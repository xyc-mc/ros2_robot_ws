import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger


JOINT_NAMES = (
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
)

# ROS controller order is FR, FL, RR, RL. The policy uses FL, FR, RL, RR.
POLICY_FROM_CONTROLLER = np.array((3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8))
CROUCH_POSITION = np.array(
    (-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
     -0.50, 1.36, -2.65, 0.50, 1.36, -2.65),
    dtype=np.float32,
)
STAND_POSITION = np.array((0.0, 0.9, -1.8) * 4, dtype=np.float32)
POLICY_DEFAULT = np.array(
    (-0.15, 0.55, -1.5, 0.15, 0.55, -1.5,
     -0.15, 0.7, -1.5, 0.15, 0.7, -1.5),
    dtype=np.float32,
)
JOINT_LOWER = np.array((-0.80, -1.04, -2.69) * 4, dtype=np.float32)
JOINT_UPPER = np.array((0.80, 4.18, -0.92) * 4, dtype=np.float32)


def _smooth_step(value: float) -> float:
    value = min(1.0, max(0.0, value))
    return value * value * (3.0 - 2.0 * value)


def _inverse_rotate(q_xyzw: np.ndarray, vector: np.ndarray) -> np.ndarray:
    q_vec = q_xyzw[:3]
    q_w = q_xyzw[3]
    return (
        vector * (2.0 * q_w * q_w - 1.0)
        - 2.0 * q_w * np.cross(q_vec, vector)
        + 2.0 * q_vec * np.dot(q_vec, vector)
    )


class A1Controller(Node):
    def __init__(self) -> None:
        super().__init__("a1_controller")
        self._group = ReentrantCallbackGroup()
        self._control_group = MutuallyExclusiveCallbackGroup()
        self._inference_group = MutuallyExclusiveCallbackGroup()
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._lock = threading.RLock()

        self.declare_parameter("reference_rate", 100.0)
        self.declare_parameter("inference_rate", 50.0)
        self.declare_parameter("feedback_rate", 200.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("state_timeout", 0.25)
        self.declare_parameter("stand_settle_duration", 0.5)
        self.declare_parameter("stand_duration", 3.0)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("locomotion_mode", "rl")
        self.declare_parameter("policy_file", "policy_act_inference_stair.pt")
        self.declare_parameter("stamped_cmd_vel_topic", "/cmd_vel_stamped")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter(
            "reference_topic", "/a1_low_level_controller/reference"
        )
        self.declare_parameter("max_linear_x", 0.6)
        self.declare_parameter("max_linear_y", 0.35)
        self.declare_parameter("max_angular_z", 0.9)

        self._reference_rate = float(self.get_parameter("reference_rate").value)
        self._inference_rate = float(self.get_parameter("inference_rate").value)
        feedback_rate = float(self.get_parameter("feedback_rate").value)
        self._feedback_period = 1.0 / max(feedback_rate, 1.0)
        self._command_timeout = float(self.get_parameter("command_timeout").value)
        self._state_timeout = float(self.get_parameter("state_timeout").value)
        self._settle_duration = float(self.get_parameter("stand_settle_duration").value)
        self._stand_duration = float(self.get_parameter("stand_duration").value)
        self._enabled = bool(self.get_parameter("auto_start").value)
        self._mode = str(self.get_parameter("locomotion_mode").value).lower()
        self._max_command = np.array(
            (
                float(self.get_parameter("max_linear_x").value),
                float(self.get_parameter("max_linear_y").value),
                float(self.get_parameter("max_angular_z").value),
            ),
            dtype=np.float32,
        )
        self._positions = np.zeros(12, dtype=np.float32)
        self._velocities = np.zeros(12, dtype=np.float32)
        self._orientation = np.array((0.0, 0.0, 0.0, 1.0), dtype=np.float32)
        self._angular_velocity = np.zeros(3, dtype=np.float32)
        self._command = np.zeros(3, dtype=np.float32)
        self._active_target = STAND_POSITION.copy()
        self._stand_start = CROUCH_POSITION.copy()
        self._phase = "waiting"
        self._phase_started = time.monotonic()
        self._last_joint_state = 0.0
        self._last_imu = 0.0
        self._last_joint_callback = 0.0
        self._last_imu_callback = 0.0
        self._last_command = 0.0
        self._warned_stale = False
        self._fall_latched = False

        self._torch = None
        self._policy = None
        self._history = None
        self._last_action = None
        self._history_initialized = False
        if self._mode == "rl":
            self._load_policy()
        elif self._mode != "stand":
            self.get_logger().warning(
                f"Unknown locomotion_mode '{self._mode}', using stand mode"
            )
            self._mode = "stand"

        self._reference_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("reference_topic").value),
            1,
        )
        self._state_pub = self.create_publisher(String, "~/state", 1)
        feedback_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            feedback_qos,
            callback_group=self._group,
        )
        self.create_subscription(
            Imu,
            "/imu",
            self._imu_callback,
            feedback_qos,
            callback_group=self._group,
        )
        self.create_subscription(
            TwistStamped,
            str(self.get_parameter("stamped_cmd_vel_topic").value),
            self._stamped_command_callback,
            10,
            callback_group=self._group,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self._command_callback,
            10,
            callback_group=self._group,
        )
        self.create_service(
            SetBool, "~/enable", self._enable_callback, callback_group=self._group
        )
        self.create_service(
            Trigger, "~/stand", self._stand_callback, callback_group=self._group
        )
        self.create_service(
            Trigger, "~/passive", self._passive_callback, callback_group=self._group
        )
        self.create_timer(
            1.0 / self._reference_rate,
            self._control_callback,
            callback_group=self._control_group,
            clock=self._steady_clock,
        )
        self.create_timer(
            1.0 / self._inference_rate,
            self._inference_callback,
            callback_group=self._inference_group,
            clock=self._steady_clock,
        )
        self.create_timer(
            0.5,
            self._publish_state,
            callback_group=self._group,
            clock=self._steady_clock,
        )

        self.get_logger().info(
            "A1 controller ready: 500 Hz C++ PD loop, 50 Hz RL inference; "
            "waiting for /joint_states and /imu"
        )

    def _load_policy(self) -> None:
        try:
            import torch

            share = Path(get_package_share_directory("a1_controller"))
            policy_path = share / "policy" / str(self.get_parameter("policy_file").value)
            self._policy = torch.jit.load(str(policy_path), map_location="cpu")
            self._policy.eval()
            self._torch = torch
            self._history = torch.zeros((5, 45), dtype=torch.float32)
            self._last_action = torch.zeros(12, dtype=torch.float32)
            with torch.inference_mode():
                output = self._policy.act_inference(self._history.reshape(1, 225))
            if tuple(output.shape) not in ((1, 12), (12,)):
                raise RuntimeError(f"unexpected policy output shape {tuple(output.shape)}")
            self.get_logger().info(f"Loaded A1 TorchScript policy: {policy_path}")
        except Exception as error:  # Keep the robot standing if policy loading fails.
            self._mode = "stand"
            self._policy = None
            self.get_logger().error(f"RL policy unavailable, falling back to stand: {error}")

    def _joint_state_callback(self, message: JointState) -> None:
        callback_time = time.monotonic()
        with self._lock:
            if callback_time - self._last_joint_callback < self._feedback_period:
                return
            self._last_joint_callback = callback_time
        index = {name: position for position, name in enumerate(message.name)}
        if any(name not in index for name in JOINT_NAMES):
            return
        positions = np.array([message.position[index[name]] for name in JOINT_NAMES])
        if len(message.velocity) == len(message.name):
            velocities = np.array([message.velocity[index[name]] for name in JOINT_NAMES])
        else:
            velocities = np.zeros(12)
        if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(velocities)):
            return
        with self._lock:
            self._positions = positions.astype(np.float32)
            self._velocities = velocities.astype(np.float32)
            self._last_joint_state = callback_time

    def _imu_callback(self, message: Imu) -> None:
        callback_time = time.monotonic()
        with self._lock:
            if callback_time - self._last_imu_callback < self._feedback_period:
                return
            self._last_imu_callback = callback_time
        orientation = np.array(
            (
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
                message.orientation.w,
            ),
            dtype=np.float32,
        )
        angular_velocity = np.array(
            (
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ),
            dtype=np.float32,
        )
        norm = float(np.linalg.norm(orientation))
        if not np.all(np.isfinite(orientation)) or not np.all(np.isfinite(angular_velocity)):
            return
        if norm < 1.0e-6:
            return
        with self._lock:
            self._orientation = orientation / norm
            self._angular_velocity = angular_velocity
            self._last_imu = callback_time

    def _store_command(self, message: Twist) -> None:
        command = np.array(
            (message.linear.x, message.linear.y, message.angular.z), dtype=np.float32
        )
        if not np.all(np.isfinite(command)):
            command.fill(0.0)
        command = np.clip(command, -self._max_command, self._max_command)
        with self._lock:
            self._command = command
            self._last_command = time.monotonic()

    def _stamped_command_callback(self, message: TwistStamped) -> None:
        self._store_command(message.twist)

    def _command_callback(self, message: Twist) -> None:
        self._store_command(message)

    def _enable_callback(self, request: SetBool.Request, response: SetBool.Response):
        with self._lock:
            self._enabled = request.data
            if request.data:
                self._fall_latched = False
                self._begin_stand_locked()
            else:
                self._phase = "crouch"
                self._phase_started = time.monotonic()
        response.success = True
        response.message = "enabled" if request.data else "crouching"
        return response

    def _stand_callback(self, _request: Trigger.Request, response: Trigger.Response):
        with self._lock:
            self._enabled = True
            self._mode = "stand"
            self._fall_latched = False
            self._begin_stand_locked()
        response.success = True
        response.message = "standing"
        return response

    def _passive_callback(self, _request: Trigger.Request, response: Trigger.Response):
        with self._lock:
            self._enabled = False
            self._phase = "crouch"
            self._phase_started = time.monotonic()
        response.success = True
        response.message = "crouching"
        return response

    def _begin_stand_locked(self) -> None:
        self._stand_start = self._positions.copy()
        self._phase = "standing"
        self._phase_started = time.monotonic()
        self._history_initialized = False
        if self._history is not None:
            self._history.zero_()
        if self._last_action is not None:
            self._last_action.zero_()

    def _state_ready_locked(self, now: float) -> bool:
        return (
            now - self._last_joint_state <= self._state_timeout
            and now - self._last_imu <= self._state_timeout
        )

    def _control_callback(self) -> None:
        now = time.monotonic()
        with self._lock:
            if not self._state_ready_locked(now):
                if self._last_joint_state > 0.0 and not self._warned_stale:
                    self.get_logger().warning(
                        "A1 state feedback is stale; low-level controller will hold crouch"
                    )
                    self._warned_stale = True
                return

            self._warned_stale = False
            upright = 1.0 - 2.0 * (
                self._orientation[0] * self._orientation[0]
                + self._orientation[1] * self._orientation[1]
            )
            if upright < 0.5 and not self._fall_latched:
                self.get_logger().error(
                    "A1 fall protection triggered; call ~/enable to recover after reset"
                )
                self._fall_latched = True
                self._enabled = False
                self._phase = "crouch"
                self._phase_started = now
            if self._phase == "waiting":
                self._phase = "crouch"
                self._phase_started = now

            elapsed = now - self._phase_started
            if self._phase == "crouch":
                target = CROUCH_POSITION
                kp = np.full(12, 12.0)
                kd = np.full(12, 2.0)
                if self._enabled and elapsed >= self._settle_duration:
                    self._begin_stand_locked()
            elif self._phase == "standing":
                ratio = _smooth_step(elapsed / max(self._stand_duration, 1.0e-3))
                target = self._stand_start + ratio * (STAND_POSITION - self._stand_start)
                kp_leg = (
                    15.0 + 80.0 * ratio,
                    15.0 + 80.0 * ratio,
                    25.0 + 115.0 * ratio,
                )
                kd_leg = (
                    1.5 + 3.5 * ratio,
                    1.5 + 3.5 * ratio,
                    2.0 + 5.0 * ratio,
                )
                kp = np.array(kp_leg * 4)
                kd = np.array(kd_leg * 4)
                if elapsed >= self._stand_duration:
                    self._phase = "active"
                    self._phase_started = now
                    self._active_target = STAND_POSITION.copy()
            else:
                target = self._active_target
                kp = np.full(12, 80.0)
                kd = np.full(12, 1.0)

            reference = np.concatenate((target, kp, kd)).astype(float).tolist()

        output = Float64MultiArray()
        output.data = reference
        self._reference_pub.publish(output)

    def _inference_callback(self) -> None:
        if self._mode != "rl" or self._policy is None:
            return
        now = time.monotonic()
        with self._lock:
            if self._phase != "active" or not self._state_ready_locked(now):
                return
            positions = self._positions.copy()
            velocities = self._velocities.copy()
            orientation = self._orientation.copy()
            angular_velocity = self._angular_velocity.copy()
            command = self._command.copy()
            if now - self._last_command > self._command_timeout:
                command.fill(0.0)

        q_policy = positions[POLICY_FROM_CONTROLLER]
        qd_policy = velocities[POLICY_FROM_CONTROLLER]
        projected_gravity = _inverse_rotate(
            orientation, np.array((0.0, 0.0, -1.0), dtype=np.float32)
        )
        observation = np.concatenate(
            (
                angular_velocity * 0.25,
                projected_gravity,
                command * np.array((2.0, 2.0, 0.25), dtype=np.float32),
                q_policy - POLICY_DEFAULT,
                qd_policy * 0.05,
                self._last_action.numpy(),
            )
        ).astype(np.float32)

        try:
            with self._torch.inference_mode():
                obs = self._torch.from_numpy(observation)
                if not self._history_initialized:
                    self._history = obs.unsqueeze(0).repeat(5, 1)
                    self._history_initialized = True
                else:
                    self._history = self._torch.cat(
                        (self._history[1:], obs.unsqueeze(0)), dim=0
                    )
                action = self._policy.act_inference(
                    self._history.reshape(1, 225)
                ).cpu().reshape(12)
                if not bool(self._torch.isfinite(action).all()):
                    raise RuntimeError("policy returned non-finite action")
                self._last_action = action.clone()
                policy_target = POLICY_DEFAULT + 0.25 * action.numpy()
            controller_target = policy_target[POLICY_FROM_CONTROLLER]
            controller_target = np.clip(controller_target, JOINT_LOWER, JOINT_UPPER)
            with self._lock:
                self._active_target = controller_target.astype(np.float32)
        except Exception as error:
            self.get_logger().error(f"A1 policy inference failed; holding stand: {error}")
            with self._lock:
                self._active_target = STAND_POSITION.copy()
            self._mode = "stand"

    def _publish_state(self) -> None:
        message = String()
        with self._lock:
            message.data = f"{self._phase}:{self._mode}"
        self._state_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = A1Controller()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, Exception):
            pass


if __name__ == "__main__":
    main()
