#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace a1_low_level_controller
{
namespace
{
constexpr std::size_t kJointCount = 12;
constexpr std::size_t kReferenceSize = 3 * kJointCount;

const std::vector<std::string> kDefaultJointNames = {
  "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
  "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
  "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
  "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
};

const std::array<double, kJointCount> kCrouchPosition = {
  -0.35, 1.36, -2.65,
  0.35, 1.36, -2.65,
  -0.50, 1.36, -2.65,
  0.50, 1.36, -2.65,
};

struct JointReference
{
  std::array<double, kJointCount> position = kCrouchPosition;
  std::array<double, kJointCount> kp = {
    12.0, 12.0, 12.0, 12.0, 12.0, 12.0,
    12.0, 12.0, 12.0, 12.0, 12.0, 12.0,
  };
  std::array<double, kJointCount> kd = {
    2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
    2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
  };
};
}  // namespace

class A1EffortController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    try {
      auto_declare<std::vector<std::string>>("joints", kDefaultJointNames);
      auto_declare<std::string>(
        "reference_topic", "/a1_low_level_controller/reference");
      auto_declare<double>("torque_limit", 33.5);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", error.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    reference_buffer_.initRT(JointReference{});
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration configuration;
    configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names_) {
      configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    }
    return configuration;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration configuration;
    configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names_) {
      configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
      configuration.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return configuration;
  }

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    if (joint_names_.size() != kJointCount) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu joints, got %zu",
        kJointCount, joint_names_.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    auto sorted_names = joint_names_;
    std::sort(sorted_names.begin(), sorted_names.end());
    if (std::adjacent_find(sorted_names.begin(), sorted_names.end()) != sorted_names.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Joint names must be unique");
      return controller_interface::CallbackReturn::ERROR;
    }

    torque_limit_ = get_node()->get_parameter("torque_limit").as_double();
    if (!std::isfinite(torque_limit_) || torque_limit_ <= 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "torque_limit must be finite and positive");
      return controller_interface::CallbackReturn::ERROR;
    }

    const auto reference_topic =
      get_node()->get_parameter("reference_topic").as_string();
    if (reference_topic.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "reference_topic must not be empty");
      return controller_interface::CallbackReturn::ERROR;
    }

    reference_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      reference_topic, rclcpp::QoS(1),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr message) {
        if (message->data.size() != kReferenceSize) {
          RCLCPP_WARN(
            get_node()->get_logger(), "Ignoring reference with %zu values; expected %zu",
            message->data.size(), kReferenceSize);
          return;
        }

        JointReference next;
        for (std::size_t index = 0; index < kJointCount; ++index) {
          const double position = message->data[index];
          const double kp = message->data[kJointCount + index];
          const double kd = message->data[2 * kJointCount + index];
          if (!std::isfinite(position) || !std::isfinite(kp) || !std::isfinite(kd)) {
            RCLCPP_WARN(get_node()->get_logger(), "Ignoring non-finite A1 reference");
            return;
          }
          next.position[index] = position;
          next.kp[index] = std::clamp(kp, 0.0, 200.0);
          next.kd[index] = std::clamp(kd, 0.0, 20.0);
        }
        reference_buffer_.writeFromNonRT(next);
      });

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Configured synchronous A1 PD controller on %s", reference_topic.c_str());
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    if (command_interfaces_.size() != kJointCount ||
      state_interfaces_.size() != 2 * kJointCount)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unexpected interface count: %zu command, %zu state",
        command_interfaces_.size(), state_interfaces_.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    initialized_ = false;
    for (auto & command_interface : command_interfaces_) {
      command_interface.set_value(0.0);
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    for (auto & command_interface : command_interfaces_) {
      command_interface.set_value(0.0);
    }
    initialized_ = false;
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period) override
  {
    const double dt = period.seconds();
    if (!std::isfinite(dt) || dt <= 0.0) {
      return controller_interface::return_type::OK;
    }

    std::array<double, kJointCount> position{};
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      position[joint] = state_interfaces_[2 * joint].get_value();
      if (!std::isfinite(position[joint])) {
        for (auto & command_interface : command_interfaces_) {
          command_interface.set_value(0.0);
        }
        return controller_interface::return_type::ERROR;
      }
    }

    if (!initialized_) {
      last_position_ = position;
      filtered_velocity_.fill(0.0);
      initialized_ = true;
    }

    const JointReference reference = *reference_buffer_.readFromRT();
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      const double raw_velocity = (position[joint] - last_position_[joint]) / dt;
      const double velocity =
        0.35 * filtered_velocity_[joint] + 0.65 * raw_velocity;
      const double effort =
        reference.kp[joint] * (reference.position[joint] - position[joint]) -
        reference.kd[joint] * velocity;

      command_interfaces_[joint].set_value(
        std::clamp(effort, -torque_limit_, torque_limit_));
      last_position_[joint] = position[joint];
      filtered_velocity_[joint] = velocity;
    }

    return controller_interface::return_type::OK;
  }

private:
  std::vector<std::string> joint_names_ = kDefaultJointNames;
  double torque_limit_ = 33.5;
  bool initialized_ = false;
  std::array<double, kJointCount> last_position_{};
  std::array<double, kJointCount> filtered_velocity_{};
  realtime_tools::RealtimeBuffer<JointReference> reference_buffer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    reference_subscription_;
};
}  // namespace a1_low_level_controller

PLUGINLIB_EXPORT_CLASS(
  a1_low_level_controller::A1EffortController,
  controller_interface::ControllerInterface)
