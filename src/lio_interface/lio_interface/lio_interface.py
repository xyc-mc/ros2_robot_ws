import rclpy
import numpy as np
import tf_transformations
from array import array
from copy import deepcopy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField


class LioInterface(Node):
    def __init__(self):
        super().__init__('lio_interface_node')

        self.cur_map_to_odom = None
        self.cur_lio_odom = None
        self.warned_missing_map_to_odom_for_cloud = False
        self.warned_missing_lio_odom_for_cloud = False
        self.warned_missing_map_to_odom_for_odom = False
        self.warned_invalid_cloud = False

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('sensor_frame', 'sensor')
        self.declare_parameter('input_cloud_topic', '/cloud_registered_body')
        self.declare_parameter('body_to_sensor_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('body_to_sensor_rpy', [0.0, 0.0, 0.0])

        self.map_frame = self.get_parameter('map_frame').value
        self.sensor_frame = self.get_parameter('sensor_frame').value
        self.input_cloud_topic = self.get_parameter('input_cloud_topic').value
        body_to_sensor_xyz = self.read_vector_parameter('body_to_sensor_xyz', 3)
        body_to_sensor_rpy = self.read_vector_parameter('body_to_sensor_rpy', 3)
        self.T_body_to_sensor = self.xyz_rpy_to_matrix(body_to_sensor_xyz, body_to_sensor_rpy)

        self.odom_pub = self.create_publisher(Odometry, '/state_estimation', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/registered_scan', 10)

        self.lio_odom_sub = self.create_subscription(Odometry, '/Odometry', self.lio_odom_callback, 10)
        self.lio_pointcloud_sub = self.create_subscription(PointCloud2, self.input_cloud_topic, self.lio_pointcloud_callback, 10)
        self.map_to_odom_sub = self.create_subscription(Odometry, "/map_to_odom", self.cb_save_map_to_odom, 1)
        
    def lio_pointcloud_callback(self, msg):
        if self.cur_map_to_odom is None:
            if not self.warned_missing_map_to_odom_for_cloud:
                self.get_logger().warn('Waiting for /map_to_odom before publishing /registered_scan')
                self.warned_missing_map_to_odom_for_cloud = True
            return
        if self.cur_lio_odom is None:
            if not self.warned_missing_lio_odom_for_cloud:
                self.get_logger().warn('Waiting for /Odometry before publishing /registered_scan')
                self.warned_missing_lio_odom_for_cloud = True
            return

        try:
            msg_pointcloud = self.transform_cloud_to_map(msg, self.cur_map_to_odom, self.cur_lio_odom)
        except ValueError as exc:
            if not self.warned_invalid_cloud:
                self.get_logger().error(f'Failed to transform {self.input_cloud_topic}: {exc}')
                self.warned_invalid_cloud = True
            return

        self.pointcloud_pub.publish(msg_pointcloud)

    def read_vector_parameter(self, name, expected_size):
        value = list(self.get_parameter(name).value)
        if len(value) != expected_size:
            raise ValueError(f'Parameter "{name}" must contain {expected_size} values')
        return [float(item) for item in value]

    def pose_to_matrix(self, pose):
        transform = tf_transformations.quaternion_matrix([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ])
        transform[:3, 3] = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
        ]
        return transform

    def xyz_rpy_to_matrix(self, xyz, rpy):
        transform = tf_transformations.euler_matrix(rpy[0], rpy[1], rpy[2])
        transform[:3, 3] = xyz
        return transform
        
    def transform_cloud_to_map(self, cloud_msg, map_to_camera_init_msg, camera_init_to_body_msg):
        fields = {field.name: field for field in cloud_msg.fields}
        required_fields = ('x', 'y', 'z')
        for field_name in required_fields:
            if field_name not in fields:
                raise ValueError(f'PointCloud2 is missing field "{field_name}"')
            field = fields[field_name]
            if field.datatype != PointField.FLOAT32 or field.count != 1:
                raise ValueError(f'PointCloud2 field "{field_name}" must be FLOAT32 count 1')

        if cloud_msg.point_step <= 0:
            raise ValueError('PointCloud2 point_step must be positive')
        if cloud_msg.row_step < cloud_msg.point_step * cloud_msg.width:
            raise ValueError('PointCloud2 row_step is smaller than point_step * width')

        raw_data = bytearray(cloud_msg.data)
        expected_size = cloud_msg.row_step * cloud_msg.height
        if len(raw_data) < expected_size:
            raise ValueError('PointCloud2 data is smaller than row_step * height')

        transform = (
            self.pose_to_matrix(map_to_camera_init_msg.pose.pose) @
            self.pose_to_matrix(camera_init_to_body_msg.pose.pose)
        )
        rotation = transform[:3, :3]
        translation = transform[:3, 3]

        if cloud_msg.width > 0 and cloud_msg.height > 0:
            dtype_prefix = '>' if cloud_msg.is_bigendian else '<'
            xyz_dtype = np.dtype({
                'names': required_fields,
                'formats': [dtype_prefix + 'f4'] * 3,
                'offsets': [fields[name].offset for name in required_fields],
                'itemsize': cloud_msg.point_step,
            })
            xyz_view = np.ndarray(
                shape=(cloud_msg.height, cloud_msg.width),
                dtype=xyz_dtype,
                buffer=raw_data,
                strides=(cloud_msg.row_step, cloud_msg.point_step),
            )

            xyz = np.empty((cloud_msg.height, cloud_msg.width, 3), dtype=np.float64)
            xyz[..., 0] = xyz_view['x']
            xyz[..., 1] = xyz_view['y']
            xyz[..., 2] = xyz_view['z']

            transformed_xyz = np.matmul(xyz, rotation.T) + translation
            xyz_view['x'] = transformed_xyz[..., 0].astype(np.float32)
            xyz_view['y'] = transformed_xyz[..., 1].astype(np.float32)
            xyz_view['z'] = transformed_xyz[..., 2].astype(np.float32)

        transformed_msg = PointCloud2()
        transformed_msg.header.stamp = cloud_msg.header.stamp
        transformed_msg.header.frame_id = self.map_frame
        transformed_msg.height = cloud_msg.height
        transformed_msg.width = cloud_msg.width
        transformed_msg.fields = deepcopy(cloud_msg.fields)
        transformed_msg.is_bigendian = cloud_msg.is_bigendian
        transformed_msg.point_step = cloud_msg.point_step
        transformed_msg.row_step = cloud_msg.row_step
        transformed_msg.data = array('B', raw_data)
        transformed_msg.is_dense = cloud_msg.is_dense

        return transformed_msg

    def lio_odom_callback(self, msg):
        self.cur_lio_odom = msg

        if self.cur_map_to_odom is None:
            if not self.warned_missing_map_to_odom_for_odom:
                self.get_logger().warn('Waiting for /map_to_odom before publishing /state_estimation')
                self.warned_missing_map_to_odom_for_odom = True
            return

        T_map_to_odom = self.pose_to_matrix(self.cur_map_to_odom.pose.pose)
        T_odom_to_body = self.pose_to_matrix(msg.pose.pose)
        T_map_to_sensor = T_map_to_odom @ T_odom_to_body @ self.T_body_to_sensor

        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = self.map_frame
        odom_msg.child_frame_id = self.sensor_frame

        quat = tf_transformations.quaternion_from_matrix(T_map_to_sensor)
        odom_msg.pose.pose.position.x = float(T_map_to_sensor[0, 3])
        odom_msg.pose.pose.position.y = float(T_map_to_sensor[1, 3])
        odom_msg.pose.pose.position.z = float(T_map_to_sensor[2, 3])
        odom_msg.pose.pose.orientation.x = float(quat[0])
        odom_msg.pose.pose.orientation.y = float(quat[1])
        odom_msg.pose.pose.orientation.z = float(quat[2])
        odom_msg.pose.pose.orientation.w = float(quat[3])

        odom_msg.pose.covariance = msg.pose.covariance

        self.odom_pub.publish(odom_msg)
    
    def cb_save_map_to_odom(self, msg):
        self.cur_map_to_odom = msg

def main():
    rclpy.init()
    node = LioInterface()
    rclpy.spin(node)
    rclpy.shutdown()




        
