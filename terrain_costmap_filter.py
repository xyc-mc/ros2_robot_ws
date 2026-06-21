#!/usr/bin/env python3
import math
import struct
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


FIELD_FORMATS = {
    PointField.INT8: ("b", 1),
    PointField.UINT8: ("B", 1),
    PointField.INT16: ("h", 2),
    PointField.UINT16: ("H", 2),
    PointField.INT32: ("i", 4),
    PointField.UINT32: ("I", 4),
    PointField.FLOAT32: ("f", 4),
    PointField.FLOAT64: ("d", 8),
}


class TerrainCostmapFilter(Node):
    """Filter terrain-analysis PointCloud2 for costmap obstacle marking.

    The terrain_analysis node publishes obstacle height above ground in the
    PointXYZI intensity field. Points with height <= passable_height are treated
    as traversable and removed from the output cloud.
    """

    def __init__(self) -> None:
        super().__init__("terrain_costmap_filter")

        self.declare_parameter("input_topic", "/terrain_map")
        self.declare_parameter("output_topic", "/terrain_map_costmap")
        self.declare_parameter("height_field", "intensity")
        self.declare_parameter("passable_height", 0.6)
        self.declare_parameter("qos_depth", 5)
        self.declare_parameter("qos_reliability", "reliable")
        self.declare_parameter("output_frame_id", "")
        self.declare_parameter("stamp_with_now", False)
        self.declare_parameter("project_to_ground", False)
        self.declare_parameter("projected_z", 0.0)
        self.declare_parameter("debug_every_n", 30)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.height_field_name = self.get_parameter("height_field").value
        self.passable_height = float(self.get_parameter("passable_height").value)
        qos_depth = int(self.get_parameter("qos_depth").value)
        self.output_frame_id = self.get_parameter("output_frame_id").value
        self.stamp_with_now = bool(self.get_parameter("stamp_with_now").value)
        self.project_to_ground = bool(self.get_parameter("project_to_ground").value)
        self.projected_z = float(self.get_parameter("projected_z").value)
        self.debug_every_n = int(self.get_parameter("debug_every_n").value)

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=max(qos_depth, 1),
            reliability=self._parse_reliability(
                self.get_parameter("qos_reliability").value
            ),
        )

        self.publisher = self.create_publisher(PointCloud2, output_topic, qos_profile)
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.cloud_callback, qos_profile
        )

        self.cloud_count = 0
        self.warned_missing_height_field = False
        self.warned_project_to_ground = False

        self.get_logger().info(
            f"subscribed to {input_topic}, publishing {output_topic}; "
            f"keeping points with {self.height_field_name} > {self.passable_height:.3f} m"
        )

    @staticmethod
    def _parse_reliability(value: str) -> ReliabilityPolicy:
        if str(value).lower() in ("best_effort", "besteffort", "best-effort"):
            return ReliabilityPolicy.BEST_EFFORT
        return ReliabilityPolicy.RELIABLE

    @staticmethod
    def _find_field(cloud_msg: PointCloud2, name: str) -> Optional[PointField]:
        for field in cloud_msg.fields:
            if field.name == name:
                return field
        return None

    @staticmethod
    def _field_format(
        field: PointField, is_bigendian: bool
    ) -> Optional[Tuple[str, int]]:
        if field.datatype not in FIELD_FORMATS:
            return None

        format_code, size = FIELD_FORMATS[field.datatype]
        endian = ">" if is_bigendian else "<"
        return f"{endian}{format_code}", size

    def cloud_callback(self, cloud_msg: PointCloud2) -> None:
        height_field = self._find_field(cloud_msg, self.height_field_name)
        if height_field is None:
            if not self.warned_missing_height_field:
                self.get_logger().error(
                    f"PointCloud2 has no '{self.height_field_name}' field; "
                    "cannot filter terrain height."
                )
                self.warned_missing_height_field = True
            return

        height_format = self._field_format(height_field, cloud_msg.is_bigendian)
        if height_format is None:
            self.get_logger().error(
                f"Unsupported datatype for field '{self.height_field_name}': "
                f"{height_field.datatype}"
            )
            return

        z_field = self._find_field(cloud_msg, "z")
        z_format = None
        if self.project_to_ground:
            if z_field is not None and z_field.datatype == PointField.FLOAT32:
                z_format = self._field_format(z_field, cloud_msg.is_bigendian)
            elif not self.warned_project_to_ground:
                self.get_logger().warn(
                    "project_to_ground is enabled, but z is not a float32 field; "
                    "publishing original z values."
                )
                self.warned_project_to_ground = True

        height_unpack_format, _ = height_format
        point_step = cloud_msg.point_step
        point_count = cloud_msg.width * cloud_msg.height
        data_view = memoryview(cloud_msg.data)
        data_len = len(data_view)

        filtered_data = bytearray()
        kept_count = 0
        valid_count = 0

        for point_index in range(point_count):
            point_offset = point_index * point_step
            if point_offset + point_step > data_len:
                break

            height = struct.unpack_from(
                height_unpack_format,
                data_view,
                point_offset + height_field.offset,
            )[0]

            if not math.isfinite(float(height)):
                continue

            valid_count += 1
            if float(height) <= self.passable_height:
                continue

            if self.project_to_ground and z_field is not None and z_format is not None:
                point_bytes = bytearray(data_view[point_offset : point_offset + point_step])
                struct.pack_into(z_format[0], point_bytes, z_field.offset, self.projected_z)
                filtered_data.extend(point_bytes)
            else:
                filtered_data.extend(data_view[point_offset : point_offset + point_step])

            kept_count += 1

        output_msg = PointCloud2()
        output_msg.header = cloud_msg.header
        if self.stamp_with_now:
            output_msg.header.stamp = self.get_clock().now().to_msg()
        if self.output_frame_id:
            output_msg.header.frame_id = self.output_frame_id

        output_msg.height = 1
        output_msg.width = kept_count
        output_msg.fields = cloud_msg.fields
        output_msg.is_bigendian = cloud_msg.is_bigendian
        output_msg.point_step = point_step
        output_msg.row_step = point_step * kept_count
        output_msg.data = bytes(filtered_data)
        output_msg.is_dense = cloud_msg.is_dense

        self.publisher.publish(output_msg)
        self._log_debug(point_count, valid_count, kept_count)

    def _log_debug(self, total_count: int, valid_count: int, kept_count: int) -> None:
        self.cloud_count += 1
        if self.debug_every_n <= 0 or self.cloud_count % self.debug_every_n != 0:
            return

        removed_count = valid_count - kept_count
        self.get_logger().info(
            f"cloud #{self.cloud_count}: input={total_count}, "
            f"valid_height={valid_count}, removed_passable={removed_count}, "
            f"output={kept_count}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TerrainCostmapFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
