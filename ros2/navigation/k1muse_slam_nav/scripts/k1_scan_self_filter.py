#!/usr/bin/env python3
import math
import xml.etree.ElementTree as ET

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def _parse_float_list(value, expected_count):
    parts = value.split()
    if len(parts) != expected_count:
        raise ValueError(f"expected {expected_count} values, got {len(parts)}")
    return [float(part) for part in parts]


def _rotate_vector(qx, qy, qz, qw, x, y, z):
    # Quaternion-vector rotation: v' = v + 2*w*(q x v) + 2*(q x (q x v)).
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


class K1ScanSelfFilter(Node):
    def __init__(self):
        super().__init__("k1_scan_self_filter")

        self.declare_parameter("input_topic", "/scan_raw")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("target_frame", "base_footprint")
        self.declare_parameter("base_link_name", "base_link")
        self.declare_parameter("footprint_padding", 0.02)
        self.declare_parameter("fallback_half_x", 0.198)
        self.declare_parameter("fallback_half_y", 0.198)
        self.declare_parameter("robot_description", "")

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.padding = float(self.get_parameter("footprint_padding").value)
        self.bounds = self._load_bounds()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(LaserScan, self.output_topic, 10)
        self.subscription = self.create_subscription(
            LaserScan,
            self.input_topic,
            self._on_scan,
            10,
        )

        self.get_logger().info(
            f"K1 scan self filter: {self.input_topic} -> {self.output_topic} "
            f"frame={self.target_frame} "
            f"bounds=[{self.bounds[0]:.3f},{self.bounds[1]:.3f}]x"
            f"[{self.bounds[2]:.3f},{self.bounds[3]:.3f}] padding={self.padding:.3f}"
        )

    def _load_bounds(self):
        robot_description = str(self.get_parameter("robot_description").value or "")
        base_link_name = str(self.get_parameter("base_link_name").value)
        if robot_description:
            try:
                root = ET.fromstring(robot_description)
                link = next(
                    (item for item in root.findall("link") if item.attrib.get("name") == base_link_name),
                    None,
                )
                if link is not None:
                    box_node = self._first_box(link, "collision") or self._first_box(link, "visual")
                    if box_node is not None:
                        origin_node, geometry_node = box_node
                        size = _parse_float_list(geometry_node.find("box").attrib["size"], 3)
                        origin = [0.0, 0.0, 0.0]
                        if origin_node is not None and "xyz" in origin_node.attrib:
                            origin = _parse_float_list(origin_node.attrib["xyz"], 3)
                        half_x = size[0] / 2.0 + self.padding
                        half_y = size[1] / 2.0 + self.padding
                        return (
                            origin[0] - half_x,
                            origin[0] + half_x,
                            origin[1] - half_y,
                            origin[1] + half_y,
                        )
            except Exception as exc:  # noqa: BLE001 - keep robot bringup alive with fallback bounds.
                self.get_logger().warning(f"Failed to parse robot_description footprint: {exc}")

        half_x = float(self.get_parameter("fallback_half_x").value) + self.padding
        half_y = float(self.get_parameter("fallback_half_y").value) + self.padding
        self.get_logger().warning(
            f"Using fallback self-filter footprint half_x={half_x:.3f} half_y={half_y:.3f}"
        )
        return (-half_x, half_x, -half_y, half_y)

    @staticmethod
    def _first_box(link, tag_name):
        for item in link.findall(tag_name):
            geometry = item.find("geometry")
            if geometry is None or geometry.find("box") is None:
                continue
            return item.find("origin"), geometry
        return None

    def _on_scan(self, msg):
        try:
            stamp = Time.from_msg(msg.header.stamp)
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                stamp,
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"No TF {msg.header.frame_id} -> {self.target_frame} for scan self-filter: {exc}",
                throttle_duration_sec=5.0,
            )
            return

        output = LaserScan()
        output.header = msg.header
        output.angle_min = msg.angle_min
        output.angle_max = msg.angle_max
        output.angle_increment = msg.angle_increment
        output.time_increment = msg.time_increment
        output.scan_time = msg.scan_time
        output.range_min = msg.range_min
        output.range_max = msg.range_max
        output.ranges = list(msg.ranges)
        output.intensities = list(msg.intensities)

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        q = transform.transform.rotation

        raw_finite = 0
        filtered = 0
        min_x, max_x, min_y, max_y = self.bounds

        angle = msg.angle_min
        for index, range_value in enumerate(msg.ranges):
            if math.isfinite(range_value):
                raw_finite += 1
                source_x = range_value * math.cos(angle)
                source_y = range_value * math.sin(angle)
                source_z = 0.0
                rx, ry, rz = _rotate_vector(q.x, q.y, q.z, q.w, source_x, source_y, source_z)
                target_x = rx + tx
                target_y = ry + ty
                _ = rz + tz
                if min_x <= target_x <= max_x and min_y <= target_y <= max_y:
                    output.ranges[index] = float("nan")
                    filtered += 1
            angle += msg.angle_increment

        ratio = filtered / raw_finite if raw_finite else 0.0
        self.get_logger().info(
            f"scan self-filter raw_finite={raw_finite} filtered={filtered} "
            f"ratio={ratio:.3f} tf=ok",
            throttle_duration_sec=5.0,
        )
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = K1ScanSelfFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
