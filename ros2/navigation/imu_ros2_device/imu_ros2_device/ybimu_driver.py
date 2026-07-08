#!/usr/bin/env python3

from YbImuLib import YbImuSerial

import fcntl
import math
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32MultiArray


STANDARD_GRAVITY = 9.80665


class YbImuDriver(Node):
    def __init__(self, name):
        super().__init__(name)
        self.robot = None
        self.port_lock = None
        self.last_valid_quaternion = (1.0, 0.0, 0.0, 0.0)
        self.declare_parameter("port", "/dev/myimu")
        self.declare_parameter("fallback_ports", ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"])
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("publish_rate", 10.0)

        # 参数化 publisher：默认只启用主 IMU 数据，其他调试 topic 按需打开。
        # 默认只发 /imu/data（标准 sensor_msgs/Imu，含 orientation+gyro+accel），
        # 其余 (data_raw / mag / baro / euler) 默认关。调试需要再 -p enable_*:=true。
        self.declare_parameter("imu_data_topic", "imu/data")
        self.declare_parameter("enable_imu_data", True)
        self.declare_parameter("imu_topic", "imu/data_raw")
        self.declare_parameter("enable_imu_raw", False)
        self.declare_parameter("mag_topic", "imu/mag")
        self.declare_parameter("enable_mag", False)
        self.declare_parameter("baro_topic", "baro")
        self.declare_parameter("enable_baro", False)
        self.declare_parameter("euler_topic", "euler")
        self.declare_parameter("enable_euler", False)

        self.frame_id = self.get_parameter("frame_id").value
        # Publisher 槽位，按 enable_* 决定是否实例化
        self.imuDataPublisher = None
        self.imuPublisher = None
        self.magPublisher = None
        self.baroPublisher = None
        self.eulerPublisher = None

    def init_topic(self):
        port = self.get_parameter("port").value
        fallback_ports = list(self.get_parameter("fallback_ports").value)
        port_list = [port] + [fallback for fallback in fallback_ports if fallback != port]
        seen_devices = set()

        for candidate_port in port_list:
            resolved_port = str(Path(candidate_port).resolve())
            if resolved_port in seen_devices:
                continue
            seen_devices.add(resolved_port)

            try:
                self.port_lock = self.acquire_port_lock(resolved_port)
                self.robot = YbImuSerial(candidate_port)
                self.get_logger().info("Open Ybimu Port OK:%s" % candidate_port)
                break
            except BlockingIOError:
                self.get_logger().warn(
                    "Ybimu port %s is already locked by another driver process" % candidate_port
                )
                self.port_lock = None
            except Exception as exc:
                self.get_logger().warn("Failed to open Ybimu port %s: %s" % (candidate_port, exc))
                if self.port_lock is not None:
                    self.port_lock.close()
                    self.port_lock = None

        if self.robot is None:
            self.get_logger().error("---------Fail To Open Ybimu Serial------------")
            return False

        self.robot.create_receive_threading()

        # 按 enable_* 参数选择性创建 publisher。
        enabled_topics = []
        if self.get_parameter("enable_imu_data").value:
            topic = self.get_parameter("imu_data_topic").value
            self.imuDataPublisher = self.create_publisher(Imu, topic, 100)
            enabled_topics.append(topic)
        if self.get_parameter("enable_imu_raw").value:
            topic = self.get_parameter("imu_topic").value
            self.imuPublisher = self.create_publisher(Imu, topic, 100)
            enabled_topics.append(topic)
        if self.get_parameter("enable_mag").value:
            topic = self.get_parameter("mag_topic").value
            self.magPublisher = self.create_publisher(MagneticField, topic, 100)
            enabled_topics.append(topic)
        if self.get_parameter("enable_baro").value:
            topic = self.get_parameter("baro_topic").value
            self.baroPublisher = self.create_publisher(Float32MultiArray, topic, 100)
            enabled_topics.append(topic)
        if self.get_parameter("enable_euler").value:
            topic = self.get_parameter("euler_topic").value
            self.eulerPublisher = self.create_publisher(Float32MultiArray, topic, 100)
            enabled_topics.append(topic)

        if enabled_topics:
            self.get_logger().info("Publishing: " + ", ".join(enabled_topics))
        else:
            self.get_logger().warn("No IMU publisher enabled — set enable_* params to true")

        publish_rate = float(self.get_parameter("publish_rate").value)
        if publish_rate <= 0.0:
            self.get_logger().warn("Invalid publish_rate %.3f, using 10 Hz" % publish_rate)
            publish_rate = 10.0
        self.timer = self.create_timer(1.0 / publish_rate, self.pub_data)
        return True

    def acquire_port_lock(self, resolved_port):
        lock_name = resolved_port.strip("/").replace("/", "_")
        lock_path = Path("/tmp") / ("%s.lock" % lock_name)
        lock_file = lock_path.open("w")
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        lock_file.write(str(self.get_clock().now().nanoseconds))
        lock_file.flush()
        return lock_file

    def get_valid_quaternion(self, q0, q1, q2, q3):
        quaternion = (float(q0), float(q1), float(q2), float(q3))
        norm_sq = sum(value * value for value in quaternion)
        if norm_sq < 1e-12 or not all(math.isfinite(value) for value in quaternion):
            return self.last_valid_quaternion

        norm = math.sqrt(norm_sq)
        normalized = tuple(value / norm for value in quaternion)
        self.last_valid_quaternion = normalized
        return normalized

    def pub_data(self):
        if self.robot is None:
            return

        time_stamp = self.get_clock().now()

        # Imu 消息：data 和 data_raw 用同一份内容（标准 sensor_msgs/Imu 完整字段）
        need_imu = (self.imuDataPublisher is not None) or (self.imuPublisher is not None)
        if need_imu:
            ax, ay, az = self.robot.get_accelerometer_data()
            gx, gy, gz = self.robot.get_gyroscope_data()
            q0, q1, q2, q3 = self.robot.get_imu_quaternion_data()
            q0, q1, q2, q3 = self.get_valid_quaternion(q0, q1, q2, q3)

            imu = Imu()
            imu.header.stamp = time_stamp.to_msg()
            imu.header.frame_id = self.frame_id
            imu.linear_acceleration.x = ax * STANDARD_GRAVITY
            imu.linear_acceleration.y = ay * STANDARD_GRAVITY
            imu.linear_acceleration.z = az * STANDARD_GRAVITY
            imu.angular_velocity.x = gx
            imu.angular_velocity.y = gy
            imu.angular_velocity.z = gz
            imu.orientation.w = q0
            imu.orientation.x = q1
            imu.orientation.y = q2
            imu.orientation.z = q3

            if self.imuDataPublisher is not None:
                self.imuDataPublisher.publish(imu)
            if self.imuPublisher is not None:
                self.imuPublisher.publish(imu)

        if self.magPublisher is not None:
            mx, my, mz = self.robot.get_magnetometer_data()
            mag = MagneticField()
            mag.header.stamp = time_stamp.to_msg()
            mag.header.frame_id = self.frame_id
            mag.magnetic_field.x = mx
            mag.magnetic_field.y = -my
            mag.magnetic_field.z = mz
            self.magPublisher.publish(mag)

        if self.baroPublisher is not None:
            height, temperature, pressure, pressure_contrast = self.robot.get_baro_data()
            baro = Float32MultiArray()
            baro.data = [height, temperature, pressure, pressure_contrast]
            self.baroPublisher.publish(baro)

        if self.eulerPublisher is not None:
            roll, pitch, yaw = self.robot.get_imu_attitude_data(True)
            euler = Float32MultiArray()
            euler.data = [roll, pitch, yaw]
            self.eulerPublisher.publish(euler)


def main(args=None):
    rclpy.init(args=args)
    node = YbImuDriver("ybimu_node")
    try:
        if node.init_topic():
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if node.port_lock is not None:
            node.port_lock.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
