#!/usr/bin/env python3
"""Record all odometry estimator inputs and outputs in one headed CSV event log."""

import argparse
from collections import Counter
import csv
from pathlib import Path

import rclpy
from flow_range_sensor_node.msg import OpticalFlow
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Bool


IDENTITY_COLUMNS = ['category', 'topic', 'message_type']
MESSAGE_INFO_COLUMNS = [
    'callback_time_ns',
    'source_time_ns',
    'received_time_ns',
    'publication_sequence_number',
    'reception_sequence_number',
]
HEADER_COLUMNS = ['stamp_sec', 'stamp_nanosec', 'frame_id']


def indexed_columns(prefix, length):
    return [f'{prefix}_{index}' for index in range(length)]


CSV_COLUMNS = [
    *IDENTITY_COLUMNS,
    *MESSAGE_INFO_COLUMNS,
    *HEADER_COLUMNS,
    'calibrated',
    'imu_orientation_x', 'imu_orientation_y', 'imu_orientation_z', 'imu_orientation_w',
    *indexed_columns('imu_orientation_covariance', 9),
    'imu_angular_velocity_x', 'imu_angular_velocity_y', 'imu_angular_velocity_z',
    *indexed_columns('imu_angular_velocity_covariance', 9),
    'imu_linear_acceleration_x', 'imu_linear_acceleration_y', 'imu_linear_acceleration_z',
    *indexed_columns('imu_linear_acceleration_covariance', 9),
    'flow_motion_detected', 'flow_delta_x', 'flow_delta_y', 'flow_quality',
    'flow_observation', 'flow_raw_data_sum', 'flow_raw_data_max', 'flow_raw_data_min',
    'flow_shutter', 'flow_integration_time', 'flow_range_valid', 'flow_ground_distance',
    'range_radiation_type', 'range_field_of_view', 'range_min', 'range_max', 'range_distance',
    'odom_child_frame_id',
    'odom_position_x', 'odom_position_y', 'odom_position_z',
    'odom_orientation_x', 'odom_orientation_y', 'odom_orientation_z', 'odom_orientation_w',
    *indexed_columns('odom_pose_covariance', 36),
    'odom_linear_velocity_x', 'odom_linear_velocity_y', 'odom_linear_velocity_z',
    'odom_angular_velocity_x', 'odom_angular_velocity_y', 'odom_angular_velocity_z',
    *indexed_columns('odom_twist_covariance', 36),
]


def message_info_field(info, name, default=0):
    if hasattr(info, 'get'):
        return info.get(name, default)
    return getattr(info, name, default)


def indexed_values(prefix, values):
    return {f'{prefix}_{index}': value for index, value in enumerate(values)}


class OdomBlackboxCsvRecorder(Node):
    def __init__(self, args):
        super().__init__('odom_blackbox_csv_recorder')
        output = Path(args.output_directory)
        output.mkdir(parents=True, exist_ok=True)
        self.output_path = output / 'odom_blackbox.csv'
        self.handle = self.output_path.open(
            'x', encoding='utf-8', newline='', buffering=1,
        )
        self.writer = csv.DictWriter(
            self.handle, fieldnames=CSV_COLUMNS, extrasaction='raise', restval='',
        )
        self.writer.writeheader()
        self.counts = Counter()

        self.create_subscription(
            Imu, args.raw_imu_topic,
            lambda message, info: self.record_imu(
                'raw_input', args.raw_imu_topic, message, info,
            ),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            OpticalFlow, args.flow_topic,
            lambda message, info: self.record_flow(args.flow_topic, message, info),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Range, args.range_topic,
            lambda message, info: self.record_range(args.range_topic, message, info),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu, args.calculated_imu_topic,
            lambda message, info: self.record_imu(
                'calculated_output', args.calculated_imu_topic, message, info,
            ),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, args.odom_topic,
            lambda message, info: self.record_odom(args.odom_topic, message, info),
            qos_profile_sensor_data,
        )
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool, args.calibrated_topic,
            lambda message, info: self.record_calibrated(
                args.calibrated_topic, message, info,
            ),
            status_qos,
        )
        self.get_logger().info(f'Recording odometry CSV in {self.output_path}')

    def base_row(self, category, topic, message_type, info, message=None):
        row = {
            'category': category,
            'topic': topic,
            'message_type': message_type,
            'callback_time_ns': self.get_clock().now().nanoseconds,
            'source_time_ns': message_info_field(info, 'source_timestamp'),
            'received_time_ns': message_info_field(info, 'received_timestamp'),
            'publication_sequence_number': message_info_field(
                info, 'publication_sequence_number',
            ),
            'reception_sequence_number': message_info_field(
                info, 'reception_sequence_number',
            ),
        }
        if message is not None and hasattr(message, 'header'):
            row.update({
                'stamp_sec': message.header.stamp.sec,
                'stamp_nanosec': message.header.stamp.nanosec,
                'frame_id': message.header.frame_id,
            })
        return row

    def write(self, row):
        self.writer.writerow(row)
        self.counts[row['topic']] += 1

    def record_imu(self, category, topic, message, info):
        row = self.base_row(category, topic, 'sensor_msgs/msg/Imu', info, message)
        row.update({
            'imu_orientation_x': message.orientation.x,
            'imu_orientation_y': message.orientation.y,
            'imu_orientation_z': message.orientation.z,
            'imu_orientation_w': message.orientation.w,
            'imu_angular_velocity_x': message.angular_velocity.x,
            'imu_angular_velocity_y': message.angular_velocity.y,
            'imu_angular_velocity_z': message.angular_velocity.z,
            'imu_linear_acceleration_x': message.linear_acceleration.x,
            'imu_linear_acceleration_y': message.linear_acceleration.y,
            'imu_linear_acceleration_z': message.linear_acceleration.z,
            **indexed_values('imu_orientation_covariance', message.orientation_covariance),
            **indexed_values(
                'imu_angular_velocity_covariance', message.angular_velocity_covariance,
            ),
            **indexed_values(
                'imu_linear_acceleration_covariance', message.linear_acceleration_covariance,
            ),
        })
        self.write(row)

    def record_flow(self, topic, message, info):
        row = self.base_row(
            'raw_input', topic, 'flow_range_sensor_node/msg/OpticalFlow', info, message,
        )
        row.update({
            'flow_motion_detected': message.motion_detected,
            'flow_delta_x': message.delta_x,
            'flow_delta_y': message.delta_y,
            'flow_quality': message.quality,
            'flow_observation': message.observation,
            'flow_raw_data_sum': message.raw_data_sum,
            'flow_raw_data_max': message.raw_data_max,
            'flow_raw_data_min': message.raw_data_min,
            'flow_shutter': message.shutter,
            'flow_integration_time': message.integration_time,
            'flow_range_valid': message.range_valid,
            'flow_ground_distance': message.ground_distance,
        })
        self.write(row)

    def record_range(self, topic, message, info):
        row = self.base_row(
            'raw_input', topic, 'sensor_msgs/msg/Range', info, message,
        )
        row.update({
            'range_radiation_type': message.radiation_type,
            'range_field_of_view': message.field_of_view,
            'range_min': message.min_range,
            'range_max': message.max_range,
            'range_distance': message.range,
        })
        self.write(row)

    def record_odom(self, topic, message, info):
        pose = message.pose.pose
        twist = message.twist.twist
        row = self.base_row(
            'calculated_output', topic, 'nav_msgs/msg/Odometry', info, message,
        )
        row.update({
            'odom_child_frame_id': message.child_frame_id,
            'odom_position_x': pose.position.x,
            'odom_position_y': pose.position.y,
            'odom_position_z': pose.position.z,
            'odom_orientation_x': pose.orientation.x,
            'odom_orientation_y': pose.orientation.y,
            'odom_orientation_z': pose.orientation.z,
            'odom_orientation_w': pose.orientation.w,
            'odom_linear_velocity_x': twist.linear.x,
            'odom_linear_velocity_y': twist.linear.y,
            'odom_linear_velocity_z': twist.linear.z,
            'odom_angular_velocity_x': twist.angular.x,
            'odom_angular_velocity_y': twist.angular.y,
            'odom_angular_velocity_z': twist.angular.z,
            **indexed_values('odom_pose_covariance', message.pose.covariance),
            **indexed_values('odom_twist_covariance', message.twist.covariance),
        })
        self.write(row)

    def record_calibrated(self, topic, message, info):
        row = self.base_row(
            'calculated_output', topic, 'std_msgs/msg/Bool', info,
        )
        row['calibrated'] = message.data
        self.write(row)

    def close(self):
        self.handle.flush()
        self.handle.close()
        counts = ', '.join(f'{topic}={count}' for topic, count in self.counts.items())
        self.get_logger().info(f'Closed odometry CSV: {counts}')


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-directory', required=True)
    parser.add_argument('--raw-imu-topic', required=True)
    parser.add_argument('--flow-topic', required=True)
    parser.add_argument('--range-topic', required=True)
    parser.add_argument('--calculated-imu-topic', required=True)
    parser.add_argument('--odom-topic', required=True)
    parser.add_argument('--calibrated-topic', required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = None
    try:
        node = OdomBlackboxCsvRecorder(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
