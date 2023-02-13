#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
import pandas as pd
import time


class RelativeTracking(Node):
    def __init__(self):
        super().__init__('relative_tracking')
        self.declare_parameter('world', 'world')
        self.declare_parameter('relative_tracking_name', 'hololens')
        self.declare_parameter('update_frequency', 120.0)
        self.declare_parameter('to_track', ['hololens'])

        self.world = self.get_parameter(
            'world').get_parameter_value().double_value
        self.update_frequency = self.get_parameter(
            'update_frequency').get_parameter_value().double_value
        self.relative_tracking_name = self.get_parameter(
            'relative_tracking_name').get_parameter_value().string_value
        self.children = self.get_parameter(
            'to_track').get_parameter_value().string_array_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.transform_df = pd.DataFrame(columns=[
                                         'timestamp', 'frame', 'tvec_x', 'tvec_y', 'tvec_y', 'quat_x', 'quat_y', 'quat_z', 'quat_w'])

        self.timer = self.create_timer(
            (1.0/self.update_frequency), self.record_relative_frames)

    def record_relative_frames(self):
        for child in self.children:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.relative_tracking_name,
                    child,
                    rclpy.time.Time())

                translation = [t.transform.translation.x,
                    t.transform.translation.y, t.transform.translation.z]
                rotation = [t.transform.rotation.x, t.transform.rotation.y,
                    t.transform.rotation.z, t.transform.rotation.w]

                self.transform_df.loc[len(self.transform_df.index)] = [str(rclpy.time.Time().nanoseconds) + f", {child}", 
                child, translation[0], translation[1], translation[2], rotation[0], rotation[1], rotation[2], rotation[3]]

            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {self.world} to {child}: {ex} : May occur when system is spinng up')
        
    def shutdown(self):
        current_time = time.strftime("%Y%m%d_%H%M%S")
        file_name = 'relative_tracking_' + current_time + '.csv'
        self.transform_df.to_csv(file_name, sep=',', header=True, index=False)

def main(args=None):
    rclpy.init(args=args)
    rt = RelativeTracking()
    rclpy.spin(rt)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
