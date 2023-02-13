#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class OffsetBroadcaster(Node):
    def __init__(self):
        super().__init__('offset_publisher')
        self.declare_parameter('world', 'world')
        self.declare_parameter('update_frequency', 120.0)
        self.declare_parameter('offset_frame', 'hololens') # what to offset
        self.declare_parameter('relative_tracking_name', 'hololens_with_offset') # what to name the offset frame
        self.declare_parameter('translation_offset', [0.0, 0.0, 0.0])
        self.declare_parameter('rotation_offset', [1.0, 0.0, 0.0, 0.0])


        self.world = self.get_parameter('world').get_parameter_value().string_value
        self.update_frequency = self.get_parameter('update_frequency').get_parameter_value().double_value
        self.offset_frame = self.get_parameter('offset_frame').get_parameter_value().string_value
        self.relative_tracking_name = self.get_parameter('relative_tracking_name').get_parameter_value().string_value
        self.translation_offset = self.get_parameter('translation_offset').get_parameter_value().double_array_value
        self.rotation_offset = self.get_parameter('rotation_offset').get_parameter_value().double_array_value

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer((1.0/self.update_frequency), self.broadcast_offset)

    def broadcast_offset(self):
        try:
            no_offset = self.tf_buffer.lookup_transform(
                self.world,
                self.offset_frame,
                rclpy.time.Time())

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.world} to {self.offset_frame}: {ex} : May occur while VRPN is connecting.')
            return

        offset = TransformStamped()

        offset.header.stamp = self.get_clock().now().to_msg()
        offset.header.frame_id = self.offset_frame
        offset.child_frame_id = self.relative_tracking_name

        offset.transform.translation.x = no_offset.transform.translation.x + self.translation_offset[0]
        offset.transform.translation.y = no_offset.transform.translation.y + self.translation_offset[1]
        offset.transform.translation.z = no_offset.transform.translation.z + self.translation_offset[2]
        offset.transform.rotation.x = no_offset.transform.rotation.x + self.rotation_offset[0]
        offset.transform.rotation.y = no_offset.transform.rotation.y + self.rotation_offset[1]
        offset.transform.rotation.z = no_offset.transform.rotation.z + self.rotation_offset[2]
        offset.transform.rotation.w = no_offset.transform.rotation.w + self.rotation_offset[3]

        self.tf_static_broadcaster.sendTransform(offset)

def main(args=None):
    rclpy.init(args=args)
    ob = OffsetBroadcaster()
    rclpy.spin(ob)
    rclpy.shutdown()

if __name__ == '__main__':
    main()