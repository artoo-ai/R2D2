#!/usr/bin/env python3
# Copyright 2019 Rico.ai
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from r2d2.frame.arms import Arms
from std_msgs.msg import String


class R2D2ArmSubscriber(Node):

    def __init__(self):
        super().__init__('arm_ctrl_sub')                        # Create Node Name
        self.subscription = self.create_subscription(           # Create Subscriber
            String,                                             # Take string messages
            'r2d2_arm_topic',                                   # Create topic name
            self.listener_callback,                             # Set Callback
            10)
        self.subscription                                       # prevent unused variable warning

        # Create the R2D2 Arm
        self.r2d2_arms = Arms()

    def listener_callback(self, msg):
        self.get_logger().info('R2D2 Arm Ctrl Sub: "%s"' % msg.data)

        if msg.data=="arm_open":
            self.r2d2_arms.open_arms()
            self.r2d2_arms.close_arms()
            self.get_logger().info('R2D2 Open arms : "%s"' % msg.data)
        if msg.data=="test_arms":
            self.r2d2_arms.test_arms()
            self.get_logger().info('R2D2 Test Arms : "%s"' % msg.data)
        if msg.data=="open_top_arm"
            self.r2d2_arms.open_top_arm()
            self.get_logger().info('R2D2 Open Top Arms : "%s"' % msg.data)
        if msg.data=="open_bottom_arm"
            self.r2d2_arms.open_bottom_arm
            self.get_logger().info('R2D2 Open Bottom Arms : "%s"' % msg.data)
        if msg.data=="close_top_arm"
            self.r2d2_arms.close_top_arm()
            self.get_logger().info('R2D2 Close Top Arms : "%s"' % msg.data)
        if msg.data=="close_bottom_arm"
            self.r2d2_arms.close_bottom_arm()
            self.get_logger().info('R2D2 Close Bottom Arms : "%s"' % msg.data)

def main(args=None):
    """
    Create ROS2 Node.
    """
    # Create ROS2 Python
    rclpy.init(args=args)

    # Create Node
    r2d2_arm_subscriber = R2D2ArmSubscriber()

    # Keep Node Alive
    rclpy.spin(r2d2_arm_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    r2d2_arm_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
