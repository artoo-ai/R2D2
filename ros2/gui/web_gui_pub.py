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

import time
from flask import Flask, render_template, Response, request, redirect, url_for
from threading import Thread
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class R2d2WebGui(Node):

    def __init__(self):
        """
        Initialize the R2D2 Web GUI.
        """
        super().__init__('r2d2_web_gui')                        # Create Node Name

        # Create a Arm publisher to publish arm topics
        self.arm_publisher = self.create_publisher(String, 'r2d2_arm_topic')

        # Init the web server
        self.init_web_server()


    def init_web_server(self):
        template_dir =os.path.dirname(os.path.realpath(__file__))
        template_dir = os.path.join(template_dir, 'web')
        print(template_dir)

        app = Flask(__name__, template_folder=template_dir, static_url_path='/static')

        @app.route("/")
        def index():
            return render_template('index.html')

        @app.route("/test_arms/", methods=['GET', 'POST'])
        def test_arms():
            self.pub_test_arms()
            return render_template('index.html')

        @app.route("/open_top_arm/", methods=['GET', 'POST'])
        def open_top_arm():
            self.pub_open_top_arm()
            return render_template('index.html')

        @app.route("/open_bottom_arm/", methods=['GET', 'POST'])
        def open_bottom_arm():
            self.pub_open_bottom_arm()
            return render_template('index.html')

        app.run(host='0.0.0.0')

    def pub_test_arms(self):
        """
        Test the arms.
        :return:
        """
        msg = String()
        msg.data = "test_arms"
        self.arm_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def pub_open_top_arm(self):
        """
        Open then close the top arm.
        """
        # Open Top Arm
        msg = String()
        msg.data = "open_top_arm"
        self.arm_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

        # Wait a period of time
        time.sleep(0.5)

        # Close Top Arm
        msg.data = "close_top_arm"
        self.arm_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def pub_open_bottom_arm(self):
        """
        Open then close the bottom arm.
        """
        # Open Bottom Arm
        msg = String()
        msg.data = "open_bottom_arm"
        self.arm_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

        # Wait a period of time
        time.sleep(0.5)

        # Close Top Arm
        msg.data = "close_bottom_arm"
        self.arm_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    r2d2_web_gui = R2d2WebGui()

    rclpy.spin(r2d2_web_gui)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    r2d2_web_gui.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
