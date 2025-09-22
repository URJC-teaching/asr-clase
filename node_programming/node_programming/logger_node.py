# Copyright 2025 Intelligent Robotics Lab
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

class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')
        self.counter = 0
        self.timer_ = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(f'Counter: {self.counter}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    logger_node = LoggerNode()
    rclpy.spin(logger_node)
    logger_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
