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
from std_msgs.msg import Int32

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('publisher_node')
    publisher = node.create_publisher(Int32, 'int_topic', 10)
    message = Int32()
    message.data = 0

    rate = node.create_rate(2)  # 2 Hz
    
    while rclpy.ok():
        rclpy.spin_once(node)
        publisher.publish(message)
        message.data += 1
        rate.sleep()
    rclpy.shutdown()

if __name__ == '__main__':
    main()