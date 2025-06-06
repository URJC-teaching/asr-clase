// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("logger_node");


  rclcpp::Time start = node->now();

  rclcpp::Rate loop_rate(10ms);

  rclcpp::Time start_time = node->now();
  int counter = 0;
  while (rclcpp::ok() && node->now() - start < 5s) {
    rclcpp::Time current = node->now();
    rclcpp::Duration diff = current - start;

    RCLCPP_ERROR(node->get_logger(), "Hello %d. %.2lfs", counter++, diff.seconds());

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
