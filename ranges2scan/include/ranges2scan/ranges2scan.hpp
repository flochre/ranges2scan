// Copyright 2023 flochre
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

#ifndef RANGES2SCAN_HPP_
#define RANGES2SCAN_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <vector>

#include "ranges2scan/range_info.hpp"

class Ranges2Scan : public rclcpp::Node {
public:
  Ranges2Scan();

private:
  void rangeDataCallback(const sensor_msgs::msg::Range::SharedPtr range_data);

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
  std::string laser_scan_frame_id_;

  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> range_data_subs_;
  std::vector<float> ranges_;
  std::vector<RangeInfo*> ranges_info_;

  // Function to add range sensor subscriptions based on parameters
  void addRangeSensorSubscriptions();
};

#endif  // RANGES2SCAN_HPP_
