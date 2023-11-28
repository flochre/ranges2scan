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

#include "ranges2scan/ranges2scan.hpp"

#include <cmath>  // for M_PI, round

Ranges2Scan::Ranges2Scan() : Node("ranges_to_scan") {
  // Create a TF2 buffer and listener
  tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  // Create a publisher for LaserScan messages
  declare_parameter("scan_topic", std::string("laser_scan_topic"));
  auto laser_scan_topic = get_parameter("scan_topic").as_string();
  laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(laser_scan_topic, 10);
  
  declare_parameter("frame_id", std::string("laser_frame"));
  laser_scan_frame_id_ = get_parameter("frame_id").as_string();

  // Load parameters and subscribe to range sensors based on the parameters
  addRangeSensorSubscriptions();

  // Initialize the ranges vector with 360 values initialized to 0
  ranges_.resize(360, 0.0);
}

void Ranges2Scan::addRangeSensorSubscriptions() {
  // Retrieve a list of range sensor topics from parameters
  declare_parameter("range_sensor_topics", std::vector<std::string>());
  auto range_sensor_topics = get_parameter("range_sensor_topics").as_string_array();
  
  RCLCPP_INFO(
    this->get_logger(), 
    "We will listen to the following %ld topic(s) to generate the scan in the %s frame_id", 
    range_sensor_topics.size(), laser_scan_frame_id_.c_str()
  );
  std::stringstream ss;
  for (auto & value : range_sensor_topics) {
    RCLCPP_INFO(this->get_logger(), "%s", value.c_str());
  }

  for (const std::string& topic_name : range_sensor_topics) {
    // Create a subscription for a range sensor
    auto range_data_sub = this->create_subscription<sensor_msgs::msg::Range>(
        topic_name, 10, std::bind(&Ranges2Scan::rangeDataCallback, this, std::placeholders::_1));

    // Store the subscription in the vector
    range_data_subs_.push_back(range_data_sub);
  }
}

void Ranges2Scan::rangeDataCallback(const sensor_msgs::msg::Range::SharedPtr range_data) {
  // Define a TransformStamped message for the transformation
  geometry_msgs::msg::TransformStamped transformStamped;

  try {
    // Lookup the transformation from sensor frame to laser_scan_frame_id_ frame
    transformStamped = tfBuffer_->lookupTransform(laser_scan_frame_id_, range_data->header.frame_id, this->now());
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
    return;
  }

  // Create a PointStamped for the range sensor data
  geometry_msgs::msg::PointStamped sensor_data_point;
  sensor_data_point.header = range_data->header;
  sensor_data_point.point.x = range_data->range;

  // Use tf2 to transform the sensor data to the laser_scan_frame_id_ frame
  geometry_msgs::msg::PointStamped transformed_data;
  tf2::doTransform(sensor_data_point, transformed_data, transformStamped);

  // Use tf2 to calculate polar coordinates
  tf2::Transform polar_transform;
  tf2::Vector3 polar_coords(transformed_data.point.x, transformed_data.point.y, 0.0);
  polar_transform.setOrigin(polar_coords);

  // Get the angle in radians
  double angle_radians = std::atan2(transformed_data.point.y, transformed_data.point.x);

  // Convert the angle to degrees and round to the nearest integer
  int angle_degrees = static_cast<int>(round(angle_radians * 180.0 / M_PI));

  // the index starts with 0 and ends at 359
  // the angle is between -180 and 179
  int index = (angle_degrees + 180) % 360;

  double distance = polar_transform.getOrigin().length();

  RCLCPP_DEBUG(
    this->get_logger(), 
    "Received range : %s / distance %f / new_frame : %s / new_distance : %f / angle_degree : %d / index : %d", 
    range_data->header.frame_id.c_str(), range_data->range, laser_scan_frame_id_.c_str(), distance, angle_degrees, index
  );

  bool frame_id_found = false;
  for(auto & range_info : ranges_info_){
    // For every iteration only one will be new
    if(!frame_id_found && range_info->get_frame_id() == range_data->header.frame_id){
      range_info->set_distance(distance);
      range_info->set_index(index);
      frame_id_found = true;
    }

    // on every iterations we need to reset all the ranges
    if (range_info->get_index() >= 0 && range_info->get_index() < 360) {
    // if (range_info->get_index() >= -180 && range_info->get_index() < 180) {
      ranges_[range_info->get_index()] = range_info->get_distance();
    } else {
      RCLCPP_WARN(this->get_logger(), 
        "Received index : %d out of the range of atan2 .. Should never happens",
        range_info->get_index()
      );
    }

  }

  // if at the end of the loop no frame_id has been found, then the object does not exist yet
  if(!frame_id_found){
    ranges_info_.push_back(new RangeInfo(range_data->header.frame_id));
  }
  
  auto laser_scan = sensor_msgs::msg::LaserScan();

  // Create a LaserScan message for the transformed data
  // laser_scan.header = range_data->header;
  laser_scan.header.stamp = range_data->header.stamp;
  laser_scan.header.frame_id = laser_scan_frame_id_;
  // laser_scan.angle_min = -std::atan2(0.5 * range_data->field_of_view, range_data->range);
  laser_scan.angle_min = - 179 * M_PI / 180.0;
  laser_scan.angle_max = M_PI;
  // laser_scan.angle_increment = range_data->field_of_view / (range_data->point_step - 1);
  laser_scan.angle_increment = M_PI / 180.0;
  laser_scan.range_min = range_data->min_range;
  laser_scan.range_max = range_data->max_range;

  // Set the ranges in the LaserScan message
  laser_scan.ranges = ranges_;

  // Publish the LaserScan message
  laser_scan_pub_->publish(laser_scan);

  // Reset the ranges vector to 0 for the next iteration
  std::fill(ranges_.begin(), ranges_.end(), 0.0);
}


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ranges2Scan>());
  rclcpp::shutdown();
  return 0;
}
