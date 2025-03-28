

#ifndef INTERFACES_ROSBAGRANGEDATAPROCESSORROS_H
#define INTERFACES_ROSBAGRANGEDATAPROCESSORROS_H

#pragma once
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <memory>
#include "multiviperfrog/config/optimization_data.h"
#include "multiviperfrog/interfaces/RosDataProcessor.h"
//#include "open3d_slam/time.hpp"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace interfaces {

class RosRosbagDataProcessor : public RosDataProcessor {
  using BASE = RosDataProcessor;

 public:
  RosRosbagDataProcessor(ros::NodeHandlePtr nh);
  ~RosRosbagDataProcessor() override = default;

  void initializeInput() override;
  void setupProcessing(double topics_sync_upper_thr) override;
  void startProcessing(core::WorkFlowManager* manager, double topics_sync_upper_thr) override;

 private:
  rosbag::Bag bag_;

  void preprocessRosbagForSynchronization(const rosbag::Bag& bag, double& minimum_delay, const std::vector<std::string>& topics);

  void readRosbagWithSync(core::WorkFlowManager* manager, const rosbag::Bag& bag, double minimum_delay,
                          const std::vector<std::string>& topics);

  void saveRosbagMessages(const std::vector<rosbag::MessageInstance>& synced_messages, config::RosToPreprocessingData& output_data);

  // ROS
  // Visualization Marker Array Publisher
  ros::Publisher cam1_marker_pub_;
  ros::Publisher cam2_marker_pub_;
  // Visualization Marker Topic
  std::string cam1_marker_topic_ = "/cam1_visualization_marker_array";
  std::string cam2_marker_topic_ = "/cam2_visualization_marker_array";
  // Visualization Marker Array Message
  visualization_msgs::MarkerArray cam1_marker_array_msg_;
  visualization_msgs::MarkerArray cam2_marker_array_msg_;
};

}  // namespace interfaces

#endif  // INTERFACES_ROSBAGRANGEDATAPROCESSORROS_H