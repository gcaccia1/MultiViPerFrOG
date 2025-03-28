
// ROS
#include <ros/package.h>

// Workspace
#include "multiviperfrog/interfaces/RosDataProcessor.h"

namespace interfaces {

RosDataProcessor::RosDataProcessor(ros::NodeHandlePtr nh) : nh_(nh) {
  // Package Path
  packagePath_ = ros::package::getPath("multiviperfrog");
  // Full path to the YAML configuration file
  std::string yamlFilePath = packagePath_ + "/config/config.yaml";
  std::cout << "RosDataProcessor: yaml file path: " << yamlFilePath << std::endl;
  // Load the YAML configuration file
  yamlConfig_ = YAML::LoadFile(yamlFilePath);
}

void RosDataProcessor::initCommonRosStuff() {
  // TODO: remove redundancy on 2 types of pointclous and structure to handle inference data instead (max 2 cam in that case)!
  // define rosbag name
  rosbagFilename_ = packagePath_ + yamlConfig_["rosbag_path"].as<std::string>();
  std::cout << "RosDataProcessor: rosbag path: " << rosbagFilename_ << std::endl;
  std::string data_type = yamlConfig_["input_data_source"].as<std::string>(); // "ground_truth" or "inference"

  // allocate size for topics
  topics.resize(9);
  // Assign topics dynamically
  topics[0] = "/camera1/depth/3Dflow/" + data_type + "/points";
  topics[1] = "/camera2/depth/3Dflow/" + data_type + "/points";
  topics[2] = "/camera2/depth/3Dflow/" + data_type + "/points"; // for now bypassed in absence of cam3
  topics[3] = "/camera1/depth/3Dflow/" + data_type + "/points";
  topics[4] = "/camera2/depth/3Dflow/" + data_type + "/points";
  topics[5] = "/camera2/depth/3Dflow/" + data_type + "/points"; // for now bypassed in absence of cam3
  topics[6] = "/camera1/pose/" + data_type;
  topics[7] = "/camera2/pose/" + data_type;
  topics[8] = "/camera2/pose/" + data_type; // for now bypassed in absence of cam3

  cloudOutputTopic_ = "/camera1/depth/color/points/republished";
  cloudOutputTopicDownsampled_ = "/camera1/depth/color/points/as_optimized";
  cloudOutputTopicOverlap_ = "/camera1/depth/color/points/overlap";
  cloudOutputTopicOverlapDownsampled_ = "/camera1/depth/color/points/overlap/as_optimized";

  cloudOutputTopic2_ = "/camera2/depth/color/points/republished";
  cloudOutputTopicDownsampled2_ = "/camera2/depth/color/points/as_optimized";
  cloudOutputTopicOverlap2_ = "/camera2/depth/color/points/overlap";
  cloudOutputTopicOverlapDownsampled2_ = "/camera2/depth/color/points/overlap/as_optimized";

  // TODO somehow make topic management more flexible and less hardcoded
}

void RosDataProcessor::initializeOutput() {
  tfBroadcaster_.reset(new tf2_ros::TransformBroadcaster());
  pc_pub_ = std::make_shared<ros::Publisher>(nh_->advertise<sensor_msgs::PointCloud2>(cloudOutputTopic_, 1));
  pc_downsampled_pub_ = std::make_shared<ros::Publisher>(nh_->advertise<sensor_msgs::PointCloud2>(cloudOutputTopicDownsampled_, 1));
  pc_overlap_pub_ = std::make_shared<ros::Publisher>(nh_->advertise<sensor_msgs::PointCloud2>(cloudOutputTopicOverlap_, 1));
  pc_overlap_downsampled_pub_ =
      std::make_shared<ros::Publisher>(nh_->advertise<sensor_msgs::PointCloud2>(cloudOutputTopicOverlapDownsampled_, 1));

  pc_pub2_ = std::make_shared<ros::Publisher>(nh_->advertise<sensor_msgs::PointCloud2>(cloudOutputTopic2_, 1));
  pc_downsampled_pub2_ = std::make_shared<ros::Publisher>(nh_->advertise<sensor_msgs::PointCloud2>(cloudOutputTopicDownsampled2_, 1));
  pc_overlap_pub2_ = std::make_shared<ros::Publisher>(nh_->advertise<sensor_msgs::PointCloud2>(cloudOutputTopicOverlap2_, 1));
  pc_overlap_downsampled_pub2_ =
      std::make_shared<ros::Publisher>(nh_->advertise<sensor_msgs::PointCloud2>(cloudOutputTopicOverlapDownsampled2_, 1));

  std::cout << "RosDataProcessor: Output initialized." << std::endl;
}

void RosDataProcessor::publishData(const config::PostprocessingToRosData& data, const config::ExperimentConfig& exp_config) {
  // publish the data
  // TODO make this dynamic with camera_id, now badly hardcoded to case of 2 cameras and no permutations
  // TODO check if there is any effect/delay if the data is published in a different order
  // publish always
  // transforms
  tfBroadcaster_->sendTransform(data.Cpose_start__T__Ccurrent_GT_msg[0]);
  if (exp_config.run_odo_ICP) {
    tfBroadcaster_->sendTransform(data.Cpose_start__T__Ccurrent_icp_msg[0]);
  }
  if (exp_config.run_odo_SVD) {
    tfBroadcaster_->sendTransform(data.Cpose_start__T__Ccurrent_svd_msg[0]);
  }
  if (exp_config.optimize_odometries || exp_config.optimize_multicam) {
    tfBroadcaster_->sendTransform(data.Cpose_start__T__Ccurrent_estim_msg[0]);
  }
  // pointclouds
  pc_pub_->publish(data.Cn0__t__Cn0_P0_o3d_msg[0]);
  //  if (exp_config.optimize_odometries || exp_config.run_odo_ICP || exp_config.run_odo_SVD) {
  pc_downsampled_pub_->publish(data.Cn0__t__Cn0_P0_o3d_downsampled_msg[0]);
  //  }
  // publish only if there are more than 1 camera
  if (exp_config.num_cameras > 1) {
    // transforms
    tfBroadcaster_->sendTransform(data.Cpose_start__T__Ccurrent_GT_msg[1]);
    if (exp_config.run_odo_ICP) {
      tfBroadcaster_->sendTransform(data.Cpose_start__T__Ccurrent_icp_msg[1]);
    }
    if (exp_config.run_odo_SVD) {
      tfBroadcaster_->sendTransform(data.Cpose_start__T__Ccurrent_svd_msg[1]);
    }
    if (exp_config.optimize_odometries || exp_config.optimize_multicam) {
      tfBroadcaster_->sendTransform(data.Cpose_start__T__Ccurrent_estim_msg[1]);
    }
    // pointclouds
    pc_pub2_->publish(data.Cn0__t__Cn0_P0_o3d_msg[1]);
    //    if (exp_config.optimize_odometries || exp_config.run_odo_ICP || exp_config.run_odo_SVD) {
    pc_downsampled_pub2_->publish(data.Cn0__t__Cn0_P0_o3d_downsampled_msg[1]);
    //    }
    if (exp_config.optimize_multicam) {
      //      pc_overlap_pub_->publish(data.Cn0__t__Cn0_P0_o3d_overlap_msg[0]);
      pc_overlap_downsampled_pub_->publish(data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_msg[0]);
      //      pc_overlap_pub2_->publish(data.Cn0__t__Cn0_P0_o3d_overlap_msg[1]);
      pc_overlap_downsampled_pub2_->publish(data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_msg[1]);
    }
  }
}

}  // namespace interfaces
