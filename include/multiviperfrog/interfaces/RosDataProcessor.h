
#ifndef INTERFACES_DATAPROCESSORROS_H
#define INTERFACES_DATAPROCESSORROS_H

// System
#include <yaml-cpp/yaml.h>

// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "tf2_ros/buffer.h"

// Workspace
#include <open3d/Open3D.h>
#include "multiviperfrog/config/experiment_config.h"
#include "multiviperfrog/config/optimization_data.h"
//#include "open3d_conversions/open3d_conversions.h"

namespace core {
class WorkFlowManager;
}

namespace interfaces {

class RosDataProcessor {
 public:
  RosDataProcessor(ros::NodeHandlePtr nh);
  virtual ~RosDataProcessor() = default;
  virtual void initializeInput() = 0;
  virtual void setupProcessing(double topics_sync_upper_thr) = 0;
  virtual void startProcessing(core::WorkFlowManager* manager, double topics_sync_upper_thr) = 0;
  virtual void initializeOutput();
  virtual void publishData(const config::PostprocessingToRosData& data, const config::ExperimentConfig& exp_config);
  void initCommonRosStuff();

 protected:
  ros::NodeHandlePtr nh_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  std::shared_ptr<ros::Publisher> pc_pub_;
  std::shared_ptr<ros::Publisher> pc_downsampled_pub_;
  std::shared_ptr<ros::Publisher> pc_overlap_pub_;
  std::shared_ptr<ros::Publisher> pc_overlap_downsampled_pub_;
  std::shared_ptr<ros::Publisher> pc_pub2_;
  std::shared_ptr<ros::Publisher> pc_downsampled_pub2_;
  std::shared_ptr<ros::Publisher> pc_overlap_pub2_;
  std::shared_ptr<ros::Publisher> pc_overlap_downsampled_pub2_;
  std::string rosbagFilename_;
  std::vector<std::string> topics;
  std::string cloudOutputTopic_;
  std::string cloudOutputTopicDownsampled_;
  std::string cloudOutputTopicOverlap_;
  std::string cloudOutputTopicOverlapDownsampled_;
  std::string cloudOutputTopic2_;
  std::string cloudOutputTopicDownsampled2_;
  std::string cloudOutputTopicOverlap2_;
  std::string cloudOutputTopicOverlapDownsampled2_;

  // Yaml config
  YAML::Node yamlConfig_;
  std::string packagePath_;
  //        std::string cloudTopic1_;
  //        std::string cloudTopic2_;
  //        std::string cloudTopic3_;
  //        std::string cloudTopic4_;
  //        std::string cloudTopic5_;
  //        std::string cloudTopic6_;
  //        std::string poseTopic1_;
  //        std::string poseTopic2_;
  //        std::string poseTopic3_;
};

}  // namespace interfaces

#endif  // INTERFACES_DATAPROCESSORROS_H
