
#include "multiviperfrog/interfaces/RosOnlineDataProcessor.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "multiviperfrog/core/workflow_manager.h"
#include "multiviperfrog/utils/data_conversions.h"
//#include "open3d_slam/frames.hpp"
//#include "open3d_slam/time.hpp"

namespace interfaces {

RosOnlineDataProcessor::RosOnlineDataProcessor(ros::NodeHandlePtr nh) : BASE(nh) {}

void RosOnlineDataProcessor::initializeInput() {
  std::cout << "RosOnlineDataProcessor: initializing ros stuff..." << std::endl;
  initCommonRosStuff();
  initOnlineRosInputStuff();
  std::cout << "RosOnlineDataProcessor: done!" << std::endl;
}

void RosOnlineDataProcessor::initOnlineRosInputStuff() {
  tfBuffer_.reset(new tf2_ros::Buffer());
  tfListener_.reset(new tf2_ros::TransformListener(*tfBuffer_));
}

void RosOnlineDataProcessor::setupProcessing(double topics_sync_upper_thr) {
  std::cout << "RosOnlineDataProcessor: registering synchronized callback..." << std::endl;
  setupOnlineSyncProcessing(topics_sync_upper_thr);
  std::cout << "RosOnlineDataProcessor: done!" << std::endl;
}

void RosOnlineDataProcessor::startProcessing(core::WorkFlowManager* manager, double topics_sync_upper_thr) {
  throw std::runtime_error("RosOnlineDataProcessor: startProcessing not implemented yet. Exiting... ");
}

// approximate time synchronizer (clouds and poses)
//  extremely complex structure proposed by gpt to overcome some issues with the previous implementation that used boost::bind
// but had some issues with the number of arguments
struct SyncCallbackStruct {
  void operator()(const sensor_msgs::PointCloud2ConstPtr& msg1, const sensor_msgs::PointCloud2ConstPtr& msg2,
                  const sensor_msgs::PointCloud2ConstPtr& msg3, const sensor_msgs::PointCloud2ConstPtr& msg4,
                  const sensor_msgs::PointCloud2ConstPtr& msg5, const sensor_msgs::PointCloud2ConstPtr& msg6,
                  const geometry_msgs::PoseStampedConstPtr& msg7, const geometry_msgs::PoseStampedConstPtr& msg8,
                  const geometry_msgs::PoseStampedConstPtr& msg9) const {
    // Call the actual syncCallback method
    // Assuming you have access to the `this` pointer somehow
    this_ptr->syncCallback(msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8, msg9);
  }

  // Store a pointer to the RosOnlineDataProcessor object
  RosOnlineDataProcessor* this_ptr;
  using result_type = void;  // Define the result_type that Boost requires
};

void RosOnlineDataProcessor::setupOnlineSyncProcessing(const double topics_sync_upper_thr) {
  cloudSubscriber_1_.subscribe(*nh_, topics[0], 1);
  cloudSubscriber_2_.subscribe(*nh_, topics[1], 1);
  cloudSubscriber_3_.subscribe(*nh_, topics[2], 1);
  cloudSubscriber_4_.subscribe(*nh_, topics[3], 1);
  cloudSubscriber_5_.subscribe(*nh_, topics[4], 1);
  cloudSubscriber_6_.subscribe(*nh_, topics[5], 1);
  poseSubscriber_1_.subscribe(*nh_, topics[6], 1);
  poseSubscriber_2_.subscribe(*nh_, topics[7], 1);
  poseSubscriber_3_.subscribe(*nh_, topics[8], 1);

  sync_.reset(new Sync(MySyncPolicy(10), cloudSubscriber_1_, cloudSubscriber_2_, cloudSubscriber_3_, cloudSubscriber_4_, cloudSubscriber_5_,
                       cloudSubscriber_6_, poseSubscriber_1_, poseSubscriber_2_, poseSubscriber_3_));
  //    sync_->setInterMessageLowerBound(0, ros::Duration(0.1)); //minimum time gap that must exist between consecutive messages from the
  //    same topic
  sync_->setMaxIntervalDuration(
      ros::Duration(topics_sync_upper_thr));  // maximum allowable time interval between messages from different topics
  SyncCallbackStruct callback;
  callback.this_ptr = this;  // Assign the current object to the struct
  sync_->registerCallback(callback);
}

void RosOnlineDataProcessor::syncCallback(
    const sensor_msgs::PointCloud2ConstPtr& C1_points_msg_ptr, const sensor_msgs::PointCloud2ConstPtr& C2_points_msg_ptr,
    const sensor_msgs::PointCloud2ConstPtr& C3_points_msg_ptr, const sensor_msgs::PointCloud2ConstPtr& C1_rel_flow_msg_ptr,
    const sensor_msgs::PointCloud2ConstPtr& C2_rel_flow_msg_ptr, const sensor_msgs::PointCloud2ConstPtr& C3_rel_flow_msg_ptr,
    const geometry_msgs::PoseStampedConstPtr& C1_pose_msg_ptr, const geometry_msgs::PoseStampedConstPtr& C2_pose_msg_ptr,
    const geometry_msgs::PoseStampedConstPtr& C3_pose_msg_ptr) {
  std::cout << "RosOnlineDataProcessor: messages arrived. \n";
  // TODO: encapsulate into a function like in RosRosbagDataProcessor::saveRosbagMessages
  config::RosToPreprocessingData output_data;
  output_data.Camera_points_msg_ptr[0] = C1_points_msg_ptr;
  output_data.Camera_points_msg_ptr[1] = C2_points_msg_ptr;
  output_data.Camera_points_msg_ptr[2] = C3_points_msg_ptr;
  output_data.Camera_rel_flow_msg_ptr[0] = C1_rel_flow_msg_ptr;
  output_data.Camera_rel_flow_msg_ptr[1] = C2_rel_flow_msg_ptr;
  output_data.Camera_rel_flow_msg_ptr[2] = C3_rel_flow_msg_ptr;
  output_data.Camera_pose_msg_ptr[0] = C1_pose_msg_ptr;
  output_data.Camera_pose_msg_ptr[1] = C2_pose_msg_ptr;
  output_data.Camera_pose_msg_ptr[2] = C3_pose_msg_ptr;

  core::WorkFlowManager::updateRosInputFields(output_data);
}

}  // namespace interfaces
