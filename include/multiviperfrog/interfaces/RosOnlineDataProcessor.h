

#ifndef INTERFACES_ONLINERANGEDATAPROCESSORROS_H
#define INTERFACES_ONLINERANGEDATAPROCESSORROS_H

#pragma once
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include "multiviperfrog/config/optimization_data.h"
#include "multiviperfrog/interfaces/RosDataProcessor.h"

namespace interfaces {

class RosOnlineDataProcessor : public RosDataProcessor {
  using BASE = RosDataProcessor;

 public:
  RosOnlineDataProcessor(ros::NodeHandlePtr nh);
  ~RosOnlineDataProcessor() override = default;
  void initializeInput() override;
  void setupProcessing(double topics_sync_upper_thr) override;
  void startProcessing(core::WorkFlowManager* manager,
                       double topics_sync_upper_thr) override;  // not implemented in .cpp, not needed for now
  void syncCallback(const sensor_msgs::PointCloud2ConstPtr& C1_points_msg, const sensor_msgs::PointCloud2ConstPtr& C2_points_msg,
                    const sensor_msgs::PointCloud2ConstPtr& C3_points_msg, const sensor_msgs::PointCloud2ConstPtr& C1_rel_flow_msg,
                    const sensor_msgs::PointCloud2ConstPtr& C2_rel_flow_msg, const sensor_msgs::PointCloud2ConstPtr& C3_rel_flow_msg,
                    const geometry_msgs::PoseStampedConstPtr& C2_pose_msg, const geometry_msgs::PoseStampedConstPtr& C1_pose_msg,
                    const geometry_msgs::PoseStampedConstPtr& C3_pose_msg);

 private:
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  tf::TransformListener tfl_;

  void initOnlineRosInputStuff();
  void setupOnlineSyncProcessing(const double topics_sync_upper_thr);

  message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSubscriber_1_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSubscriber_2_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSubscriber_3_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSubscriber_4_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSubscriber_5_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSubscriber_6_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> poseSubscriber_1_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> poseSubscriber_2_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> poseSubscriber_3_;

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped>
      MySyncPolicy;

  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
};

}  // namespace interfaces

#endif  // INTERFACES_ONLINERANGEDATAPROCESSORROS_H
