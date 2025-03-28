

#ifndef INTERFACES_ROS_INPUT_ADAPTER_H
#define INTERFACES_ROS_INPUT_ADAPTER_H

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "multiviperfrog/interfaces/RosOnlineDataProcessor.h"
#include "multiviperfrog/interfaces/RosRosbagDataProcessor.h"
#include "multiviperfrog/interfaces/input_interface.h"

namespace interfaces {

class ROSInputAdapter : public InputInterface<config::RosToPreprocessingData> {
 public:
  ROSInputAdapter(ros::NodeHandlePtr nh, ::core::WorkFlowManager* manager, bool isProcessAsFastAsPossible, double topics_sync_upper_thr);

  ~ROSInputAdapter() override = default;

  bool initialize() override;
  bool getData(config::RosToPreprocessingData& data) override;
  bool isDataAvailable() const override;
  std::shared_ptr<RosDataProcessor> getDataProcessor() const;

 private:
  ros::NodeHandlePtr nh_;
  core::WorkFlowManager* manager_;
  std::shared_ptr<RosDataProcessor> dataProcessor_;
  std::shared_ptr<open3d::geometry::PointCloud> m4_pcd_ptr;
  std::shared_ptr<open3d::geometry::PointCloud> m5_pcd_ptr;

  bool isProcessAsFastAsPossible_;
  double topics_sync_upper_thr_;

  void initializeDataProcessor(bool isProcessAsFastAsPossible);
};

}  // namespace interfaces

#endif  // INTERFACES_ROS_INPUT_ADAPTER_H
