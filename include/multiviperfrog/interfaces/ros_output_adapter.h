

#ifndef INTERFACES_ROS_OUTPUT_ADAPTER_H
#define INTERFACES_ROS_OUTPUT_ADAPTER_H

#include <ros/ros.h>
#include "multiviperfrog/interfaces/RosDataProcessor.h"
#include "multiviperfrog/interfaces/RosOnlineDataProcessor.h"
#include "multiviperfrog/interfaces/output_interface.h"

namespace interfaces {

class ROSOutputAdapter : public OutputInterface<config::PostprocessingToRosData, config::ExperimentConfig> {
 public:
  ROSOutputAdapter(ros::NodeHandlePtr nh, std::shared_ptr<RosDataProcessor> existingProcessor = nullptr);
  ~ROSOutputAdapter() override = default;

  bool initialize() override;
  bool sendData(const config::PostprocessingToRosData& data, const config::ExperimentConfig& exp_config) override;
  bool isDataReady() const override;

 private:
  ros::NodeHandlePtr nh_;
  std::shared_ptr<RosDataProcessor> dataProcessor_;
  void initializeDataProcessor();
};

}  // namespace interfaces

#endif  // INTERFACES_ROS_OUTPUT_ADAPTER_H
