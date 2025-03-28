

#include "multiviperfrog/interfaces/ros_input_adapter.h"
#include <iostream>
#include <utility>
#include "multiviperfrog/config/optimization_data.h"
#include "multiviperfrog/core/workflow_manager.h"

namespace interfaces {

ROSInputAdapter::ROSInputAdapter(ros::NodeHandlePtr nh, ::core::WorkFlowManager* manager, bool isProcessAsFastAsPossible,
                                 double topics_sync_upper_thr)
    : nh_(nh), manager_(manager), isProcessAsFastAsPossible_(isProcessAsFastAsPossible), topics_sync_upper_thr_(topics_sync_upper_thr) {}

void ROSInputAdapter::initializeDataProcessor(bool isProcessAsFastAsPossible) {
  if (!dataProcessor_) {
    if (isProcessAsFastAsPossible) {
      std::cout << "ROSInputAdapter: initializing RosRosbagDataProcessor..." << std::endl;
      dataProcessor_ = std::make_shared<RosRosbagDataProcessor>(nh_);
    } else {
      std::cout << "ROSInputAdapter: initializing RosOnlineDataProcessor..." << std::endl;
      dataProcessor_ = std::make_shared<RosOnlineDataProcessor>(nh_);
    }
  }
  dataProcessor_->initializeInput();
  std::cout << "ROSInputAdapter: done!" << std::endl;
}

std::shared_ptr<RosDataProcessor> ROSInputAdapter::getDataProcessor() const {
  return dataProcessor_;
}

bool ROSInputAdapter::initialize() {
  initializeDataProcessor(isProcessAsFastAsPossible_);
  // TODO: implement way to check if node is up and running, or some other initialization check
  dataProcessor_->setupProcessing(topics_sync_upper_thr_);
  std::cout << "ROSInputAdapter: ready to process synchronized data with max delay: " << topics_sync_upper_thr_ << " seconds." << std::endl;
  return true;
}

bool ROSInputAdapter::getData(config::RosToPreprocessingData& data) {
  if (isProcessAsFastAsPossible_) {
    if (!core::WorkFlowManager::rosbag_data_available_) {  // this is true only if the WorkFlowManager has not yet called the reading of the
                                                           // rosbag, this flag is also put as false at the endo fo the rosbag, but then
                                                           // nobody will call getData anymore.
      dataProcessor_->startProcessing(manager_,
                                      topics_sync_upper_thr_);  // TODO remove need to specify topics_sync_upper_thr_ here, somehow pass it
                                                                // from setupProcessing in the RosbagRangeDataProcessor
      return false;
    } else {  // this is true only if the WorkFlowManager has already called the reading of the rosbag once
      data = core::WorkFlowManager::Ros_input_global_buffer_;
      //                std::cout << "ROSInputAdapter: forwarded rosbag data." << std::endl;
      return true;
    }
  } else {
    if (core::WorkFlowManager::callback_data_received_) {
      // TODO: enforce some check on emptyness of data in the global buffer
      data = core::WorkFlowManager::Ros_input_global_buffer_;
      std::cout << "ROSInputAdapter: forwarded callback data." << std::endl;
      core::WorkFlowManager::callback_data_received_ = false;
      return true;
    }
  }
  return false;
}

bool ROSInputAdapter::isDataAvailable() const {
  return ros::ok();
}

}  // namespace interfaces
