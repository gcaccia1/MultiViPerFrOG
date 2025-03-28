

#include "multiviperfrog/interfaces/ros_output_adapter.h"
#include <iostream>

namespace interfaces {

ROSOutputAdapter::ROSOutputAdapter(ros::NodeHandlePtr nh, std::shared_ptr<RosDataProcessor> existingProcessor)
    : nh_(nh), dataProcessor_(existingProcessor){};

//    ROSOutputAdapter::~ROSOutputAdapter() {
//        // Destructor implementation
//        // Clean up resources if necessary
//    }

bool ROSOutputAdapter::initialize() {
  // Initialize ROS broadcasters
  initializeDataProcessor();
  std::cout << "ROSOutputAdapter: RosDataProcessor (unified output) initialized." << std::endl;
  return true;
}

void ROSOutputAdapter::initializeDataProcessor() {
  if (!dataProcessor_) {
    std::cout << "ROSOutputAdapter: Initializing new RosDataProcessor..." << std::endl;
    // Create a new dataProcessor_ if none is passed
    dataProcessor_ = std::make_shared<RosOnlineDataProcessor>(nh_);
  } else {
    std::cout << "ROSOutputAdapter: Using existing RosDataProcessor (online or rosbag)..." << std::endl;
  }
  dataProcessor_->initializeOutput();
  // implemented in the base class RosDataProcessor as a non pure virtual function so that
  // both RosOnlineDataProcessor and RosRosbagDataProcessor can use the same output method
}

bool ROSOutputAdapter::sendData(const config::PostprocessingToRosData& data, const config::ExperimentConfig& exp_config) {
  //        std::cout << "ROSOutputAdapter sending data..." << std::endl;
  // Publish data to ROS topics
  dataProcessor_->publishData(data, exp_config);  // TODO make this dynamic with camera_id, now badly hardcoded
  // Handle any exceptions during publishing
  return true;  // Return false if publishing fails
}

bool ROSOutputAdapter::isDataReady() const {
  // Check if data is ready to be sent
  // Implement logic to determine data readiness
  return true;  // Return true if data is ready, false otherwise
}

}  // namespace interfaces
