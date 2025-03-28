#include "multiviperfrog/core/workflow_manager.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "multiviperfrog");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
  // set sleep rate
  //    ros::Rate rate(1);
  try {
    auto manager = new core::WorkFlowManager(nh);
    manager->runMultiViPerFrOG();
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return -1;
  }
}
