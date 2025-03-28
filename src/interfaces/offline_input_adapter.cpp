

#include "multiviperfrog/interfaces/offline_input_adapter.h"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace interfaces {

OfflineInputAdapter::OfflineInputAdapter() {
  // Constructor implementation
}

OfflineInputAdapter::~OfflineInputAdapter() {
  // Destructor implementation
  // Clean up loading resources if necessary
}

bool OfflineInputAdapter::initialize() {
  // Initialize loading
  // Handle any errors during initialization
  std::cout << "OfflineInputAdapter initialized (not needed here!)." << std::endl;
  return true;
}

bool OfflineInputAdapter::getData(config::PreprocessingToOptimizationData& data) {
  // Retrieve data from files
  std::cout << "OfflineInputAdapter loading data." << std::endl;
  // snippet to load files exported by python (the ROS interface is still not ready)

  std::string filename_Ca0TCb0 = "/home/user/Documents/rosbag_python_evaluation_output/GT_transform_Ca0TCb0.json";
  std::string filename_Ca1TCb1 = "/home/user/Documents/rosbag_python_evaluation_output/GT_transform_Ca1TCb1.json";
  std::string filename_Ca0TCa1 = "/home/user/Documents/rosbag_python_evaluation_output/GT_transform_Ca0TCa1.json";
  std::string filename_Cb0TCb1 = "/home/user/Documents/rosbag_python_evaluation_output/GT_transform_Cb0TCb1.json";

  data.Cself0__T__Cother0[0] = loadTransformFromJson(filename_Ca0TCb0);
  data.Cself1__T__Cother1[0] = loadTransformFromJson(filename_Ca1TCb1);
  data.Cn0__T__Cn1[0] = loadTransformFromJson(filename_Ca0TCa1);
  data.Cn0__T__Cn1[1] = loadTransformFromJson(filename_Cb0TCb1);

  // load pcd file with open3d
  // TODO:NOTE removed the support "estimated" (e.g. RAFT3D) point clouds, only GT for consistency with ROS callbacks which are not readu
  // for it
  data.Cn1__t__Cn1_P1_o3d_ptr[0] = open3d::io::CreatePointCloudFromFile(
      "/home/user/Documents/rosbag_python_evaluation_output/"
      "biased_flowed_points_cam1_full_in_cam1_next_reference_frame.pcd");
  //        data.m2_estimated_pcd_ptr = open3d::io::CreatePointCloudFromFile("/home/user/Documents/rosbag_python_evaluation_output/"
  //                                                             "biased_flowed_points_cam1_full_in_cam1_next_reference_frame.pcd"); // TODO
  //                                                             comment, here temporarely as GT
  ////                        "biased_estimated_flowed_points_cam1_full_in_cam1_next_reference_frame.pcd"); // TODO uncomment
  data.Cn1__t__Cn1_P1_o3d_ptr[1] = open3d::io::CreatePointCloudFromFile(
      "/home/user/Documents/rosbag_python_evaluation_output/"
      "biased_flowed_points_cam2_full_in_cam2_next_reference_frame.pcd");
  //        data.m3_estimated_pcd_ptr = open3d::io::CreatePointCloudFromFile("/home/user/Documents/rosbag_python_evaluation_output/"
  //                                                                     "biased_flowed_points_cam2_full_in_cam2_next_reference_frame.pcd");
  //                                                                     // TODO comment, here temporarely as GT
  ////                        "biased_estimated_flowed_points_cam2_full_in_cam2_next_reference_frame.pcd"); // TODO uncomment
  data.Cn0__t__Cn0_P0_o3d_ptr[0] = open3d::io::CreatePointCloudFromFile(
      "/home/user/Documents/rosbag_python_evaluation_output/"
      "current_points_and_true_flow_cam1_full_in_cam1_current_reference_frame.pcd");
  data.Cn0__t__Cn0_P0_o3d_ptr[1] = open3d::io::CreatePointCloudFromFile(
      "/home/user/Documents/rosbag_python_evaluation_output/"
      "current_points_and_true_flow_cam2_full_in_cam2_current_reference_frame.pcd");
  // Check for data validity and availability
  // TODO: implement throwing an error if the number of points is different or no files are loaded

  std::cout << "OfflineInputAdapter finished loading data." << std::endl;
  return true;  // Return false if no more data is available
}

bool OfflineInputAdapter::isDataAvailable() const {
  // Implement logic to check if more data is available
  // state that is not implemented yet
  std::cout << "OfflineInputAdapter has no data control implemented yet." << std::endl;
  return true;
}

// load 4x4 matrices from serialized arrays in .json files (exported from python) //TODO: remove return, that's python style
Eigen::Transform<double, 3, Eigen::Isometry> OfflineInputAdapter::loadTransformFromJson(const std::string& filename) {
  std::ifstream ifs(filename);
  json j = json::parse(ifs);
  Eigen::Transform<double, 3, Eigen::Isometry> Tf;
  auto vec0 = j.at("zero").get<std::vector<double>>();
  auto vec1 = j.at("one").get<std::vector<double>>();
  auto vec2 = j.at("two").get<std::vector<double>>();
  Tf.matrix() << vec0[0], vec0[1], vec0[2], vec0[3], vec1[0], vec1[1], vec1[2], vec1[3], vec2[0], vec2[1], vec2[2], vec2[3], 0, 0, 0, 1;
  //    std::cout << "Loaded Matrix from: " << filename + "\n" << Tf.matrix() << std::endl;
  ifs.close();
  return Tf;
}

}  // namespace interfaces
