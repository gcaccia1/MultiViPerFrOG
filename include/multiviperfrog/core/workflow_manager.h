

#ifndef CORE_WORKFLOW_MANAGER_H
#define CORE_WORKFLOW_MANAGER_H
#include <ros/ros.h>
#include <iostream>
#include <memory>
#include <vector>
#include "multiviperfrog/config/ceres_config.h"
#include "multiviperfrog/config/experiment_config.h"
#include "multiviperfrog/config/optimization_data.h"
#include "multiviperfrog/core/experiment_module.h"
#include "multiviperfrog/core/optimization/optimization.h"
#include "multiviperfrog/core/postprocessing/postprocessing.h"
#include "multiviperfrog/core/preprocessing/preprocessing.h"
#include "multiviperfrog/interfaces/RosDataProcessor.h"
#include "multiviperfrog/interfaces/holoscan_input_adapter.h"
#include "multiviperfrog/interfaces/holoscan_output_adapter.h"
#include "multiviperfrog/interfaces/input_interface.h"
#include "multiviperfrog/interfaces/offline_input_adapter.h"
#include "multiviperfrog/interfaces/output_interface.h"
#include "multiviperfrog/interfaces/ros_input_adapter.h"
#include "multiviperfrog/interfaces/ros_output_adapter.h"

namespace core {

// initialize class WorkFlowManager

class WorkFlowManager {
 public:
  WorkFlowManager(ros::NodeHandlePtr nh);
  ~WorkFlowManager();
  void runMultiViPerFrOG();
  static void updateRosInputFields(config::RosToPreprocessingData& data);
  void runProcessingCore();
  // KDTree stuff
  open3d::geometry::KDTreeFlann& GetKDTree() { return kdtree; }
  std::shared_ptr<open3d::geometry::PointCloud> GetPersistentCloudKNN() { return persistent_cloud_knn; }
  // Function to set both the KDTree and the associated point cloud
  void SetKDTreeAndCloud(const std::shared_ptr<open3d::geometry::PointCloud>& cloud) {
    persistent_cloud_knn = cloud;
    kdtree.SetGeometry(*persistent_cloud_knn);
  }
  // Accessors to references of static variables
  static std::vector<int>& getSnippetStartIndices() { return snippet_start_indices_; }
  static std::vector<int>& getSnippetEndIndices() { return snippet_end_indices_; }
  // Estimates
  static std::vector<Eigen::Isometry3d>& getCam1EstSnippetStartPoses() { return cam1_snippet_start_poses_est_; }
  static std::vector<Eigen::Isometry3d>& getCam1EstSnippetEndPoses() { return cam1_snippet_end_poses_est_; }
  static std::vector<Eigen::Isometry3d>& getCam2EstSnippetStartPoses() { return cam2_snippet_start_poses_est_; }
  static std::vector<Eigen::Isometry3d>& getCam2EstSnippetEndPoses() { return cam2_snippet_end_poses_est_; }
  // GT
  static std::vector<Eigen::Isometry3d>& getCam1GtSnippetStartPoses() { return cam1_snippet_start_poses_gt_; }
  static std::vector<Eigen::Isometry3d>& getCam1GtSnippetEndPoses() { return cam1_snippet_end_poses_gt_; }
  static std::vector<Eigen::Isometry3d>& getCam2GtSnippetStartPoses() { return cam2_snippet_start_poses_gt_; }
  static std::vector<Eigen::Isometry3d>& getCam2GtSnippetEndPoses() { return cam2_snippet_end_poses_gt_; }
  // SVD
  static std::vector<Eigen::Isometry3d>& getCam1SVDSnippetStartPoses() { return cam1_snippet_start_poses_svd_; }
  static std::vector<Eigen::Isometry3d>& getCam1SVDSnippetEndPoses() { return cam1_snippet_end_poses_svd_; }
  static std::vector<Eigen::Isometry3d>& getCam2SVDSnippetStartPoses() { return cam2_snippet_start_poses_svd_; }
  static std::vector<Eigen::Isometry3d>& getCam2SVDSnippetEndPoses() { return cam2_snippet_end_poses_svd_; }

  // Member Variables
  static config::RosToPreprocessingData Ros_input_global_buffer_;
  static bool callback_data_received_;
  static bool rosbag_data_available_;
  static bool isFirstTimeRunning_;
  static int camera_id_;
  // moved to public to allow reading exp_monfig_ from Readrosbag
  static config::CeresConfig base_config_;
  config::ExperimentConfig exp_config_;

 private:
  ros::NodeHandlePtr nh_;
  static int num_ros_data_received_;
  static std::chrono::time_point<std::chrono::system_clock> start_again_;
  // single frame and deque data structures
  static config::RosToPreprocessingData ros_to_preprocessing_data_;
  static std::deque<config::RosToPreprocessingData> ros_to_preprocessing_data_buffer_;
  static config::PreprocessingToOptimizationData preprocessing_to_optimization_data_;
  static config::OptimizationToPostprocessingData optimization_to_postprocessing_data_;
  static config::PostprocessingOdometryBuffer postprocessing_odometry_buffer_;
  static config::PostprocessingData postprocessing_data_;
  static config::PostprocessingToRosData postprocessing_to_ros_data_;
  static int ros_processing_window_size;  // how many messages are accumulated before sending to preprocessing
  static int ros_processing_window_step;  // how many message are deleted from the buffer after sending to preprocessing
  std::shared_ptr<interfaces::RosDataProcessor> sharedDataProcessor = nullptr;
  open3d::geometry::KDTreeFlann kdtree;
  std::shared_ptr<open3d::geometry::PointCloud> persistent_cloud_knn;

  // Variable for specific evaluation (non-static)
  // Index Counter
  static int frame_index_counter_;
  static int pair_index_counter_;
  static int snippet_start_index_;
  static int snippet_end_index_;

  // Vector of indices
  static std::vector<int> snippet_start_indices_;
  static std::vector<int> snippet_end_indices_;
  // Vectors of estimated poses (both cameras, both start and end)
  static std::vector<Eigen::Isometry3d> cam1_snippet_start_poses_est_;
  static std::vector<Eigen::Isometry3d> cam1_snippet_end_poses_est_;
  static std::vector<Eigen::Isometry3d> cam2_snippet_start_poses_est_;
  static std::vector<Eigen::Isometry3d> cam2_snippet_end_poses_est_;
  // Vectors of GT poses (both cameras, both start and end)
  static std::vector<Eigen::Isometry3d> cam1_snippet_start_poses_gt_;
  static std::vector<Eigen::Isometry3d> cam1_snippet_end_poses_gt_;
  static std::vector<Eigen::Isometry3d> cam2_snippet_start_poses_gt_;
  static std::vector<Eigen::Isometry3d> cam2_snippet_end_poses_gt_;
  // Vectors of SVD poses (both cameras, both start and end)
  static std::vector<Eigen::Isometry3d> cam1_snippet_start_poses_svd_;
  static std::vector<Eigen::Isometry3d> cam1_snippet_end_poses_svd_;
  static std::vector<Eigen::Isometry3d> cam2_snippet_start_poses_svd_;
  static std::vector<Eigen::Isometry3d> cam2_snippet_end_poses_svd_;
};
}  // namespace core
#endif  // CORE_WORKFLOW_MANAGER_H
