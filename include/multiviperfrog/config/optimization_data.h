

#ifndef CONFIG_OPTIMIZATION_DATA_H
#define CONFIG_OPTIMIZATION_DATA_H

#include <geometry_msgs/PoseStamped.h>
#include <open3d/Open3D.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <optional>
#include <tuple>
// #include "core/workflow_manager.h"

namespace config {

static const int max_num_cameras = 3;  // TODO: make dynamic with a config

// INTERNAL OPTIMIZATION
struct OptimizationData {
  // MEASURES
  // measured c2c transformations
  Eigen::Vector3d Ca0__t__Ca0_Cb0_measured;
  Eigen::Quaternion<double> Ca0__q__Cb0_measured;
  Eigen::Vector3d Ca1__t__Ca1_Cb1_measured;
  Eigen::Quaternion<double> Ca1__q__Cb1_measured;
  // measured odometries
  Eigen::Vector3d Ca0__t__Ca0_Ca1_measured;
  Eigen::Quaternion<double> Ca0__q__Ca1_measured;
  Eigen::Vector3d Cb0__t__Cb0_Cb1_measured;
  Eigen::Quaternion<double> Cb0__q__Cb1_measured;
  // measured point clouds
  Eigen::Matrix4Xd Ca0__t__Ca0_P0_measured;
  Eigen::Matrix4Xd Cb0__t__Cb0_P0_measured;
  // measured flowed points
  Eigen::Matrix4Xd Ca1__t__Ca1_P1_measured;
  Eigen::Matrix4Xd Cb1__t__Cb1_P1_measured;
  // measured scene flow
  Eigen::Matrix4Xd Cb0__t__P0_P1_measured;
  // measured Cb0_SF knowledge matrix, used to show constrain-ability of the problem either via known Sflow or static Sflow
  Eigen::Matrix4Xd Cb0__t__P0_P1_knowledge;
  // Cb0_SF boolean mask to select points of known true scene flow
  std::vector<std::vector<bool>> Cb0__t__P0_P1_mask{max_num_cameras};

  // ESTIMATES
  // estimated c2c transformations
  Eigen::Vector3d Ca0__t__Ca0_Cb0_estim;
  Eigen::Quaternion<double> Ca0__q__Cb0_estim;
  Eigen::Vector3d Ca1__t__Ca1_Cb1_estim;
  Eigen::Quaternion<double> Ca1__q__Cb1_estim;
  // estimated odometries
  Eigen::Vector3d Ca0__t__Ca0_Ca1_estim;
  Eigen::Quaternion<double> Ca0__q__Ca1_estim;
  Eigen::Vector3d Cb0__t__Cb0_Cb1_estim;
  Eigen::Quaternion<double> Cb0__q__Cb1_estim;
  // estimated scene flow
  Eigen::Matrix4Xd Cb0__t__P0_P1_estim;
  // estimated rigidity labels
  Eigen::VectorXd rigidity_labels_a_estim;
  Eigen::VectorXd rigidity_labels_b_estim;
  // estimated labels_to_SF_Scaling
  double labels_to_SF_scaling_estim;
};

// INPUTS OF EXPERIMENT // TODO change names of structs also below
struct FileToOptimizationData {  // now actually same as PreprocessingToOptimizationData, TODO remove this struct
  std::shared_ptr<open3d::geometry::PointCloud> Ca1__t__Ca1_P1_o3d_ptr;  // previously m2 (not on paper)
  std::shared_ptr<open3d::geometry::PointCloud> Cb1__t__Cb1_P1_o3d_ptr;  // previously m3 (not on paper)
  std::shared_ptr<open3d::geometry::PointCloud> Cc1__t__Cc1_P1_o3d_ptr;  // not previously defined
  std::shared_ptr<open3d::geometry::PointCloud> Ca0__t__Ca0_P0_o3d_ptr;  // previously m4 (not on paper)
  std::shared_ptr<open3d::geometry::PointCloud> Cb0__t__Cb0_P0_o3d_ptr;  // previously m5 (not on paper)
  std::shared_ptr<open3d::geometry::PointCloud> Cc0__t__Cc0_P0_o3d_ptr;  // not previously defined
  Eigen::Isometry3d world__T__Ca;
  Eigen::Isometry3d world__T__Cb;
  Eigen::Isometry3d world__T__Cc;
  Eigen::Transform<double, 3, Eigen::Isometry> Ca0__T__Cb0;
  Eigen::Transform<double, 3, Eigen::Isometry> Ca1__T__Cb1;
  Eigen::Transform<double, 3, Eigen::Isometry> Ca0__T__Cc0;
  Eigen::Transform<double, 3, Eigen::Isometry> Ca1__T__Cc1;
  Eigen::Transform<double, 3, Eigen::Isometry> Cb0__T__Cc0;
  Eigen::Transform<double, 3, Eigen::Isometry> Cb1__T__Cc1;
  Eigen::Transform<double, 3, Eigen::Isometry> Ca0__T__Ca1;
  Eigen::Transform<double, 3, Eigen::Isometry> Cb0__T__Cb1;
  Eigen::Transform<double, 3, Eigen::Isometry> Cc0__T__Cc1;
};

struct RosToPreprocessingData {
  // initialize vectors of shared pointers with the size of the # of cameras
  std::vector<sensor_msgs::PointCloud2ConstPtr> Camera_points_msg_ptr{max_num_cameras};    // Cx0__t__Cx0_P0_msg_ptr;
  std::vector<sensor_msgs::PointCloud2ConstPtr> Camera_rel_flow_msg_ptr{max_num_cameras};  // Cx1__t__Cx1_P1_msg_ptr; // TODO 02-2025 REMOVE, no more used by preprocessRosData
  std::vector<geometry_msgs::PoseStampedConstPtr> Camera_pose_msg_ptr{max_num_cameras};    // world__T__Cx0_msg_ptr;
};

struct HoloscanToPreprocessingData {};

struct PreprocessingToOptimizationData {  // as for output of vision blender! NOTE: in real life you would also gt the tf at t1!
  // full pointclouds
  std::vector<std::shared_ptr<open3d::geometry::PointCloud>> Cn0__t__Cn0_P0_o3d_ptr{max_num_cameras};   // points at t0
  std::vector<std::shared_ptr<open3d::geometry::PointCloud>> Cn1__t__Cn1_P1_o3d_ptr{max_num_cameras};   // flowed points
  std::vector<std::shared_ptr<open3d::geometry::PointCloud>> Cn1__t__Cn1_P1o_o3d_ptr{max_num_cameras};  // points at t1

  // Optional tool mask and confidence values (one optional vector per camera)
  std::vector<std::optional<std::vector<uint8_t>>> tool_mask{max_num_cameras};       // Tool mask (optional per camera)
  std::vector<std::optional<std::vector<float>>> confidence_depth{max_num_cameras};  // Depth confidence (optional per camera)
  std::vector<std::optional<std::vector<float>>> confidence_flow{max_num_cameras};   // Flow confidence (optional per camera)

  // downsampled pointclouds
  std::vector<std::shared_ptr<open3d::geometry::PointCloud>> Cn0__t__Cn0_P0_o3d_downsampled_ptr{max_num_cameras};
  std::vector<std::shared_ptr<open3d::geometry::PointCloud>> Cn1__t__Cn1_P1_o3d_downsampled_ptr{max_num_cameras};
  std::vector<std::shared_ptr<open3d::geometry::PointCloud>> Cn1__t__Cn1_P1o_o3d_downsampled_ptr{max_num_cameras};
  // vectors to store the indexes of the points selected for the uniform downsampling from each original pcd
  std::vector<std::vector<size_t>> downsample_indices{max_num_cameras};

  // pointclouds just for the overlap
  std::vector<std::shared_ptr<open3d::geometry::PointCloud>> Cn0__t__Cn0_P0_o3d_overlap_ptr{max_num_cameras};
  std::vector<std::shared_ptr<open3d::geometry::PointCloud>> Cn1__t__Cn1_P1_o3d_overlap_ptr{max_num_cameras};
  // vectors to store the indexes pairs of the points in the overlap (defined in each original pcd, respectively)
  // mapping the indexes of the overlap (position on vector) to the original cloud (index at that position)
  // NOTE, for other, the cloud is already downsampled and the KDTree is build on the downsampled cloud, for self it is full resolution
  std::vector<std::vector<size_t>> overlap_indices_self{max_num_cameras};
  std::vector<std::vector<size_t>> overlap_indices_other{max_num_cameras};
  // pointclouds just for the overlap downsampled
  //  TODO make a smarter logic that overwrites the overlap with the downsampled and "indexes the indexes"
  std::vector<std::shared_ptr<open3d::geometry::PointCloud>> Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr{max_num_cameras};
  std::vector<std::shared_ptr<open3d::geometry::PointCloud>> Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr{max_num_cameras};
  // vectors to store the indexes of the points selected for the uniform downsampling from each overlap pcd
  std::vector<std::vector<size_t>> downsample_indices_overlap{
      max_num_cameras};  // TODO, uniform, this was used by downsampleOverlapRegion, new ones below
  // mapping the indexes of the overlap downsampled (position on vector) to the original downsampled cloud (index at that position)
  std::vector<std::vector<size_t>> downsample_indices_overlap_self{max_num_cameras};
  std::vector<std::vector<size_t>> downsample_indices_overlap_other{max_num_cameras};
  // Saving indexes in a tuple (triplet) where .first is the index in the downsampled cloud_a, .second is the index in the overlap clouds
  // (no matter if a or b they share same indexes), and .third is the index in the downsampled cloud_b TODO: does not work for >2 cameras
  std::vector<std::tuple<std::optional<int>, int, std::optional<int>>> triplets_indexes_downsampled_in_overlap;

  // GT values of TFs and poses
  std::vector<tf2::Stamped<Eigen::Isometry3d>> world__T__Cn0{max_num_cameras};
  std::vector<tf2::Stamped<Eigen::Isometry3d>> world__T__Cn1{max_num_cameras};
  std::vector<Eigen::Transform<double, 3, Eigen::Isometry>> Cn0__T__Cn1{max_num_cameras};
  // TODO: find a way to redefine the following variables in a more general way
  std::vector<Eigen::Transform<double, 3, Eigen::Isometry>> Cself0__T__Cother0{max_num_cameras};
  std::vector<Eigen::Transform<double, 3, Eigen::Isometry>> Cself1__T__Cother1{max_num_cameras};

  // this above can be accessed before global optimization and filled with the odometry/ICP/other outputs
  // initial estimates of the parameters (otherwise these values are set to identity by initializeOptimizationParameters)
  std::vector<Eigen::Transform<double, 3, Eigen::Isometry>> Cn0__T__Cn1_init{max_num_cameras};
  std::vector<Eigen::Transform<double, 3, Eigen::Isometry>> Cself0__T__Cother0_init{max_num_cameras};
  std::vector<Eigen::Transform<double, 3, Eigen::Isometry>> Cself1__T__Cother1_init{max_num_cameras};
  std::vector<Eigen::Matrix4Xd> Cn0__t__P0_P1_init{max_num_cameras};
  std::vector<Eigen::VectorXd> rigidity_labels_n_init{max_num_cameras};
  // measures
  std::vector<std::vector<bool>> Cn0__t__P0_P1_mask{max_num_cameras};
  // this is a measure, the previous same "init" variable is just to initialize the optimization
  std::vector<Eigen::Matrix4Xd> Cn0__t__P0_P1_knowledge{max_num_cameras};
};

// OUTPUTS OF OPTIMIZATION //TODO: implement postprocessing
struct OptimizationToPostprocessingData {
  // optimization output
  std::vector<tf2::Stamped<Eigen::Isometry3d>> Ccurrent__T__Cnext_estim{max_num_cameras};  // TODO rename everywhere with _estim
  std::vector<tf2::Stamped<Eigen::Isometry3d>> Cself0__T__Cother0_estim{max_num_cameras};
  std::vector<tf2::Stamped<Eigen::Isometry3d>> Cself1__T__Cother1_estim{max_num_cameras};
  // only one of the values of Cn0__t__P0_P1_estim is needed, as for convention SF is in Cb frame
  std::vector<Eigen::Matrix4Xd> Cn0__t__P0_P1_estim{max_num_cameras};
  // rigidity_labels_n_estim not needed, unless one recomputes the labels in the global optimization step.
  std::vector<Eigen::VectorXd> rigidity_labels_n_estim{max_num_cameras};

  // output of other preprocessing steps, hosted in this struct for uniformity of the output to postprocessing
  // currently these measures are computed in postprocessing, but they could also be computed in preprocessing
  std::vector<tf2::Stamped<Eigen::Isometry3d>> Ccurrent__T__Cnext_GT{max_num_cameras};
  std::vector<tf2::Stamped<Eigen::Isometry3d>> Ccurrent__T__Cnext_svd{max_num_cameras};
  std::vector<tf2::Stamped<Eigen::Isometry3d>> Ccurrent__T__Cnext_icp{max_num_cameras};
  // add the point clouds as downsampled for first step (not just the overlap, for usage in second step to compute abs SF from odometry)
  std::vector<Eigen::Matrix4Xd> Ca0__t__Ca0_P0_measured{max_num_cameras};
  std::vector<Eigen::Matrix4Xd> Ca1__t__Ca1_P1_measured {max_num_cameras};
};

struct PostprocessingOdometryBuffer {
  std::vector<std::string> parent_frame_id{max_num_cameras};
  std::vector<tf2::Stamped<Eigen::Isometry3d>> Cpose_start__T__Ccurrent_estim{max_num_cameras};
  std::vector<tf2::Stamped<Eigen::Isometry3d>> Cpose_start__T__Ccurrent_GT{max_num_cameras};
  std::vector<tf2::Stamped<Eigen::Isometry3d>> Cpose_start__T__Ccurrent_svd{max_num_cameras};
  std::vector<tf2::Stamped<Eigen::Isometry3d>> Cpose_start__T__Ccurrent_icp{max_num_cameras};
};

struct PostprocessingData {
  std::vector<std::string> parent_frame_id{max_num_cameras};
  std::vector<tf2::Stamped<Eigen::Isometry3d>> Cpose_start__T__Ccurrent_estim{max_num_cameras};
  std::vector<tf2::Stamped<Eigen::Isometry3d>> Cpose_start__T__Ccurrent_GT{max_num_cameras};
  std::vector<tf2::Stamped<Eigen::Isometry3d>> Cpose_start__T__Ccurrent_svd{max_num_cameras};
  std::vector<tf2::Stamped<Eigen::Isometry3d>> Cpose_start__T__Ccurrent_icp{max_num_cameras};
  std::vector<std::shared_ptr<open3d::geometry::PointCloud>> Cn0__t__Cn0_P0_o3d_ptr{max_num_cameras};
  std::vector<std::shared_ptr<open3d::geometry::PointCloud>> Cn0__t__Cn0_P0_o3d_downsampled_ptr{max_num_cameras};
  std::vector<std::shared_ptr<open3d::geometry::PointCloud>> Cn0__t__Cn0_P0_o3d_overlap_ptr{max_num_cameras};
  std::vector<std::shared_ptr<open3d::geometry::PointCloud>> Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr{max_num_cameras};
};

struct PostprocessingToRosData {
  std::vector<geometry_msgs::TransformStamped> Cpose_start__T__Ccurrent_estim_msg{max_num_cameras};
  std::vector<geometry_msgs::TransformStamped> Cpose_start__T__Ccurrent_GT_msg{max_num_cameras};
  std::vector<geometry_msgs::TransformStamped> Cpose_start__T__Ccurrent_svd_msg{max_num_cameras};
  std::vector<geometry_msgs::TransformStamped> Cpose_start__T__Ccurrent_icp_msg{max_num_cameras};
  std::vector<sensor_msgs::PointCloud2> Cn0__t__Cn0_P0_o3d_msg{max_num_cameras};
  std::vector<sensor_msgs::PointCloud2> Cn0__t__Cn0_P0_o3d_downsampled_msg{max_num_cameras};
  std::vector<sensor_msgs::PointCloud2> Cn0__t__Cn0_P0_o3d_overlap_msg{max_num_cameras};
  std::vector<sensor_msgs::PointCloud2> Cn0__t__Cn0_P0_o3d_overlap_downsampled_msg{max_num_cameras};
};

struct PostprocessingToHoloscanData {};
struct OptimizationToRosData {};
struct OptimizationToHoloscanData {};

}  // namespace config

#endif  // CONFIG_OPTIMIZATION_DATA_H
