

#ifndef UTILS_PREPROCESSING_UTILS_H
#define UTILS_PREPROCESSING_UTILS_H

#include <open3d/Open3D.h>
#include <Eigen/Geometry>  // For Eigen::Isometry3d and transformations
#include <fstream>
#include <functional>
#include <iostream>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>
#include "multiviperfrog/core/optimization/optimization.h"
#include "multiviperfrog/utils/KDTreeFlann.h"

namespace core {
class WorkFlowManager;
}

namespace utils {

// Declare preprocessing utility functions here
// region***  simple kinematics (folded here)
//
Eigen::Isometry3d CamOdoFromWorldPoses(const Eigen::Isometry3d& T_from_world_to_cam_current,
                                       const Eigen::Isometry3d& T_from_world_to_cam_next);
// perhaps redundant with a function inside of optimization.cpp
// TODO: make generci naming (e.g. source and target)

Eigen::Matrix3Xd transformPointsEigen(const Eigen::Isometry3d& src_T_dst, const Eigen::Matrix3Xd& points_src);

void downsampleO3dPointCloud(std::shared_ptr<open3d::geometry::PointCloud>& point_cloud, const config::ExperimentConfig& exp_config);

// custom downsample with Eigen
Eigen::Matrix4Xd downsampleCustomEigenMatrix(const Eigen::Matrix4Xd& matrix, std::vector<int>& indices,
                                             const config::ExperimentConfig& exp_config);
// custom downsample with Open3D
std::shared_ptr<open3d::geometry::PointCloud> downsampleCustomOpen3DPointCloud(const std::shared_ptr<open3d::geometry::PointCloud>& cloud,
                                                                               std::vector<size_t>& indices,
                                                                               const config::ExperimentConfig& exp_config,
                                                                               const bool random_downsample = false);
// Function to create a new o3d point cloud based on selected indices
std::shared_ptr<open3d::geometry::PointCloud> CreateSelectedPointCloud(const std::shared_ptr<open3d::geometry::PointCloud>& cloud,
                                                                       const std::vector<size_t>& indices);

// Function to create a new o3d point cloud based on selected indices pairs
std::shared_ptr<open3d::geometry::PointCloud> CreateSelectedPointCloudFromPairs(const std::shared_ptr<open3d::geometry::PointCloud>& cloud,
                                                                                const std::vector<std::pair<int, int>>& pairs,
                                                                                const bool is_first_cloud);

std::vector<std::tuple<std::optional<int>, int, std::optional<int>>> mergeVectorsToTriplet(
    const std::vector<std::pair<int, int>>& vector_a, const std::vector<std::pair<int, int>>& vector_b);

void AssignO3dPcdFieldsGivenIndex(const std::shared_ptr<open3d::geometry::PointCloud>& source_cloud,
                                  std::shared_ptr<open3d::geometry::PointCloud>& new_cloud, const int idx);

std::vector<std::pair<int, int>> RecoverNewPcdIndex(const std::vector<std::pair<int, int>>& pairs_a,
                                                    const std::vector<std::pair<int, int>>& pairs_b);

// Function to compute reproducible random downsample indices
std::vector<size_t> ComputeRandomDownsampleIndices(int seed, int N_desired, int N_max);
// Function to compute uniform downsample indices
std::vector<size_t> ComputeUniformDownsampleIndices(int H, int W, int N_desired);
// Function to compute the smallest integer ratio that reflects the aspect ratio
int ComputeMinimumPoints(int H, int W);
// Helper function to compute the greatest common divisor (GCD)
int GCD(int a, int b);

// endregion

//----------------------------------------------------------------------------------------------------------------------
// region***  advanced kinematics (folded here)
// functions from optimization.cpp, not yet called from preprocessing
void ComputeICPwithO3D(const std::shared_ptr<open3d::geometry::PointCloud>& Csource__t__Csource_P_measured_o3d,
                       const std::shared_ptr<open3d::geometry::PointCloud>& Ctarget__t__Ctarget_P_measured_o3d,
                       Eigen::Transform<double, 3, Eigen::Isometry>* Ctarget__T__Csource_o3d, const config::ExperimentConfig& exp_config);

void GetPairsOfPointsCloserThanThreshold(const Eigen::MatrixXd& cloud_a, const Eigen::MatrixXd& cloud_b, double threshold,
                                         std::vector<std::pair<int, int>>& pairs);

void GetPairsOfPointsCloserThanThresholdO3d(
    core::WorkFlowManager* manager, const std::shared_ptr<const open3d::geometry::PointCloud>& cloud_a, double threshold,
    std::vector<size_t>& output_indices_a, std::vector<size_t>& output_indices_b, bool reset_kdtree_geometry = true,
    bool minimize_duplicates = false,  // if true, the function will search in more KNN neighbors
    bool avoid_duplicates = false,     // if true, the function will skip if the point in cloud_b is already taken
    const std::shared_ptr<const open3d::geometry::PointCloud>& cloud_b_unregistered = nullptr,
    const Eigen::Transform<double, 3, Eigen::Isometry>* cloud_a__T__cloud_b = nullptr);

void GetPairsOfPointsCloserThanThresholdO3dParallel(  // TODO setup and import openMP
    const std::shared_ptr<const open3d::geometry::PointCloud>& cloud_a,
    const std::shared_ptr<const open3d::geometry::PointCloud>& cloud_b_unregistered,
    const Eigen::Transform<double, 3, Eigen::Isometry>& cloud_a__T__cloud_b, double threshold, std::vector<std::pair<int, int>>& pairs);

void makePointsSubsetIndices(std::vector<int>& indices_vector, int num_indices, int indices_max_range, unsigned int seed);

Eigen::Isometry3d computeAbsoluteOrientation(const Eigen::MatrixXd& points_src, const Eigen::MatrixXd& points_dst, bool use_horn = false);

Eigen::Matrix3d computeRotationFromCovarianceSVD(const Eigen::Matrix3d& W);
Eigen::Matrix3d computeRotationFromCovarianceHORN(const Eigen::Matrix3d& W);

Eigen::Isometry3d ransacAbsoluteOrientation(const Eigen::MatrixXd& points_A, const Eigen::MatrixXd& points_B, int max_iterations,
                                            double inlier_threshold, int sample_size, bool use_horn = false);

// endregion
}  // namespace utils

#endif  // UTILS_PREPROCESSING_UTILS_H
