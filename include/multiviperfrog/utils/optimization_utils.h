

#ifndef UTILS_OPTIMIZATION_UTILS_H
#define UTILS_OPTIMIZATION_UTILS_H

#include <open3d/Open3D.h>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <tuple>
#include <vector>
#include "ceres/ceres.h"
#include "multiviperfrog/config/ceres_config.h"
#include "multiviperfrog/config/experiment_config.h"
#include "multiviperfrog/core/optimization/cost_functions/odo_and_trueflow_from_biasedflow_errorterm_a.h"

namespace utils {

// region*** functions from optimization.cpp (folded here)

// compute the ADD metric for two transforms and a set of points (e.g. x1_GT, x1_noisy, and m2 to reconstruct x3)
void ComputeADDMetricForTransforms(const Eigen::Transform<double, 3, Eigen::Isometry>& Csource__T__Ctarget_GT,
                                   const Eigen::Transform<double, 3, Eigen::Isometry>& Csource__T__Ctarget_estimated,
                                   const Eigen::Matrix4Xd& Ctarget__t__Ctarget_P1_GT, double* ADD_Csource__t__Csource_P1);

// compute the ADD metric for two sets of points (e.g. Cb0_SF_GT and Cb0_SF_estimated)
void ComputeADDMetricForSF(const Eigen::Matrix4Xd& Csource_SF_GT, const Eigen::Matrix4Xd& Csource_SF_estimated, double* ADD_Csource_SF);

// add noise to the translation part of the transform
void AddNoiseToTransformTranslation(Eigen::Transform<double, 3, Eigen::Isometry>* Tf, double noise_std_dev, unsigned int seed);

// add noise to the rotation part of the transform using the tangent space
void AddNoiseToTransformRotationTangent(Eigen::Transform<double, 3, Eigen::Isometry>* Tf, double noise_std_dev, unsigned int seed);

void AddNoiseToTransform(Eigen::Transform<double, 3, Eigen::Isometry>* Tf, double noise_std_dev, unsigned int seed);

void AddNoiseToPoints(Eigen::Matrix4Xd* Points, double noise_std_dev, unsigned int seed);

// print of output performance TODO change variables name from init to GT, unify in one function with above
bool OutputOptimizedParams(const Eigen::Transform<double, 3, Eigen::Isometry>& Ca1__T__Cb1_,        // measure m1
                           const Eigen::Matrix4Xd& Ca1__t__Ca1_P1_measured_,                        // measure m2
                           const Eigen::Matrix4Xd& Cb1__t__Cb1_P1_measured_,                        // measure m3
                           const Eigen::Matrix4Xd& Cb0__t__Cb0_P0_measured_,                        // measure m5
                           const Eigen::Transform<double, 3, Eigen::Isometry>& Ca0__T__Ca1_GT_,     // parameter x1 GT
                           const Eigen::Transform<double, 3, Eigen::Isometry>& Ca0__T__Ca1_estim_,  // parameter x1
                           const Eigen::Transform<double, 3, Eigen::Isometry>& Cb0__T__Cb1_GT_,     // parameter x2 GT
                           const Eigen::Transform<double, 3, Eigen::Isometry>& Cb0__T__Cb1_estim_,  // parameter x2
                           const Eigen::Matrix4Xd& Cb0__t__P0_P1_GT_,                               // parameter Cb0_SF GT
                           const Eigen::Matrix4Xd& Cb0__t__P0_P1_estim_,                            // parameter Cb0_SF
                           const Eigen::Transform<double, 3, Eigen::Isometry>& Ca0__T__Cb0_GT_,     // parameter x5 GT
                           const Eigen::Transform<double, 3, Eigen::Isometry>& Ca0__T__Cb0_estim_,  // parameter x5
                           struct config::ExperimentConfig config);

void OutputMetricsToFile(config::ExperimentConfig& config, const config::output_settings& output);

// plotting
void PlotCloudsCorrespondences(std::shared_ptr<open3d::geometry::PointCloud> pcd1, std::shared_ptr<open3d::geometry::PointCloud> pcd2);

void PlotCloudsCorrespondencesPlusAnotherCloud(std::shared_ptr<open3d::geometry::PointCloud> pcd1,
                                               std::shared_ptr<open3d::geometry::PointCloud> pcd2,
                                               std::shared_ptr<open3d::geometry::PointCloud> pcd3);
void PlotCloudsCorrespondences3clouds(std::shared_ptr<open3d::geometry::PointCloud> pcd1,
                                      std::shared_ptr<open3d::geometry::PointCloud> pcd2,
                                      std::shared_ptr<open3d::geometry::PointCloud> pcd3);

void PlotCloudsCorrespondences4clouds(std::shared_ptr<open3d::geometry::PointCloud> pcd1,
                                      std::shared_ptr<open3d::geometry::PointCloud> pcd2,
                                      std::shared_ptr<open3d::geometry::PointCloud> pcd3,
                                      std::shared_ptr<open3d::geometry::PointCloud> pcd4);

// endregion

std::vector<size_t> getUpperPercentileIndices(const Eigen::VectorXd& values, double& avg_upper_percentile, double percentile = 90.0);

std::vector<size_t> getHigherThanThresholdIndices(const Eigen::VectorXd& values, double threshold);

std::vector<size_t> findMatchingIndicesInVector(const std::vector<size_t>& upper_percentile_indices,
                                                                                           const std::vector<size_t>& indexes_downsampled_in_overlap) ;
std::pair<std::vector<size_t>, std::vector<size_t>> findMatchingTripletIndices(
    const std::vector<size_t>& upper_percentile_indices_a, const std::vector<size_t>& upper_percentile_indices_b,
    const std::vector<std::tuple<std::optional<int>, int, std::optional<int>>> triplets_indexes_downsampled_in_overlap);

Eigen::Matrix4Xd computeAbsSceneFlowFromOdometry(const Eigen::Isometry3d Ccurrent__T__Cnext_estim, const Eigen::Matrix4Xd Cn0__t__Cn0_P0,
                                                 const Eigen::Matrix4Xd Cn1__t__Cn1_P1);

std::vector<Eigen::Vector3d> o3dColorsVectorFromRigidityLabelsGreenRed(const Eigen::VectorXd& labels);
}  // namespace utils

#endif  // UTILS_OPTIMIZATION_UTILS_H
