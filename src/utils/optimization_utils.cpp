

#include "multiviperfrog/utils/optimization_utils.h"

namespace utils {

// region*** functions from optimization.cpp (folded here)

// compute the ADD metric for two transforms and a set of points (e.g. x1_GT, x1_noisy, and m2 to reconstruct x3)
void ComputeADDMetricForTransforms(const Eigen::Transform<double, 3, Eigen::Isometry>& Csource__T__Ctarget_GT,
                                   const Eigen::Transform<double, 3, Eigen::Isometry>& Csource__T__Ctarget_estimated,
                                   const Eigen::Matrix4Xd& Ctarget__t__Ctarget_P1_GT, double* ADD_Csource__t__Csource_P1) {
  // compute the ADD metric
  *ADD_Csource__t__Csource_P1 = 0.0;
  for (int i = 0; i < Ctarget__t__Ctarget_P1_GT.cols(); i++) {
    Eigen::Vector4d Ctarget__t__Ctarget_P1i_GT = Ctarget__t__Ctarget_P1_GT.col(i);
    Eigen::Vector4d Csource__t__Csource_P1_GT = Csource__T__Ctarget_GT * Ctarget__t__Ctarget_P1i_GT;
    Eigen::Vector4d Csource__t__Csource_P1_estimated = Csource__T__Ctarget_estimated * Ctarget__t__Ctarget_P1i_GT;
    *ADD_Csource__t__Csource_P1 += (Csource__t__Csource_P1_GT - Csource__t__Csource_P1_estimated).norm();
  }
  // normalize the ADD metric
  *ADD_Csource__t__Csource_P1 /= Ctarget__t__Ctarget_P1_GT.cols();
}

// compute the ADD metric for two sets of points (e.g. Cb0_SF_GT and Cb0_SF_estimated)
void ComputeADDMetricForSF(const Eigen::Matrix4Xd& Csource_SF_GT, const Eigen::Matrix4Xd& Csource_SF_estimated, double* ADD_Csource_SF) {
  // compute the ADD metric
  *ADD_Csource_SF = 0.0;
  for (int i = 0; i < Csource_SF_GT.cols(); i++) {
    Eigen::Vector4d Csource_SFi_GT = Csource_SF_GT.col(i);
    Eigen::Vector4d Csource_SFi_estimated = Csource_SF_estimated.col(i);
    *ADD_Csource_SF += (Csource_SFi_GT - Csource_SFi_estimated).norm();
  }
  // normalize the ADD metric
  *ADD_Csource_SF /= Csource_SF_GT.cols();
}

// add noise to the translation part of the transform
void AddNoiseToTransformTranslation(Eigen::Transform<double, 3, Eigen::Isometry>* Tf, double noise_std_dev, unsigned int seed) {
  // generate random noise
  srand(seed);
  Eigen::Vector3d noise;
  noise.setRandom();  // actually using rand(), check for better random number generator
  noise *= noise_std_dev;
  // print value of noise
  //        std::cout << "noise translation:\n" << noise.transpose() << std::endl;
  // update the translation part of the transform
  Tf->translation() += noise;
}

// add noise to the rotation part of the transform using the tangent space
void AddNoiseToTransformRotationTangent(Eigen::Transform<double, 3, Eigen::Isometry>* Tf, double noise_std_dev, unsigned int seed) {
  // generate random noise
  srand(seed);
  Eigen::Vector3d noise;
  noise.setRandom();  // actually using rand(), check for better random number generator
  noise *= noise_std_dev;
  // TODO: CHECK should this not be a quaternion? do we simply add noise to the coefficients?
  // define quaternion
  Eigen::Quaterniond q = Eigen::Quaterniond(Tf->linear());
  Eigen::Quaterniond* q_noisy = new Eigen::Quaterniond;
  // add noise to the rotation part of the transform using the tangent space
  ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold;
  quaternion_manifold->Plus(q.coeffs().data(), noise.data(), q_noisy->coeffs().data());
  q_noisy->normalize();
  // print value of noise
  //    std::cout << "noise rotation:\n" << noise.transpose() << std::endl;
  // update the rotation part of the transform
  Tf->linear() = q_noisy->toRotationMatrix();
}

void AddNoiseToTransform(Eigen::Transform<double, 3, Eigen::Isometry>* Tf, double noise_std_dev, unsigned int seed) {
  // add noise to the translation part of the transform
  AddNoiseToTransformTranslation(Tf, noise_std_dev, seed);
  // add noise to the rotation part of the transform using the tangent space
  AddNoiseToTransformRotationTangent(Tf, noise_std_dev, seed);
}

void AddNoiseToPoints(Eigen::Matrix4Xd* Points, double noise_std_dev, unsigned int seed) {
  srand(seed);
  for (int i = 0; i < Points->cols(); i++) {
    // generate random noise
    Eigen::Vector3d noise;
    noise.setRandom();  // actually using rand(), check for better random number generator
    noise *= noise_std_dev;
    // print value of noise
    //        std::cout << "noise point:#" <<i<<"\n"<< noise.transpose() << std::endl;
    // update the translation part of the transform
    Points->block(0, i, 3, 1) += noise;
  }
}

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
                           struct config::ExperimentConfig config) {
  std::cout << "-------------- Optimized Parameters Errors-------------" << std::endl;

  // print the results of optimization compared to the GT values
  // x1
  std::cout << "Ca0__T__Ca1_error:\n" << (Ca0__T__Ca1_estim_ * Ca0__T__Ca1_GT_.inverse()).matrix() << std::endl;
  // x2
  std::cout << "Cb0__T__Cb1_error:\n" << (Cb0__T__Cb1_estim_ * Cb0__T__Cb1_GT_.inverse()).matrix() << std::endl;
  // x5
  std::cout << "Ca0__T__Cb0_error:\n" << (Ca0__T__Cb0_estim_ * Ca0__T__Cb0_GT_.inverse()).matrix() << std::endl;
  // Cb0_SF
  std::cout << "Cb0__t__P0_P1_error(avg):\n" << (Cb0__t__P0_P1_estim_ - Cb0__t__P0_P1_GT_).transpose().colwise().mean() << std::endl;

  // reconstruct the measures from the estimated parameters
  // m3
  Eigen::Matrix4Xd Cb0__t__Cb0_P1_estim_;
  Cb0__t__Cb0_P1_estim_.setZero(4, config.num_points);
  Cb0__t__Cb0_P1_estim_.row(3) = Eigen::VectorXd::Ones(config.num_points);
  Cb0__t__Cb0_P1_estim_.block(0, 0, 3, config.num_points) =
      Cb0__t__Cb0_P0_measured_.block(0, 0, 3, config.num_points) + Cb0__t__P0_P1_estim_.block(0, 0, 3, config.num_points);
  auto Cb1__t__Cb1_P1_estim_ = Cb0__T__Cb1_estim_.matrix().inverse() * Cb0__t__Cb0_P1_estim_;
  // m2
  auto Ca1__t__Ca1_P1_estim_ = Ca0__T__Ca1_estim_.matrix().inverse() * Ca0__T__Cb0_estim_ * Cb0__t__Cb0_P1_estim_;
  // m1
  auto Ca1__T__Cb1_estim_ = Ca0__T__Ca1_estim_.inverse() * Ca0__T__Cb0_estim_ * Cb0__T__Cb1_estim_;

  // test criterion for true convergence check (only on the measures)
  float tolerance_Tf = 1e-4;
  float tolerance_points = 1e-4;
  if (!Ca1__t__Ca1_P1_estim_.isApprox(Ca1__t__Ca1_P1_measured_, tolerance_points)) {
    std::cout << "m2_reconstructed error over threshold of: " << tolerance_points << " :(" << std::endl;
    return false;
  } else if (!Cb1__t__Cb1_P1_estim_.isApprox(Cb1__t__Cb1_P1_measured_, tolerance_points)) {
    std::cout << "m3_reconstructed error over threshold of: " << tolerance_points << " :(" << std::endl;
    return false;
  } else if (!Ca1__T__Cb1_estim_.isApprox(Ca1__T__Cb1_, tolerance_Tf)) {
    std::cout << "m1_reconstructed error over threshold of: " << tolerance_Tf << " :(" << std::endl;
    return false;
  } else {
    std::cout << "Optimization proofed!! :):):)" << std::endl;
    return true;
  }
}

void OutputMetricsToFile(config::ExperimentConfig& config, const config::output_settings& output) {
  // TODO: modify the script so that it adds the header only once and then appends the results so it can be iterated
  std::cout << "-----------------Saving Metrics To file----------------" << std::endl;
  std::cout << "Output directory: " << config.output_directory << std::endl;
  std::cout << "Experiment name: " << config.experiment_name << std::endl;
  std::string filename = config.output_directory + "/" + config.experiment_name + ".csv";
  // add the header and generate file f it is the first time
  if (config.header_flag == 0) {
    std::ofstream outfile;
    outfile.open(filename, std::ios::out);
    if (!outfile.is_open()) {
      std::cerr << "Error: Unable to open file " << filename << " for appending." << std::endl;
      return;
    }
    // TODO: NOTE on 06-03-2024 changed the header to stick with the new variables convention (paper). change later also in the code!
    outfile << "total_avg_opt_time, ADD_metric_input_DA, ADD_metric_input_x3, ADD_metric_output_x3, "
               "ADD_metric_input_x4, ADD_metric_output_x4, ADD_metric_input_x1, "
               "ADD_metric_output_x1, ADD_metric_input_x2, ADD_metric_output_x2, "
               "ADD_metric_input_x5, ADD_metric_output_x5, "
               "num_points, num_known_SF, noise_SD, noise_DA_SD, "
               "alpha_span, seed, tot_time_rep, "
               "alpha_pointsA, alpha_pointsB, alpha_DataAssC0, alpha_DataAssC1, "
               "alpha_chain, alpha_SF, alpha_odoA, alpha_odoB, "
               "alpha_C2C0, alpha_C2C1"
            << std::endl;
    config.header_flag = 1;
    outfile.close();
  }
  // append the results
  std::ofstream outfile(config.output_directory + "/" + config.experiment_name + ".csv", std::ios::app);
  outfile << output.total_avg_opt_time << ", " << output.ADD_metric_input_DA_avg << ", " << output.ADD_metric_input_x1_avg << ", "
          << output.ADD_metric_output_x1_avg << ", " << output.ADD_metric_input_x2_avg << ", " << output.ADD_metric_output_x2_avg << ", "
          << output.ADD_metric_input_x5_avg << ", " << output.ADD_metric_output_x5_avg << ", " << output.ADD_metric_input_x6_avg << ", "
          << output.ADD_metric_output_x6_avg << ", " << output.ADD_metric_input_SF_avg << ", " << output.ADD_metric_output_SF_avg << ", "
          << config.num_points << ", " << config.num_known_SF << ", " << config.noise_SD << ", " << config.noise_DA_SD << ", "
          << config.alpha_span << ", " << config.seed << ", " << config.tot_time_rep + 1 << ", " << config.alpha_pointsA << ", "
          << config.alpha_pointsB << ", " << config.alpha_DataAssC0 << ", " << config.alpha_DataAssC1 << ", " << config.alpha_chain << ", "
          << config.alpha_SF << ", " << config.alpha_odoA << ", " << config.alpha_odoB << ", " << config.alpha_C2C0 << ", "
          << config.alpha_C2C1 << std::endl;
  outfile.close();
  std::cout << "----------------------Metrics Saved--------------------" << std::endl;  // TODO add writing error check
}

// plotting
void PlotCloudsCorrespondences(std::shared_ptr<open3d::geometry::PointCloud> pcd1, std::shared_ptr<open3d::geometry::PointCloud> pcd2) {
  // create a set of lines connectiong each corresponding point in the two pointclouds using open3d.geometry.LineSet using the
  // open3d::geometry::PointCloud>() objects as input
  auto lines = std::make_shared<open3d::geometry::LineSet>();
  for (int i = 0; i < pcd1->points_.size(); i++) {
    lines->points_.push_back(pcd1->points_[i]);
    lines->points_.push_back(pcd2->points_[i]);
    lines->lines_.push_back(Eigen::Vector2i(2 * i, 2 * i + 1));
    // set lines color to light blue
    lines->colors_.push_back(Eigen::Vector3d(0.7, 0.7, 1.0));
  }
      open3d::visualization::DrawGeometries({pcd1, pcd2, lines});
//  open3d::visualization::DrawGeometries({pcd2, lines});
}

void PlotCloudsCorrespondencesPlusAnotherCloud(std::shared_ptr<open3d::geometry::PointCloud> pcd1,
                                               std::shared_ptr<open3d::geometry::PointCloud> pcd2,
                                               std::shared_ptr<open3d::geometry::PointCloud> pcd3) {
  // create a set of lines connectiong each corresponding point in the two pointclouds using open3d.geometry.LineSet using the
  // open3d::geometry::PointCloud>() objects as input
  auto lines = std::make_shared<open3d::geometry::LineSet>();
  for (int i = 0; i < pcd1->points_.size(); i++) {
    lines->points_.push_back(pcd1->points_[i]);
    lines->points_.push_back(pcd2->points_[i]);
    lines->lines_.push_back(Eigen::Vector2i(2 * i, 2 * i + 1));
  }
  open3d::visualization::DrawGeometries({pcd1, pcd2, lines, pcd3});
}
void PlotCloudsCorrespondences3clouds(std::shared_ptr<open3d::geometry::PointCloud> pcd1,
                                      std::shared_ptr<open3d::geometry::PointCloud> pcd2,
                                      std::shared_ptr<open3d::geometry::PointCloud> pcd3) {
  // create a set of lines connectiong each corresponding point in the two pointclouds using open3d.geometry.LineSet using the
  // open3d::geometry::PointCloud>() objects as input
  auto lines12 = std::make_shared<open3d::geometry::LineSet>();
  auto lines13 = std::make_shared<open3d::geometry::LineSet>();
  for (int i = 0; i < pcd1->points_.size(); ++i) {
    lines12->points_.push_back(pcd1->points_[i]);
    lines12->points_.push_back(pcd2->points_[i]);
    lines12->lines_.push_back(Eigen::Vector2i(2 * i, 2 * i + 1));
    lines13->points_.push_back(pcd1->points_[i]);
    lines13->points_.push_back(pcd3->points_[i]);
    lines13->lines_.push_back(Eigen::Vector2i(2 * i, 2 * i + 1));
  }
  open3d::visualization::DrawGeometries({pcd1, pcd2, pcd3, lines12, lines13});
}

void PlotCloudsCorrespondences4clouds(std::shared_ptr<open3d::geometry::PointCloud> pcd1,
                                      std::shared_ptr<open3d::geometry::PointCloud> pcd2,
                                      std::shared_ptr<open3d::geometry::PointCloud> pcd3,
                                      std::shared_ptr<open3d::geometry::PointCloud> pcd4) {
  // create a set of lines connectiong each corresponding point in the two pointclouds using open3d.geometry.LineSet using the
  // open3d::geometry::PointCloud>() objects as input
  auto lines12 = std::make_shared<open3d::geometry::LineSet>();
  auto lines13 = std::make_shared<open3d::geometry::LineSet>();
  auto lines14 = std::make_shared<open3d::geometry::LineSet>();
  auto lines23 = std::make_shared<open3d::geometry::LineSet>();
  auto lines34 = std::make_shared<open3d::geometry::LineSet>();
  for (int i = 0; i < pcd1->points_.size(); ++i) {
    lines12->points_.push_back(pcd1->points_[i]);
    lines12->points_.push_back(pcd2->points_[i]);
    lines12->lines_.push_back(Eigen::Vector2i(2 * i, 2 * i + 1));
    lines13->points_.push_back(pcd1->points_[i]);
    lines13->points_.push_back(pcd3->points_[i]);
    lines13->lines_.push_back(Eigen::Vector2i(2 * i, 2 * i + 1));
    lines14->points_.push_back(pcd1->points_[i]);
    lines14->points_.push_back(pcd4->points_[i]);
    lines14->lines_.push_back(Eigen::Vector2i(2 * i, 2 * i + 1));
    lines23->points_.push_back(pcd2->points_[i]);
    lines23->points_.push_back(pcd3->points_[i]);
    lines23->lines_.push_back(Eigen::Vector2i(2 * i, 2 * i + 1));
    lines34->points_.push_back(pcd3->points_[i]);
    lines34->points_.push_back(pcd4->points_[i]);
    lines34->lines_.push_back(Eigen::Vector2i(2 * i, 2 * i + 1));
  }
  // set lines12 as green
  lines12->colors_ = std::vector<Eigen::Vector3d>(lines12->lines_.size(), Eigen::Vector3d(0, 0, 1));
  lines13->colors_ = std::vector<Eigen::Vector3d>(lines13->lines_.size(), Eigen::Vector3d(0, 1, 0));
  lines14->colors_ = std::vector<Eigen::Vector3d>(lines14->lines_.size(), Eigen::Vector3d(1, 0, 0));

  open3d::visualization::DrawGeometries({pcd1, pcd2, pcd3, pcd4, lines12, lines13, lines14, lines23, lines34});
}

// endregion

std::vector<size_t> getUpperPercentileIndices(const Eigen::VectorXd& values, double& avg_upper_percentile, double percentile) {
  size_t n_top_values = static_cast<size_t>(std::ceil((1.0 - percentile) * values.size()));

  // Sort indices based on corresponding values in descending order
  std::vector<size_t> indices(values.size());
  std::iota(indices.begin(), indices.end(), 0);  // Fill with 0, 1, ..., values.size() - 1
  std::sort(indices.begin(), indices.end(), [&values](size_t i1, size_t i2) {
    return values[i1] > values[i2];  // Sort by value in descending order
  });

  // Select the top n_top_values indices
  std::vector<size_t> top_indices(indices.begin(), indices.begin() + n_top_values);

  // Calculate the average of the top 1.0-percentile values
  avg_upper_percentile = 0.0;
  for (size_t i = 0; i < n_top_values; ++i) {
    avg_upper_percentile += values[indices[i]];
  }
  avg_upper_percentile /= n_top_values;

  return top_indices;
}

std::vector<size_t> getHigherThanThresholdIndices(const Eigen::VectorXd& values, double threshold) {
  // Find indices of elements higher than the threshold
  std::vector<size_t> indices;
  for (size_t i = 0; i < values.size(); ++i) {
    if (values[i] >= threshold) {
      indices.push_back(i);
    }
  }

  return indices;
}

std::vector<size_t> findMatchingIndicesInVector(const std::vector<size_t>& upper_percentile_indices,
                                            const std::vector<size_t>& indexes_downsampled_in_overlap) {
  std::vector<size_t> matching_indices;
  // Convert upper_percentile_indices to an unordered_set for faster lookups
  std::unordered_set<size_t> upper_percentile_set(upper_percentile_indices.begin(), upper_percentile_indices.end());
  // Iterate over the vector of pairs
  for (size_t i = 0; i < indexes_downsampled_in_overlap.size(); ++i) {
    // Check if .first of the pair exists in upper_percentile_set
    if (upper_percentile_set.count(indexes_downsampled_in_overlap[i])) {
      matching_indices.push_back(i);  // Add index of the pair to matching_indices
    }
  }
  return matching_indices;
}

std::pair<std::vector<size_t>, std::vector<size_t>> findMatchingTripletIndices(
    const std::vector<size_t>& upper_percentile_indices_a, const std::vector<size_t>& upper_percentile_indices_b,
    const std::vector<std::tuple<std::optional<int>, int, std::optional<int>>> triplets_indexes_downsampled_in_overlap) {
  std::vector<size_t> matching_indices_a;
  std::vector<size_t> matching_indices_b;
  // Convert upper_percentile_indices to an unordered_set for faster lookups
  if (upper_percentile_indices_a.empty() || upper_percentile_indices_b.empty()) {
    throw std::runtime_error("findMatchingTripletIndices: upper_percentile_indices_a or upper_percentile_indices_b is empty");
  }
  std::unordered_set<size_t> upper_percentile_set_a(upper_percentile_indices_a.begin(), upper_percentile_indices_a.end());
  std::unordered_set<size_t> upper_percentile_set_b(upper_percentile_indices_b.begin(), upper_percentile_indices_b.end());
  // if triplets_indexes_downsampled_in_overlap is not None or empty
  if (triplets_indexes_downsampled_in_overlap.empty()) {
    throw std::runtime_error("findMatchingTripletIndices: triplets_indexes_downsampled_in_overlap is empty");
  }
  // Iterate over the vector of triplets
  for (size_t i = 0; i < triplets_indexes_downsampled_in_overlap.size(); ++i) {
    auto first = std::get<0>(triplets_indexes_downsampled_in_overlap[i]);
    auto third = std::get<2>(triplets_indexes_downsampled_in_overlap[i]);
    if (first.has_value() && upper_percentile_set_a.count(static_cast<size_t>(first.value()))) {
      matching_indices_a.push_back(i);
    }
    if (third.has_value() && upper_percentile_set_b.count(static_cast<size_t>(third.value()))) {
      matching_indices_b.push_back(i);
    }
  }
  return {matching_indices_a, matching_indices_b};
}

Eigen::Matrix4Xd computeAbsSceneFlowFromOdometry(const Eigen::Isometry3d Ccurrent__T__Cnext_estim, const Eigen::Matrix4Xd Cn0__t__Cn0_P0,
                                                 const Eigen::Matrix4Xd Cn1__t__Cn1_P1) {
  // compute the absolute scene flow from odometry and relative
  Eigen::Matrix4Xd Cn0__t__P0_P1 = Eigen::Matrix4Xd::Zero(4, Cn0__t__Cn0_P0.cols());
  Cn0__t__P0_P1 = (Ccurrent__T__Cnext_estim.matrix() * Cn1__t__Cn1_P1) - Cn0__t__Cn0_P0;
  Cn0__t__P0_P1.row(3) = Eigen::VectorXd::Ones(Cn0__t__P0_P1.cols());
  return Cn0__t__P0_P1;
}

std::vector<Eigen::Vector3d> o3dColorsVectorFromRigidityLabelsGreenRed(const Eigen::VectorXd& labels) {
  std::vector<Eigen::Vector3d> colors;
  colors.resize(labels.size());
  for (int i = 0; i < labels.size(); i++) {
    colors[i] = Eigen::Vector3d(1.0 - labels[i], labels[i], 0.0);
  }
  return colors;
}

}  // namespace utils
