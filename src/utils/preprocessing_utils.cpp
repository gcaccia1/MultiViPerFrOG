

#include "multiviperfrog/utils/preprocessing_utils.h"
#include <omp.h>
#include "multiviperfrog/core/workflow_manager.h"
#include "multiviperfrog/utils/KDTreeFlann.h"

namespace utils {

// Implement preprocessing utility functions here
// region***  simple kinematics (folded here)
// Function to compute the transformation from the current frame to the next frame
Eigen::Isometry3d CamOdoFromWorldPoses(const Eigen::Isometry3d& T_from_world_to_cam_current,
                                       const Eigen::Isometry3d& T_from_world_to_cam_next) {
  // Calculate the transformation from cam1 in the current frame to cam1 in the next frame
  Eigen::Isometry3d T_cam1_from_current_to_next = T_from_world_to_cam_current.inverse() * T_from_world_to_cam_next;
  return T_cam1_from_current_to_next;
}

Eigen::Matrix3Xd transformPointsEigen(const Eigen::Isometry3d& src_T_dst, const Eigen::Matrix3Xd& points_src) {
  // Transform the points using the given transformation matrix
  // points_dst = dst_T_src * points_src IMPORTANT!!
  Eigen::Matrix3Xd points_dst = src_T_dst.linear().transpose() * points_src - src_T_dst.translation().replicate(1, points_src.cols());
  return points_dst;
}

void downsampleO3dPointCloud(std::shared_ptr<open3d::geometry::PointCloud>& point_cloud, const config::ExperimentConfig& exp_config) {
  if (exp_config.downsample_strategy == "uniform") {
    point_cloud = point_cloud->UniformDownSample(exp_config.downsample_interval);
  } else if (exp_config.downsample_strategy == "random") {
    point_cloud = point_cloud->RandomDownSample(exp_config.downsample_ratio);
  } else if (exp_config.downsample_strategy == "voxel") {
    point_cloud = point_cloud->VoxelDownSample(exp_config.downsample_voxel_size);
  } else {
    throw std::runtime_error("downsampleO3dPointCloud: no downsampling method selected [random, uniform, voxel]!");
  }
}

// custom downsample with Eigen
Eigen::Matrix4Xd downsampleCustomEigenMatrix(const Eigen::Matrix4Xd& matrix, std::vector<size_t>& indices,
                                             const config::ExperimentConfig& exp_config) {
  // TODO: implement logic like in downsampleCustomOpen3DPointCloud to handle random and uniform downsampling
  // returns a downsampled flat point cloud, and saves the indices of the selected points in the provided vector
  int H = exp_config.cloud_original_height;
  int W = exp_config.cloud_original_width;
  int N_desired = exp_config.desired_downsampled_points_number_uniform;
  // Ensure the matrix has the expected number of columns
  if (matrix.cols() != H * W) {
    throw std::runtime_error("Matrix size does not match the provided grid dimensions.");
  }
  // Compute indices for downsampling
  indices = ComputeUniformDownsampleIndices(H, W, N_desired);
  // Create the downsampled matrix
  Eigen::Matrix4Xd downsampled_matrix(4, indices.size());
  for (size_t i = 0; i < indices.size(); ++i) {
    downsampled_matrix.col(i) = matrix.col(indices[i]);
  }
  return downsampled_matrix;
}
// custom downsample with Open3D
std::shared_ptr<open3d::geometry::PointCloud> downsampleCustomOpen3DPointCloud(const std::shared_ptr<open3d::geometry::PointCloud>& cloud,
                                                                               std::vector<size_t>& indices,
                                                                               const config::ExperimentConfig& exp_config,
                                                                               bool random_downsample) {
  // returns a downsampled flat point cloud, and saves the indices of the selected points in the provided vector

  if (random_downsample) {
    int seed = exp_config.seed_random_downsampling;
    int N_desired = exp_config.desired_downsampled_points_number_random;
    int N_max = cloud->points_.size();
    if (N_desired > N_max) {
      throw std::runtime_error(
          "downsampleCustomOpen3DPointCloud: "
          "Requested number of random samples exceeds size of the cloud.");
    }
    // Compute indices for downsampling
    indices = ComputeRandomDownsampleIndices(seed, N_desired, N_max);
  } else {
    int H = exp_config.cloud_original_height;
    int W = exp_config.cloud_original_width;
    int N_desired = exp_config.desired_downsampled_points_number_uniform;
    // Ensure the cloud has the expected number of points
    if (cloud->points_.size() != H * W) {
      throw std::runtime_error(
          "downsampleCustomOpen3DPointCloud: "
          "Point cloud size does not match the provided HxW.");
    }
    // Compute indices for downsampling
    indices = ComputeUniformDownsampleIndices(H, W, N_desired);
  }
  // createa cloud with the selected indices
  return CreateSelectedPointCloud(cloud, indices);
}

// Function to create a new o3d point cloud based on selected indices
std::shared_ptr<open3d::geometry::PointCloud> CreateSelectedPointCloud(const std::shared_ptr<open3d::geometry::PointCloud>& cloud,
                                                                       const std::vector<size_t>& indices) {
  // Create a new point cloud using make_shared
  auto selected_cloud = std::make_shared<open3d::geometry::PointCloud>();
  selected_cloud->points_.reserve(indices.size());
  // Loop through the provided indices and copy points, colors, and normals
  for (int idx : indices) {
    selected_cloud->points_.push_back(cloud->points_[idx]);
    // Copy colors if they exist
    if (!cloud->colors_.empty()) {
      selected_cloud->colors_.push_back(cloud->colors_[idx]);
    }
    // Copy normals if they exist
    if (!cloud->normals_.empty()) {
      selected_cloud->normals_.push_back(cloud->normals_[idx]);
    }
  }
  return selected_cloud;
}

// this funtion is used to merge two vectors of pairs into a vector of triplets with optional values avoiding duplicates in the second value
// of each pair, alway access the optional values with "if (triplet.first)"...
std::vector<std::tuple<std::optional<int>, int, std::optional<int>>> mergeVectorsToTriplet(
    const std::vector<std::pair<int, int>>& vector_a, const std::vector<std::pair<int, int>>& vector_b) {
  std::unordered_map<int, std::pair<std::optional<int>, std::optional<int>>> combined_map;
  // Process vector_a
  for (const auto& [first_a, second] : vector_a) {
    combined_map[second].first = first_a;
  }
  // Process vector_b
  for (const auto& [first_b, second] : vector_b) {
    combined_map[second].second = first_b;
  }
  // Build the result vector
  std::vector<std::tuple<std::optional<int>, int, std::optional<int>>> merged_vector;
  for (const auto& [second, firsts] : combined_map) {
    merged_vector.emplace_back(firsts.first, second, firsts.second);
  }
  // Sort merged_vector by the `second` value (middle element of each triplet)
  std::sort(merged_vector.begin(), merged_vector.end(),
            [](const std::tuple<std::optional<int>, int, std::optional<int>>& a,
               const std::tuple<std::optional<int>, int, std::optional<int>>& b) { return std::get<1>(a) < std::get<1>(b); });

  return merged_vector;
}

// Function to create a new o3d point cloud based on selected indices pairs
std::shared_ptr<open3d::geometry::PointCloud> CreateSelectedPointCloudFromPairs(const std::shared_ptr<open3d::geometry::PointCloud>& cloud,
                                                                                const std::vector<std::pair<int, int>>& pairs,
                                                                                const bool is_first_cloud = true) {
  // Create a new point cloud using make_shared
  auto selected_cloud = std::make_shared<open3d::geometry::PointCloud>();
  selected_cloud->points_.reserve(pairs.size());
  // Loop through the provided indices and copy points, colors, and normals
  for (const auto& pair : pairs) {
    int idx;
    if (is_first_cloud) {
      idx = pair.first;
    } else {
      idx = pair.second;
    }
    AssignO3dPcdFieldsGivenIndex(cloud, selected_cloud, idx);
  }
  return selected_cloud;
}

void AssignO3dPcdFieldsGivenIndex(const std::shared_ptr<open3d::geometry::PointCloud>& source_cloud,
                                  std::shared_ptr<open3d::geometry::PointCloud>& new_cloud, const int idx) {
  new_cloud->points_.push_back(source_cloud->points_[idx]);
  // Copy colors if they exist
  if (!source_cloud->colors_.empty()) {
    new_cloud->colors_.push_back(source_cloud->colors_[idx]);
  }
  // Copy normals if they exist
  if (!source_cloud->normals_.empty()) {
    new_cloud->normals_.push_back(source_cloud->normals_[idx]);
  }
}

#include <unordered_map>
#include <utility>  // For std::pair
#include <vector>

// Function used to convert the indexes of original KDTREE.second into the indexes of the pointclouds generated from the overlap region
// results very cryptic as a description, and it is indeed. TODO: clarify when shown that it works, before forgetting the logic involved!!
std::vector<std::pair<int, int>> RecoverNewPcdIndex(const std::vector<std::pair<int, int>>& pairs_a,
                                                    const std::vector<std::pair<int, int>>& pairs_b) {
  // Step 1: Create an unordered_map from pairs_b for efficient lookup
  std::unordered_map<int, int> pairs_b_map;
  for (size_t i = 0; i < pairs_b.size(); ++i) {
    pairs_b_map[pairs_b[i].second] = i;  // Map pairs_b.second to its index
  }
  // Step 2: Create the resulting vector for pairs_c
  std::vector<std::pair<int, int>> pairs_c;

  // Step 3: Search pairs_a elements in pairs_b_map
  for (const auto& pair_a : pairs_a) {
    auto it = pairs_b_map.find(pair_a.second);
    if (it != pairs_b_map.end()) {
      // pair_a.second found in pairs_b, add to pairs_c
      pairs_c.emplace_back(pair_a.first, it->second);
    } else {
      // Try to find pair_a.second + 1
      auto it_plus = pairs_b_map.find(pair_a.second + 1);
      if (it_plus != pairs_b_map.end()) {
        pairs_c.emplace_back(pair_a.first, it_plus->second);
      } else {
        // Try to find pair_a.second - 1
        auto it_minus = pairs_b_map.find(pair_a.second - 1);
        if (it_minus != pairs_b_map.end()) {
          pairs_c.emplace_back(pair_a.first, it_minus->second);
        }
        // If neither is found, do nothing
      }
    }
  }
  return pairs_c;
}

// Function to compute reproducible random downsample indices
std::vector<size_t> ComputeRandomDownsampleIndices(int seed, int N_desired, int N_max) {
  // Initialize the random number generator
  srand(seed);
  // Create a vector to store the indices
  std::vector<size_t> indices;
  // Generate random indices
  for (int i = 0; i < N_desired; ++i) {
    int idx = rand() % N_max;
    // Check if the index is already in the vector
    while (std::find(indices.begin(), indices.end(), idx) != indices.end()) {
      idx = rand() % N_max;
    }
    indices.push_back(idx);
  }
  //        std::cout<< "ComputeRandomDownsampleIndices: Generated " << indices.size() << " random indices." << std::endl;
  return indices;
}
// Function to compute uniform downsample indices
std::vector<size_t> ComputeUniformDownsampleIndices(int H, int W, int N_desired) {
  // Compute the minimum number of samples based on the aspect ratio
  int N_min = ComputeMinimumPoints(H, W);
  int N_max = H * W;
  std::vector<size_t> indices;
  // Clamp N_desired within the valid range
  if (N_desired > N_max) {
    //            std::cout << "Requested number of samples exceeds the maximum possible. "
    //                      << "Using maximum number of samples: " << N_max << std::endl;
    N_desired = N_max;
  } else if (N_desired < N_min) {
    //            std::cout << "Requested number of samples is less than the minimum allowed. "
    //                      << "Using minimum number of samples: " << N_min << std::endl;
    N_desired = N_min;
  }
  // Aspect ratio
  double aspect_ratio = static_cast<double>(H) / W;
  // Initialize variables
  int best_num_points = N_max;
  int best_row_step = 1;
  int best_col_step = 1;
  double min_diff = std::abs(N_max - N_desired);
  // Estimate initial desired numbers of rows and columns
  double estimated_cols = sqrt(N_desired / aspect_ratio);
  double estimated_rows = estimated_cols * aspect_ratio;
  // Explore around the estimated values to find the best match
  for (int delta = -5; delta <= 5; ++delta) {
    int desired_cols = static_cast<int>(std::round(estimated_cols)) + delta;
    if (desired_cols < 1) continue;
    int desired_rows = static_cast<int>(std::round(estimated_rows)) + static_cast<int>(delta * aspect_ratio);
    if (desired_rows < 1) continue;
    // Compute sampling steps
    int row_step = static_cast<int>(std::floor(static_cast<double>(H) / desired_rows));
    int col_step = static_cast<int>(std::floor(static_cast<double>(W) / desired_cols));
    // Ensure steps are at least 1
    if (row_step < 1) row_step = 1;
    if (col_step < 1) col_step = 1;
    // Compute actual number of downsampled points
    int downsampled_rows = (H + row_step - 1) / row_step;
    int downsampled_cols = (W + col_step - 1) / col_step;
    int num_points = downsampled_rows * downsampled_cols;
    // Check if this is closer to the desired number
    double diff = std::abs(num_points - N_desired);
    if (diff < min_diff) {
      min_diff = diff;
      best_num_points = num_points;
      best_row_step = row_step;
      best_col_step = col_step;
    }
    // If exact match is found, break
    if (diff == 0) {
      break;
    }
  }
  // Inform the user if the exact number cannot be achieved
  if (best_num_points != N_desired) {
    //            std::cout << "Exact number of requested samples cannot be achieved. "
    //                      << "Using closest possible number of samples: " << best_num_points << std::endl;
  }
  // Generate indices based on the best steps found
  int idx = 0;
  for (int i = 0; i < H; i += best_row_step) {
    for (int j = 0; j < W; j += best_col_step) {
      int index = i * W + j;  // Linear index
      indices.push_back(index);
      ++idx;
      // Break if we reach the desired number of points
      if (idx >= best_num_points) {
        break;
      }
    }
    if (idx >= best_num_points) {
      break;
    }
  }
  return indices;
}
// Function to compute the smallest integer ratio that reflects the aspect ratio
int ComputeMinimumPoints(int H, int W) {
  int gcd = GCD(H, W);
  int min_rows = H / gcd;
  int min_cols = W / gcd;
  return min_rows * min_cols;
}
// Helper function to compute the greatest common divisor (GCD)
int GCD(int a, int b) {
  return b == 0 ? a : GCD(b, a % b);
}

// endregion
//----------------------------------------------------------------------------------------------------------------------
// region***  advanced kinematics (folded here)
// function to compute the transformation between two pointclouds using ICP with Open3D, returns Csource__T__Ctarget
void ComputeICPwithO3D(const std::shared_ptr<open3d::geometry::PointCloud>& Csource__t__Csource_P_measured_o3d,
                       const std::shared_ptr<open3d::geometry::PointCloud>& Ctarget__t__Ctarget_P_measured_o3d,
                       Eigen::Transform<double, 3, Eigen::Isometry>* Csource__T__Ctarget_o3d, const config::ExperimentConfig& exp_config) {
  // Start Timer
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  // Create a copy of the source and target pointclouds
  auto Csource__t__Csource_P_measured_o3d_copy = std::make_shared<open3d::geometry::PointCloud>(*Csource__t__Csource_P_measured_o3d);
  auto Ctarget__t__Ctarget_P_measured_o3d_copy = std::make_shared<open3d::geometry::PointCloud>(*Ctarget__t__Ctarget_P_measured_o3d);

  //        //downsample the pointclouds
  if (exp_config.downsample_ICP) {
    downsampleO3dPointCloud(Csource__t__Csource_P_measured_o3d_copy, exp_config);
    downsampleO3dPointCloud(Ctarget__t__Ctarget_P_measured_o3d_copy, exp_config);
  }

  // End Timer
  std::chrono::steady_clock::time_point end_downsample = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_downsample = (end_downsample - start);
  std::cout
      << "WorkFlowManager: Execution time (runProcessingCore:preprocessing:computeOdometryRigidClosedForm:ComputeICPwithO3D:Downsample: "
      << elapsed_downsample.count() * 1000 << " millliseconds" << std::endl;

  // Start Timer
  std::chrono::steady_clock::time_point start_normals = std::chrono::steady_clock::now();
  // normals estimation
  Csource__t__Csource_P_measured_o3d_copy->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01, 10));  // TODO undo hardcoded
  // normals normalization (why?)
  Csource__t__Csource_P_measured_o3d_copy->NormalizeNormals();
  // End Timer
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed = (end - start_normals);
  std::cout << "WorkFlowManager: Execution time "
               "(runProcessingCore:preprocessing:computeOdometryRigidClosedForm:ComputeICPwithO3D:EstimateNormals: "
            << elapsed.count() * 1000 << " millliseconds" << std::endl;

  // Start Timer
  std::chrono::steady_clock::time_point start_icp = std::chrono::steady_clock::now();
  // compute the transformation between the two pointclouds using ICP
  auto result = open3d::pipelines::registration::RegistrationICP(
      *Ctarget__t__Ctarget_P_measured_o3d_copy, *Csource__t__Csource_P_measured_o3d_copy, 0.01, Eigen::Matrix4d::Identity(),
      open3d::pipelines::registration::TransformationEstimationPointToPlane(),
      open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 20));

  // End Timer
  std::chrono::steady_clock::time_point end_icp = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_icp = (end_icp - start_icp);
  std::cout << "WorkFlowManager: Execution time (runProcessingCore:preprocessing:computeOdometryRigidClosedForm:ComputeICPwithO3D:ICP: "
            << elapsed_icp.count() * 1000 << " millliseconds" << std::endl;
  // set the estimated transformation
  Csource__T__Ctarget_o3d->matrix() = result.transformation_.cast<double>();
}

void GetPairsOfPointsCloserThanThreshold(const Eigen::MatrixXd& cloud_a, const Eigen::MatrixXd& cloud_b, double threshold,
                                         std::vector<std::pair<int, int>>& pairs) {
  // Build KDTree for cloud_b
  auto kdtree_b = std::make_shared<KDTreeFlann>();
  // add the points to the KDTree
  kdtree_b->SetMatrixData(cloud_b);
  // Query KDTree of cloud_b for each point in cloud_a
  std::vector<int> indices;
  std::vector<double> distances;
  int max_nn = 1;
  std::cout << "now searching for KNN pairs" << std::endl;
  for (size_t i = 0; i < cloud_a.cols(); i++) {
    kdtree_b->SearchHybrid(cloud_a.block(0, i, 3, 1), threshold, max_nn, indices, distances);
    if (indices.size() > 0) {
      pairs.emplace_back(i, indices[0]);
    }
  }
  std::cout << "Finished searching for KNN pairs" << std::endl;
}

void GetPairsOfPointsCloserThanThresholdO3d(core::WorkFlowManager* manager,
                                            const std::shared_ptr<const open3d::geometry::PointCloud>& cloud_a, double threshold,
                                            std::vector<size_t>& output_indices_a, std::vector<size_t>& output_indices_b,
                                            bool reset_kdtree_geometry, bool minimize_duplicates, bool avoid_duplicates,
                                            const std::shared_ptr<const open3d::geometry::PointCloud>& cloud_b_unregistered,
                                            const Eigen::Transform<double, 3, Eigen::Isometry>* cloud_a__T__cloud_b) {
  // Start Timer
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  auto& kdtree_b = manager->GetKDTree();
  std::chrono::steady_clock::time_point start_kdtree_reset = std::chrono::steady_clock::now();
  if (reset_kdtree_geometry) {
    if (!cloud_b_unregistered || !cloud_a__T__cloud_b) {
      throw std::invalid_argument(
          "GetPairsOfPointsCloserThanThresholdO3d: cloud_b_unregistered and cloud_a__T__cloud_b must be provided when "
          "reset_kdtree_geometry is true.");
    }
    std::cout << "GetPairsOfPointsCloserThanThresholdO3d: resetting KDTree geometry" << std::endl;
    // Create a transformed copy of cloud_b_unregistered
    auto transformed_cloud_b = std::make_shared<open3d::geometry::PointCloud>(*cloud_b_unregistered);
    transformed_cloud_b->Transform(cloud_a__T__cloud_b->matrix());
    // Set the KDTree with the transformed cloud
    manager->SetKDTreeAndCloud(transformed_cloud_b);
  }
  std::chrono::steady_clock::time_point start_loop = std::chrono::steady_clock::now();

  if (minimize_duplicates) {
    double squared_threshold = threshold * threshold;
    int max_nn = 10;  // maximum number of neighbors to consider within the radius
    auto cloud_b = manager->GetPersistentCloudKNN();
    // Initialize the matched flags for cloud_b
    std::vector<bool> is_matched_b(cloud_b->points_.size(), false);

    for (size_t i = 0; i < cloud_a->points_.size(); ++i) {
      std::vector<int> indices;
      std::vector<double> distances;

      // Perform hybrid search within the radius
      int num_neighbors = kdtree_b.SearchHybrid(cloud_a->points_[i], threshold, max_nn, indices, distances);
      int selected_idx_b = -1;
      if (num_neighbors > 0) {
        // Try to find a free neighbor within the radius
        for (int j = 0; j < num_neighbors; ++j) {
          int idx_b = indices[j];
          if (!is_matched_b[idx_b]) {
            selected_idx_b = idx_b;
            is_matched_b[idx_b] = true;
            break;
          }
        }
        if (selected_idx_b == -1) {
          // All neighbors within radius are already matched
          // Select the closest one among them
          selected_idx_b = indices[0];  // indices are sorted by distance
        }
      }
      if (selected_idx_b != -1) {
        // Store the indexes of a and b
        output_indices_a.push_back(i);
        output_indices_b.push_back(selected_idx_b);
      }
    }
  } else if (avoid_duplicates) {
    int max_nn = 1;
    auto cloud_b = manager->GetPersistentCloudKNN();
    std::vector<bool> is_matched_b(cloud_b->points_.size(), false);
    std::vector<std::pair<int, double>> previous_sq_distances_b(cloud_b->points_.size(), std::make_pair(-1, 1.0));
    std::vector<int> idx_b_to_pair_idx(cloud_b->points_.size(), -1);  // New mapping

    for (size_t i = 0; i < cloud_a->points_.size(); ++i) {
      std::vector<int> indices;
      std::vector<double> distances_a;
      int num_neighbors = kdtree_b.SearchHybrid(cloud_a->points_[i], threshold, max_nn, indices, distances_a);
      if (num_neighbors > 0) {
        int idx_b = indices[0];
        double current_sq_distance = distances_a[0] * distances_a[0];
        if (!is_matched_b[idx_b]) {
          is_matched_b[idx_b] = true;
          previous_sq_distances_b[idx_b] = std::make_pair(i, current_sq_distance);
          output_indices_a.push_back(i);
          output_indices_b.push_back(idx_b);
          idx_b_to_pair_idx[idx_b] = output_indices_a.size() - 1;  // Update the mapping
        } else {
          if (previous_sq_distances_b[idx_b].second < 1.0) {
            if (current_sq_distance < previous_sq_distances_b[idx_b].second) {
              previous_sq_distances_b[idx_b] = std::make_pair(i, current_sq_distance);
              int pair_idx = idx_b_to_pair_idx[idx_b];
              output_indices_a[pair_idx] = i;  // Update the pair
            }
          } else {
            throw std::runtime_error("GetPairsOfPointsCloserThanThresholdO3d: BUG: point already matched but no valid previous_distance");
          }
        }
      }
    }
  } else {  // Perform regular hybrid search for each point in cloud_a (might produce duplicates if both clouds are dense)
    for (size_t i = 0; i < cloud_a->points_.size(); ++i) {
      std::vector<int> indices;
      std::vector<double> distances;
      // Perform hybrid search within the radius
      int num_neighbors = kdtree_b.SearchHybrid(cloud_a->points_[i], threshold, 1, indices, distances);
      if (num_neighbors > 0) {
        output_indices_a.push_back(i);
        output_indices_b.push_back(indices[0]);
      }
    }
  }
  //    auto Persistent_cloud_b = std::make_shared<open3d::geometry::PointCloud>(*manager->GetPersistentCloudKNN());
  //    auto cloud_a_test = std::make_shared<open3d::geometry::PointCloud>(*cloud_a);
  //    // plot with cloud_a and b in neutralcolors
  //    cloud_a_test->PaintUniformColor(Eigen::Vector3d(1., 0., 0.));
  //    Persistent_cloud_b->PaintUniformColor(Eigen::Vector3d(0., 0., 1.));
  //    // set colors of points founds closer than threshold to green
  //    for (const auto& j : output_indices_a) {
  //      cloud_a_test->colors_[j] = Eigen::Vector3d(0., 1., 0.);
  //      //  Persistent_cloud_b->colors_[pair.second] = Eigen::Vector3d(0., 1., 0.);
  //    }
  //    open3d::visualization::DrawGeometries({cloud_a_test, Persistent_cloud_b});
  // End Timer
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_get_kdtree = (start_kdtree_reset - start);
  std::chrono::duration<double> elapsed_kdtree_reset = (start_loop - start_kdtree_reset);
  std::chrono::duration<double> elapsed_loop = (end - start_loop);
  std::chrono::duration<double> elapsed_total = (end - start);
  //    std::cout << "GetPairsOfPointsCloserThanThresholdO3d: KDTree creation time: " << elapsed_get_kdtree.count() * 1000 << "
  //    milliseconds"
  //              << std::endl;
  //    std::cout << "GetPairsOfPointsCloserThanThresholdO3d: KDTree reset time: " << elapsed_kdtree_reset.count() * 1000 << " milliseconds"
  //              << std::endl;
  //    std::cout << "GetPairsOfPointsCloserThanThresholdO3d: Loop time: " << elapsed_loop.count() * 1000 << " milliseconds" << std::endl;
  std::cout << "GetPairsOfPointsCloserThanThresholdO3d: Hybrid search for overlap pairs total time: " << elapsed_total.count() * 1000
            << " milliseconds" << std::endl;
  // print size of pairs
  std::cout << "GetPairsOfPointsCloserThanThresholdO3d: found " << output_indices_a.size() << " pairs of points closer than " << threshold
            << "m" << std::endl;
}

void GetPairsOfPointsCloserThanThresholdO3dParallel(  // TODO check why openMP does not work!!
    const std::shared_ptr<const open3d::geometry::PointCloud>& cloud_a,
    const std::shared_ptr<const open3d::geometry::PointCloud>& cloud_b_unregistered,
    const Eigen::Transform<double, 3, Eigen::Isometry>& cloud_a__T__cloud_b, double threshold, std::vector<std::pair<int, int>>& pairs) {
  // create a hardcopy of cloud b
  auto cloud_b = std::make_shared<open3d::geometry::PointCloud>(*cloud_b_unregistered);
  // register the pointclouds
  cloud_b->Transform(cloud_a__T__cloud_b.matrix());
  // Build KDTree for cloud_b
  open3d::geometry::KDTreeFlann kdtree_b;
  kdtree_b.SetGeometry(*cloud_b);

  // Precompute squared threshold to avoid sqrt computations
  double threshold_sq = threshold * threshold;

  // Prepare a thread-safe vector to store pairs
  std::vector<std::pair<int, int>> local_pairs;

  std::cout << "Now searching for hybrid pairs in parallel" << std::endl;

#pragma omp parallel
  {
    std::vector<int> indices;
    std::vector<double> distances;
    std::vector<std::pair<int, int>> thread_pairs;

    int max_nn = 1;

#pragma omp for nowait
    for (int i = 0; i < static_cast<int>(cloud_a->points_.size()); ++i) {
      // Clear previous results
      indices.clear();
      distances.clear();

      // Perform hybrid search
      int num_neighbors = kdtree_b.SearchHybrid(cloud_a->points_[i], threshold, max_nn, indices, distances);

      if (num_neighbors > 0) {
        // Store the pair
        thread_pairs.emplace_back(i, indices[0]);
      }
    }

// Merge thread-local pairs into the main vector
#pragma omp critical
    pairs.insert(pairs.end(), thread_pairs.begin(), thread_pairs.end());
  }

  std::cout << "Finished searching for hybrid pairs" << std::endl;
}

void makePointsSubsetIndices(std::vector<int>& indices_vector, int num_indices, int indices_max_range, unsigned int seed) {
  srand(seed);  // NOTE: important to keep this seed, otherwise rand() will generate the same sequence of numbers
  for (int i = 0; i < num_indices; i++) {
    int idx = rand() % indices_max_range;
    // check if the index is already in the vector
    while (std::find(indices_vector.begin(), indices_vector.end(), idx) != indices_vector.end()) {
      idx = rand() % indices_max_range;
    }
    indices_vector.push_back(idx);  // TODO: check for better random number generator
  }
}

// Function to compute absolute orientation between two sets of 3D points, returns src_T_dst
Eigen::Isometry3d computeAbsoluteOrientation(const Eigen::MatrixXd& points_src, const Eigen::MatrixXd& points_dst, bool use_horn) {
  // NOTE: returns the transformation from points_src to points_dst
  // Start Timer
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  // Ensure points_dst and points_src have the same number of points
  if (points_dst.cols() != points_src.cols()) {
    throw std::runtime_error("computeAbsoluteOrientation: point sets must have the same number of points.");
  }

  // 1. Compute the center of mass
  Eigen::Vector3d center_of_mass_dst = points_dst.rowwise().mean();
  Eigen::Vector3d center_of_mass_src = points_src.rowwise().mean();

  // 2. Subtract the center of mass
  Eigen::MatrixXd centered_points_dst = points_dst.colwise() - center_of_mass_dst;
  Eigen::MatrixXd centered_points_src = points_src.colwise() - center_of_mass_src;

  // 3. Compute the cross-covariance matrix W
  Eigen::Matrix3d W = centered_points_dst * centered_points_src.transpose();

  // 4. Compute rotation using SVD or Horn's method
  Eigen::Matrix3d R;
  if (use_horn) {
    R = computeRotationFromCovarianceHORN(W);
  } else {
    R = computeRotationFromCovarianceSVD(W);
  }

  // enforce orthonormality on the rotation matrix
  R = R.colwise().normalized();

  // 5. Compute translation
  Eigen::Vector3d t = center_of_mass_src - R * center_of_mass_dst;

  // Assemble the transformation matrix
  Eigen::Isometry3d src_T_dst = Eigen::Isometry3d::Identity();
  src_T_dst.linear() = R;
  src_T_dst.translation() = t;
  // End Timer
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed = (end - start);
  //        std::cout << "WorkFlowManager: Execution time
  //        (runProcessingCore:preprocessing:computeOdometryRigidClosedForm:computeAbsoluteOrientation:(1 camera): " << elapsed.count() *
  //        1000 << " millliseconds" << std::endl;
  return src_T_dst;
}

// Function to compute rotation from covariance using SVD
Eigen::Matrix3d computeRotationFromCovarianceSVD(const Eigen::Matrix3d& W) {
  // Perform SVD
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  // Compute the rotation matrix
  Eigen::Matrix3d R = V * U.transpose();

  // Ensure a valid rotation matrix (det(R) should be +1, correct for reflection if needed)
  if (R.determinant() < 0) {
    std::cout << "computeRotationFromCovarianceSVD: det(R) < 0, reflection detected! Correcting..." << std::endl;
    V.col(2) *= -1;  // Flip the last col of V
    R = V * U.transpose();
  }
  return R;
}

// Function to compute rotation from covariance using Horn's method
Eigen::Matrix3d computeRotationFromCovarianceHORN(const Eigen::Matrix3d& W) {
  // Create a 4x4 matrix Q based on the cross-covariance matrix W
  Eigen::Matrix4d Q;
  Q << W.trace(), W(1, 2) - W(2, 1), W(2, 0) - W(0, 2), W(0, 1) - W(1, 0), W(1, 2) - W(2, 1), W(0, 0) - W(1, 1) - W(2, 2),
      W(0, 1) + W(1, 0), W(0, 2) + W(2, 0), W(2, 0) - W(0, 2), W(0, 1) + W(1, 0), W(1, 1) - W(0, 0) - W(2, 2), W(1, 2) + W(2, 1),
      W(0, 1) - W(1, 0), W(0, 2) + W(2, 0), W(1, 2) + W(2, 1), W(2, 2) - W(0, 0) - W(1, 1);

  // Calculate the eigenvalues and eigenvectors of Q
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(Q);
  Eigen::Vector4d eigenvalues = solver.eigenvalues();
  Eigen::Matrix4d eigenvectors = solver.eigenvectors();

  // Extract the quaternion with the largest eigenvalue
  Eigen::Vector4d quaternion = eigenvectors.col(3);  // Last column has the largest eigenvalue

  // Convert quaternion to a rotation matrix
  Eigen::Quaterniond q(quaternion(0), quaternion(1), quaternion(2), quaternion(3));
  q.normalize();
  Eigen::Matrix3d R = q.toRotationMatrix();

  return R;
}

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>
#include <stdexcept>
#include <unordered_set>

// function to estimate the absolute orientation between two sets of 3D points using RANSAC returns best src_T_dst
Eigen::Isometry3d ransacAbsoluteOrientation(const Eigen::MatrixXd& points_A, const Eigen::MatrixXd& points_B, int max_iterations,
                                            double inlier_threshold,  // Euclidean distance threshold in meters
                                            int sample_size,          // Number of points to sample for each iteration
                                            bool use_horn) {
  // Ensure points_A and points_B have the same number of points
  if (points_A.cols() != points_B.cols()) {
    throw std::runtime_error("ransacAbsoluteOrientation: point sets must have the same number of points.");
  }

  // Ensure the sample size is feasible
  if (sample_size > points_A.cols()) {
    throw std::runtime_error(
        "ransacAbsoluteOrientation: Sample size cannot be greater than the number of points available. Available points: " +
        std::to_string(points_A.cols()) + ", Requested points: " + std::to_string(sample_size));
  }

  int best_inlier_count = 0;
  Eigen::Isometry3d best_transformation;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, points_A.cols() - 1);

  for (int iter = 0; iter < max_iterations; ++iter) {
    // Randomly sample unique indices
    std::unordered_set<int> indices_set;
    while (indices_set.size() < sample_size) {
      int index = dis(gen);
      indices_set.insert(index);
    }
    std::vector<int> indices(indices_set.begin(), indices_set.end());

    // Create matrices for sampled points
    Eigen::MatrixXd sample_A(3, sample_size);
    Eigen::MatrixXd sample_B(3, sample_size);
    for (int i = 0; i < sample_size; ++i) {
      sample_A.col(i) = points_A.col(indices[i]);
      sample_B.col(i) = points_B.col(indices[i]);
    }

    // Estimate transformation using the sampled points
    Eigen::Isometry3d estimated_transformation = computeAbsoluteOrientation(sample_A, sample_B, use_horn);

    // Count inliers
    int inlier_count = 0;
    for (int i = 0; i < points_A.cols(); ++i) {
      Eigen::Vector3d transformed_point =
          estimated_transformation.linear().transpose() * points_A.col(i) - estimated_transformation.translation();
      Eigen::Vector3d diff = points_B.col(i) - transformed_point;
      if (diff.norm() < inlier_threshold) {
        ++inlier_count;
      } else {
      }
    }

    // Update the best transformation if more inliers are found
    if (inlier_count > best_inlier_count) {
      best_inlier_count = inlier_count;
      best_transformation = estimated_transformation;
    }
  }
  return best_transformation;
}

// endregion
}  // namespace utils
