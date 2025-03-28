
#include "multiviperfrog/core/preprocessing/preprocessing.h"

namespace core {
namespace preprocessing {

std::vector<config::PreprocessingToOptimizationData> temp_output_data_previous_buffer_{config::max_num_cameras};

void preprocessRosDataBuffer(const std::deque<config::RosToPreprocessingData>& input_buffer, const config::ExperimentConfig& exp_config,
                             config::PreprocessingToOptimizationData* output_data, int camera_id) {
  // Start timer
  auto start_preprocess_buffer = std::chrono::high_resolution_clock::now();
  config::PreprocessingToOptimizationData temp_output_data_0;
  config::PreprocessingToOptimizationData temp_output_data_1;
  //            std::cout << "Preprocessing: accumulated buffer of size : " << input_buffer.size() << std::endl;
  // Start timer
  auto start_preprocess_single = std::chrono::high_resolution_clock::now();
  if (WorkFlowManager::isFirstTimeRunning_) {
    // Preprocess the first element of the buffer
    preprocessRosData(input_buffer[0], exp_config, &temp_output_data_0, camera_id);
    // Preprocess the second element of the buffer
    preprocessRosData(input_buffer[1], exp_config, &temp_output_data_1, camera_id);
    temp_output_data_previous_buffer_[camera_id] = temp_output_data_1;
  } else {  // if not first time running, avoid processing the first element again
    temp_output_data_0 = temp_output_data_previous_buffer_[camera_id];
    // Preprocess the second element of the buffer
    preprocessRosData(input_buffer[1], exp_config, &temp_output_data_1, camera_id);
    temp_output_data_previous_buffer_[camera_id] = temp_output_data_1;
  }

  // pointclouds are still only from the first element of the buffer
  //  measured points at t0
  output_data->Cn0__t__Cn0_P0_o3d_ptr[camera_id] = temp_output_data_0.Cn0__t__Cn0_P0_o3d_ptr[camera_id];
  // flowed points at t1
  output_data->Cn1__t__Cn1_P1_o3d_ptr[camera_id] = temp_output_data_0.Cn1__t__Cn1_P1_o3d_ptr[camera_id];
  // measured points at t1, only for ICP benchmark, no data association TODO: find better variable name
  output_data->Cn1__t__Cn1_P1o_o3d_ptr[camera_id] = temp_output_data_1.Cn0__t__Cn0_P0_o3d_ptr[camera_id];
  // optional vectors for tool_mask, confidence_depth, confidence_flow (are relative to the points at t0)
  output_data->tool_mask[camera_id] = temp_output_data_0.tool_mask[camera_id];
  output_data->confidence_depth[camera_id] = temp_output_data_0.confidence_depth[camera_id];
  output_data->confidence_flow[camera_id] = temp_output_data_0.confidence_flow[camera_id];

  // chain the poses of each data to get the relative poses (requires both poses at t0 and t1)
  // odometries
  output_data->Cn0__T__Cn1[camera_id] = temp_output_data_0.world__T__Cn0[camera_id].inverse() * temp_output_data_1.world__T__Cn0[camera_id];

  // copy also the world poses (at time t0 and t1)
  // NOTE: the field Cx1 is used only in the buffer output
  output_data->world__T__Cn0[camera_id] = temp_output_data_0.world__T__Cn0[camera_id];
  output_data->world__T__Cn1[camera_id] = temp_output_data_1.world__T__Cn0[camera_id];

  // compute absolute flow from transformation and subtraction
  std::vector<std::shared_ptr<open3d::geometry::PointCloud>> Cn0__t__Cn1_P1_o3d_ptr{config::max_num_cameras};
  Cn0__t__Cn1_P1_o3d_ptr[camera_id] =  // here variable name abused, true only after transformation
      std::make_shared<open3d::geometry::PointCloud>(*output_data->Cn1__t__Cn1_P1_o3d_ptr[camera_id]);

  // transform Ca0__t__Ca1_P1_o3d_ptr with Ca0__T__Ca1 using o3d
  Cn0__t__Cn1_P1_o3d_ptr[camera_id]->Transform(output_data->Cn0__T__Cn1[camera_id].matrix());

  // Ensure point clouds are not nullptr
  if (!Cn0__t__Cn1_P1_o3d_ptr[camera_id] || !output_data->Cn0__t__Cn0_P0_o3d_ptr[camera_id]) {
    throw std::runtime_error("Preproccessing: One or more PointCloud pointers are null.");
  }

  // Initialize normals with zero vectors
  size_t point_count = output_data->Cn0__t__Cn0_P0_o3d_ptr[camera_id]->points_.size();
  output_data->Cn0__t__Cn0_P0_o3d_ptr[camera_id]->normals_.resize(point_count, Eigen::Vector3d::Zero());

  // Start timer
  auto start_abs_GTflow_calculation = std::chrono::high_resolution_clock::now();
  // subtract the point coordinates of the flowed transformed points and the original points to get the absolute flow
  // TODO: parallelize this for loop with openmp
  for (size_t i = 0; i < Cn0__t__Cn1_P1_o3d_ptr[camera_id]->points_.size(); ++i) {
    // Subtract the original point from the transformed point (gives the absolute scene flow in the camera frame at t0)
    output_data->Cn0__t__Cn0_P0_o3d_ptr[camera_id]->normals_[i] =
        Cn0__t__Cn1_P1_o3d_ptr[camera_id]->points_[i] - output_data->Cn0__t__Cn0_P0_o3d_ptr[camera_id]->points_[i];
  }
  // End timer
  auto end_abs_GTflow_calculation = std::chrono::high_resolution_clock::now();
  // Calculate elapsed time
  std::chrono::duration<double> elapsed_preprocess_buffer = (end_abs_GTflow_calculation - start_preprocess_buffer);
  std::cout << "WorkFlowManager: Execution time (runProcessingCore:preprocessing:preprocessRosDataBuffer: "
               "total): "
            << elapsed_preprocess_buffer.count() * 1000 << " milliseconds" << std::endl;
}

void preprocessRosData(const config::RosToPreprocessingData& input_data, const config::ExperimentConfig& exp_config,
                       config::PreprocessingToOptimizationData* output_data, int camera_id) {
  // TODO: add check if the point cloud pointer is empty
  // points at t0
  std::shared_ptr<open3d::geometry::PointCloud> Camera_points_o3d_ptr = std::make_shared<open3d::geometry::PointCloud>();

  // convert ros messages to open3d pointclouds
  // optional vectors for tool_mask, confidence_depth, confidence_flow
  std::optional<std::vector<uint8_t>> tool_mask;
  std::optional<std::vector<float>> confidence_depth;
  std::optional<std::vector<float>> confidence_flow;

  // pointclouds a t0
  ::utils::PointCLoud2ToOpen3dCustom(input_data.Camera_points_msg_ptr[camera_id], *Camera_points_o3d_ptr,
                                     tool_mask, confidence_depth, confidence_flow);

//// print size of optional vectors, if any
//  if (tool_mask) {std::cout << "Preprocessing:preprocessRosData: Camera_points_o3d_ptr tool_mask size: " << tool_mask->size() << std::endl;}
//    else {std::cout << "Preprocessing:preprocessRosData: Camera_points_o3d_ptr tool_mask not available." << std::endl;}
//  if (confidence_depth) {std::cout << "Preprocessing:preprocessRosData: Camera_points_o3d_ptr confidence_depth size: " << confidence_depth->size() << std::endl;}
//    else {std::cout << "Preprocessing:preprocessRosData: Camera_points_o3d_ptr confidence_depth not available." << std::endl;}
//  if (confidence_flow) {std::cout << "Preprocessing:preprocessRosData: Camera_points_o3d_ptr confidence_flow size: " << confidence_flow->size() << std::endl;}
//    else {std::cout << "Preprocessing:preprocessRosData: Camera_points_o3d_ptr confidence_flow not available." << std::endl;}

  // downsample the point clouds
  if (exp_config.downsample_before_preprocessing) {
    throw std::runtime_error(
        "Preprocessing:preprocessRosData: Downsample before preprocessing temporary verboten."
        "TODO: create odometry_only global flag, since downsampling here conflicts "
        "with the KNN search for the global optimization.");
    if (exp_config.downsample_strategy != "uniform") {
      throw std::runtime_error("Preprocessing:preprocessRosData: Downsample strategy not supported, select uniform.");
    }
    ::utils::downsampleO3dPointCloud(Camera_points_o3d_ptr, exp_config);
  }
  // add points at t0 with GT relative flow to get equivalent of biased_flowed_points_camX_in_camX_next_reference_frame
  // TODO: decide if needed to keep o3d, or if it is better to convert to Eigen directly from the ros messages
  // TODO: parallelize all the for loops with openmp

  // flowed points in ref frame of cam at t1 (cloned from the points at t0)
  std::shared_ptr<open3d::geometry::PointCloud> Camera_points_flowed_o3d_ptr =
      std::make_shared<open3d::geometry::PointCloud>(*Camera_points_o3d_ptr);

  // add relative SF to the points at t0 to get the flowed points at t1
  for (size_t i = 0; i < Camera_points_o3d_ptr->points_.size(); ++i) {
    Camera_points_flowed_o3d_ptr->points_[i] += Camera_points_flowed_o3d_ptr->normals_[i];
  }
  // convert poses to Eigen::Isometry3d
  tf2::Stamped<Eigen::Isometry3d> Camera_pose = ::utils::PoseStampedMsgToEigenIsometryStamped(input_data.Camera_pose_msg_ptr[camera_id]);
  // add the pointclouds and poses to the output data
  output_data->Cn0__t__Cn0_P0_o3d_ptr[camera_id] = Camera_points_o3d_ptr;
  output_data->Cn1__t__Cn1_P1_o3d_ptr[camera_id] = Camera_points_flowed_o3d_ptr;
  output_data->world__T__Cn0[camera_id] = Camera_pose;
  // Assign optional vectors to output data
  if (tool_mask.has_value()) {output_data->tool_mask[camera_id] = std::move(tool_mask);}
  if (confidence_depth.has_value()) {output_data->confidence_depth[camera_id] = std::move(confidence_depth);}
  if (confidence_flow.has_value()) {output_data->confidence_flow[camera_id] = std::move(confidence_flow);}
}

void computeCam2Cam(config::PreprocessingToOptimizationData* data, const config::ExperimentConfig& exp_config, int cam_id_self,
                    int cam_id_other, bool estimate_from_points, bool compute_from_poses) {
  // Compute the transformation between the cameras
  if (estimate_from_points) {
    // TODO implement the estimation of the transformation between the cameras from the points (ICP??)
    // that would require editing the structure of the processing so to have the data available not in ros
    // format (currently done by preprocessRosData) but before the preprocessRosDataBuffer is called,
    // otherwise one need to wait for 2 frames, while this computation can already be done in parallel
    // when the first frame arrives
    throw std::runtime_error(
        "Preprocessing:computeCam2Cam: Estimation of the transformation between"
        "cameras from the points not implemented yet.");
  } else if (compute_from_poses) {  // TODO add way to know if one or both frames already arrived (not needed now)
    // Compute the transformation between the cameras from the poses
    data->Cself0__T__Cother0[cam_id_self] = data->world__T__Cn0[cam_id_self].inverse() * data->world__T__Cn0[cam_id_other];
    data->Cself1__T__Cother1[cam_id_self] = data->world__T__Cn1[cam_id_self].inverse() * data->world__T__Cn1[cam_id_other];
  } else {
    throw std::runtime_error(
        "Preprocessing:computeCam2Cam: No method to compute the transformation between"
        "cameras selected.");
  }
}

void computeOdometryRigidClosedForm(const config::PreprocessingToOptimizationData& data, const config::ExperimentConfig& exp_config,
                                    config::OptimizationToPostprocessingData* output_data, int camera_id, bool use_horn, bool use_ransac) {
  auto start_compute_absolute_orientation = std::chrono::high_resolution_clock::now();
  Eigen::Matrix3Xd Cn0__t__Cn0_P0_eigen = ::utils::Open3dToEigenMatrix(data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id]);
  Eigen::Matrix3Xd Cn1__t__Cn1_P1_eigen = ::utils::Open3dToEigenMatrix(data.Cn1__t__Cn1_P1_o3d_downsampled_ptr[camera_id]);
  Eigen::Isometry3d Cn0__T__Cn1_SVD = Eigen::Isometry3d::Identity();
  if (use_ransac) {
    // TODO make ransac parameters configurable
    unsigned int max_iterations = 10;
    double inlier_threshold = 0.0001;  // TODO make so that is sam e as Huber loss threshold
                                       //                int sample_size = floor(Ca0__t__Ca0_P0_eigen.cols() / max_iterations);
    int sample_size = 3;
    // TODO POOR time performance!! get rid of all the for loops or use external library
    Cn0__T__Cn1_SVD = ::utils::ransacAbsoluteOrientation(Cn0__t__Cn0_P0_eigen, Cn1__t__Cn1_P1_eigen, max_iterations, inlier_threshold,
                                                         sample_size, use_horn);
  } else {
    Cn0__T__Cn1_SVD = ::utils::computeAbsoluteOrientation(Cn0__t__Cn0_P0_eigen, Cn1__t__Cn1_P1_eigen, use_horn);
  }
  // TODO implement the same for the other cameras but also make a smarter handling of the stamping, here hardcoded!
  auto end_compute_absolute_orientation = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_compute_absolute_orientation =
      (end_compute_absolute_orientation - start_compute_absolute_orientation);
  std::cout << "WorkFlowManager: Execution time "
               "(runProcessingCore:preprocessing:computeOdometryRigidClosedForm: Total): "
            << elapsed_compute_absolute_orientation.count() * 1000 << " milliseconds" << std::endl;
  output_data->Ccurrent__T__Cnext_svd[camera_id] = tf2::Stamped<Eigen::Isometry3d>(Cn0__T__Cn1_SVD, data.world__T__Cn1[camera_id].stamp_,
                                                                                   data.world__T__Cn1[camera_id].frame_id_ + "_SVD");
}

void computeOdometryRigidIterative(const config::PreprocessingToOptimizationData& data, const config::ExperimentConfig& exp_config,
                                   config::OptimizationToPostprocessingData* output_data, int camera_id) {
  Eigen::Isometry3d Cn0__T__Cn1_ICP = Eigen::Isometry3d::Identity();
  utils::ComputeICPwithO3D(data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id],
                           data.Cn1__t__Cn1_P1o_o3d_downsampled_ptr[camera_id],  // NOTE: these are the points at t1, not the flowed ones
                           &Cn0__T__Cn1_ICP, exp_config);
  output_data->Ccurrent__T__Cnext_icp[camera_id] = tf2::Stamped<Eigen::Isometry3d>(Cn0__T__Cn1_ICP, data.world__T__Cn1[camera_id].stamp_,
                                                                                   data.world__T__Cn1[camera_id].frame_id_ + "_ICP");
}

void downsamplePointClouds(config::PreprocessingToOptimizationData& data, const config::ExperimentConfig& exp_config, int camera_id) {
  std::vector<size_t> downsample_indices_P0;
  std::vector<size_t> downsample_indices_P1;

  // Custom downsampling that saves indexes of the selected points
  data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id] =
      utils::downsampleCustomOpen3DPointCloud(data.Cn0__t__Cn0_P0_o3d_ptr[camera_id], downsample_indices_P0, exp_config);
  data.Cn1__t__Cn1_P1_o3d_downsampled_ptr[camera_id] =
      utils::downsampleCustomOpen3DPointCloud(data.Cn1__t__Cn1_P1_o3d_ptr[camera_id], downsample_indices_P1, exp_config);
  data.Cn1__t__Cn1_P1o_o3d_downsampled_ptr[camera_id] =
      utils::downsampleCustomOpen3DPointCloud(data.Cn1__t__Cn1_P1o_o3d_ptr[camera_id], downsample_indices_P1, exp_config);

  if (downsample_indices_P0 == downsample_indices_P0) {
    data.downsample_indices[camera_id] = downsample_indices_P0;
  } else {
    throw std::runtime_error("Preprocessing:downsamplePointClouds: Downsample indices not equal btw P0 and P1.");
  }
  //            // deprecated function for this stage, now using custom downsampling (see above)
  //            // Downsample the point clouds
  //            utils::downsampleO3dPointCloud(data.Cn0__t__Cn0_P0_o3d_ptr[camera_id], exp_config);
  //            utils::downsampleO3dPointCloud(data.Cn1__t__Cn1_P1_o3d_ptr[camera_id], exp_config);
}

void indexSamplingOverlapRegion(core::WorkFlowManager* manager, config::PreprocessingToOptimizationData& data,
                                const config::ExperimentConfig& exp_config, int camera_id_self, int camera_id_other) {
  // TODO: change logic when >2 cameras. for works only if camera_id_self = 0 and camera_id_other = 1
  int permutation;
  if (camera_id_self == 0 & camera_id_other == 1) {
    permutation = 0;
  } else {
    throw std::runtime_error("Preprocessing:downsampleOverlapRegion: Camera ids not supported!!.");
  }

  std::vector<size_t> indices_downsampled_self;
  std::vector<size_t> indices_overlap_downsampled_self;
  // declare an identity transform for the KNN search named_ identity
  Eigen::Isometry3d identity_transform = Eigen::Isometry3d::Identity();

  utils::GetPairsOfPointsCloserThanThresholdO3d(manager, data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_self],
                                                exp_config.downsampledcloud_to_overlap_threshold, indices_downsampled_self,
                                                indices_overlap_downsampled_self, true, false, false,
                                                data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self], &identity_transform);

  //     //TODO: downsample somehow the overlap_downsampled cloud further to the size of the indices_overlap_downsampled_other so to
  //     maintain
  //     // constant downsample each data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr by skipping every other index using ->SelectByIndex()
  //    std::vector<size_t> every_other_index;
  //    for (size_t i = 0; i < data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self]->points_.size(); i += 2) {
  //      every_other_index.push_back(i);
  //    }
  //    data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self] =
  //        data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self]->SelectByIndex(every_other_index);
  //    data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other] =
  //        data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other]->SelectByIndex(every_other_index);
  //
  //    // print size of data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self] and
  //    // data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other]
  //    std::cout << "Preprocessing:downsampleOverlapRegion: size of data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self]: "
  //              << data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self]->points_.size() << std::endl;
  //    std::cout << "Preprocessing:downsampleOverlapRegion: size of data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other]: "
  //              << data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other]->points_.size() << std::endl;

  int size_of_non_overlap_points_in_self =
      data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_self]->points_.size() - indices_downsampled_self.size();
  if (size_of_non_overlap_points_in_self < 0) {
    throw std::runtime_error("Preprocessing:downsampleOverlapRegion: size_of_non_overlap_points_in_self is negative.");
  }

  // remove all all the overlapping points from the downsampled cloud of the other camera, and add all the points from the overlap
  auto& original_pcd_t0 = data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_self];
  auto new_pcd_t0 = original_pcd_t0->SelectByIndex(indices_downsampled_self, /*invert=*/true);
  auto& overlap_pcd_t0 = data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self];
  new_pcd_t0->points_.insert(new_pcd_t0->points_.end(), overlap_pcd_t0->points_.begin(), overlap_pcd_t0->points_.end());
  new_pcd_t0->colors_.insert(new_pcd_t0->colors_.end(), overlap_pcd_t0->colors_.begin(), overlap_pcd_t0->colors_.end());
  new_pcd_t0->normals_.insert(new_pcd_t0->normals_.end(), overlap_pcd_t0->normals_.begin(), overlap_pcd_t0->normals_.end());
  *original_pcd_t0 = *new_pcd_t0;

  auto& original_pcd_t1 = data.Cn1__t__Cn1_P1_o3d_downsampled_ptr[camera_id_self];
  auto new_pcd_t1 = original_pcd_t1->SelectByIndex(indices_downsampled_self, /*invert=*/true);
  auto& overlap_pcd_t1 = data.Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_self];
  new_pcd_t1->points_.insert(new_pcd_t1->points_.end(), overlap_pcd_t1->points_.begin(), overlap_pcd_t1->points_.end());
  new_pcd_t1->colors_.insert(new_pcd_t1->colors_.end(), overlap_pcd_t1->colors_.begin(), overlap_pcd_t1->colors_.end());
  new_pcd_t1->normals_.insert(new_pcd_t1->normals_.end(), overlap_pcd_t1->normals_.begin(), overlap_pcd_t1->normals_.end());
  *original_pcd_t1 = *new_pcd_t1;

  // create a std::vector of indices initialized in increasing value from 0 to size of overlap_pcd_t0
  std::vector<size_t> indices_overlap_pcd_t0(overlap_pcd_t0->points_.size());
  std::iota(indices_overlap_pcd_t0.begin(), indices_overlap_pcd_t0.end(), 0);
  // add the value of size_of_non_overlap_points_in_self to all the values in indices_overlap_pcd_t0
  std::transform(indices_overlap_pcd_t0.begin(), indices_overlap_pcd_t0.end(), indices_overlap_pcd_t0.begin(),
                 [size_of_non_overlap_points_in_self](size_t i) { return i + size_of_non_overlap_points_in_self; });
  data.downsample_indices_overlap_self[permutation] =
      indices_overlap_pcd_t0;  // indexing correspond with an offset for SELF between downsampled and overlap
  data.downsample_indices_overlap_other[permutation] =
      data.overlap_indices_other[permutation];  // indexing correspond perfectly for OTHER between downsampled and overlap

  //  // color of red only the points of data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_self] given the indexes of
  //  data.downsample_indices_overlap_self[permutation]
  //        for (size_t i = 0; i < data.downsample_indices_overlap_self[permutation].size(); i++) {
  //          data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_self]->colors_[data.downsample_indices_overlap_self[permutation][i]] =
  //          Eigen::Vector3d(1, 0, 0);
  //        }
  //  // color of blue only the points of data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_other] given the indexes of
  //  data.downsample_indices_overlap_other[permutation]
  //        for (size_t i = 0; i < data.downsample_indices_overlap_other[permutation].size(); i++) {
  //          data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_other]->colors_[data.downsample_indices_overlap_other[permutation][i]] =
  //          Eigen::Vector3d(0, 0, 1);
  //        }

  //  // paint t0 in red and t1 in green using paint_uniform_color
  //  data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_other]->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
  //  data.Cn1__t__Cn1_P1_o3d_downsampled_ptr[camera_id_other]->PaintUniformColor(Eigen::Vector3d(0, 1, 0));
  //
  // o3d plot of the downsampled cloud of the other camera in the same plot using utils::PlotCloudsCorrespondences
  //  utils::PlotCloudsCorrespondences(data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_self],
  //                                   data.Cn1__t__Cn1_P1_o3d_downsampled_ptr[camera_id_self]);
  //  utils::PlotCloudsCorrespondences(data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_other],
  //                                   data.Cn1__t__Cn1_P1_o3d_downsampled_ptr[camera_id_other]);

  // TODO: NOTE not done for data.Cn1__t__Cn1_P1o_o3d_downsampled_ptr[camera_id_other] since it is not used in the optimization (ICP only)

  //  // o3d plot of the downsampled cloud of the other camera
  //  open3d::visualization::DrawGeometries({data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self],
  //                                         data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_self]},
  //                                        "Downsampled cloud of the self camera");
  //      open3d::visualization::DrawGeometries({data.Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_self],data.Cn1__t__Cn1_P1_o3d_downsampled_ptr[camera_id_self]},
  //                                            "Downsampled cloud of the other camera");
  //  std::cout << "Size of data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_other]"
  //            << data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_other]->points_.size() << std::endl;
  //  open3d::visualization::DrawGeometries({data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_self]}, "Downsampled cloud of the self
  //  camera"); std::cout << "Size of data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_self]"
  //            << data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_self]->points_.size() << std::endl;

  //  throw std::runtime_error("Preprocessing:downsampleOverlapRegion: we stop here for now.");

  //
  ////  // set time computation for this function
  ////  auto start_indexsampling_overlap_region = std::chrono::high_resolution_clock::now();
  ////  // Create voxel grids
  ////  double voxel_size = 0.001;  // Adjust voxel size as needed
  ////  auto voxel_grid_a = open3d::geometry::VoxelGrid::CreateFromPointCloud(*data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_self],
  /// voxel_size); /  auto voxel_grid_b =
  /// open3d::geometry::VoxelGrid::CreateFromPointCloud(*data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_other], voxel_size); /  //
  /// Create a hash set of voxel indices from voxel_grid_a /  std::unordered_set<Eigen::Vector3i,
  /// open3d::geometry::VoxelGrid::VoxelHasher> voxel_indices_a; /  for (const auto &voxel : voxel_grid_a->GetVoxels()) { /
  /// voxel_indices_a.insert(voxel.grid_index_); /  } /  // set time computation for this function /  auto
  /// end_indexsampling_overlap_region = std::chrono::high_resolution_clock::now(); / std::chrono::duration<double>
  /// elapsed_indexsampling_overlap_region = /      (end_indexsampling_overlap_region - start_indexsampling_overlap_region); /  std::cout
  /// << "WorkFlowManager: Execution time " /
  ///"(runProcessingCore:preprocessing:indexSamplingOverlapRegion: Total): " /            << elapsed_indexsampling_overlap_region.count()
  ///*
  /// 1000 << " milliseconds" << std::endl; /  throw std::runtime_error("Preprocessing:downsampleOverlapRegion: we stop here for now.");
  //
  //  std::vector<std::vector<std::pair<int, int>>> pairs_indexes_downsampled_in_overlap{2};
  //
  //  // search for KNN on the overlap for each point in each downsampled cloud (reusing the KDTree previously computed for full cloud_b)
  //  // KDTree previously computed for full cloud_b was already transformed in cloud_a frame
  //  Eigen::Transform<double, 3, Eigen::Isometry> downsampled__T__overlap;
  //  downsampled__T__overlap.setIdentity();
  //  utils::GetPairsOfPointsCloserThanThresholdO3d(manager, data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_self],
  //                                                exp_config.downsampledcloud_to_overlap_threshold,
  //                                                pairs_indexes_downsampled_in_overlap[camera_id_self],
  //                                                false);  // reusing existing KDTree
  //  //  define a new temporary pointcloud
  //  auto Cn0__t__Cn0_P0_o3d_downsampled_other_ptr_transformed =
  //      std::make_shared<open3d::geometry::PointCloud>(*data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_other]);
  //  // transform the downsampled cloud in the frame of cam a to match the KDtree frame (see comment above)
  //  Cn0__t__Cn0_P0_o3d_downsampled_other_ptr_transformed->Transform(data.Cself0__T__Cother0[permutation].matrix());
  //  utils::GetPairsOfPointsCloserThanThresholdO3d(manager, Cn0__t__Cn0_P0_o3d_downsampled_other_ptr_transformed,
  //                                                exp_config.downsampledcloud_to_overlap_threshold,
  //                                                pairs_indexes_downsampled_in_overlap[camera_id_other],
  //                                                false);  // reusing existing KDTree
  //  //  // print first 10pairs of pairs_indexes_downsampled_in_overlap[0] and pairs_indexes_downsampled_in_overlap[1]
  //  //  std::cout << "Preprocessing:downsampleOverlapRegion: First 10 pairs of indexes in overlap for camera 0: \n";
  //  //  for (int i = 0; i < 10; i++) {
  //  //    std::cout << pairs_indexes_downsampled_in_overlap[0][i].first << ", " << pairs_indexes_downsampled_in_overlap[0][i].second <<
  //  "\n";
  //  //  }
  //  //  std::cout << std::endl;
  //  //
  //  //  std::cout << "Preprocessing:downsampleOverlapRegion: First 10 pairs of indexes in overlap for camera 1: \n";
  //  //  for (int i = 0; i < 10; i++) {
  //  //    std::cout << pairs_indexes_downsampled_in_overlap[1][i].first << ", " << pairs_indexes_downsampled_in_overlap[1][i].second <<
  //  "\n";
  //  //  }
  //  //  std::cout << std::endl;
  //  // NOTE, pair.second will now be indexes of the original cloud, so we transform pair.second in overlap indexes
  //  auto pairs_indexes_downsampled_in_overlap_self_test =
  //      utils::RecoverNewPcdIndex(pairs_indexes_downsampled_in_overlap[camera_id_self], data.overlap_pairs_indices[permutation]);
  //  auto pairs_indexes_downsampled_in_overlap_other_test =
  //      utils::RecoverNewPcdIndex(pairs_indexes_downsampled_in_overlap[camera_id_other], data.overlap_pairs_indices[permutation]);
  //
  //  //  // print size of pairs_indexes_downsampled_in_overlap_self_test and pairs_indexes_downsampled_in_overlap_other_test
  //  //  std::cout << "Preprocessing:downsampleOverlapRegion: size of pairs_indexes_downsampled_in_overlap_self_test: "
  //  //            << pairs_indexes_downsampled_in_overlap_self_test.size() << std::endl;
  //  //  std::cout << "Preprocessing:downsampleOverlapRegion: size of pairs_indexes_downsampled_in_overlap_other_test: "
  //  //            << pairs_indexes_downsampled_in_overlap_other_test.size() << std::endl;
  //  //
  //  //  auto cloud_a_test = std::make_shared<open3d::geometry::PointCloud>(*data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_self]);
  //  //  auto cloud_a_overlap_test =
  //  std::make_shared<open3d::geometry::PointCloud>(*data.Cn0__t__Cn0_P0_o3d_overlap_ptr[camera_id_self]);
  //  //  auto cloud_b_test = std::make_shared<open3d::geometry::PointCloud>(*Cn0__t__Cn0_P0_o3d_downsampled_other_ptr_transformed);
  //  //  auto cloud_b_overlap_test =
  //  std::make_shared<open3d::geometry::PointCloud>(*data.Cn0__t__Cn0_P0_o3d_overlap_ptr[camera_id_other]);
  //  //  auto cloud_b_full_test = std::make_shared<open3d::geometry::PointCloud>(*data.Cn0__t__Cn0_P0_o3d_ptr[camera_id_other]);
  //  //  cloud_b_overlap_test->Transform(data.Cself0__T__Cother0[permutation].matrix());
  //  //  cloud_b_full_test->Transform(data.Cself0__T__Cother0[permutation].matrix());
  //  //  // plot with cloud_a in neutralcolors
  //  //  cloud_a_test->PaintUniformColor(Eigen::Vector3d(1., 0., 1.));
  //  //  cloud_a_overlap_test->PaintUniformColor(Eigen::Vector3d(0.4, 0., 0.4));
  //  //  cloud_b_test->PaintUniformColor(Eigen::Vector3d(0., 1., 1.));
  //  //  cloud_b_overlap_test->PaintUniformColor(Eigen::Vector3d(0., 0.4, 0.4));
  //  //
  //  //  // pprint size of overlap clouds
  //  //  std::cout << "Preprocessing:downsampleOverlapRegion: size of cloud_a_overlap_test: " << cloud_a_overlap_test->points_.size() <<
  //  //  std::endl; std::cout << "Preprocessing:downsampleOverlapRegion: size of cloud_b_overlap_test: " <<
  //  //  cloud_b_overlap_test->points_.size() << std::endl;
  //  //
  //  //  for (const auto& pair : pairs_indexes_downsampled_in_overlap_self_test) {
  //  //    cloud_a_overlap_test->colors_[pair.second] = Eigen::Vector3d(1., 0., 0.);
  //  //  }
  //  //  for (const auto& pair : pairs_indexes_downsampled_in_overlap_other_test) {
  //  //    cloud_b_overlap_test->colors_[pair.second] = Eigen::Vector3d(0., 1., 0.);
  //  //  }
  //  //
  //  //  // iterate through pairs_indexes_downsampled_in_overlap[camera_id_self],
  //  //  open3d::visualization::DrawGeometries({cloud_a_test, cloud_a_overlap_test}, "Pointclouds in overlap");
  //  //  open3d::visualization::DrawGeometries({cloud_b_test, cloud_b_overlap_test}, "Pointclouds in overlap");
  //
  //  // index pointclouds with the overlap indexes and plot with O3d with
  //  //  // print first 10pairs of pairs_indexes_downsampled_in_overlap_self_test and pairs_indexes_downsampled_in_overlap_other_test
  //  //  std::cout << "Preprocessing:downsampleOverlapRegion: First 10 pairs of indexes in overlap for camera 0: \n";
  //  //  for (int i = 0; i < 10; i++) {
  //  //    std::cout << pairs_indexes_downsampled_in_overlap_self_test[i].first << ", " <<
  //  //    pairs_indexes_downsampled_in_overlap_self_test[i].second
  //  //              << "\n";
  //  //  }
  //  //  std::cout << std::endl;
  //  //
  //  //  std::cout << "Preprocessing:downsampleOverlapRegion: First 10 pairs of indexes in overlap for camera 1: \n";
  //  //  for (int i = 0; i < 10; i++) {
  //  //    std::cout << pairs_indexes_downsampled_in_overlap_other_test[i].first << ", "
  //  //              << pairs_indexes_downsampled_in_overlap_other_test[i].second << "\n";
  //  //  }
  //  //  std::cout << std::endl;
  //
  //  // debug somehow
  //  pairs_indexes_downsampled_in_overlap[camera_id_self] = pairs_indexes_downsampled_in_overlap_self_test;
  //  pairs_indexes_downsampled_in_overlap[camera_id_other] = pairs_indexes_downsampled_in_overlap_other_test;
  //
  //  // create a  vector to store both indexes lists, nota that we want to select the points in the overlap that
  //  // are present in each of the downsampled clouds so we need a list with both of their indexes,
  //
  //  std::vector<std::tuple<std::optional<int>, int, std::optional<int>>> pairs_indexes_downsampled_in_overlap_cameras_joined =
  //      utils::mergeVectorsToTriplet(pairs_indexes_downsampled_in_overlap[camera_id_self],
  //                                   pairs_indexes_downsampled_in_overlap[camera_id_other]);
  //
  //  // TODO make all this crap elagantly initialized to avoid visual clutter
  //  // make sure that the point clouds are initialized and not empty
  //  if (!data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self]) {
  //    data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self] = std::make_shared<open3d::geometry::PointCloud>();
  //    data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self]->points_.reserve(
  //        pairs_indexes_downsampled_in_overlap_cameras_joined.size());
  //  }
  //  if (!data.Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_self]) {
  //    data.Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_self] = std::make_shared<open3d::geometry::PointCloud>();
  //    data.Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_self]->points_.reserve(
  //        pairs_indexes_downsampled_in_overlap_cameras_joined.size());
  //  }
  //  if (!data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other]) {
  //    data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other] = std::make_shared<open3d::geometry::PointCloud>();
  //    data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other]->points_.reserve(
  //        pairs_indexes_downsampled_in_overlap_cameras_joined.size());
  //  }
  //  if (!data.Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_other]) {
  //    data.Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_other] = std::make_shared<open3d::geometry::PointCloud>();
  //    data.Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_other]->points_.reserve(
  //        pairs_indexes_downsampled_in_overlap_cameras_joined.size());
  //  }
  //
  //  for (const auto& triplet : pairs_indexes_downsampled_in_overlap_cameras_joined) {
  //    int i = std::get<1>(triplet);  // Access the `.second` element of the triplet which contains the index values of the overlap
  //    utils::AssignO3dPcdFieldsGivenIndex(data.Cn0__t__Cn0_P0_o3d_overlap_ptr[camera_id_self],
  //                                        data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self], i);
  //    utils::AssignO3dPcdFieldsGivenIndex(data.Cn1__t__Cn1_P1_o3d_overlap_ptr[camera_id_self],
  //                                        data.Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_self], i);
  //    utils::AssignO3dPcdFieldsGivenIndex(data.Cn0__t__Cn0_P0_o3d_overlap_ptr[camera_id_other],
  //                                        data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other], i);
  //    utils::AssignO3dPcdFieldsGivenIndex(data.Cn1__t__Cn1_P1_o3d_overlap_ptr[camera_id_other],
  //                                        data.Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_other], i);
  //  }
  //  // save indexes to the output data
  //  data.triplets_indexes_downsampled_in_overlap = pairs_indexes_downsampled_in_overlap_cameras_joined;
}

void downsampleOverlapRegion(config::PreprocessingToOptimizationData& data, const config::ExperimentConfig& exp_config, int camera_id_self,
                             int camera_id_other) {
  // TODO: change logic when >2 cameras. for works only if camera_id_self = 0 and camera_id_other = 1
  int permutation;
  if (camera_id_self == 0 & camera_id_other == 1) {
    permutation = 0;
  } else {
    throw std::runtime_error("Preprocessing:downsampleOverlapRegion: Camera ids not supported!!.");
  }

  std::vector<std::vector<size_t>> downsample_indices_P0{2};  // HARDCODED for overlap of only 2 cameras, no overlap of 3 cameras
  std::vector<std::vector<size_t>> downsample_indices_P1{2};  // HARDCODED

  data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self] = utils::downsampleCustomOpen3DPointCloud(
      data.Cn0__t__Cn0_P0_o3d_overlap_ptr[camera_id_self], downsample_indices_P0[camera_id_self], exp_config, true);
  data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other] = utils::downsampleCustomOpen3DPointCloud(
      data.Cn0__t__Cn0_P0_o3d_overlap_ptr[camera_id_other], downsample_indices_P0[camera_id_other], exp_config, true);
  data.Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_self] = utils::downsampleCustomOpen3DPointCloud(
      data.Cn1__t__Cn1_P1_o3d_overlap_ptr[camera_id_self], downsample_indices_P1[camera_id_self], exp_config, true);
  data.Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_other] = utils::downsampleCustomOpen3DPointCloud(
      data.Cn1__t__Cn1_P1_o3d_overlap_ptr[camera_id_other], downsample_indices_P1[camera_id_other], exp_config, true);
  if ((downsample_indices_P0 == downsample_indices_P1) &&
      (downsample_indices_P0[camera_id_self] == downsample_indices_P0[camera_id_other])) {
    data.downsample_indices_overlap[permutation] = downsample_indices_P0[camera_id_self];
  } else {
    throw std::runtime_error("Preprocessing:downsampleOverlapRegion: Downsample indices not equal btw P0 and P1 or self and other.");
  }
}

void getOverlapIndices(core::WorkFlowManager* manager, config::PreprocessingToOptimizationData& data,
                       const config::ExperimentConfig& exp_config, int camera_id_self, int camera_id_other) {
  // TODO: change logic when >2 cameras. for works only if camera_id_self = 0 and camera_id_other = 1
  int permutation;
  if (camera_id_self == 0 & camera_id_other == 1) {
    permutation = 0;
  } else {
    throw std::runtime_error("Preprocessing:getOverlapIndices: Camera ids not supported!!.");
  }
  //  Eigen::Transform<double, 3, 1> Cother0__T__Cself0 = data.Cself0__T__Cother0[permutation].inverse();
  utils::GetPairsOfPointsCloserThanThresholdO3d(manager, data.Cn0__t__Cn0_P0_o3d_ptr[camera_id_self], exp_config.points_overlap_threshold,
                                                data.overlap_indices_self[permutation], data.overlap_indices_other[permutation],
                                                true,   // create new KDTree
                                                false,  // minimize duplicated matches
                                                true,   // avoids completely duplicated points
                                                data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_other],
                                                &data.Cself0__T__Cother0[permutation]);
}

void extractOverlapPoints(config::PreprocessingToOptimizationData& data, config::ExperimentConfig& exp_config, int camera_id_self,
                          int camera_id_other) {
  // TODO: change logic when >2 cameras. for works only if camera_id_self = 0 and camera_id_other = 1
  int permutation;
  if (camera_id_self == 0 & camera_id_other == 1) {
    permutation = 0;
  } else {
    throw std::runtime_error("Preprocessing:extractOverlapPoints: Camera ids not supported!!.");
  }
  // print size of data.overlap_indices_self[permutation] and data.overlap_indices_other[permutation]
  std::cout << "Preprocessing:extractOverlapPoints: size of data.overlap_indices_self[permutation]: "
            << data.overlap_indices_self[permutation].size() << std::endl;
  std::cout << "Preprocessing:extractOverlapPoints: size of data.overlap_indices_other[permutation]: "
            << data.overlap_indices_other[permutation].size() << std::endl;

  // points at t0
  data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self] =
      utils::CreateSelectedPointCloud(data.Cn0__t__Cn0_P0_o3d_ptr[camera_id_self], data.overlap_indices_self[permutation]);
  data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other] =
      utils::CreateSelectedPointCloud(data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id_other], data.overlap_indices_other[permutation]);

  // flowed points at t1
  data.Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_self] =
      utils::CreateSelectedPointCloud(data.Cn1__t__Cn1_P1_o3d_ptr[camera_id_self], data.overlap_indices_self[permutation]);
  data.Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_other] =
      utils::CreateSelectedPointCloud(data.Cn1__t__Cn1_P1_o3d_downsampled_ptr[camera_id_other], data.overlap_indices_other[permutation]);

  //  auto test_cloud_other_t0 =
  //      std::make_shared<open3d::geometry::PointCloud>(*data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other]);
  //  auto test_cloud_other_t1 =
  //      std::make_shared<open3d::geometry::PointCloud>(*data.Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_other]);
  //  test_cloud_other_t0->Transform(data.Cself0__T__Cother0[permutation].matrix());
  //  test_cloud_other_t1->Transform(data.Cself0__T__Cother0[permutation].matrix());
  //  // plot cloud corrrespondences
  //  utils::PlotCloudsCorrespondences(data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self], test_cloud_other_t0);
  //  utils::PlotCloudsCorrespondences(data.Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_self], test_cloud_other_t1);

  // print size of all 4 clouds
  std::cout << "Preprocessing:extractOverlapPoints: size of Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self]: "
            << data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self]->points_.size() << std::endl;
  std::cout << "Preprocessing:extractOverlapPoints: size of Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other]: "
            << data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other]->points_.size() << std::endl;
  std::cout << "Preprocessing:extractOverlapPoints: size of Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_self]: "
            << data.Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_self]->points_.size() << std::endl;
  std::cout << "Preprocessing:extractOverlapPoints: size of Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_other]: "
            << data.Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_other]->points_.size() << std::endl;

  // check if the number of points is the same
  if (data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self]->points_.size() !=
          data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_other]->points_.size() ||
      data.Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_self]->points_.size() !=
          data.Cn1__t__Cn1_P1_o3d_overlap_downsampled_ptr[camera_id_other]->points_.size()) {
    throw std::runtime_error(
        "Preprocessing:extractOverlapPoints: Number of points in the overlap point clouds"
        "not equal.");
  } else {
    exp_config.num_points_overlap[permutation] = data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id_self]->points_.size();
  }
}

}  // namespace preprocessing
}  // namespace core
