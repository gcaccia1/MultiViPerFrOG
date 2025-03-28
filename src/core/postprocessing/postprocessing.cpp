

#include "multiviperfrog/core/postprocessing/postprocessing.h"
#include <fstream>

namespace core {
namespace postprocessing {

void postprocessOptimizationDataForRos(const config::PostprocessingData& input_data, config::PostprocessingToRosData& output_data,
                                       const config::ExperimentConfig& exp_config, int camera_id) {
  utils::EigenIsometryStampedToTransformStampedMsg(input_data.parent_frame_id[camera_id],
                                                   input_data.Cpose_start__T__Ccurrent_estim[camera_id],
                                                   output_data.Cpose_start__T__Ccurrent_estim_msg[camera_id]);
  utils::EigenIsometryStampedToTransformStampedMsg(input_data.parent_frame_id[camera_id], input_data.Cpose_start__T__Ccurrent_GT[camera_id],
                                                   output_data.Cpose_start__T__Ccurrent_GT_msg[camera_id]);
  utils::EigenIsometryStampedToTransformStampedMsg(input_data.parent_frame_id[camera_id],
                                                   input_data.Cpose_start__T__Ccurrent_svd[camera_id],
                                                   output_data.Cpose_start__T__Ccurrent_svd_msg[camera_id]);
  utils::EigenIsometryStampedToTransformStampedMsg(input_data.parent_frame_id[camera_id],
                                                   input_data.Cpose_start__T__Ccurrent_icp[camera_id],
                                                   output_data.Cpose_start__T__Ccurrent_icp_msg[camera_id]);

  utils::Open3dToPointCloud2Custom(input_data.Cn0__t__Cn0_P0_o3d_ptr[camera_id], output_data.Cn0__t__Cn0_P0_o3d_msg[camera_id],
                                   input_data.Cpose_start__T__Ccurrent_GT[camera_id].frame_id_,
                                   input_data.Cpose_start__T__Ccurrent_GT[camera_id].stamp_, exp_config.cloud_original_height,
                                   exp_config.cloud_original_width);
  //  if (exp_config.optimize_odometries || exp_config.run_odo_ICP || exp_config.run_odo_SVD) {
  utils::Open3dToPointCloud2Custom(
      input_data.Cn0__t__Cn0_P0_o3d_downsampled_ptr[camera_id], output_data.Cn0__t__Cn0_P0_o3d_downsampled_msg[camera_id],
      input_data.Cpose_start__T__Ccurrent_GT[camera_id].frame_id_, input_data.Cpose_start__T__Ccurrent_GT[camera_id].stamp_,
      exp_config.cloud_original_height, exp_config.cloud_original_width);
  //  }
  if (exp_config.optimize_multicam) {
    //    utils::Open3dToPointCloud2Custom(
    //        input_data.Cn0__t__Cn0_P0_o3d_overlap_ptr[camera_id], output_data.Cn0__t__Cn0_P0_o3d_overlap_msg[camera_id],
    //        input_data.Cpose_start__T__Ccurrent_GT[camera_id].frame_id_, input_data.Cpose_start__T__Ccurrent_GT[camera_id].stamp_, 1,
    //        exp_config.num_points_overlap[0]);  // now hardcoded to overlap of camera 0 and 1

    utils::Open3dToPointCloud2Custom(
        input_data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_ptr[camera_id], output_data.Cn0__t__Cn0_P0_o3d_overlap_downsampled_msg[camera_id],
        input_data.Cpose_start__T__Ccurrent_GT[camera_id].frame_id_, input_data.Cpose_start__T__Ccurrent_GT[camera_id].stamp_, 1,
        exp_config.num_points_overlap_downsampled);  // theoretically same for all overlap regions
  }
}

void updateOdometry(const config::OptimizationToPostprocessingData& input_data, config::PostprocessingOdometryBuffer* odometry_buffer,
                    int camera_id) {
  utils::accumulateStampedOdometry(input_data.Ccurrent__T__Cnext_estim[camera_id],
                                   odometry_buffer->Cpose_start__T__Ccurrent_estim[camera_id]);
  utils::accumulateStampedOdometry(input_data.Ccurrent__T__Cnext_GT[camera_id], odometry_buffer->Cpose_start__T__Ccurrent_GT[camera_id]);
  utils::accumulateStampedOdometry(input_data.Ccurrent__T__Cnext_svd[camera_id], odometry_buffer->Cpose_start__T__Ccurrent_svd[camera_id]);
  utils::accumulateStampedOdometry(input_data.Ccurrent__T__Cnext_icp[camera_id], odometry_buffer->Cpose_start__T__Ccurrent_icp[camera_id]);
}

void initializeOdometry(const config::PreprocessingToOptimizationData& input_data, config::PostprocessingOdometryBuffer* odometry_buffer,
                        int camera_id) {
  odometry_buffer->Cpose_start__T__Ccurrent_estim[camera_id] = input_data.world__T__Cn0[camera_id];
  odometry_buffer->Cpose_start__T__Ccurrent_estim[camera_id].frame_id_ =
      odometry_buffer->Cpose_start__T__Ccurrent_estim[camera_id].frame_id_ + "_OPT";
  odometry_buffer->Cpose_start__T__Ccurrent_GT[camera_id] = input_data.world__T__Cn0[camera_id];
  odometry_buffer->Cpose_start__T__Ccurrent_GT[camera_id].frame_id_ =
      odometry_buffer->Cpose_start__T__Ccurrent_GT[camera_id].frame_id_ + "_GT";
  odometry_buffer->Cpose_start__T__Ccurrent_svd[camera_id] = input_data.world__T__Cn0[camera_id];
  odometry_buffer->Cpose_start__T__Ccurrent_svd[camera_id].frame_id_ =
      odometry_buffer->Cpose_start__T__Ccurrent_svd[camera_id].frame_id_ + "_SVD";
  odometry_buffer->Cpose_start__T__Ccurrent_icp[camera_id] = input_data.world__T__Cn0[camera_id];
  odometry_buffer->Cpose_start__T__Ccurrent_icp[camera_id].frame_id_ =
      odometry_buffer->Cpose_start__T__Ccurrent_icp[camera_id].frame_id_ + "_ICP";
}

void saveOdometry(const config::PostprocessingOdometryBuffer& odometry_buffer, config::PostprocessingData* postprocessing_data,
                  int camera_id) {
  postprocessing_data->parent_frame_id[camera_id] = odometry_buffer.parent_frame_id[camera_id];
  postprocessing_data->Cpose_start__T__Ccurrent_estim[camera_id] = odometry_buffer.Cpose_start__T__Ccurrent_estim[camera_id];
  postprocessing_data->Cpose_start__T__Ccurrent_GT[camera_id] = odometry_buffer.Cpose_start__T__Ccurrent_GT[camera_id];
  postprocessing_data->Cpose_start__T__Ccurrent_svd[camera_id] = odometry_buffer.Cpose_start__T__Ccurrent_svd[camera_id];
  postprocessing_data->Cpose_start__T__Ccurrent_icp[camera_id] = odometry_buffer.Cpose_start__T__Ccurrent_icp[camera_id];
}
 void saveEigenIsometryStampedToFreiburgStyle(const config::PostprocessingData& input_data, const int camera_id, const int frame_index_counter, const config::ExperimentConfig& exp_config) {
   // filename should be exp_config.rosbag_path (removing the last part of the string after the last /) + "Cam" + camera_id + "estimated_poses.txt"
   // like if rosbag_path: '/media/user/FastData/Blender_export/MICCAI_2025_official_renders_no_breathing/no_tools/2_moving_no_tools/vision_blender_MICCAI_600_frames_10hz_2_moving_no_tools.bag'
   // file shuld be saved in '/media/user/FastData/Blender_export/MICCAI_2025_official_renders_no_breathing/no_tools/2_moving_no_tools/Cam0estimated_poses.txt'
        std::string base_dir = exp_config.package_path + "/output";
        // Construct filename dynamically based on camera_id
        std::string filename = base_dir + "/Cam" + std::to_string(camera_id + 1) + "_estimated_poses.txt";
        // Determine file open mode (overwrite if first frame, append otherwise)
        std::ios_base::openmode mode = (frame_index_counter - 1 == 0) ? std::ios::trunc : std::ios::app;
        // Open file in the correct mode
        std::ofstream file(filename, mode);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << " for writing.\n";
            return;
        }
        // Extract translation
        Eigen::Vector3d translation = input_data.Cpose_start__T__Ccurrent_estim[camera_id].translation();
        // convert from rotation matrix to quaternion// Extract rotation as quaternion
        Eigen::Quaterniond quaternion(input_data.Cpose_start__T__Ccurrent_estim[camera_id].rotation());
        // Write data in Freiburg format where the first element of each row is ###### where the most left elements are flled with frame_index_counter
        file << std::setw(6) << std::setfill('0') << frame_index_counter-1 << " "
             << std::fixed << std::setprecision(6)
             << translation.x() << " " << translation.y() << " " << translation.z()
              << " " << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << " " <<quaternion.w() << "\n";
        // Close file
        file.close();
 }

}  // namespace postprocessing
}  // namespace core
