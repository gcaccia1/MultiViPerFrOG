

#ifndef UTILS_DATA_CONVERSIONS_H
#define UTILS_DATA_CONVERSIONS_H

#include <geometry_msgs/TransformStamped.h>
#include <open3d/Open3D.h>
#include <sensor_msgs/PointCloud2.h>
#include <optional>
#include "multiviperfrog/config/optimization_data.h"

namespace utils {

// Convert ROS PointCloud2 message to OptimizationData
//    void convertRosToOptimizationData(const sensor_msgs::PointCloud2& msg, config::OptimizationData& data);

// Convert ROS TransformStamped message to Eigen Isometry
void convertRosToEigen(const geometry_msgs::TransformStamped& msg, Eigen::Isometry3d& transform);

void EigenIsometryStampedToTransformStampedMsg(const std::string& parent_frame_id, const tf2::Stamped<Eigen::Isometry3d>& input_data,
                                               geometry_msgs::TransformStamped& output_data);

tf2::Stamped<Eigen::Isometry3d> PoseStampedMsgToEigenIsometryStamped(const geometry_msgs::PoseStampedConstPtr& pose_msg);

void accumulateStampedOdometry(const tf2::Stamped<Eigen::Isometry3d>& Ccurrent__T__Cnext,
                               tf2::Stamped<Eigen::Isometry3d>& Cpose_start__T__Ccurrent);

void PointCLoud2ToOpen3dCustom(const sensor_msgs::PointCloud2ConstPtr& ros_pc2, open3d::geometry::PointCloud& o3d_pc,
                                std::optional<std::vector<uint8_t>>& tool_mask,
                                std::optional<std::vector<float>>& confidence_depth,
                                std::optional<std::vector<float>>& confidence_flow);

void Open3dToPointCloud2Custom(const std::shared_ptr<open3d::geometry::PointCloud>& o3d_pc, sensor_msgs::PointCloud2& ros_pc2,
                               const std::string& frame_id, const ros::Time& stamp, int height = 1, int width = 0);

void splitROSPC2RGBFields(uint32_t packed_rgb, uint8_t& r, uint8_t& g, uint8_t& b);

Eigen::Matrix3Xd Open3dToEigenMatrix(const std::shared_ptr<open3d::geometry::PointCloud>& point_cloud);
// Convert OptimizationData to ROS messages if needed

}  // namespace utils

#endif  // UTILS_DATA_CONVERSIONS_H
