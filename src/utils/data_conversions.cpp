

#include "multiviperfrog/utils/data_conversions.h"
#include <open3d_conversions/open3d_conversions.h>
#include <tf2_eigen/tf2_eigen.h>

namespace utils {

//    void convertRosToOptimizationData(const sensor_msgs::PointCloud2& msg, config::OptimizationData& data) {
//          std::cout << "Conversion not implemented yet!" << std::endl;
////        open3d_conversions::rosToOpen3d(msg, *data.point_cloud);
//    }

//    void convertRosToEigen(const geometry_msgs::TransformStamped& msg, Eigen::Isometry3d& transform) {
//            std::cout << "Conversion not implemented yet!" << std::endl;
////        transform = tf2::transformToEigen(msg);
//    }

tf2::Stamped<Eigen::Isometry3d> PoseStampedMsgToEigenIsometryStamped(const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  //        Eigen::Isometry3d eigen_transform = Eigen::Isometry3d::Identity();  // Initialize as identity to be safe
  // Create a Stamped<Eigen::Isometry3d> for output
  tf2::Stamped<Eigen::Isometry3d> eigen_stamped_transform;
  // Convert pose_msg's pose to Eigen type
  tf2::fromMsg(*pose_msg, eigen_stamped_transform);
  //        // Get the Eigen::Isometry3d from the stamped version
  //        eigen_transform = eigen_stamped_transform;
  // TODO comment this line and change the return type to
  // tf2::Stamped<Eigen::Isometry3d> if you want to return the stamped version

  return eigen_stamped_transform;
}

void EigenIsometryStampedToTransformStampedMsg(const std::string& parent_frame_id, const tf2::Stamped<Eigen::Isometry3d>& input_data,
                                               geometry_msgs::TransformStamped& output_data) {
  // Set the header (timestamp and frame id)
  output_data.header.stamp = input_data.stamp_;
  output_data.header.frame_id =
      parent_frame_id;  // in this case the parent frame id is passed as an argument (Cpose_start for odometry case)
  output_data.child_frame_id = input_data.frame_id_;  // this is the frame_id from the callback, it is the camera frame id

  // Convert Eigen Isometry to translation
  Eigen::Vector3d translation = input_data.translation();
  output_data.transform.translation.x = translation.x();
  output_data.transform.translation.y = translation.y();
  output_data.transform.translation.z = translation.z();

  // Convert Eigen Isometry to rotation (quaternion)
  Eigen::Quaterniond rotation(input_data.rotation());
  output_data.transform.rotation.x = rotation.x();
  output_data.transform.rotation.y = rotation.y();
  output_data.transform.rotation.z = rotation.z();
  output_data.transform.rotation.w = rotation.w();
}

void accumulateStampedOdometry(const tf2::Stamped<Eigen::Isometry3d>& Ccurrent__T__Cnext,
                               tf2::Stamped<Eigen::Isometry3d>& Cpose_start__T__Ccurrent) {
  // Concatenate the incoming relative transformation to the cumulative transformation
  Cpose_start__T__Ccurrent.matrix() = Cpose_start__T__Ccurrent.matrix() * Ccurrent__T__Cnext.matrix();
  Cpose_start__T__Ccurrent.stamp_ = Ccurrent__T__Cnext.stamp_;
  Cpose_start__T__Ccurrent.frame_id_ = Ccurrent__T__Cnext.frame_id_;
}

void PointCLoud2ToOpen3dCustom(const sensor_msgs::PointCloud2ConstPtr& ros_pc2, open3d::geometry::PointCloud& o3d_pc,
                              std::optional<std::vector<uint8_t>>& tool_mask,
                              std::optional<std::vector<float>>& confidence_depth,
                              std::optional<std::vector<float>>& confidence_flow) {

  bool has_mask = false, has_confidence_depth = false, has_confidence_flow = false;
  for (const auto& field : ros_pc2->fields) {
    if (field.name == "mask") has_mask = true;
    if (field.name == "confidence_depth") has_confidence_depth = true;
    if (field.name == "confidence_flow") has_confidence_flow = true;
  }
  size_t point_count = ros_pc2->height * ros_pc2->width;
  // Allocate only if the field exists
  if (has_mask) tool_mask.emplace().reserve(point_count);
  if (has_confidence_depth) confidence_depth.emplace().reserve(point_count);
  if (has_confidence_flow) confidence_flow.emplace().reserve(point_count);

  if (ros_pc2->fields.size() == 3) {
    if (ros_pc2->fields[0].name == "x") {
      //                std::cout << "Found three fields: x, y, z. \n";
      sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_x(*ros_pc2, "x");
      sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_y(*ros_pc2, "y");
      sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_z(*ros_pc2, "z");
      o3d_pc.points_.reserve(point_count);
      for (size_t i = 0; i < point_count; ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
        o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
      }
    } else if (ros_pc2->fields[0].name == "dx") {  // TODO check if it can actually be possible to have only dxdydz w/o xyz
                                                   //                std::cout << "Found three fields: dx, dy, dz. \n";
      sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_dx(*ros_pc2, "dx");
      sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_dy(*ros_pc2, "dy");
      sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_dz(*ros_pc2, "dz");
      o3d_pc.normals_.reserve(point_count);
      for (size_t i = 0; i < point_count; ++i, ++ros_pc2_dx, ++ros_pc2_dy, ++ros_pc2_dz) {
        o3d_pc.normals_.push_back(Eigen::Vector3d(*ros_pc2_dx, *ros_pc2_dy, *ros_pc2_dz));
      }
    } else {
      std::cout << "Warning: unsupported point cloud format, found three fields. Expect either xyz or dxdydz. \n";
      return;
    }
  }  // TODO: make debug messages dynamic with the actual number and names of fields
  else if (ros_pc2->fields.size() > 3) {
    if (ros_pc2->fields[0].name == "x") {
      sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_x(*ros_pc2, "x");
      sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_y(*ros_pc2, "y");
      sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_z(*ros_pc2, "z");
      o3d_pc.points_.reserve(point_count);
      for (size_t i = 0; i < point_count; ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
        o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
      }
    } else {
      std::cout << "Warning: unsupported point cloud format, found >3 fields. Expected xyz as first three.  \n";
      return;
    }
    if (ros_pc2->fields[3].name == "rgb") {
      //                std::cout << "Found 4 fields: x, y, z, rgb. \n";

      sensor_msgs::PointCloud2ConstIterator<uint32_t> ros_pc2_rgb(*ros_pc2, "rgb");
      o3d_pc.colors_.reserve(point_count);
      for (size_t i = 0; i < point_count; ++i, ++ros_pc2_rgb) {
        uint8_t r, g, b;
        splitROSPC2RGBFields(*ros_pc2_rgb, r, g, b);
        o3d_pc.colors_.push_back(Eigen::Vector3d(((int)(r)) / 255.0, ((int)(g)) / 255.0, ((int)(b)) / 255.0));
      }
    } else if (ros_pc2->fields[3].name == "intensity") {
      //                std::cout << "Found 4 fields: x, y, z, intensity. \n";
      sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_i(*ros_pc2, "intensity");
      for (size_t i = 0; i < point_count; ++i, ++ros_pc2_i) {
        o3d_pc.colors_.push_back(Eigen::Vector3d(*ros_pc2_i, *ros_pc2_i, *ros_pc2_i));
        // TODO: NOTE: intesity is copied to all three channels, this is not correct, but it is a placeholder
      }
    } else if (ros_pc2->fields[3].name == "dx") {
      //                std::cout << "Found 6 fields: x, y, z, dx, dy, dz. \n";
      sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_dx(*ros_pc2, "dx");
      sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_dy(*ros_pc2, "dy");
      sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_dz(*ros_pc2, "dz");
      o3d_pc.normals_.reserve(point_count);
      for (size_t i = 0; i < point_count; ++i, ++ros_pc2_dx, ++ros_pc2_dy, ++ros_pc2_dz) {
        o3d_pc.normals_.push_back(Eigen::Vector3d(*ros_pc2_dx, *ros_pc2_dy, *ros_pc2_dz));
      }
    } else {
      std::cout << "Warning: unsupported point cloud format, found >3 fields. Expected rgb or intensity or dxdydx after xyz.  \n";
      return;
    }
    if (ros_pc2->fields.size() > 6 && ros_pc2->fields[3].name == "dx" && ros_pc2->fields[6].name == "rgb") {
        //                std::cout << "Found 9 fields: x, y, z, dx, dy, dz, rgb. \n";
        sensor_msgs::PointCloud2ConstIterator<uint32_t> ros_pc2_rgb(*ros_pc2, "rgb");
        o3d_pc.colors_.reserve(point_count);
        for (size_t i = 0; i < point_count; ++i, ++ros_pc2_rgb) {
          uint8_t r, g, b;
          splitROSPC2RGBFields(*ros_pc2_rgb, r, g, b);
          o3d_pc.colors_.push_back(Eigen::Vector3d(((int)(r)) / 255.0, ((int)(g)) / 255.0, ((int)(b)) / 255.0));
        }
    }
  } else {
    std::cout << "Warning: not enough fields found: " << ros_pc2->fields.size() << " fields. Expected 3 or 6 or 9 or 12. \n";
    return;
  }
  // Load optional fields
  if (has_mask) {
    sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_mask(*ros_pc2, "mask");
    for (size_t i = 0; i < point_count; ++i, ++ros_pc2_mask) {
      tool_mask->push_back(*ros_pc2_mask);
    }
  }

  if (has_confidence_depth) {
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_conf_depth(*ros_pc2, "confidence_depth");
    for (size_t i = 0; i < point_count; ++i, ++ros_pc2_conf_depth) {
      confidence_depth->push_back(*ros_pc2_conf_depth);
    }
  }

  if (has_confidence_flow) {
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_conf_flow(*ros_pc2, "confidence_flow");
    for (size_t i = 0; i < point_count; ++i, ++ros_pc2_conf_flow) {
      confidence_flow->push_back(*ros_pc2_conf_flow);
    }
  }
}

void splitROSPC2RGBFields(uint32_t packed_rgb, uint8_t& r, uint8_t& g, uint8_t& b) {
  r = (packed_rgb >> 16) & 0xFF;  // Extract the red channel
  g = (packed_rgb >> 8) & 0xFF;   // Extract the green channel
  b = packed_rgb & 0xFF;          // Extract the blue channel
}

void Open3dToPointCloud2Custom(const std::shared_ptr<open3d::geometry::PointCloud>& o3d_pc, sensor_msgs::PointCloud2& ros_pc2,
                               const std::string& frame_id, const ros::Time& stamp, int height, int width) {
  // Set up the ROS PointCloud2 header
  ros_pc2.header.stamp = stamp;
  ros_pc2.header.frame_id = frame_id;

  // Use PointCloud2Modifier for "xyz" and "rgb" fields
  sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
  if (o3d_pc->HasColors()) {
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  } else {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  }

  // Set height and width for ordered/unordered point clouds
  if (height > 1) {
    ros_pc2.height = height;  // Ordered cloud
    ros_pc2.width = (width > 0) ? width : o3d_pc->points_.size() / height;
  } else {
    ros_pc2.height = 1;  // Unordered cloud
    ros_pc2.width = o3d_pc->points_.size();
  }

  // Resize the point cloud to fit the number of points
  modifier.resize(o3d_pc->points_.size());

  // Add normals (dx, dy, dz) fields if the point cloud has normals
  if (o3d_pc->HasNormals()) {
    int offset = ros_pc2.point_step;  // Start offset after xyz (and optionally rgb)

    sensor_msgs::PointField point_field_dx, point_field_dy, point_field_dz;

    point_field_dx.name = "normal_x";
    point_field_dx.offset = offset;
    point_field_dx.datatype = sensor_msgs::PointField::FLOAT32;
    point_field_dx.count = 1;
    ros_pc2.fields.push_back(point_field_dx);
    offset += sizeof(float);

    point_field_dy.name = "normal_y";
    point_field_dy.offset = offset;
    point_field_dy.datatype = sensor_msgs::PointField::FLOAT32;
    point_field_dy.count = 1;
    ros_pc2.fields.push_back(point_field_dy);
    offset += sizeof(float);

    point_field_dz.name = "normal_z";
    point_field_dz.offset = offset;
    point_field_dz.datatype = sensor_msgs::PointField::FLOAT32;
    point_field_dz.count = 1;
    ros_pc2.fields.push_back(point_field_dz);
    offset += sizeof(float);

    ros_pc2.point_step = offset;
    ros_pc2.row_step = ros_pc2.point_step * ros_pc2.width;
    ros_pc2.data.resize(ros_pc2.height * ros_pc2.row_step);
  }

  // Fill the XYZ (and optionally RGB) data
  sensor_msgs::PointCloud2Iterator<float> iter_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(ros_pc2, "z");

  std::optional<sensor_msgs::PointCloud2Iterator<uint8_t>> iter_r, iter_g, iter_b;
  if (o3d_pc->HasColors()) {
    iter_r.emplace(ros_pc2, "r");
    iter_g.emplace(ros_pc2, "g");
    iter_b.emplace(ros_pc2, "b");
  }

  // If the point cloud has normals (dx, dy, dz)
  std::optional<sensor_msgs::PointCloud2Iterator<float>> iter_dx, iter_dy, iter_dz;
  if (o3d_pc->HasNormals()) {
    iter_dx.emplace(ros_pc2, "normal_x");  // previously dx dy dz, now unformed with rviz visualization plugin
    iter_dy.emplace(ros_pc2, "normal_y");  // https://github.com/UCR-Robotics/pointcloud2_normal_rviz_plugin
    iter_dz.emplace(ros_pc2, "normal_z");
  }

  for (size_t i = 0; i < o3d_pc->points_.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
    const Eigen::Vector3d& point = o3d_pc->points_[i];
    *iter_x = point(0);
    *iter_y = point(1);
    *iter_z = point(2);

    // Fill RGB if available
    if (o3d_pc->HasColors() && iter_r && iter_g && iter_b) {
      const Eigen::Vector3d& color = o3d_pc->colors_[i];
      *(*iter_r) = static_cast<uint8_t>(255 * color(0));
      *(*iter_g) = static_cast<uint8_t>(255 * color(1));
      *(*iter_b) = static_cast<uint8_t>(255 * color(2));
      ++(*iter_r);
      ++(*iter_g);
      ++(*iter_b);
    }

    // Fill normals (dx, dy, dz) if available
    if (o3d_pc->HasNormals() && iter_dx && iter_dy && iter_dz) {
      const Eigen::Vector3d& normal = o3d_pc->normals_[i];
      *(*iter_dx) = normal(0);
      *(*iter_dy) = normal(1);
      *(*iter_dz) = normal(2);
      ++(*iter_dx);
      ++(*iter_dy);
      ++(*iter_dz);
    }
  }
}

Eigen::Matrix3Xd Open3dToEigenMatrix(const std::shared_ptr<open3d::geometry::PointCloud>& point_cloud) {
  // Get the number of points in the point cloud
  size_t num_points = point_cloud->points_.size();

  // Initialize an Eigen matrix of size 3 x num_points
  Eigen::Matrix3Xd points_matrix(3, num_points);

  // Iterate through all points in the Open3D point cloud and assign them to the Eigen matrix
  for (size_t i = 0; i < num_points; ++i) {
    points_matrix.col(i) = point_cloud->points_[i];
  }
  return points_matrix;
}

}  // namespace utils
