
#ifndef INTERFACES_HELPERS_ROS_H
#define INTERFACES_HELPERS_ROS_H

#pragma once
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <open3d/geometry/MeshBase.h>
#include <open3d/geometry/PointCloud.h>
#include <ros/publisher.h>
#include <ros/time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
//#include "open3d_slam/time.hpp"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"

namespace interfaces {  // TODO change to interfaces

class SubmapCollection;

void publishSubmapCoordinateAxes(const SubmapCollection& submaps, const std::string& frame_id, const ros::Time& timestamp,
                                 const ros::Publisher& pub);
geometry_msgs::Point createPoint(double x, double y, double z);
void drawAxes(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double scale, double line_width, visualization_msgs::Marker* marker);

void assembleColoredPointCloud(const SubmapCollection& submaps, open3d::geometry::PointCloud* cloud);

void publishCloud(const open3d::geometry::PointCloud& cloud, const std::string& frame_id, const ros::Time& timestamp, ros::Publisher& pub);

geometry_msgs::Pose getPose(const Eigen::MatrixXd& T);

geometry_msgs::TransformStamped toRos(const Eigen::Matrix4d& Mat, const ros::Time& time, const std::string& frame,
                                      const std::string& childFrame);

void publishTfTransform(const Eigen::Matrix4d& Mat, const ros::Time& time, const std::string& frame, const std::string& childFrame,
                        tf2_ros::TransformBroadcaster* broadcaster);
bool lookupTransform(const std::string& target_frame, const std::string& source_frame, const ros::Time& time,
                     std::shared_ptr<tf2_ros::Buffer> tfBuffer, Eigen::Isometry3d* transform);
bool lookupTransformOld(const std::string& target_frame, const std::string& source_frame, const ros::Time& time,
                        const tf::TransformListener& tfl, Eigen::Isometry3d* transform);

//ros::Time toRos(Time time);

//Time fromRos(const ::ros::Time& time);

template <typename Msg>
void publishIfSubscriberExists(const Msg& msg, const ros::Publisher& pub) {
  if (pub.getNumSubscribers() > 0) {
    pub.publish(msg);
  }
}

template <typename T>
inline void printKey(const std::string& key, T value) {
  std::cout << "\033[92m"
            << "Open3d SLAM "
            << "\033[0m" << key << "  set to: " << value << std::endl;
}

template <>
inline void printKey(const std::string& key, std::vector<double> vector) {
  std::cout << "\033[92m"
            << "Open3d SLAM "
            << "\033[0m" << key << " set to: ";
  for (const auto& element : vector) {
    std::cout << element << ",";
  }
  std::cout << std::endl;
}

template <typename T>
T tryGetParam(const std::string& key, const ros::NodeHandle& privateNode) {
  T value;
  if (privateNode.getParam(key, value)) {
    printKey(key, value);
    return value;
  } else {
    if (privateNode.getParam("/" + key, value)) {
      printKey("/" + key, value);
      return value;
    }

    throw std::runtime_error("Open3d SLAM - " + key + " not specified.");
  }
}

// from previous file Color.h (in o3d_slam)
class Color : public std_msgs::ColorRGBA {
 public:
  Color();
  Color(double red, double green, double blue);
  Color(double red, double green, double blue, double alpha);
  Color operator*(double scalar) const;

  static const Color White() { return Color(1.0, 1.0, 1.0); }
  static const Color Black() { return Color(0.0, 0.0, 0.0); }
  static const Color Gray() { return Color(0.5, 0.5, 0.5); }
  static const Color Red() { return Color(1.0, 0.0, 0.0); }
  static const Color Green() { return Color(0.0, 1.0, 0.0); }
  static const Color Blue() { return Color(0.0, 0.0, 1.0); }
  static const Color Yellow() { return Color(1.0, 1.0, 0.0); }
  static const Color Orange() { return Color(1.0, 0.5, 0.0); }
  static const Color Purple() { return Color(0.5, 0.0, 1.0); }
  static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
  static const Color Teal() { return Color(0.0, 1.0, 1.0); }
  static const Color Pink() { return Color(1.0, 0.0, 0.5); }
  static const Color Magenta() { return Color(0.78, 0.0, 0.9); }
  static const int numColors_ = 13;
  static const Color getColor(int colorCode);
};

} /* namespace interfaces */

#endif  // INTERFACES_HELPERS_ROS_H
