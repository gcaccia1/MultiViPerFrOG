
#include <visualization_msgs/MarkerArray.h>

#ifndef ROS_VISUALIZATION_H
#define ROS_VISUALIZATION_H

void addToMarkerArray(visualization_msgs::MarkerArray& marker_array_msg_, const std::vector<Eigen::Isometry3d>& cam_poses,
                      const std::string& ns, const float scale, const float r, const float g, const float b) {
  for (size_t i = 0; i < cam_poses.size(); ++i) {
    Eigen::Vector3d position = cam_poses[i].translation();
    // ROS Marker for Position
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // COLOR: Green
    marker.color.a = 1.0f;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    // Scale
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    // Add Marker to Marker Array
    marker_array_msg_.markers.push_back(marker);
  }
}

#endif  // ROS_VISUALIZATION_H
