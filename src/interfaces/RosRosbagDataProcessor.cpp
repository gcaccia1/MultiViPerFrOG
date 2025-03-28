
#include "multiviperfrog/interfaces/RosRosbagDataProcessor.h"
#include <open3d/Open3D.h>
#include <ros/ros.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>
#include "multiviperfrog/core/workflow_manager.h"
#include "multiviperfrog/utils/data_conversions.h"
#include "multiviperfrog/utils/evaluation.h"
#include "multiviperfrog/utils/ros_visualization.h"
//#include "open3d_slam/frames.hpp"

namespace interfaces {

RosRosbagDataProcessor::RosRosbagDataProcessor(ros::NodeHandlePtr nh) : BASE(nh) {
  // Initialize Marker Publisher
  cam1_marker_pub_ = nh_->advertise<visualization_msgs::MarkerArray>(cam1_marker_topic_, 1);
  cam2_marker_pub_ = nh_->advertise<visualization_msgs::MarkerArray>(cam2_marker_topic_, 1);
  // Initialize Marker Array
  cam1_marker_array_msg_.markers.resize(0);
  cam2_marker_array_msg_.markers.resize(0);
}

void RosRosbagDataProcessor::initializeInput() {
  initCommonRosStuff();
  std::cout << "RosRosbagDataProcessor: selected rosbag: " << rosbagFilename_ << "\n";
}
// TODO: check if the data are ok as const reference??
void RosRosbagDataProcessor::setupProcessing(double topics_sync_upper_thr) {
  std::cout << "RosRosbagDataProcessor: opening bag and checking delay..." << std::endl;

  bag_.open(rosbagFilename_, rosbag::bagmode::Read);
  // Preprocess to find minimum synchronization delay
  preprocessRosbagForSynchronization(bag_, topics_sync_upper_thr, topics);
}

void RosRosbagDataProcessor::startProcessing(core::WorkFlowManager* manager, double topics_sync_upper_thr) {
  std::cout << "RosRosbagDataProcessor: starting to read from rosbag..." << std::endl;
  // Read and sync messages with the calculated delay, calls the processing core when sync is found
  readRosbagWithSync(manager, bag_, topics_sync_upper_thr, topics);
  bag_.close();
}

// implementation with synchronization here and below
void RosRosbagDataProcessor::preprocessRosbagForSynchronization(const rosbag::Bag& bag, double& accepted_delay,
                                                                const std::vector<std::string>& topics) {
  // Store timestamps for each topic in a vector of vectors
  std::vector<std::vector<ros::Time>> topic_timestamps(topics.size());

  // Traverse the bag and collect timestamps for each relevant topic
  rosbag::View view(bag);
  for (const rosbag::MessageInstance& msg_instance : view) {
    for (size_t i = 0; i < topics.size(); ++i) {
      if (msg_instance.getTopic() == topics[i]) {
        topic_timestamps[i].push_back(msg_instance.getTime());
      }
    }
  }

  // Determine the minimum number of messages in any topic (for sync)
  int n_sets = std::numeric_limits<int>::max();
  for (const auto& timestamps : topic_timestamps) {
    n_sets = std::min(n_sets, static_cast<int>(timestamps.size()));
  }

  // Search for the minimum delay that allows synchronization of n sets
  double calculated_minimum_delay = std::numeric_limits<double>::max();

  for (int i = 0; i < n_sets; ++i) {
    // Reference time from the first topic's i-th message
    ros::Time reference_time = topic_timestamps[0][i];

    // Find the max time difference between the reference and other topics' timestamps
    double max_diff = 0.0;
    for (size_t j = 1; j < topic_timestamps.size(); ++j) {
      double diff = std::abs((topic_timestamps[j][i] - reference_time).toSec());
      max_diff = std::max(max_diff, diff);
    }

    // Update the minimum delay if needed
    calculated_minimum_delay = std::min(calculated_minimum_delay, max_diff);
  }

  // Validate against the accepted delay threshold
  if (calculated_minimum_delay > accepted_delay) {
    //            // Update the delay and number of synchronized sets
    //            minimum_delay = calculated_minimum_delay;
    throw std::runtime_error(
        "RosRosbagDataProcessor: minimum delay found exceeds the accepted threshold. "
        "Set threshold to at least " +
        std::to_string(calculated_minimum_delay));
  }
}

void RosRosbagDataProcessor::readRosbagWithSync(core::WorkFlowManager* manager, const rosbag::Bag& bag, double minimum_delay,
                                                const std::vector<std::string>& topics) {
  // Buffer for the topics
  std::vector<std::deque<rosbag::MessageInstance>> topic_buffers(topics.size());
  // Traverse the messages in the bag and synchronize them
  rosbag::View view(bag);
  core::WorkFlowManager::rosbag_data_available_ = true;
  // iterate over the messages in the bag, when sync is found, calls the processing core
  for (const rosbag::MessageInstance& msg_instance : view) {
    if (ros::ok()) {
      // Fill topic buffers
      for (size_t i = 0; i < topics.size(); ++i) {
        if (msg_instance.getTopic() == topics[i]) {
          topic_buffers[i].push_back(msg_instance);
        }
      }
      // Check if we have enough messages in all buffers for synchronization
      bool all_buffers_have_data = true;
      for (const auto& buffer : topic_buffers) {
        if (buffer.empty()) {
          all_buffers_have_data = false;
          break;
        }
      }
      if (all_buffers_have_data) {
        // Use the first message in the first buffer as reference time
        ros::Time reference_time = topic_buffers[0].front().getTime();

        // Calculate time differences
        bool within_delay = true;
        for (size_t i = 1; i < topic_buffers.size(); ++i) {
          ros::Duration diff = topic_buffers[i].front().getTime() - reference_time;
          if (std::abs(diff.toSec()) > minimum_delay) {
            within_delay = false;
            break;
          }
        }
        if (within_delay) {
          // Create a vector of synchronized messages (just the front element of each buffer)
          std::vector<rosbag::MessageInstance> synced_messages;
          for (auto& buffer : topic_buffers) {
            synced_messages.push_back(buffer.front());
          }
          if (!synced_messages.empty()) {
            config::RosToPreprocessingData output_data;
            saveRosbagMessages(synced_messages, output_data);
            manager->runProcessingCore();
          } else {
            std::cout << "RosRosbagDataProcessor: no synchronized messages found yet, continuing...\n" << std::endl;
            continue;
          }
          // Remove last synchronized messages from the buffers
          for (auto& buffer : topic_buffers) {
            buffer.pop_front();
          }
        } else {
          // trow an error if the delay is too high
          throw std::runtime_error(
              "RosRosbagDataProcessor: desynchronization detected in the bag. "
              "Increase the delay threshold");
        }
      }
    } else {
      throw std::runtime_error("RosRosbagDataProcessor: user terminated manually, exiting...");
    }
  }
  core::WorkFlowManager::rosbag_data_available_ = false;
  std::cout << "RosRosbagDataProcessor: finished reading rosbag." << std::endl;
  // Print Snippet Start and End Indices
  auto& start_indices = core::WorkFlowManager::getSnippetStartIndices();
  auto& end_indices = core::WorkFlowManager::getSnippetEndIndices();
  assert(start_indices.size() == end_indices.size());
  for (size_t i = 0; i < start_indices.size(); ++i) {
    std::cout << "Snippet " << i << " start: " << start_indices[i] << " end: " << end_indices[i] << std::endl;
  }
  // Print Relative Pose per snippet for cam1 --------------------------------
  // Estimate
  auto& cam1_est_start_poses = core::WorkFlowManager::getCam1EstSnippetStartPoses();
  auto& cam1_est_end_poses = core::WorkFlowManager::getCam1EstSnippetEndPoses();
  // GT
  auto& cam1_gt_start_poses = core::WorkFlowManager::getCam1GtSnippetStartPoses();
  auto& cam1_gt_end_poses = core::WorkFlowManager::getCam1GtSnippetEndPoses();
  // SVD
  auto& cam1_SVD_start_poses = core::WorkFlowManager::getCam1SVDSnippetStartPoses();
  auto& cam1_SVD_end_poses = core::WorkFlowManager::getCam1SVDSnippetEndPoses();
  // Print and visualize
  std::cout << "Size of start_poses for cam1: " << cam1_est_start_poses.size() << std::endl;
  const float scale = 0.001;
  // Visualize Estimated Pose Positions as Point Markers
  addToMarkerArray(cam1_marker_array_msg_, cam1_est_start_poses, "cam_poses_start_est", scale, 0.0, 1.0, 1.0);
  addToMarkerArray(cam1_marker_array_msg_, cam1_est_end_poses, "cam_poses_end_est", scale, 0.0, 1.0, 1.0);
  // Visualize GT Pose Positions as Point Markers
  addToMarkerArray(cam1_marker_array_msg_, cam1_gt_start_poses, "cam_poses_start_gt", scale, 0.0, 1.0, 0.0);
  addToMarkerArray(cam1_marker_array_msg_, cam1_gt_end_poses, "cam_poses_end_gt", scale, 0.0, 1.0, 0.0);
  // Visualize SVD Pose Positions as Point Markers
  addToMarkerArray(cam1_marker_array_msg_, cam1_SVD_start_poses, "cam_poses_start_SVD", scale, 1.0, 1.0, 0.0);
  addToMarkerArray(cam1_marker_array_msg_, cam1_SVD_end_poses, "cam_poses_end_SVD", scale, 1.0, 1.0, 0.0);
  // Publish Marker Array
  cam1_marker_pub_.publish(cam1_marker_array_msg_);
  std::cout << "RosRosbagDataProcessor: published cam1 start poses as markers." << std::endl;

  // Print Relative Pose per snippet for cam2 --------------------------------
  // Estimates
  auto& cam2_est_start_poses = core::WorkFlowManager::getCam2EstSnippetStartPoses();
  auto& cam2_est_end_poses = core::WorkFlowManager::getCam2EstSnippetEndPoses();
  // GT
  auto& cam2_gt_start_poses = core::WorkFlowManager::getCam2GtSnippetStartPoses();
  auto& cam2_gt_end_poses = core::WorkFlowManager::getCam2GtSnippetEndPoses();
  // SVD
  auto& cam2_SVD_start_poses = core::WorkFlowManager::getCam2SVDSnippetStartPoses();
  auto& cam2_SVD_end_poses = core::WorkFlowManager::getCam2SVDSnippetEndPoses();

  // Print and visualize
  std::cout << "Size of start_poses2: " << cam2_est_start_poses.size() << std::endl;
  // Visualize Estimated Pose Positions as Point Markers
  addToMarkerArray(cam2_marker_array_msg_, cam2_est_start_poses, "cam_poses", scale, 0.0, 1.0, 1.0);
  addToMarkerArray(cam2_marker_array_msg_, cam2_est_end_poses, "cam_poses", scale, 0.0, 1.0, 1.0);
  // Visualize GT Pose Positions as Point Markers
  addToMarkerArray(cam2_marker_array_msg_, cam2_gt_start_poses, "cam_poses_gt", scale, 0.0, 1.0, 0.0);
  addToMarkerArray(cam2_marker_array_msg_, cam2_gt_end_poses, "cam_poses_gt", scale, 0.0, 1.0, 0.0);
  // Visualize SVD Pose Positions as Point Markers
  addToMarkerArray(cam2_marker_array_msg_, cam2_SVD_start_poses, "cam_poses_SVD", scale, 1.0, 1.0, 0.0);
  addToMarkerArray(cam2_marker_array_msg_, cam2_SVD_end_poses, "cam_poses_SVD", scale, 1.0, 1.0, 0.0);
  // Publish Marker Array
  cam2_marker_pub_.publish(cam2_marker_array_msg_);
  std::cout << "RosRosbagDataProcessor: published cam2 start poses as markers." << std::endl;

  // Quantitative Evaluation
  if (manager->exp_config_.run_odo_SVD) {
    std::cout << "Cam 1 -------------SVD--------------" << std::endl;
    evaluateTrajectorySnippets(cam1_gt_start_poses, cam1_gt_end_poses, cam1_SVD_start_poses, cam1_SVD_end_poses);
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "Cam 2 -------------SVD--------------" << std::endl;
    evaluateTrajectorySnippets(cam2_gt_start_poses, cam2_gt_end_poses, cam2_SVD_start_poses, cam2_SVD_end_poses);
    std::cout << "-----------------------------------" << std::endl;
  }
  // Cam 1
  std::cout << "Cam 1 --------------OPT-------------" << std::endl;
  evaluateTrajectorySnippets(cam1_gt_start_poses, cam1_gt_end_poses, cam1_est_start_poses, cam1_est_end_poses);
  std::cout << "-----------------------------------" << std::endl;
  // Cam 2
  std::cout << "Cam 2 --------------OPT-------------" << std::endl;
  evaluateTrajectorySnippets(cam2_gt_start_poses, cam2_gt_end_poses, cam2_est_start_poses, cam2_est_end_poses);
  std::cout << "-----------------------------------" << std::endl;
}

void RosRosbagDataProcessor::saveRosbagMessages(const std::vector<rosbag::MessageInstance>& synced_messages,
                                                config::RosToPreprocessingData& output_data) {
  if (synced_messages.size() < 9) {  // TODO here hardcoded, make dynamic
    throw std::runtime_error("RosRosbagDataProcessor: insufficient number of topics for output.");
  }

  // Extract synchronized messages from buffers and store in output_data
  output_data.Camera_points_msg_ptr[0] = synced_messages[0].instantiate<sensor_msgs::PointCloud2>();
  output_data.Camera_points_msg_ptr[1] = synced_messages[1].instantiate<sensor_msgs::PointCloud2>();
  output_data.Camera_points_msg_ptr[2] = synced_messages[2].instantiate<sensor_msgs::PointCloud2>();
  output_data.Camera_rel_flow_msg_ptr[0] = synced_messages[3].instantiate<sensor_msgs::PointCloud2>();
  output_data.Camera_rel_flow_msg_ptr[1] = synced_messages[4].instantiate<sensor_msgs::PointCloud2>();
  output_data.Camera_rel_flow_msg_ptr[2] = synced_messages[5].instantiate<sensor_msgs::PointCloud2>();
  output_data.Camera_pose_msg_ptr[0] = synced_messages[6].instantiate<geometry_msgs::PoseStamped>();
  output_data.Camera_pose_msg_ptr[1] = synced_messages[7].instantiate<geometry_msgs::PoseStamped>();
  output_data.Camera_pose_msg_ptr[2] = synced_messages[8].instantiate<geometry_msgs::PoseStamped>();

  // Process the output_data further if necessary
  core::WorkFlowManager::updateRosInputFields(output_data);
}

}  // namespace interfaces
