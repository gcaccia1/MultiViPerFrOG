

#ifndef EVALUATION_H
#define EVALUATION_H

void evaluateTrajectorySnippets(const std::vector<Eigen::Isometry3d>& snippet_start_poses_gt,
                                const std::vector<Eigen::Isometry3d>& snippet_end_poses_gt,
                                const std::vector<Eigen::Isometry3d>& snippet_start_poses_est,
                                const std::vector<Eigen::Isometry3d>& snippet_end_poses_est) {
  // Evaluate the trajectory snippets
  std::cout << "Bon giorno! Evaluation for snippets happening now." << std::endl;
  // Compute relative poses for each snippet
  std::vector<Eigen::Isometry3d> snippet_rel_poses_gt;
  std::vector<Eigen::Isometry3d> snippet_rel_poses_est;
  for (size_t i = 0; i < snippet_start_poses_gt.size(); ++i) {
    Eigen::Isometry3d rel_pose_gt = snippet_start_poses_gt[i].inverse() * snippet_end_poses_gt[i];
    Eigen::Isometry3d rel_pose_est = snippet_start_poses_est[i].inverse() * snippet_end_poses_est[i];
    snippet_rel_poses_gt.push_back(rel_pose_gt);
    snippet_rel_poses_est.push_back(rel_pose_est);
  }
  // Compute the relative poses error
  std::vector<double> rel_translation_error;
  std::vector<double> rel_rotation_error;
  for (size_t i = 0; i < snippet_rel_poses_gt.size(); ++i) {
    Eigen::Isometry3d rel_pose_gt = snippet_rel_poses_gt[i];
    Eigen::Isometry3d rel_pose_est = snippet_rel_poses_est[i];
    Eigen::Isometry3d rel_pose_error = rel_pose_gt.inverse() * rel_pose_est;
    // Translation
    double translation_error = rel_pose_error.translation().norm() * 1000.0; // Convert to mm
    rel_translation_error.push_back(translation_error);
    // Rotation
    Eigen::AngleAxisd rot_error(rel_pose_error.rotation());
    double angle_error = rot_error.angle();
    double angle_error_degrees = angle_error * (180.0 / M_PI); // Convert to degrees
    rel_rotation_error.push_back(angle_error_degrees);
  }
  // Print everything
  double averageTranslationError = 0.0;
  double standardDeviationTranslationError = 0.0;
  double averageRotationError = 0.0;
  double standardDeviationRotationError = 0.0;
  for (size_t i = 0; i < snippet_rel_poses_gt.size(); ++i) {
    std::cout << "Snippet " << i << " translation error: " << rel_translation_error[i] << " rotation error: " << rel_rotation_error[i]
              << std::endl;
    averageTranslationError += rel_translation_error[i];
    averageRotationError += rel_rotation_error[i];
  }
  averageTranslationError /= snippet_rel_poses_gt.size();
  averageRotationError /= snippet_rel_poses_gt.size();
  for (size_t i = 0; i < snippet_rel_poses_gt.size(); ++i) {
    standardDeviationTranslationError +=
        (rel_translation_error[i] - averageTranslationError) * (rel_translation_error[i] - averageTranslationError);
    standardDeviationRotationError += (rel_rotation_error[i] - averageRotationError) * (rel_rotation_error[i] - averageRotationError);
  }
  standardDeviationTranslationError = sqrt(standardDeviationTranslationError / snippet_rel_poses_gt.size());
  standardDeviationRotationError = sqrt(standardDeviationRotationError / snippet_rel_poses_gt.size());
  // Print averages
  std::cout << "Average translation error (+- SD): " << averageTranslationError << " (+- " << standardDeviationTranslationError << ")"
            << std::endl;
  std::cout << " Average rotation error (+- SD): " << averageRotationError << " (+- " << standardDeviationRotationError << ")" << std::endl;
}

#endif  // EVALUATION_H
