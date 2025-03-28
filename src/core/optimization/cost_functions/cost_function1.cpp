

#include "multiviperfrog/core/optimization/cost_functions/cost_function1.h"

namespace core {
namespace optimization {
namespace cost_functions {

CostFunction1::CostFunction1(const Eigen::Vector3d& observed_point) : observed_point_(observed_point) {}

bool CostFunction1::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
  // Parameters: pose[0..3] = quaternion (w, x, y, z), pose[4..6] = translation (x, y, z)
  const double* pose = parameters[0];

  Eigen::Quaterniond q(pose[0], pose[1], pose[2], pose[3]);
  Eigen::Vector3d t(pose[4], pose[5], pose[6]);

  // Transform point
  Eigen::Vector3d transformed_point = q * observed_point_ + t;

  // Define target point (e.g., origin)
  Eigen::Vector3d target_point = Eigen::Vector3d::Zero();

  // Compute residuals
  Eigen::Vector3d res = transformed_point - target_point;
  residuals[0] = res[0];
  residuals[1] = res[1];
  residuals[2] = res[2];

  // Optionally compute jacobians
  if (jacobians && jacobians[0]) {
    // Implement jacobian computation if necessary
  }

  return true;
}

}  // namespace cost_functions
}  // namespace optimization
}  // namespace core
