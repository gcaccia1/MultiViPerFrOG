
#ifndef CONFIG_CERES_CONFIG_H
#define CONFIG_CERES_CONFIG_H

#include "ceres/ceres.h"

namespace config {

struct CeresConfig {
  ceres::LinearSolverType solver = ceres::SPARSE_NORMAL_CHOLESKY;
  int max_num_iterations = 10;
  // num_threads, ceres::DENSE_QR, dense_linear_algebra_library_type
};

}  // namespace config

#endif  // CONFIG_CERES_CONFIG_H
