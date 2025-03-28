

#ifndef CORE_OPTIMIZATION_COST_FUNCTIONS_SF_KNOWLEDGE_ERRORTERM_H
#define CORE_OPTIMIZATION_COST_FUNCTIONS_SF_KNOWLEDGE_ERRORTERM_H
#include <utility>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/autodiff_cost_function.h"
namespace core::optimization::cost_functions {
class SFKnowledgeErrorterm {
 public:
  SFKnowledgeErrorterm(const Eigen::Vector3d C0__t__P0_P1_knowledge)  // Cb0_SF knowledge
      : C0__t__P0_P1_knowledge_(std::move(C0__t__P0_P1_knowledge)) {}
  // TODO check difference between using std::move vs. declaring as const reference
  template <typename T>
  bool operator()(const T* const C0__t__P0_P1_ptr,  // Cb0_SF
                  T* residuals_ptr) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> C0__t__P0_P1(C0__t__P0_P1_ptr);

    // define residuals TODO: compress notation to single line
    residuals_ptr[0] = C0__t__P0_P1_knowledge_[0] - C0__t__P0_P1[0];
    residuals_ptr[1] = C0__t__P0_P1_knowledge_[1] - C0__t__P0_P1[1];
    residuals_ptr[2] = C0__t__P0_P1_knowledge_[2] - C0__t__P0_P1[2];
    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d& C0__t__P0_P1_knowledge) {
    return new ceres::AutoDiffCostFunction<SFKnowledgeErrorterm, 3, 3>(C0__t__P0_P1_knowledge);
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // TODO check what this does
      private : const Eigen::Vector3d C0__t__P0_P1_knowledge_;
};
}  // namespace core::optimization::cost_functions

#endif  // CORE_OPTIMIZATION_COST_FUNCTIONS_SF_KNOWLEDGE_ERRORTERM_H
