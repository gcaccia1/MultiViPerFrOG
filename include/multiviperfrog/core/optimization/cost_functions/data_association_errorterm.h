
#ifndef CORE_OPTIMIZATION_COST_FUNCTIONS_DATA_ASSOCIATION_ERRORTERM_H
#define CORE_OPTIMIZATION_COST_FUNCTIONS_DATA_ASSOCIATION_ERRORTERM_H

#include <utility>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/autodiff_cost_function.h"
namespace core::optimization::cost_functions {
//    template <typename T>
class DataAssociationErrorterm {
 public:
  DataAssociationErrorterm(Eigen::Vector3d Cself__t__Cself_P_measured, Eigen::Vector3d Cother__t__Cother_P_measured)
      : Cself__t__Cself_P_measured_(std::move(Cself__t__Cself_P_measured)),
        Cother__t__Cother_P_measured_(std::move(Cother__t__Cother_P_measured)) {}
  // TODO check difference between using std::move vs. declaring as const reference
  template <typename T>
  bool operator()(const T* const Cself__t__Cself_Cother_ptr,  // x5 (but could be also used to estimate any AO problem)
                  const T* const Cself__q__Cother_ptr,        // x5 (but could be also used to estimate any AO problem)
                  T* residuals_ptr) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> Cself__t__Cself_Cother(Cself__t__Cself_Cother_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> Cself__q__Cother(Cself__q__Cother_ptr);

    // m4 = x5 * m5 (Cself__t__Cself_P = Cself__t__Cself_Cother * Cother__t__Cother_P)
    Eigen::Matrix<T, 3, 1> Cself__t__Cself_P_estimated =
        Cself__t__Cself_Cother + Cself__q__Cother * Cother__t__Cother_P_measured_.template cast<T>();

    residuals_ptr[0] = Cself__t__Cself_P_measured_[0] - Cself__t__Cself_P_estimated[0];
    residuals_ptr[1] = Cself__t__Cself_P_measured_[1] - Cself__t__Cself_P_estimated[1];
    residuals_ptr[2] = Cself__t__Cself_P_measured_[2] - Cself__t__Cself_P_estimated[2];
    return true;
  }
  static ceres::CostFunction* Create(const Eigen::Vector3d& Cself__t__Cself_P_measured,
                                     const Eigen::Vector3d& Cother__t__Cother_P_measured) {
    return new ceres::AutoDiffCostFunction<DataAssociationErrorterm, 3, 3, 4>(Cself__t__Cself_P_measured, Cother__t__Cother_P_measured);
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  const Eigen::Vector3d Cself__t__Cself_P_measured_;
  const Eigen::Vector3d Cother__t__Cother_P_measured_;
};
}  // namespace core::optimization::cost_functions

#endif  // CORE_OPTIMIZATION_COST_FUNCTIONS_DATA_ASSOCIATION_ERRORTERM_H
