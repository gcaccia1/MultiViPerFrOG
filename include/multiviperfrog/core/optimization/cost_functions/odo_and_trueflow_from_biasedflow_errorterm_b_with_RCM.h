

#ifndef CORE_OPTIMIZATION_COST_FUNCTIONS_ODO_AND_TRUEFLOW_FROM_BIASEDFLOW_ERRORTERM_B_WITH_RCM_H
#define CORE_OPTIMIZATION_COST_FUNCTIONS_ODO_AND_TRUEFLOW_FROM_BIASEDFLOW_ERRORTERM_B_WITH_RCM_H

#include <utility>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/autodiff_cost_function.h"

namespace core::optimization::cost_functions {
//    template <typename T>
class OdoAndTrueflowFromBiasedflowErrortermBwithRCM {
 public:
  OdoAndTrueflowFromBiasedflowErrortermBwithRCM(Eigen::Vector3d Cb0__t__Cb0_P0_measured, Eigen::Vector3d Cb1__t__Cb1_P1_measured)
      : Cb0__t__Cb0_P0_measured_(std::move(Cb0__t__Cb0_P0_measured)), Cb1__t__Cb1_P1_measured_(std::move(Cb1__t__Cb1_P1_measured)) {}
  // TODO check difference between using std::move vs. declaring as const reference
  template <typename T>
  bool operator()(const T* const Cb0__t__Cb0_Cb1_ptr,  // x2
                  const T* const Cb0__q__Cb1_ptr,      // x2
                  const T* const Cb0__t__P0_P1_ptr,    // Cb0_SF
                  T* residuals_ptr) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> Cb0__t__Cb0_Cb1(Cb0__t__Cb0_Cb1_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> Cb0__q__Cb1(Cb0__q__Cb1_ptr);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> Cb0__t__P0_P1(Cb0__t__P0_P1_ptr);
    // Map translation vector components
    const T& t1 = Cb0__t__Cb0_Cb1_ptr[0];
    const T& t2 = Cb0__t__Cb0_Cb1_ptr[1];
    // (t3 is not needed for this constraint)
    // Map quaternion components
    const T& q0 = Cb0__q__Cb1_ptr[0];  // scalar part (w)
    const T& q1 = Cb0__q__Cb1_ptr[1];  // x
    const T& q2 = Cb0__q__Cb1_ptr[2];  // y
    const T& q3 = Cb0__q__Cb1_ptr[3];  // z
    // here the simple transformation is applied to the point as m3 = x2 * (m5 + Cb0_SF)
    Eigen::Matrix<T, 3, 1> Cb1__t__Cb1_P1_estimated = Cb0__q__Cb1.inverse() * (Cb0__t__Cb0_P0_measured_ + Cb0__t__P0_P1 - Cb0__t__Cb0_Cb1);

    residuals_ptr[0] = (q2 * q3 - q0 * q1) * t1 - (q1 * q2 - q0 * q3) * t2;
    residuals_ptr[1] = (Cb1__t__Cb1_P1_measured_[0] - Cb1__t__Cb1_P1_estimated[0]);
    residuals_ptr[2] = (Cb1__t__Cb1_P1_measured_[1] - Cb1__t__Cb1_P1_estimated[1]);
    residuals_ptr[3] = (Cb1__t__Cb1_P1_measured_[2] - Cb1__t__Cb1_P1_estimated[2]);
    return true;
  }
  static ceres::CostFunction* Create(const Eigen::Vector3d& Cb0__t__Cb0_P0_measured, const Eigen::Vector3d& Cb1__t__Cb1_P1_measured) {
    return new ceres::AutoDiffCostFunction<OdoAndTrueflowFromBiasedflowErrortermBwithRCM, 4, 3, 4, 3>(Cb0__t__Cb0_P0_measured,
                                                                                                      Cb1__t__Cb1_P1_measured);
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  const Eigen::Vector3d Cb0__t__Cb0_P0_measured_;
  const Eigen::Vector3d Cb1__t__Cb1_P1_measured_;
};
}  // namespace core::optimization::cost_functions

#endif  // CORE_OPTIMIZATION_COST_FUNCTIONS_ODO_AND_TRUEFLOW_FROM_BIASEDFLOW_ERRORTERM_B_WITH_RCM_H
