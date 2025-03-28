

#ifndef CORE_OPTIMIZATION_COST_FUNCTIONS_ODO_AND_TRUEFLOW_FROM_BIASEDFLOW_ERRORTERM_A_H
#define CORE_OPTIMIZATION_COST_FUNCTIONS_ODO_AND_TRUEFLOW_FROM_BIASEDFLOW_ERRORTERM_A_H

#include <utility>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/autodiff_cost_function.h"
namespace core::optimization::cost_functions {
//    template <typename T>
class OdoAndTrueflowFromBiasedflowErrortermA {
 public:
  OdoAndTrueflowFromBiasedflowErrortermA(Eigen::Vector3d Cb0__t__Cb0_P0_measured, Eigen::Vector3d Ca1__t__Ca1_P1_measured)
      : Cb0__t__Cb0_P0_measured_(std::move(Cb0__t__Cb0_P0_measured)), Ca1__t__Ca1_P1_measured_(std::move(Ca1__t__Ca1_P1_measured)) {}
  // TODO check difference between using std::move vs. declaring as const reference
  template <typename T>
  bool operator()(const T* const Ca0__t__Ca0_Ca1_ptr,  // x1
                  const T* const Ca0__q__Ca1_ptr,      // x1
                  const T* const Cb0__t__P0_P1_ptr,    // Cb0_SF
                  const T* const Ca0__t__Ca0_Cb0_ptr,  // x5
                  const T* const Ca0__q__Cb0_ptr,      // x5
                  T* residuals_ptr) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> Ca0__t__Ca0_Ca1(Ca0__t__Ca0_Ca1_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> Ca0__q__Ca1(Ca0__q__Ca1_ptr);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> Cb0__t__P0_P1(Cb0__t__P0_P1_ptr);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> Ca0__t__Ca0_Cb0(Ca0__t__Ca0_Cb0_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> Ca0__q__Cb0(Ca0__q__Cb0_ptr);

    // m2 = inv(x1) * x3 becomes m2 = inv(x1) * x5 * x4 (using x3 = x5 * x4).
    Eigen::Quaternion<T> Ca1__q__Ca0 = Ca0__q__Ca1.inverse();
    Ca1__q__Ca0.normalize();
    Eigen::Matrix<T, 3, 1> Ca1__t__Ca1_Cb0 = Ca1__q__Ca0 * Ca0__t__Ca0_Cb0 - Ca1__q__Ca0 * Ca0__t__Ca0_Ca1;
    Eigen::Quaternion<T> Ca1__q__Cb0 = Ca1__q__Ca0 * Ca0__q__Cb0;  // NOTE: here used right multiplication
    Ca1__q__Cb0.normalize();
    Eigen::Matrix<T, 3, 1> Ca1__t__Ca1_P1_estimated = Ca1__t__Ca1_Cb0 + Ca1__q__Cb0 * (Cb0__t__Cb0_P0_measured_ + Cb0__t__P0_P1);
    // NOTE: on quaternion multiplication (rot1 then rot2) = rot2 * rot1, if both in world frame, or rot1 * rot2 if
    // second one is in relative frame of first one. This is the case here, as Ca0__q__Cb0 is in Ca0 frame.
    residuals_ptr[0] = (Ca1__t__Ca1_P1_measured_[0] - Ca1__t__Ca1_P1_estimated[0]);
    residuals_ptr[1] = (Ca1__t__Ca1_P1_measured_[1] - Ca1__t__Ca1_P1_estimated[1]);
    residuals_ptr[2] = (Ca1__t__Ca1_P1_measured_[2] - Ca1__t__Ca1_P1_estimated[2]);
    return true;
  }
  static ceres::CostFunction* Create(const Eigen::Vector3d& Cb0__t__Cb0_P0_measured, const Eigen::Vector3d& Ca1__t__Ca1_P1_measured) {
    return new ceres::AutoDiffCostFunction<OdoAndTrueflowFromBiasedflowErrortermA, 3, 3, 4, 3, 3, 4>(Cb0__t__Cb0_P0_measured,
                                                                                                     Ca1__t__Ca1_P1_measured);
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  const Eigen::Vector3d Cb0__t__Cb0_P0_measured_;
  const Eigen::Vector3d Ca1__t__Ca1_P1_measured_;
};
}  // namespace core::optimization::cost_functions

#endif  // CORE_OPTIMIZATION_COST_FUNCTIONS_ODO_AND_TRUEFLOW_FROM_BIASEDFLOW_ERRORTERM_A_H
