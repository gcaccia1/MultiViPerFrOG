

#ifndef CORE_OPTIMIZATION_COST_FUNCTIONS_ODO_FROM_RIGIDTF1_ERRORTERM_H
#define CORE_OPTIMIZATION_COST_FUNCTIONS_ODO_FROM_RIGIDTF1_ERRORTERM_H

#include <utility>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/autodiff_cost_function.h"
namespace core::optimization::cost_functions {
class OdoFromRigidTf1Errorterm {
 public:
  OdoFromRigidTf1Errorterm(Eigen::Vector3d Ca1__t__Ca1_Cb1_measured,
                           Eigen::Quaternion<double> Ca1__q__Cb1_measured)  // measure m1 (or x6)
      : Ca1__t__Ca1_Cb1_measured_(std::move(Ca1__t__Ca1_Cb1_measured)), Ca1__q__Cb1_measured_(std::move(Ca1__q__Cb1_measured)) {}
  // TODO check difference between using std::move vs. declaring as const reference
  template <typename T>
  bool operator()(const T* const Ca0__t__Ca0_Ca1_ptr,  // x1
                  const T* const Ca0__q__Ca1_ptr,      // x1
                  const T* const Ca0__t__Ca0_Cb0_ptr,  // x5
                  const T* const Ca0__q__Cb0_ptr,      // x5
                  const T* const Cb0__t__Cb0_Cb1_ptr,  // x2
                  const T* const Cb0__q__Cb1_ptr,      // x2
                  T* residuals_ptr) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> Ca0__t__Ca0_Ca1(Ca0__t__Ca0_Ca1_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> Ca0__q__Ca1(Ca0__q__Ca1_ptr);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> Ca0__t__Ca0_Cb0(Ca0__t__Ca0_Cb0_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> Ca0__q__Cb0(Ca0__q__Cb0_ptr);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> Cb0__t__Cb0_Cb1(Cb0__t__Cb0_Cb1_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> Cb0__q__Cb1(Cb0__q__Cb1_ptr);

    // define the following concatenated hom transformations
    // Ca1__T__Cb1 = Ca1__T__Ca0 * Ca0__T__Cb0 * Cb0__T__Cb1, but in quaternion and translation
    Eigen::Quaternion<T> Ca1__q_Ca0 = Ca0__q__Ca1.inverse();
    Ca1__q_Ca0.normalize();
    Eigen::Quaternion<T> Ca1__q__Cb0 = Ca1__q_Ca0 * Ca0__q__Cb0;
    Ca1__q__Cb0.normalize();
    Eigen::Matrix<T, 3, 1> Ca1__t__Ca1_Cb1_estimated =
        Ca1__q_Ca0 * Ca0__t__Ca0_Cb0 - Ca1__q_Ca0 * Ca0__t__Ca0_Ca1 + Ca1__q__Cb0 * Cb0__t__Cb0_Cb1;
    Eigen::Quaternion<T> Ca1__q__Cb1_estimated = Ca1__q_Ca0 * Ca0__q__Cb0 * Cb0__q__Cb1;
    Ca1__q__Cb1_estimated.normalize();
    // from ceres pose_graph_3d_error_term.h example
    // "For the orientation error, we will use the standard multiplicative error
    // 2.0*vec(q_measured*inv(q_est)) as the residuals."
    Eigen::Quaternion<T> Ca1__q__Cb1_error = Ca1__q__Cb1_measured_.template cast<T>() * Ca1__q__Cb1_estimated.inverse();
    Ca1__q__Cb1_error.normalize();
    Eigen::Matrix<T, 3, 1> Ca1__q__Cb1_error_vec = T(2.0) * Ca1__q__Cb1_error.vec();
    // define residuals TODO: compress notation to single line
    residuals_ptr[0] = Ca1__t__Ca1_Cb1_measured_[0] - Ca1__t__Ca1_Cb1_estimated[0];
    residuals_ptr[1] = Ca1__t__Ca1_Cb1_measured_[1] - Ca1__t__Ca1_Cb1_estimated[1];
    residuals_ptr[2] = Ca1__t__Ca1_Cb1_measured_[2] - Ca1__t__Ca1_Cb1_estimated[2];
    residuals_ptr[3] = Ca1__q__Cb1_error_vec[0];
    residuals_ptr[4] = Ca1__q__Cb1_error_vec[1];
    residuals_ptr[5] = Ca1__q__Cb1_error_vec[2];
    //            residuals_ptr[6] = Ca1__q__Cb1_error.w(); //TODO: check if this is needed
    return true;
  }
  static ceres::CostFunction* Create(const Eigen::Vector3d& Ca1__t__Ca1_Cb1_measured,
                                     const Eigen::Quaternion<double>& Ca1__q__Cb1_measured) {
    return new ceres::AutoDiffCostFunction<OdoFromRigidTf1Errorterm, 6, 3, 4, 3, 4, 3, 4>(Ca1__t__Ca1_Cb1_measured, Ca1__q__Cb1_measured);
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // TODO check what this does
      private : const Eigen::Vector3d Ca1__t__Ca1_Cb1_measured_;
  const Eigen::Quaternion<double> Ca1__q__Cb1_measured_;
};
}  // namespace core::optimization::cost_functions

#endif  // CORE_OPTIMIZATION_COST_FUNCTIONS_ODO_FROM_RIGIDTF1_ERRORTERM_H
