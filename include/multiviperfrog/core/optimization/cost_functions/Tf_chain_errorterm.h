

#ifndef CORE_OPTIMIZATION_COST_FUNCTIONS_TF_CHAIN_ERRORTERM_H
#define CORE_OPTIMIZATION_COST_FUNCTIONS_TF_CHAIN_ERRORTERM_H

#include <utility>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/autodiff_cost_function.h"

namespace core::optimization::cost_functions {
class TfChainErrorterm {
 public:
  TfChainErrorterm() = default;

  template <typename T>
  bool operator()(const T* const Ca0__t__Ca0_Ca1_ptr, const T* const Ca0__q__Ca1_ptr, const T* const Ca0__t__Ca0_Cb0_ptr,
                  const T* const Ca0__q__Cb0_ptr, const T* const Cb0__t__Cb0_Cb1_ptr, const T* const Cb0__q__Cb1_ptr,
                  const T* const Ca1__t__Ca1_Cb1_ptr, const T* const Ca1__q__Cb1_ptr, T* residuals_ptr) const {
    // Mapping the inputs to Eigen types
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> Ca0__t__Ca0_Ca1(Ca0__t__Ca0_Ca1_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> Ca0__q__Ca1(Ca0__q__Ca1_ptr);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> Ca0__t__Ca0_Cb0(Ca0__t__Ca0_Cb0_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> Ca0__q__Cb0(Ca0__q__Cb0_ptr);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> Cb0__t__Cb0_Cb1(Cb0__t__Cb0_Cb1_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> Cb0__q__Cb1(Cb0__q__Cb1_ptr);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> Ca1__t__Ca1_Cb1(Ca1__t__Ca1_Cb1_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> Ca1__q__Cb1(Ca1__q__Cb1_ptr);

    // Compute intermediate transformations
    Eigen::Quaternion<T> Ca1__q_Ca0 = Ca0__q__Ca1.inverse();
    Ca1__q_Ca0.normalize();
    Eigen::Quaternion<T> Ca1__q__Cb0 = Ca1__q_Ca0 * Ca0__q__Cb0;
    Ca1__q__Cb0.normalize();

    // Compute the estimated transformation for the chain
    Eigen::Matrix<T, 3, 1> Ca1__t__Ca1_Cb1_estimated =
        Ca1__q_Ca0 * Ca0__t__Ca0_Cb0 - Ca1__q_Ca0 * Ca0__t__Ca0_Ca1 + Ca1__q__Cb0 * Cb0__t__Cb0_Cb1;
    Eigen::Quaternion<T> Ca1__q__Cb1_estimated = Ca1__q_Ca0 * Ca0__q__Cb0 * Cb0__q__Cb1;
    Ca1__q__Cb1_estimated.normalize();

    // Calculate the translation and rotation error with respect to the remaining parameter of the chain
    Eigen::Matrix<T, 3, 1> translation_error = Ca1__t__Ca1_Cb1 - Ca1__t__Ca1_Cb1_estimated;
    Eigen::Quaternion<T> rotation_error = Ca1__q__Cb1 * Ca1__q__Cb1_estimated.inverse();
    rotation_error.normalize();
    Eigen::Matrix<T, 3, 1> rotation_error_vec = T(2.0) * rotation_error.vec();

    // Assign residuals
    residuals_ptr[0] = translation_error[0];
    residuals_ptr[1] = translation_error[1];
    residuals_ptr[2] = translation_error[2];
    residuals_ptr[3] = rotation_error_vec[0];
    residuals_ptr[4] = rotation_error_vec[1];
    residuals_ptr[5] = rotation_error_vec[2];
    return true;
  }

  static ceres::CostFunction* Create() {
    return new ceres::AutoDiffCostFunction<TfChainErrorterm, 6, 3, 4, 3, 4, 3, 4, 3, 4>(new TfChainErrorterm());
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace core::optimization::cost_functions

#endif  // CORE_OPTIMIZATION_COST_FUNCTIONS_TF_CHAIN_ERRORTERM_H
