

#ifndef CORE_OPTIMIZATION_COST_FUNCTIONS_TF_AND_STATICLABELS_FROM_CORRESPONDENCES_ERRORTERM_EXPLICIT_SF_H
#define CORE_OPTIMIZATION_COST_FUNCTIONS_TF_AND_STATICLABELS_FROM_CORRESPONDENCES_ERRORTERM_EXPLICIT_SF_H

#include <utility>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/autodiff_cost_function.h"

namespace core::optimization::cost_functions {

class TfAndStaticLabelsFromCorrespondencesErrorTermExplicitSF {
 public:
  TfAndStaticLabelsFromCorrespondencesErrorTermExplicitSF(
      const Eigen::Vector3d& Csource__t__Csource_P0_measured,  // Point at time 0 in the camera frame at time 0
      const Eigen::Vector3d& Ctarget__t__Ctarget_P1_measured,  // Point at time 1 in the camera frame at time 1
      // NOTE the variable name will be abused if this is used between cameras, in that case it should be
      // Ctarget__t__Ctarget_P0_measured, but since it is the case of de-sync, it should be called P0, but it is
      // actually not P0, but also not P1. To keep convention in that case the function will receive an input called P0.
      // add a vector of 2 doubles as regularization weights
      const double lambda_deformation,     // Deformation weight
      const double lambda_regularization)  // Regularization weight
      : Csource__t__Csource_P0_measured_(Csource__t__Csource_P0_measured),
        Ctarget__t__Ctarget_P1_measured_(Ctarget__t__Ctarget_P1_measured),
        lambda_deformation_(lambda_deformation),
        lambda_regularization_(lambda_regularization) {}

  // NOTE: Ctarget__t__Ctarget_P1_measured can be a relative scene flow (dense input)
  // or a measured point at time 1 feature matched with point at time 0 (sparse input)
  template <typename T>
  bool operator()(const T* const Csource__t__Csource_Ctarget_ptr,  // Camera translation between t0 and t1
                  const T* const Csource__q__Ctarget_ptr,          // Camera rotation between t0 and t1
                  const T* const Csource__t__P0_P1_ptr,            // Csource_SF
                  const T* const label_ptr,                        // Binary label for the point (static or deforming)
                  T* residuals_ptr) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> Csource__t__Csource_Ctarget(Csource__t__Csource_Ctarget_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> Csource__q__Ctarget(Csource__q__Ctarget_ptr);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> Csource__t__P0_P1(Csource__t__P0_P1_ptr);
    T s = label_ptr[0];

    // Transform the flowed point from camera frame at t1 to camera frame at t0
    Eigen::Matrix<T, 3, 1> Csource__t__Csource_P1 =
        Csource__t__Csource_Ctarget + Csource__q__Ctarget * Ctarget__t__Ctarget_P1_measured_.template cast<T>();

    // Calculate the absolute error between the rigidly transformed flowed point and the measured point at t0
    Eigen::Matrix<T, 3, 1> Csource__t__P0_P1_derived =
        (Csource__t__Csource_P1 - Csource__t__Csource_P0_measured_.template cast<T>());
    Eigen::Matrix<T, 3, 1> Csource__t__P0_P1_derived_absolute =
        (Csource__t__Csource_P1 - Csource__t__Csource_P0_measured_.template cast<T>()).cwiseAbs();

    // NOTE: deformation can be between frames for odometry for a camera or between cameras in case of de-sync.
    // Residual is weighted by the label s [0 1] (taken from eq. 8,9 in https://arxiv.org/pdf/1904.08242)
    residuals_ptr[0] = lambda_deformation_ * (s * Csource__t__P0_P1_derived_absolute[0]) - lambda_regularization_ * log(s);
    residuals_ptr[1] = lambda_deformation_ * (s * Csource__t__P0_P1_derived_absolute[1]) - lambda_regularization_ * log(s);
    residuals_ptr[2] = lambda_deformation_ * (s * Csource__t__P0_P1_derived_absolute[2]) - lambda_regularization_ * log(s);
    residuals_ptr[3] = Csource__t__P0_P1[0] - Csource__t__P0_P1_derived[0];
    residuals_ptr[4] = Csource__t__P0_P1[1] - Csource__t__P0_P1_derived[1];
    residuals_ptr[5] = Csource__t__P0_P1[2] - Csource__t__P0_P1_derived[2];
    // TODO-NOTE: if  lambda_deformation_ = 0, all labels will tend to 1 (fully rigid)
    //           if lambda_regularization = 0, all labels will tend to 0 (fully deforming)
    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d& Csource__t__Csource_P0_measured,
                                     const Eigen::Vector3d& Ctarget__t__Ctarget_P1_measured, double lambda_deformation_,
                                     double lambda_regularization_) {
    return new ceres::AutoDiffCostFunction<TfAndStaticLabelsFromCorrespondencesErrorTermExplicitSF, 3, 3, 4, 3, 1>(
        new TfAndStaticLabelsFromCorrespondencesErrorTermExplicitSF(Csource__t__Csource_P0_measured, Ctarget__t__Ctarget_P1_measured,
                                                                    lambda_deformation_, lambda_regularization_));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const Eigen::Vector3d Csource__t__Csource_P0_measured_;  // Point at time 0 in the camera frame
  const Eigen::Vector3d Ctarget__t__Ctarget_P1_measured_;  // Scene flow in the camera frame
  const double lambda_deformation_;                        // error weight
  const double lambda_regularization_;                     // regularization weight
};

}  // namespace core::optimization::cost_functions

#endif  // CORE_OPTIMIZATION_COST_FUNCTIONS_TF_AND_STATICLABELS_FROM_CORRESPONDENCES_ERRORTERM_EXPLICIT_SF_H
