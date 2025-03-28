

#ifndef CORE_OPTIMIZATION_COST_FUNCTIONS_ODO_AND_TRUEFLOW_FROM_BIASEDFLOW_PLUS_LABELS_ERRORTERM_H
#define CORE_OPTIMIZATION_COST_FUNCTIONS_ODO_AND_TRUEFLOW_FROM_BIASEDFLOW_PLUS_LABELS_ERRORTERM_H

#include <utility>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/autodiff_cost_function.h"
namespace core::optimization::cost_functions {
//    template <typename T>
class OdoAndTrueflowFromBiasedflowErrortermPlusLabels {
 public:
  OdoAndTrueflowFromBiasedflowErrortermPlusLabels(Eigen::Vector3d Csource__t__Csource_P0_measured,
                                                  Eigen::Vector3d Ctarget__t__Ctarget_P1_measured)
      : Csource__t__Csource_P0_measured_(std::move(Csource__t__Csource_P0_measured)),
        Ctarget__t__Ctarget_P1_measured_(std::move(Ctarget__t__Ctarget_P1_measured)) {}
  // TODO check difference between using std::move vs. declaring as const reference
  template <typename T>
  bool operator()(const T* const Csource__t__Csource_Ctarget_ptr,  // x2
                  const T* const Csource__q__Ctarget_ptr,          // x2
                  const T* const Csource__t__P0_P1_ptr,            // Csource_SF
                  const T* const label_ptr,                        // Binary label for the point (static or deforming)
                  const T* const labels_to_SF_scaling_ptr,          // Scaling factor for the labels w.r.t. absolute SF
                  T* residuals_ptr) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> Csource__t__Csource_Ctarget(Csource__t__Csource_Ctarget_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> Csource__q__Ctarget(Csource__q__Ctarget_ptr);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> Csource__t__P0_P1(Csource__t__P0_P1_ptr);
    T s = label_ptr[0];
    T scale = labels_to_SF_scaling_ptr[0];
    // relative SF residuals
    // here the simple transformation is applied to the point as m3 = x2 * (m5 + Csource_SF)
    Eigen::Matrix<T, 3, 1> Ctarget__t__Ctarget_P1_estimated =
        Csource__q__Ctarget.inverse() * (Csource__t__Csource_P0_measured_ + Csource__t__P0_P1 - Csource__t__Csource_Ctarget);
    residuals_ptr[0] = (Ctarget__t__Ctarget_P1_measured_[0] - Ctarget__t__Ctarget_P1_estimated[0]);
    residuals_ptr[1] = (Ctarget__t__Ctarget_P1_measured_[1] - Ctarget__t__Ctarget_P1_estimated[1]);
    residuals_ptr[2] = (Ctarget__t__Ctarget_P1_measured_[2] - Ctarget__t__Ctarget_P1_estimated[2]);

    // same definition of the deformation term as in the labeling errorterm
    // Transform the flowed point from camera frame at t1 to camera frame at t0
    Eigen::Matrix<T, 3, 1> Csource__t__Csource_P1 =
        Csource__t__Csource_Ctarget + Csource__q__Ctarget * Ctarget__t__Ctarget_P1_measured_.template cast<T>();
    // Calculate the absolute error between the rigidly transformed flowed point and the measured point at t0
    Eigen::Matrix<T, 3, 1> deformation = (Csource__t__Csource_P1 - Csource__t__Csource_P0_measured_.template cast<T>()).cwiseAbs();
    // compute the norm of the deformation
    T deformation_norm = deformation.norm();
    // this residual should make sure that there is a correspondence between the deformation magnitude and the label
    residuals_ptr[3] = deformation_norm - s * scale;

    return true;
  }
  static ceres::CostFunction* Create(const Eigen::Vector3d& Csource__t__Csource_P0_measured,
                                     const Eigen::Vector3d& Ctarget__t__Ctarget_P1_measured) {
    return new ceres::AutoDiffCostFunction<OdoAndTrueflowFromBiasedflowErrortermPlusLabels, 4, 3, 4, 3, 1, 1>(Csource__t__Csource_P0_measured,
                                                                                                        Ctarget__t__Ctarget_P1_measured);
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  const Eigen::Vector3d Csource__t__Csource_P0_measured_;
  const Eigen::Vector3d Ctarget__t__Ctarget_P1_measured_;
};
}  // namespace core::optimization::cost_functions

#endif  // CORE_OPTIMIZATION_COST_FUNCTIONS_ODO_AND_TRUEFLOW_FROM_BIASEDFLOW_ERRORTERM_B_H
