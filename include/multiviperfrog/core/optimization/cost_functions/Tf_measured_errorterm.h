

#ifndef CORE_OPTIMIZATION_COST_FUNCTIONS_TF_MEASURED_ERRORTERM_H
#define CORE_OPTIMIZATION_COST_FUNCTIONS_TF_MEASURED_ERRORTERM_H
#include <utility>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/autodiff_cost_function.h"

// NOTE frames have been writte in the generic form Csource__t__Csource_Ctarget and Csource__q__Ctarget
// before they would refer to C1__t__C1_C0 and C1__q__C0, but now its generic
namespace core::optimization::cost_functions {
class TfMeasuredErrorterm {
 public:
  TfMeasuredErrorterm(const Eigen::Vector3d Csource__t__Csource_Ctarget_measured,
                      const Eigen::Quaternion<double> Csource__q__Ctarget_measured)
      : Csource__t__Csource_Ctarget_measured_(std::move(Csource__t__Csource_Ctarget_measured)),
        Csource__q__Ctarget_measured_(std::move(Csource__q__Ctarget_measured)) {}
  // TODO check difference between using std::move vs. declaring as const reference
  template <typename T>
  bool operator()(const T* const Csource__t__Csource_Ctarget_ptr,  // x1 or x2 or x5
                  const T* const Csource__q__Ctarget_ptr,          // x1 or x2 or x5
                  T* residuals_ptr) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> Csource__t__Csource_Ctarget(Csource__t__Csource_Ctarget_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> Csource__q__Ctarget(Csource__q__Ctarget_ptr);
    // "For the orientation error, we will use the standard multiplicative error
    // 2.0*vec(q_measured*inv(q_est)) as the residuals."
    Eigen::Quaternion<T> Csource__q__Ctarget_error = Csource__q__Ctarget_measured_.template cast<T>() * Csource__q__Ctarget.inverse();
    Csource__q__Ctarget_error.normalize();
    // TODO: CHECK should this not be computed as quaternion difference in the tangent space?
    Eigen::Matrix<T, 3, 1> Csource__q__Ctarget_error_vec = T(2.0) * Csource__q__Ctarget_error.vec();

    // define residuals TODO: compress notation to single line
    residuals_ptr[0] = Csource__t__Csource_Ctarget_measured_[0] - Csource__t__Csource_Ctarget[0];
    residuals_ptr[1] = Csource__t__Csource_Ctarget_measured_[1] - Csource__t__Csource_Ctarget[1];
    residuals_ptr[2] = Csource__t__Csource_Ctarget_measured_[2] - Csource__t__Csource_Ctarget[2];
    residuals_ptr[3] = Csource__q__Ctarget_error_vec[0];
    residuals_ptr[4] = Csource__q__Ctarget_error_vec[1];
    residuals_ptr[5] = Csource__q__Ctarget_error_vec[2];
    //            residuals_ptr[6] = Csource__q__Ctarget_error.w(); //TODO: check if this is needed
    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d& Csource__t__Csource_Ctarget_measured,
                                     const Eigen::Quaternion<double>& Csource__q__Ctarget_measured) {
    return new ceres::AutoDiffCostFunction<TfMeasuredErrorterm, 6, 3, 4>(Csource__t__Csource_Ctarget_measured,
                                                                         Csource__q__Ctarget_measured);
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // TODO check what this does
      private : const Eigen::Vector3d Csource__t__Csource_Ctarget_measured_;
  const Eigen::Quaternion<double> Csource__q__Ctarget_measured_;
};
}  // namespace core::optimization::cost_functions

#endif  // CORE_OPTIMIZATION_COST_FUNCTIONS_TF_MEASURED_ERRORTERM_H
