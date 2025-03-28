

#ifndef CORE_OPTIMIZATION_COST_FUNCTIONS_RCM_CONSTRAINT_ERRORTERM_H
#define CORE_OPTIMIZATION_COST_FUNCTIONS_RCM_CONSTRAINT_ERRORTERM_H
#include <utility>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/autodiff_cost_function.h"

// NOTE frames have been written in the generic form Csource__t__Csource_Ctarget and Csource__q__Ctarget

namespace core::optimization::cost_functions {
class RCMConstraintErrorterm {
 public:
  RCMConstraintErrorterm() {}
  template <typename T>
  bool operator()(const T* const Csource__t__Csource_Ctarget_ptr, const T* const Csource__q__Ctarget_ptr, T* residuals_ptr) const {
    // RCM constraint is enforced as r2,3⋅t1−r1,2⋅t2=0 where rj,k are elements of the rotation matrix Rj,k
    // and tj are the translation vectors of the relative transformation
    // written in quaternion form is (q2q3−q0q1)⋅t1−(q1q2−q0q3)⋅t2 = 0
    // where q0 is the scalar part of the quaternion and q1, q2, q3 are the vector part
    // TODO check convetion of quaternion order (w,x,y,z) or (x,y,z,w)

    // Map translation vector components
    const T& t1 = Csource__t__Csource_Ctarget_ptr[0];
    const T& t2 = Csource__t__Csource_Ctarget_ptr[1];
    // (t3 is not needed for this constraint)

    // Map quaternion components
    const T& q0 = Csource__q__Ctarget_ptr[0];  // scalar part (w)
    const T& q1 = Csource__q__Ctarget_ptr[1];  // x
    const T& q2 = Csource__q__Ctarget_ptr[2];  // y
    const T& q3 = Csource__q__Ctarget_ptr[3];  // z

    // Implement the constraint (q2q3 − q0q1)⋅t1 − (q1q2 − q0q3)⋅t2 from doi: 10.1109/LRA.2018.2809617
    residuals_ptr[0] = (q2 * q3 - q0 * q1) * t1 - (q1 * q2 - q0 * q3) * t2;
    return true;
  }

  static ceres::CostFunction* Create() { return new ceres::AutoDiffCostFunction<RCMConstraintErrorterm, 1, 3, 4>(); }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // TODO check what this does
      private:
};
}  // namespace core::optimization::cost_functions

#endif  // CORE_OPTIMIZATION_COST_FUNCTIONS_RCM_CONSTRAINT_ERRORTERM_H
