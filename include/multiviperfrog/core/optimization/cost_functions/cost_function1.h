

#ifndef CORE_OPTIMIZATION_COST_FUNCTIONS_COST_FUNCTION1_H
#define CORE_OPTIMIZATION_COST_FUNCTIONS_COST_FUNCTION1_H

#include <ceres/ceres.h>
#include <Eigen/Dense>

namespace core {
namespace optimization {
namespace cost_functions {

class CostFunction1 : public ceres::SizedCostFunction<3, 7> {
 public:
  CostFunction1(const Eigen::Vector3d& observed_point);
  virtual ~CostFunction1() = default;

  virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override;

 private:
  Eigen::Vector3d observed_point_;
};

}  // namespace cost_functions
}  // namespace optimization
}  // namespace core

#endif  // CORE_OPTIMIZATION_COST_FUNCTIONS_COST_FUNCTION1_H
