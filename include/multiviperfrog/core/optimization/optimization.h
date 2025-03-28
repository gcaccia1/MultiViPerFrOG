

#ifndef CORE_OPTIMIZATION_OPTIMIZATION_H
#define CORE_OPTIMIZATION_OPTIMIZATION_H

#include "ceres/ceres.h"
#include "multiviperfrog/config/ceres_config.h"
#include "multiviperfrog/config/experiment_config.h"
#include "multiviperfrog/config/optimization_data.h"

namespace core {
namespace optimization {

double runOptimizationMICCAI(config::OptimizationData& input, config::CeresConfig& ceres_config, config::ExperimentConfig& exp_config);

double runOptimizationSingleOdometry(config::OptimizationData& input, config::CeresConfig& ceres_config,
                                     config::ExperimentConfig& exp_config);

double runOptimizationTwoCameras(config::OptimizationData& input, config::CeresConfig& ceres_config, config::ExperimentConfig& exp_config);

double SolveOptimizationProblem(ceres::Problem* problem, config::CeresConfig& ceres_config);

}  // namespace optimization
}  // namespace core

#endif  // CORE_OPTIMIZATION_OPTIMIZATION_H
