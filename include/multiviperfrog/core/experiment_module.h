

#ifndef CORE_EXPERIMENT_MODULE_H
#define CORE_EXPERIMENT_MODULE_H

#include <open3d/Open3D.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "glog/logging.h"
#include "multiviperfrog/config/ceres_config.h"
#include "multiviperfrog/config/experiment_config.h"
#include "multiviperfrog/core/optimization/cost_functions/RCM_constraint_errorterm.h"
#include "multiviperfrog/core/optimization/cost_functions/SF_knowledge_errorterm.h"
#include "multiviperfrog/core/optimization/cost_functions/Tf_and_staticlabels_from_correspondences_errorterm.h"
#include "multiviperfrog/core/optimization/cost_functions/Tf_measured_errorterm.h"
#include "multiviperfrog/core/optimization/cost_functions/data_association_errorterm.h"
#include "multiviperfrog/core/optimization/cost_functions/odo_and_trueflow_from_biasedflow_errorterm_a.h"
#include "multiviperfrog/core/optimization/cost_functions/odo_and_trueflow_from_biasedflow_errorterm_b.h"
#include "multiviperfrog/core/optimization/cost_functions/odo_from_rigidTf1_errorterm.h"
#include "multiviperfrog/core/optimization/optimization.h"
#include "multiviperfrog/core/optimization/problems.h"
#include "multiviperfrog/interfaces/input_interface.h"
#include "multiviperfrog/interfaces/output_interface.h"
#include "multiviperfrog/utils/optimization_utils.h"
#include "multiviperfrog/utils/preprocessing_utils.h"

namespace core {
// TODO: implement the runExperiment function correctly
void runExperimentMICCAI(config::CeresConfig& base_config, config::ExperimentConfig& exp_config,
                         config::PreprocessingToOptimizationData& data_from_preprocessing);

void setupExperiment(config::CeresConfig* config, config::ExperimentConfig* exp_config);

void initializeOptimizationTwoCamerasParameters(config::PreprocessingToOptimizationData& data_from_preprocessing,
                                                const config::ExperimentConfig& exp_config);

void runExperimentSingleOdometry(config::CeresConfig& ceres_config, config::ExperimentConfig& exp_config,
                                 const config::PreprocessingToOptimizationData& data_from_preprocessing,
                                 config::OptimizationToPostprocessingData& data_to_postprocessing, int camera_id);

void runExperimentTwoCameras(config::CeresConfig& ceres_config, config::ExperimentConfig& exp_config,
                             config::PreprocessingToOptimizationData& data_from_preprocessing,
                             config::OptimizationToPostprocessingData& data_to_postprocessing, int camera_id_self, int camera_id_other);

}  // namespace core

#endif  // CORE_EXPERIMENT_MODULE_H
