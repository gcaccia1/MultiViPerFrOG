

#ifndef CORE_PREPROCESSING_PREPROCESSING_H
#define CORE_PREPROCESSING_PREPROCESSING_H

#include <deque>
#include <iostream>
#include "multiviperfrog/config/ceres_config.h"
#include "multiviperfrog/config/optimization_data.h"
#include "multiviperfrog/core/workflow_manager.h"
#include "multiviperfrog/utils/data_conversions.h"
#include "multiviperfrog/utils/preprocessing_utils.h"

namespace core {
namespace preprocessing {

void preprocessRosData(const config::RosToPreprocessingData& input_data, const config::ExperimentConfig& exp_config,
                       config::PreprocessingToOptimizationData* output_data, int camera_id);

void preprocessRosDataBuffer(const std::deque<config::RosToPreprocessingData>& input_buffer, const config::ExperimentConfig& exp_config,
                             config::PreprocessingToOptimizationData* output_data, int camera_id);

void computeCam2Cam(config::PreprocessingToOptimizationData* data, const config::ExperimentConfig& exp_config, int cam_id_self,
                    int cam_id_other, bool estimate_from_points = false, bool compute_from_poses = true);

void computeOdometryRigidClosedForm(const config::PreprocessingToOptimizationData& data, const config::ExperimentConfig& exp_config,
                                    config::OptimizationToPostprocessingData* output_data, int camera_id, bool use_horn = false,
                                    bool use_ransac = false);

void computeOdometryRigidIterative(const config::PreprocessingToOptimizationData& data, const config::ExperimentConfig& exp_config,
                                   config::OptimizationToPostprocessingData* output_data, int camera_id);

void downsamplePointClouds(config::PreprocessingToOptimizationData& data, const config::ExperimentConfig& exp_config, int camera_id);

void indexSamplingOverlapRegion(core::WorkFlowManager* manager, config::PreprocessingToOptimizationData& data,
                                const config::ExperimentConfig& exp_config, int camera_id_self, int camera_id_other);

void getOverlapIndices(core::WorkFlowManager* manager, config::PreprocessingToOptimizationData& data,
                       const config::ExperimentConfig& exp_config, int camera_id_self, int camera_id_other);

void extractOverlapPoints(config::PreprocessingToOptimizationData& data, config::ExperimentConfig& exp_config, int camera_id_self,
                          int camera_id_other);

void downsampleOverlapRegion(config::PreprocessingToOptimizationData& data, const config::ExperimentConfig& exp_config, int camera_id_self,
                             int camera_id_other);

}  // namespace preprocessing
}  // namespace core

#endif  // CORE_PREPROCESSING_PREPROCESSING_H
