

#ifndef CORE_POSTPROCESSING_POSTPROCESSING_H
#define CORE_POSTPROCESSING_POSTPROCESSING_H

#include "multiviperfrog/config/experiment_config.h"
#include "multiviperfrog/config/optimization_data.h"
#include "multiviperfrog/utils/data_conversions.h"

namespace core {
namespace postprocessing {
void postprocessOptimizationDataForRos(const config::PostprocessingData& input_data, config::PostprocessingToRosData& output_data,
                                       const config::ExperimentConfig& exp_config, int camera_id);

void updateOdometry(const config::OptimizationToPostprocessingData& input_data, config::PostprocessingOdometryBuffer* odometry_buffer,
                    int camera_id);

void initializeOdometry(const config::PreprocessingToOptimizationData& input_data, config::PostprocessingOdometryBuffer* odometry_buffer,
                        int camera_id);

void saveOdometry(const config::PostprocessingOdometryBuffer& odometry_buffer, config::PostprocessingData* postprocessing_data,
                  int camera_id);

void saveEigenIsometryStampedToFreiburgStyle(const config::PostprocessingData& input_data, const int camera_id,
                                             const int frame_index_counter, const config::ExperimentConfig& exp_config);

}  // namespace postprocessing
}  // namespace core
#endif  // CORE_POSTPROCESSING_POSTPROCESSING_H
