

#ifndef INTERFACES_OFFLINE_INPUT_ADAPTER_H
#define INTERFACES_OFFLINE_INPUT_ADAPTER_H

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "multiviperfrog/interfaces/input_interface.h"

namespace interfaces {

class OfflineInputAdapter : public InputInterface<config::PreprocessingToOptimizationData> {
 public:
  OfflineInputAdapter();

  ~OfflineInputAdapter() override;

  bool initialize() override;

  bool getData(config::PreprocessingToOptimizationData& data) override;

  bool isDataAvailable() const override;

 private:
  //        bool data_received_;
  //        config::OptimizationData buffer_data_;
  Eigen::Transform<double, 3, Eigen::Isometry> loadTransformFromJson(const std::string& filename);
};
}  // namespace interfaces

#endif  // INTERFACES_OFFLINE_INPUT_ADAPTER_H
