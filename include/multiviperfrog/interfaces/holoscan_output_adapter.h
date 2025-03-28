

#ifndef INTERFACES_HOLOSCAN_OUTPUT_ADAPTER_H
#define INTERFACES_HOLOSCAN_OUTPUT_ADAPTER_H

#include "multiviperfrog/config/experiment_config.h"
#include "multiviperfrog/interfaces/output_interface.h"

// Include Holoscan headers if available

namespace interfaces {

class HoloscanOutputAdapter : public OutputInterface<config::PostprocessingToHoloscanData, config::ExperimentConfig> {
 public:
  HoloscanOutputAdapter();
  ~HoloscanOutputAdapter() override;

  bool initialize() override;
  bool sendData(const config::PostprocessingToHoloscanData& data, const config::ExperimentConfig& exp_config) override;

 private:
  // Holoscan-specific members
  // Ensure proper resource management
};

}  // namespace interfaces

#endif  // INTERFACES_HOLOSCAN_OUTPUT_ADAPTER_H
