

#ifndef INTERFACES_HOLOSCAN_INPUT_ADAPTER_H
#define INTERFACES_HOLOSCAN_INPUT_ADAPTER_H

#include "multiviperfrog/interfaces/input_interface.h"

// Include Holoscan headers if available

namespace interfaces {

class HoloscanInputAdapter : public InputInterface<config::HoloscanToPreprocessingData> {
 public:
  HoloscanInputAdapter();
  ~HoloscanInputAdapter() override;

  bool initialize() override;
  bool getData(config::HoloscanToPreprocessingData& data) override;
  bool isDataAvailable() const override;

 private:
  // Holoscan-specific members
  // Ensure proper initialization and resource management
};

}  // namespace interfaces

#endif  // INTERFACES_HOLOSCAN_INPUT_ADAPTER_H
