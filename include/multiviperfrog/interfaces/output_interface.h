
#ifndef INTERFACES_OUTPUT_INTERFACE_H
#define INTERFACES_OUTPUT_INTERFACE_H

#include <type_traits>
#include "multiviperfrog/config/optimization_data.h"

namespace interfaces {
template <typename T, typename U>
class OutputInterface {
 public:
  virtual ~OutputInterface() = default;
  virtual bool sendData(const T& data, const U& exp_config) = 0;
  // Override base class methods if applicable
  virtual bool initialize() {
    // Default implementation, or throw if not applicable
    return true;
  }
  virtual bool isDataReady() const {
    // Example implementation
    return true;
  }
};

}  // namespace interfaces

#endif  // INTERFACES_OUTPUT_INTERFACE_H
