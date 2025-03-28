

#ifndef INTERFACES_INPUT_INTERFACE_H
#define INTERFACES_INPUT_INTERFACE_H

#include <type_traits>
#include "multiviperfrog/config/optimization_data.h"

namespace interfaces {
template <typename T>
class InputInterface {
 public:
  virtual ~InputInterface() = default;
  virtual bool getData(T& data) = 0;
  // Override base class methods if applicable
  virtual bool initialize() {
    // Default implementation, or throw if not applicable
    return true;
  }
  virtual bool isDataAvailable() const {
    // Example implementation
    return true;
  }
};

}  // namespace interfaces

#endif  // INTERFACES_INPUT_INTERFACE_H
