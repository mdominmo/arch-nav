#ifndef NAVIGATION__CORE__CONTROLLER__VEHICLE_COMMAND_HPP_
#define NAVIGATION__CORE__CONTROLLER__VEHICLE_COMMAND_HPP_

#include "dispatchers/i_command_dispatcher.hpp"

namespace arch_nav::controller {

class VehicleCommand {
 public:
  virtual void execute(dispatchers::ICommandDispatcher& dispatcher) = 0;
  virtual void cancel(dispatchers::ICommandDispatcher& dispatcher) { dispatcher.stop(); }
  virtual ~VehicleCommand() = default;
};

}  // namespace arch_nav::controller

#endif  // NAVIGATION__CORE__CONTROLLER__VEHICLE_COMMAND_HPP_
