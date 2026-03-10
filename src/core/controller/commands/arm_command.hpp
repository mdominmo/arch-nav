#ifndef NAVIGATION__CORE__CONTROLLER__COMMANDS__ARM_COMMAND_HPP_
#define NAVIGATION__CORE__CONTROLLER__COMMANDS__ARM_COMMAND_HPP_

#include "core/controller/vehicle_command.hpp"

namespace arch_nav::controller {

class ArmCommand : public VehicleCommand {
 public:
  void execute(dispatchers::ICommandDispatcher& dispatcher) override {
    dispatcher.execute_arm();
  }
};

}  // namespace arch_nav::controller

#endif  // NAVIGATION__CORE__CONTROLLER__COMMANDS__ARM_COMMAND_HPP_
