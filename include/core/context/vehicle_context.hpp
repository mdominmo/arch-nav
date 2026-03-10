#ifndef NAVIGATION__CORE__CONTEXT__VEHICLE_CONTEXT_HPP_
#define NAVIGATION__CORE__CONTEXT__VEHICLE_CONTEXT_HPP_

#include <functional>
#include <shared_mutex>

#include "core/model/vehicle/kinematics.hpp"
#include "core/model/vehicle/global_position.hpp"
#include "core/model/vehicle/behavior_subject.hpp"
#include "core/model/vehicle/vehicle_status.hpp"

namespace arch_nav::context {

using vehicle::Kinematics;
using vehicle::GlobalPosition;
using vehicle::VehicleStatus;

class VehicleContext {
 public:
  VehicleContext();

  GlobalPosition get_global_position() const;

  Kinematics get_kinematic() const;

  VehicleStatus get_vehicle_status() const;

  void update(const GlobalPosition& state);
  void update(const Kinematics& state);
  void update(const VehicleStatus& state);

  void subscribe_vehicle_status(std::function<void(const VehicleStatus&)> callback);

 private:
  mutable std::shared_mutex global_position_mutex_;
  mutable std::shared_mutex kinematic_mutex_;
  GlobalPosition global_position_;
  Kinematics kinematic_;
  vehicle::BehaviorSubject<VehicleStatus> vehicle_status_;
};

}  // namespace arch_nav::context

#endif  // NAVIGATION__CORE__CONTEXT__VEHICLE_CONTEXT_HPP_
