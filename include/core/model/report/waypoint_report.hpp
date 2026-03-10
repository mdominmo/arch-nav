#ifndef NAVIGATION__CORE__MODEL__REPORT__WAYPOINT_REPORT_HPP_
#define NAVIGATION__CORE__MODEL__REPORT__WAYPOINT_REPORT_HPP_

#include <cstddef>

#include "core/model/report/operation_report.hpp"

namespace arch_nav::report {

class WaypointReport : public OperationReport {
 public:
  explicit WaypointReport(std::size_t total_waypoints)
      : total_waypoints_(total_waypoints) {}

  std::size_t total_waypoints()     const { return total_waypoints_; }
  std::size_t completed_waypoints() const { return completed_waypoints_; }

  void increment_completed() { ++completed_waypoints_; }

 private:
  std::size_t total_waypoints_{0};
  std::size_t completed_waypoints_{0};
};

}  // namespace arch_nav::report

#endif  // NAVIGATION__CORE__MODEL__REPORT__WAYPOINT_REPORT_HPP_
