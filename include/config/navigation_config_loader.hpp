#ifndef NAVIGATION_STATE_REGISTER__CONFIG__NAVIGATION_CONFIG_LOADER_HPP_
#define NAVIGATION_STATE_REGISTER__CONFIG__NAVIGATION_CONFIG_LOADER_HPP_

#include <string>

#include "config/navigation_config.hpp"

namespace arch_nav::config {

class NavigationConfigLoader {
 public:
  static NavigationConfig load_from();
  static NavigationConfig load_from(const std::string& path);
};

}  // namespace arch_nav::config

#endif  // NAVIGATION_STATE_REGISTER__CONFIG__NAVIGATION_CONFIG_LOADER_HPP_
