#ifndef NAVIGATION__CORE__MODEL__VEHICLE__BEHAVIOR_SUBJECT_HPP_
#define NAVIGATION__CORE__MODEL__VEHICLE__BEHAVIOR_SUBJECT_HPP_

#include <functional>
#include <mutex>
#include <shared_mutex>
#include <vector>

namespace arch_nav::vehicle {

template<typename T>
class BehaviorSubject {
 public:
  explicit BehaviorSubject(T initial = T{}) : value_(std::move(initial)) {}

  T get() const {
    std::shared_lock lock(mutex_);
    return value_;
  }

  void set(const T& value) {
    std::vector<std::function<void(const T&)>> subs;
    {
      std::unique_lock lock(mutex_);
      value_ = value;
      subs = subscribers_;
    }
    for (const auto& fn : subs) fn(value);
  }

  void subscribe(std::function<void(const T&)> callback) {
    T current;
    {
      std::unique_lock lock(mutex_);
      subscribers_.push_back(callback);
      current = value_;
    }
    callback(current);
  }

 private:
  mutable std::shared_mutex mutex_;
  T value_;
  std::vector<std::function<void(const T&)>> subscribers_;
};

}  // namespace arch_nav::vehicle

#endif  // NAVIGATION__CORE__MODEL__VEHICLE__BEHAVIOR_SUBJECT_HPP_
