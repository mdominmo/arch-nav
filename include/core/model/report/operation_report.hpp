#ifndef NAVIGATION__CORE__MODEL__REPORT__OPERATION_REPORT_HPP_
#define NAVIGATION__CORE__MODEL__REPORT__OPERATION_REPORT_HPP_

namespace arch_nav::report {

enum class OperationResult { IN_PROGRESS, COMPLETED, ABORTED };

class OperationReport {
 public:
  OperationResult result() const { return result_; }
  void complete() { result_ = OperationResult::COMPLETED; }
  void abort()   { result_ = OperationResult::ABORTED; }

  OperationReport() = default;
  virtual ~OperationReport() = default;

 private:
  OperationResult result_{OperationResult::IN_PROGRESS};
};

}  // namespace arch_nav::report

#endif  // NAVIGATION__CORE__MODEL__REPORT__OPERATION_REPORT_HPP_
