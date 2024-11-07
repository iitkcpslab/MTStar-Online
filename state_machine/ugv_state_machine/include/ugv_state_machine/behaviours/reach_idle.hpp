#pragma once

#include <ugv_state_machine/ugv_state/ugv_state.hpp>

namespace mrrtpe::ugv_state_machine {

class ReachIdle {
  public:
    struct Event {};
    ReachIdle() = default;

    void execute(const Event& evt);
    ReachIdle(const statePtr state_ptr);
    bool reach_idle_;

  private:
    const statePtr ugv_state_;
};

}  // namespace mrrtpe::ugv_state_machine
