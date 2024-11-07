#pragma once

#include <ugv_state_machine/ugv_state/ugv_state.hpp>

namespace mrrtpe::ugv_state_machine {

class GoalReached {
  public:
    struct Event {};
    GoalReached() = default;

    void execute(const Event& evt);
    GoalReached(const statePtr state_ptr);

  private:
    const statePtr ugv_state_;
};

}  // namespace mrrtpe::ugv_state_machine
