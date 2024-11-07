#include <ugv_state_machine/behaviours/goal_reached.hpp>

namespace mrrtpe::ugv_state_machine {
GoalReached::GoalReached(const statePtr state_ptr)
    : ugv_state_(state_ptr) {
}
void GoalReached::execute(const Event& evt){};
}  // namespace mrrtpe::ugv_state_machine
