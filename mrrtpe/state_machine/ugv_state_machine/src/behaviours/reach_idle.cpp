#include <ugv_state_machine/behaviours/reach_idle.hpp>

namespace mrrtpe::ugv_state_machine {

void ReachIdle::execute(const Event& evt) {
    if (ugv_state_->getRobotStatus().path.points.size() > 1) {
        const std::lock_guard<std::mutex> g_l(ugv_state_->mutex_);
        ugv_state_->getRobotStatus().path.points.erase(
            ugv_state_->getRobotStatus().path.points.begin() + 1,
            ugv_state_->getRobotStatus().path.points.end());
    }
    ugv_state_->getRobotStatus().path.points.push_back(ugv_state_->getIdle());
    reach_idle_ = true;
};
ReachIdle::ReachIdle(const statePtr state_ptr)
    : ugv_state_(state_ptr)
    , reach_idle_(false){};

}  // namespace mrrtpe::ugv_state_machine
