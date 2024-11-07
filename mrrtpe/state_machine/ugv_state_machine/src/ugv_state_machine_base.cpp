#include <ugv_state_machine/ugv_state_machine_base.hpp>

namespace mrrtpe::ugv_state_machine {
StateMachineBase::StateMachineBase(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : state_ptr_(std::make_shared<UGVState>(nh, nh_private))
    , follow_anomaly_beh_(nh_private, state_ptr_)
    , goal_reached_beh_(state_ptr_)
    , reach_goal_beh_(state_ptr_)
    , reach_idle_beh_(state_ptr_)
    , startup_beh_(nh, state_ptr_) {}

void StateMachineBase::reachGoal(const ReachGoal::Event& cmd) {
    reach_goal_beh_.execute(cmd);
}

void StateMachineBase::followAnomaly(const FollowAnomaly::Event& cmd) {
    follow_anomaly_beh_.execute(cmd);
}

void StateMachineBase::reachIdle(const ReachIdle::Event& cmd) {
    reach_idle_beh_.execute(cmd);
}

}  // namespace mrrtpe::ugv_state_machine
