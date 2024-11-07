#pragma once

#include <thread>
#include <ugv_state_machine/ugv_state/ugv_state.hpp>

namespace mrrtpe::ugv_state_machine {

class ReachGoal {
  public:
    struct Event {};
    ReachGoal() = default;

    void execute(const Event& evt);
    ReachGoal(const statePtr state_ptr);
    void stopTracking();
    bool tracking_, goal_reached_;

  private:
    void pathTracker();
    geometry_msgs::Pose sampleGoal(const int& waypoint);
    const statePtr ugv_state_;
    std::thread tracker_;
};

}  // namespace mrrtpe::ugv_state_machine
