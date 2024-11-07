#pragma once

#include <ugv_state_machine/ugv_state/ugv_state.hpp>

namespace mrrtpe::ugv_state_machine {

class StartUp {
  public:
    struct Event {};
    StartUp() = default;
    StartUp(ros::NodeHandle& nh, statePtr& state_ptr);

    void execute(const Event& evt){};
    bool check() const;

  private:
    void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg);

    bool cmd_vel_zero_;

    const statePtr ugv_state_;
    ros::Subscriber cmd_vel_sub_;
};

}  // namespace mrrtpe::ugv_state_machine
