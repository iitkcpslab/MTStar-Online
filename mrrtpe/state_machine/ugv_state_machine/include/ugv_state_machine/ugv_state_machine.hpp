#pragma once

#include <ugv_state_machine/ugv_state_machine_base.hpp>

namespace mrrtpe::ugv_state_machine {

class UGVStateMachine : public StateMachineBase {
  public:
    UGVStateMachine(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void run();

  private:
    void publishCurrState(const ros::TimerEvent&);

    template<class Event> 
    uint performTask();

    template<class Behaviour> 
    inline uint executeBehaviour() {
        const std::string last_state = state_names[machine_.current_state()[0]];
        uint response = uint(machine_.process_event(typename Behaviour::Event()));
        machine_.state_ptr_->updateRobotState(machine_.current_state()[0]);
        FSM_INFO("Entered state [" << state_names[machine_.current_state()[0]] << "] from ["
                                   << last_state << "]");
        return response;
    }

    StateMachineBackend machine_;

    ros::Publisher state_pub_;
    ros::Timer state_timer_;

    double poll_rate_;
};

}  // namespace mrrtpe::ugv_state_machine
