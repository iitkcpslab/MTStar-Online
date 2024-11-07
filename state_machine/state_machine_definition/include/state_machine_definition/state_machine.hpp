#pragma once

#include <boost/msm/back/mpl_graph_fsm_check.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/euml/common.hpp>
#include <boost/msm/front/euml/operator.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>

#include <cxxabi.h>
#include <ros/ros.h>

#define FSM_INFO(X) ROS_INFO_STREAM("[FSM]: " << X)

namespace mrrtpe::state_machine {

// store available policies for easier use
struct SwitchPolicy {
    typedef boost::msm::active_state_switch_after_entry after_entry;
    typedef boost::msm::active_state_switch_after_exit after_exit;
    typedef boost::msm::active_state_switch_before_transition before_transition;
    typedef boost::msm::active_state_switch_after_transition_action after_transition;
};

template<class FSMClass> 
class FSMDef : public boost::msm::front::state_machine_def<FSMClass> {
  public:
    typedef boost::msm::back::state_machine<FSMClass> StateMachineBackend;
    typedef SwitchPolicy::before_transition active_state_switch_policy;

    template<class Event, class FSM> 
    void on_entry(Event const&, FSM&){
        FSM_INFO("entering state_machine");
    };

    template<class Event, class FSM>
    void on_exit(Event const&, FSM&){
        FSM_INFO("exiting state_machine");
    };

    template<class T, bool V = false> 
    struct State : public boost::msm::front::state<> {
        State() {
            state_name = abi::__cxa_demangle(typeid(T).name(), 0, 0, nullptr);
            verbose = V;
        }

        template<class Event, class FSM> 
        void on_entry(Event const&, FSM&) {
            if (verbose) {
                FSM_INFO("Entered " << state_name << " state");
            }
        }

        template<class Event, class FSM> 
        void on_exit(Event const&, FSM&) {
            if (verbose) {
                FSM_INFO("Exited " << state_name << " state");
            }
        }

        std::string state_name;
        bool verbose;
    };

    template<class FSM, class Event> 
    void no_transition(Event const& e, FSM&, int state) {
        FSM_INFO("no transition from state " << state << " on event " << typeid(e).name());
    }
};


}  // namespace mrrtpe::state_machine
