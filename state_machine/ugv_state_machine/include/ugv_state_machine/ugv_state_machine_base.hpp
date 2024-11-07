#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>

#include <ugv_state_machine/behaviours/follow_anomaly.hpp>
#include <ugv_state_machine/behaviours/goal_reached.hpp>
#include <ugv_state_machine/behaviours/reach_goal.hpp>
#include <ugv_state_machine/behaviours/reach_idle.hpp>
#include <ugv_state_machine/behaviours/startup.hpp>

#include <state_machine_definition/state_machine.hpp>
#include <ugv_state_machine/ugv_state/ugv_state.hpp>

namespace mrrtpe::ugv_state_machine {

class StateMachineBase : public mrrtpe::state_machine::FSMDef<StateMachineBase> {
  public:
    StateMachineBase() = default;
    StateMachineBase(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void spin();

    // State names
    std::vector<std::string> state_names = {"Rest",
        "Idle",
        "Route",
        "PatrolIdle" /* hover for mav */,
        "Follow",
        "Critical"};

    // State definitions
    struct Rest : public State<Rest> {};
    struct Idle : public State<Idle> {};
    struct Route : public State<Route> {};
    struct PatrolIdle : public State<PatrolIdle> {};
    struct Follow : public State<Follow> {};
    struct Critical : public State<Critical> {};
    typedef Rest initial_state;

    // Transition Guards
    bool checks(const StartUp::Event& cmd) {
        return startup_beh_.check();
    }

    template<class T> bool 
    routeThreadDone(const T& cmd) {
        reach_goal_beh_.stopTracking();
        return true;
    }

    // Transition actions
    void reachGoal(const ReachGoal::Event& cmd);
    void followAnomaly(const FollowAnomaly::Event& cmd);
    void reachIdle(const ReachIdle::Event& cmd);

    // clang-format off
    struct transition_table
        : boost::mpl::vector<
       //      Type     Start          Event                      Next            Action                         Guard
       // +++ ------ + ----------- + ----------------------- + ----------- + ----------------------------------- + ------------------------------------ +++
              g_row<    Rest       , StartUp::Event          , Idle        , &StateMachineBase::checks                            >,
       // +++ ------ + ----------- + ----------------------- + ----------- + ----------------------------------- + ------------------------------------ +++
              a_row<    Idle       , ReachGoal::Event        , Route       ,  &StateMachineBase::reachGoal                                   >,
       // +++ ------ + ----------- + ----------------------- + ----------- + ----------------------------------- + ------------------------------------ +++
                row<    Route      , ReachGoal::Event        , Route       ,  &StateMachineBase::reachGoal       , &StateMachineBase::routeThreadDone<ReachGoal::Event>    >,                 
       // +++ ------ + ----------- + ----------------------- + ----------- + ----------------------------------- + ------------------------------------ +++
              _row<     Route      , GoalReached::Event      , PatrolIdle                                                                 >,
       // +++ ------ + ----------- + ----------------------- + ----------- + ----------------------------------- + ------------------------------------ +++
              //  row<     Route      , InspectObstacle::Event  , Inspect     ,  &StateMachineBase::inspectingTheObstacle, &StateMachineBase::routeThreadDone<InspectObstacle::Event> >,
       // +++ ------ + ----------- + ----------------------- + ----------- + ----------------------------------- + ------------------------------------ +++
              a_row<     PatrolIdle  , FollowAnomaly::Event   , Follow     ,  &StateMachineBase::followAnomaly >,
       // +++ ------ + ----------- + ----------------------- + ----------- + ----------------------------------- + ------------------------------------ +++
              a_row<   PatrolIdle  ,  ReachGoal::Event       , Route       ,  &StateMachineBase::reachGoal                                  >,  
       // +++ ------ + ----------- + ----------------------- + ----------- + ----------------------------------- + ------------------------------------ +++
              // a_row<    Inspect    ,  ReachGoal::Event       , Route       ,  &StateMachineBase::reachGoal                        >,     
       // +++ ------ + ----------- + ----------------------- + ----------- + ----------------------------------- + ------------------------------------ +++
              // a_row<    Inspect  ,  AnomalyNotFound::Event   ,  Route    ,  &StateMachineBase::reachGoal                        >,     
       // +++ ------ + ----------- + ----------------------- + ----------- + ----------------------------------- + ------------------------------------ +++
              // row<      Inspect    ,  FollowAnomaly::Event   , Follow      ,  &StateMachineBase::followAnomaly   , &StateMachineBase::hasAnomaly>, 
       // +++ ------ + ----------- + ----------------------- + ----------- + ----------------------------------- + ------------------------------------ +++
              // row<      Follow     ,  InspectAnomaly::Event  , Inspect      ,  &StateMachineBase::inspectingTheAnomaly, &StateMachineBase::hasFollowGoal>,
       // +++ ------ + ----------- + ----------------------- + ----------- + ----------------------------------- + ------------------------------------ +++
              a_row<    Follow    ,  ReachIdle::Event       , Route       ,  &StateMachineBase::reachIdle>
       // +++ ------ + ----------- + ----------------------- + ----------- + ----------------------------------- + ------------------------------------ +++
       > {};
    // clang-format on

    statePtr state_ptr_;
    // behaviour objects
    StartUp startup_beh_;
    FollowAnomaly follow_anomaly_beh_;
    GoalReached goal_reached_beh_;
    ReachGoal reach_goal_beh_;
    ReachIdle reach_idle_beh_;

};

}  // namespace mrrtpe::ugv_state_machine