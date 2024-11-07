#include "ugv_state_machine/ugv_state_machine.hpp"
#include <std_msgs/String.h>

namespace mrrtpe::ugv_state_machine {

UGVStateMachine::UGVStateMachine(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : machine_(nh, nh_private)
    , poll_rate_(1.0) {
    utils::getParam(nh_private, "poll_rate", poll_rate_);

    machine_.start();

    state_pub_ = nh_private.advertise<std_msgs::String>("curr_state", 1);
    state_timer_ =
        nh_private.createTimer(ros::Duration(1.0/poll_rate_), &UGVStateMachine::publishCurrState, this);
}

void UGVStateMachine::run() {
    ros::spinOnce();
    std::pair<uint, uint> status{0, 0};
    while (ros::ok() && status.first != 1) {
        ros::spinOnce();
        status.first = performTask<StartUp>();
        status.second += 1;
        ros::Rate(0.5).sleep();

        if (status.second > 60) {
            ROS_ERROR_STREAM("Sensors are not being initialize");
        }
    }

    while (ros::ok()) {
        ros::spinOnce();
        ros::Rate(poll_rate_).sleep();
    }

    machine_.stop();
}

template<class Event> uint UGVStateMachine::performTask() {
    return executeBehaviour<Event>();
}

void UGVStateMachine::publishCurrState(const ros::TimerEvent&) {

    if (machine_.state_ptr_->goal_received_ || machine_.reach_idle_beh_.reach_idle_) {
        machine_.state_ptr_->goal_received_ = false;
        machine_.reach_idle_beh_.reach_idle_ = false;
        machine_.state_ptr_->moveBase().getStatus() =
            actionlib::SimpleClientGoalState::StateEnum::LOST;
        performTask<ReachGoal>();
    } else if (machine_.reach_goal_beh_.goal_reached_ && machine_.current_state()[0] == 2) {
        machine_.reach_goal_beh_.goal_reached_  = false;
        performTask<GoalReached>();
        machine_.state_ptr_->moveBase().getStatus() =
            actionlib::SimpleClientGoalState::StateEnum::LOST;
    } else if (machine_.state_ptr_->follow_goal_ && 
               machine_.current_state()[0] == 3) {
        performTask<FollowAnomaly>();
        machine_.state_ptr_->follow_goal_ = false;
    } else if (!machine_.follow_anomaly_beh_.followAnomaly() &&
               !machine_.follow_anomaly_beh_.searchAnomaly() && machine_.current_state()[0] == 4) {
        performTask<ReachIdle>();
    }

    machine_.state_ptr_->run();
    std_msgs::String state_msg;
    state_msg.data = state_names[machine_.current_state()[0]];
    state_pub_.publish(state_msg);
}

}  // namespace mrrtpe::ugv_state_machine
