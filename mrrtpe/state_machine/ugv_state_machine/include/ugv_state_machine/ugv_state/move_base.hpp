#pragma once

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <mrrtpe_msgs/GoalType.h>
#include <ros/ros.h>
#include <unordered_map>

namespace mrrtpe::ugv_state_machine {
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action;
class MoveBase {
  public:
    MoveBase() = default;
    MoveBase(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

    bool sendGoal(const geometry_msgs::Pose& goal, const mrrtpe_msgs::GoalType& goal_type);

    void cancelGoal() {
        if (goal_ac_->getState() == actionlib::SimpleClientGoalState::StateEnum::ACTIVE) {
            goal_ac_->cancelGoal();
        }
    }

    actionlib::SimpleClientGoalState::StateEnum& getStatus() {
        return feedback_;
    }

    const std::string getStatusStr() const {
        return map_.at(feedback_);
    }

  private:
    void goalDoneCallback(const actionlib::SimpleClientGoalState& state,
        const move_base_msgs::MoveBaseResultConstPtr& result);
    void activeCb();
    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

    boost::shared_ptr<action> goal_ac_;
    mrrtpe_msgs::GoalTypePtr goal_type_;
    actionlib::SimpleClientGoalState::StateEnum feedback_;

    std::unordered_map<actionlib::SimpleClientGoalState::StateEnum, std::string> map_;

    move_base_msgs::MoveBaseGoal goal_;
};
}  // namespace mrrtpe::ugv_state_machine