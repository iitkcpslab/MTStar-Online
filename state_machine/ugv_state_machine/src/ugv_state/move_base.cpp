#include <mrrtpe_utils/conversions.hpp>
#include <ugv_state_machine/ugv_state/move_base.hpp>

namespace mrrtpe::ugv_state_machine {
namespace utils = mrrtpe::mrrtpe_utils;

MoveBase::MoveBase(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    goal_ac_ = boost::make_shared<action>(nh, "move_base");
    ROS_INFO("Waiting for move_base action server to start.");
    goal_ac_->waitForServer();  // will wait for infinite time'

    goal_type_ = boost::make_shared<mrrtpe_msgs::GoalType>();
    goal_type_->goal_type = mrrtpe_msgs::GoalType::UNKNOWN;
    goal_type_->goal_id = 21342;

    map_[actionlib::SimpleClientGoalState::StateEnum::PENDING] = "PENDING";
    map_[actionlib::SimpleClientGoalState::StateEnum::ACTIVE] = "ACTIVE";
    map_[actionlib::SimpleClientGoalState::StateEnum::RECALLED] = "RECALLED";
    map_[actionlib::SimpleClientGoalState::StateEnum::REJECTED] = "REJECTED";
    map_[actionlib::SimpleClientGoalState::StateEnum::PREEMPTED] = "PREEMPTED";
    map_[actionlib::SimpleClientGoalState::StateEnum::ABORTED] = "ABORTED";
    map_[actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED] = "SUCCEEDED";
    map_[actionlib::SimpleClientGoalState::StateEnum::LOST] = "LOST";
}

bool MoveBase::sendGoal(const geometry_msgs::Pose& goal, const mrrtpe_msgs::GoalType& goal_type) {
    *goal_type_ = goal_type;

    geometry_msgs::PoseStamped goal_stamped;
    goal_stamped.header.frame_id = "world";
    goal_stamped.header.stamp = ros::Time::now();
    goal_stamped.header.seq += 1;
    goal_stamped.pose = goal;

    goal_.target_pose = goal_stamped;
    feedback_ = actionlib::SimpleClientGoalState::StateEnum::PENDING;
    goal_ac_->sendGoal(goal_,
        boost::bind(&MoveBase::goalDoneCallback, this, _1, _2),
        boost::bind(&MoveBase::activeCb, this),
        boost::bind(&MoveBase::feedbackCb, this, _1));
    return true;
}

void MoveBase::goalDoneCallback(const actionlib::SimpleClientGoalState& state,
    const move_base_msgs::MoveBaseResultConstPtr& result) {
    feedback_ = actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED;
}

void MoveBase::activeCb() {
    feedback_ = actionlib::SimpleClientGoalState::StateEnum::ACTIVE;
}

void MoveBase::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    double distance = utils::getDistance(goal_.target_pose.pose, feedback->base_position.pose);
    double yaw_diff = std::fabs(
        utils::getYaw(feedback->base_position.pose) - utils::getYaw(goal_.target_pose.pose));
    if (distance < 0.51 && yaw_diff < 1.1) {
        goal_ac_->cancelGoal();
    }
}
}  // namespace mrrtpe::ugv_state_machine
