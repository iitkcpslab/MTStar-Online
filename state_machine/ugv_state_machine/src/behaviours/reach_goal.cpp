#include <mutex>
#include <numeric>
#include <algorithm>
#include <ugv_state_machine/behaviours/reach_goal.hpp>

namespace mrrtpe::ugv_state_machine {
ReachGoal::ReachGoal(const statePtr state_ptr)
    : ugv_state_(state_ptr) {
    tracking_ = false;
    goal_reached_ = false;
}

void ReachGoal::execute(const Event& evt) {
    if (tracker_.joinable()) {
        tracker_.join();
    }
    tracker_ = std::thread(&ReachGoal::pathTracker, this);
    tracking_ = true;
    goal_reached_ = false;
}

void ReachGoal::stopTracking() {
    tracking_ = false;
    if (tracker_.joinable()) {
        tracker_.join();
    }
    // @Todo goal_reached_ true condition
}

geometry_msgs::Pose ReachGoal::sampleGoal(const int& waypoint) {
    geometry_msgs::Pose goal;
    goal.position.x = ugv_state_->getRobotStatus().path.points[waypoint].x;
    goal.position.y = ugv_state_->getRobotStatus().path.points[waypoint].y;
    goal.position.z = ugv_state_->getRobotStatus().path.points[waypoint].z;
    goal.orientation = utils::getQuat(ugv_state_->getRobotStatus().path.points[waypoint].yaw);
    return goal;
}

void ReachGoal::pathTracker() {
    geometry_msgs::Pose goal;
    auto num_waypoints = (ugv_state_->getRobotStatus().path.points.size() - 1);
    while (num_waypoints > 0 && tracking_) {
        ROS_INFO_STREAM_THROTTLE(1.0, "[ReachGoal::pathTracker]"
                                 << "tracking the goal with move_base status "
                                 << ugv_state_->moveBase().getStatusStr());

        switch (ugv_state_->moveBase().getStatus()) {
            case actionlib::SimpleClientGoalState::LOST:
                goal = sampleGoal(1);
                ugv_state_->moveBase().sendGoal(goal, ugv_state_->getRobotStatus().path.goal_type);
                break;
            case actionlib::SimpleClientGoalState::ACTIVE:
                ROS_INFO_STREAM_THROTTLE(1.0, "[ReachGoal::pathTracker]"
                                << "Goal Active");
                break;
            case actionlib::SimpleClientGoalState::SUCCEEDED:
                if (num_waypoints == 1) {
                    ROS_INFO_STREAM("[ReachGoal::pathTracker]"
                                    << "Goal SUCCEEDED");
                    const std::lock_guard<std::mutex> g_l(ugv_state_->mutex_);
                    ugv_state_->getRobotStatus().path.points.erase(
                        ugv_state_->getRobotStatus().path.points.begin() + 1,
                        ugv_state_->getRobotStatus().path.points.end());
                    num_waypoints = 0;
                    goal_reached_ = true;
                } else {
                    ugv_state_->moveBase().getStatus() =
                        actionlib::SimpleClientGoalState::StateEnum::LOST;
                }
                break;
            case actionlib::SimpleClientGoalState::REJECTED:
            case actionlib::SimpleClientGoalState::ABORTED:
                ROS_ERROR_STREAM("[ReachGoal::pathTracker]"
                                << "Goal Rejected or aborted");
                num_waypoints = 0;
                break;
            default:
                // if(num_waypoints == 1) {
                //     goal = sampleGoal(1);
                //     ugv_state_->moveBase().sendGoal(goal, ugv_state_->getRobotStatus().path.goal_type);
                // }
                break;
        }

        auto curr_pose = ugv_state_->getRobotStatus().path.points[0];
        double distance = utils::getDistance(goal, curr_pose);
        if (distance < 3.0 && num_waypoints > 1) {
            int next_waypoint = std::min(1, int(num_waypoints - 1));
            const std::lock_guard<std::mutex> g_l(ugv_state_->mutex_);
            ugv_state_->getRobotStatus().path.points.erase(
                ugv_state_->getRobotStatus().path.points.begin() + 1,
                ugv_state_->getRobotStatus().path.points.begin() + 1 + next_waypoint);
            num_waypoints -= next_waypoint;
            goal = sampleGoal(next_waypoint);
            ugv_state_->moveBase().sendGoal(goal, ugv_state_->getRobotStatus().path.goal_type);
        }
        ros::Rate(2).sleep();
    }
    ugv_state_->moveBase().cancelGoal();
    ROS_INFO_STREAM("[ReachGoal::pathTracker]"
                    << "Goal tracking end");
}
}  // namespace mrrtpe::ugv_state_machine
