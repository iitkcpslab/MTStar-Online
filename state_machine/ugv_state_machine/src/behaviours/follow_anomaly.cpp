#include <ugv_state_machine/behaviours/follow_anomaly.hpp>

namespace mrrtpe::ugv_state_machine {

FollowAnomaly::FollowAnomaly(ros::NodeHandle& nh_private, const statePtr state_ptr)
    : ugv_state_(state_ptr) {
    search_anomaly_ = false;
    follow_anomaly_ = false;
    search_anomaly_retry_ = 2;
    lost_anomaly_timeout_ = 5.0;
    anomaly_poll_time_ = 0.5;
    utils::getParam(nh_private, "search_anomaly_retry", search_anomaly_retry_);
    utils::getParam(nh_private, "lost_anomaly_timeout", lost_anomaly_timeout_);
    utils::getParam(nh_private, "anomaly_poll_time", anomaly_poll_time_);
}

void FollowAnomaly::execute(const Event& evt) {
    waitForTermination();
    search_anomaly_ = true;
    follow_anomaly_ = false;
    follow_th_ = std::thread(&FollowAnomaly::run, this);
}

void FollowAnomaly::waitForTermination() {
    if (follow_th_.joinable()) {
        follow_th_.join();
    }
}

void FollowAnomaly::run() {
    while (ros::ok() && (search_anomaly_ || follow_anomaly_)) {
        if (search_anomaly_) {
            inspect();
            search_anomaly_ = false;
        }
        if (follow_anomaly_) {
            follow();
            follow_anomaly_ = false;
        }
    }
}

void FollowAnomaly::inspect() {
    ROS_INFO_STREAM("[FollowAnomaly::inspect]"
                    << "starting search for anomaly");
    mrrtpe_msgs::Point pose = ugv_state_->getRobotStatus().path.points[0];
    pose.yaw += M_PI;

    mrrtpe_msgs::GoalType goal_type;
    goal_type.goal_type = mrrtpe_msgs::GoalType::NORMAL_GOAL;

    int retry_count = 1;

    ugv_state_->moveBase().sendGoal(utils::pointToPose(pose), goal_type);
    while (ros::ok() &&
           ugv_state_->moveBase().getStatus() !=
               actionlib::SimpleClientGoalState::StateEnum::REJECTED &&
           !ugv_state_->anomaly_received_ && retry_count < search_anomaly_retry_) {
        mrrtpe_msgs::Point curr_pose = ugv_state_->getRobotStatus().path.points[0];
        double yaw_diff = std::fabs(curr_pose.yaw - pose.yaw);

        if (yaw_diff < 0.1 || ugv_state_->moveBase().getStatus() ==
                                  actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
            ugv_state_->moveBase().cancelGoal();
            pose = ugv_state_->getRobotStatus().path.points[0];
            pose.yaw += M_PI;
            ugv_state_->moveBase().sendGoal(utils::pointToPose(pose), goal_type);
            retry_count += 1;
            ROS_INFO_STREAM("[FollowAnomaly::inspect]"
                            << "Retrying again with zero radius point turn.");
        }
        ros::Duration(anomaly_poll_time_).sleep();
    }
    if (!ugv_state_->anomaly_received_) {
        ROS_ERROR_STREAM("[FollowAnomaly::inspect]"
                         << " not able to find the anomaly");
        follow_anomaly_ = false;
        ugv_state_->moveBase().cancelGoal();
    } else {
        follow_anomaly_ = true;
        ROS_INFO_STREAM("[FollowAnomaly::inspect]"
                        << "ending search, anomaly found");
    }
}

void FollowAnomaly::follow() {
    ROS_INFO_STREAM("[FollowAnomaly::follow]"
                    << "starting to follow the anomaly");
    ros::Time t_last_update = ros::Time::now();
    while (ros::ok() && ugv_state_->moveBase().getStatus() !=
                            actionlib::SimpleClientGoalState::StateEnum::REJECTED) {
        if (ugv_state_->anomaly_received_) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position = ugv_state_->getRobotStatus().anomaly;
            pose.pose.position.x = (ugv_state_->getRobotStatus().anomaly.x +
                                     ugv_state_->getRobotStatus().path.points[0].x) /
                                 2.0;
            pose.pose.position.y = (ugv_state_->getRobotStatus().anomaly.y +
                                     ugv_state_->getRobotStatus().path.points[0].y) /
                                 2.0;
            double yaw_ref = utils::getYawFromRelativePose(
                pose.pose, ugv_state_->getRobotStatus().path.points[0]);
            pose.pose.orientation = utils::getQuat(yaw_ref);

            mrrtpe_msgs::GoalType goal_type;
            goal_type.goal_type = mrrtpe_msgs::GoalType::FOLLOW_GOAL;

            ugv_state_->moveBase().sendGoal(pose.pose, goal_type);
            t_last_update = ros::Time::now();
            const std::lock_guard<std::mutex> g_l(ugv_state_->mutex_);
            ugv_state_->anomaly_received_ = false;
        }
        ros::Rate(anomaly_poll_time_).sleep();

        if ((ros::Time::now() - t_last_update) > ros::Duration(lost_anomaly_timeout_)) {
            ROS_ERROR_STREAM("[FollowAnomaly::follow]"
                             << " lost anomaly");
            ugv_state_->moveBase().cancelGoal();
            search_anomaly_ = true;
            break;
        }
        ROS_INFO_STREAM_THROTTLE(1.0,
            "[FollowAnomaly::follow]"
                << "Following anomaly");
    }
}

}  // namespace mrrtpe::ugv_state_machine
