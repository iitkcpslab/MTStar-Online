#include <mav_move_base/mav_move_base.hpp>

namespace drdo_simulation::mav_move_base {
MAVMoveBase::MAVMoveBase(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
    : as_(nh, "move_base", boost::bind(&MAVMoveBase::process, this, _1), false)
    , fix_alt_(5.0)
    , odom_ptr_(nav_msgs::OdometryPtr()) {
    utils::getParam(nh_priv, "fix_alt", fix_alt_);

    odom_sub_ = nh.subscribe("ground_truth/odometry", 2, &MAVMoveBase::odomCb, this);
    cmd_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 1);

    while(ros::ok() && cmd_pose_pub_.getNumSubscribers() == 0) {
        ros::Duration(0.1).sleep();
    }
    as_.start();
}

bool MAVMoveBase::isReached() {
    double distance = utils::getDistance(odom_ptr_->pose.pose, goal_.pose);
    if (distance < 0.2 && std::fabs(odom_ptr_->pose.pose.position.z - fix_alt_) < 0.2) {
        return true;
    }
    return false;
}

void MAVMoveBase::odomCb(const nav_msgs::OdometryConstPtr& odom) {
    odom_ptr_ = odom;
    feedback_.base_position.pose = odom_ptr_->pose.pose;
    feedback_.base_position.header = odom_ptr_->header;
}

void MAVMoveBase::stop() {
    geometry_msgs::PoseStamped pose;
    pose.pose = odom_ptr_->pose.pose;
    cmd_pose_pub_.publish(pose);
}

void MAVMoveBase::process(const move_base_msgs::MoveBaseGoalConstPtr& goal) {
    as_.publishFeedback(feedback_);
    goal_ = goal->target_pose;
    goal_.pose.position.z = fix_alt_;
    cmd_pose_pub_.publish(goal_);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        as_.publishFeedback(feedback_);
        if (as_.isPreemptRequested()) {
            if (as_.isNewGoalAvailable()) {
                goal_ = as_.acceptNewGoal()->target_pose;
                goal_.pose.position.z = fix_alt_;
                cmd_pose_pub_.publish(goal_);
            } else {
                stop();
                as_.setPreempted();
                return;
            }
        }

        if (isReached() && as_.isActive()) {
            as_.setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
            return;
        }
        loop_rate.sleep();
    }
}
}  // namespace drdo_simulation::mav_move_base
