#pragma once

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <mrrtpe_utils/conversions.hpp>

namespace drdo_simulation::mav_move_base {
    namespace  utils = mrrtpe::mrrtpe_utils;
class MAVMoveBase {
  public:
    MAVMoveBase(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

  private:
    void odomCb(const nav_msgs::OdometryConstPtr& odom);
    void process(const move_base_msgs::MoveBaseGoalConstPtr& goal);
    bool isReached();
    void stop();

    nav_msgs::OdometryConstPtr odom_ptr_;
    geometry_msgs::PoseStamped goal_;

    double fix_alt_;

    actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as_;
    move_base_msgs::MoveBaseFeedback feedback_;
    move_base_msgs::MoveBaseResult result_;

    ros::Subscriber odom_sub_;

    ros::Publisher cmd_pose_pub_;
};
}  // namespace drdo_simulation::mav_move_base
