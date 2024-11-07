#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

#include <mrrtpe_msgs/Obstacle.h>
#include <mrrtpe_msgs/Plan.h>
#include <mrrtpe_msgs/RobotStatus.h>

#include <mrrtpe_utils/battery_state.hpp>
#include <mrrtpe_utils/conversions.hpp>
#include <mrrtpe_utils/sensor_monitor.hpp>
#include <ugv_state_machine/ugv_state/move_base.hpp>

#include <mutex>

#define BHV_INFO(X) ROS_INFO_STREAM("[BHV]: " << X)

namespace utils = mrrtpe::mrrtpe_utils;

namespace mrrtpe::ugv_state_machine {

class UGVState {
  public:
    UGVState(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void run();
    void updateRobotState(const int& state);

    mrrtpe_msgs::RobotStatus& getRobotStatus() {
        return robot_status_msg_;
    }
    bool sensorStatus() const {
        return sensor_status_;
    }
    MoveBase& moveBase() {
        return move_base_;
    }
    nav_msgs::OdometryPtr getOdom() const {
        return odom_msg_ptr_;
    }
    mrrtpe_msgs::Point getIdle() const {
        return idle_location_;
    }

  public:
    std::mutex mutex_;
    bool goal_received_;
    bool anomaly_received_;
    bool follow_goal_;

  private:
    void odometryCallback(const nav_msgs::OdometryPtr& odom);
    void mrrpCallback(const mrrtpe_msgs::Plan& msg);
    void objectDetCallback(const nav_msgs::OdometryConstPtr& msg);

    ros::Subscriber odom_sub_;
    ros::Subscriber mrrp_sub_;
    ros::Subscriber object_sub_;
  
    ros::Publisher robot_status_pub_;

    nav_msgs::OdometryPtr odom_msg_ptr_;
    mrrtpe_msgs::RobotStatus robot_status_msg_;

    mrrtpe_msgs::Point idle_location_;
    mrrtpe_msgs::Point pose_;
    std::pair<bool, nav_msgs::OdometryPtr> anomaly_pose_;

    utils::BatteryState battery_status_;
    utils::SensorMonitor<sensor_msgs::LaserScanConstPtr> laser_mon_;
    utils::SensorMonitor<sensor_msgs::BatteryStateConstPtr> battery_mon_;

    MoveBase move_base_;

    ros::Time last_state_update_;

    bool sensor_status_;
};
typedef std::shared_ptr<UGVState> statePtr;

}  // namespace mrrtpe::ugv_state_machine
