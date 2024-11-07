#pragma once

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/Odometry.h>

class FakeBattery {
public:
    FakeBattery(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    ~FakeBattery() {};
    void run();
private:
    void odomCallback(const nav_msgs::OdometryConstPtr& msgs);
    bool chargeBattery(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
    bool emptyBattery(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);

    double drain_factor_;
    sensor_msgs::BatteryState battery_state_;
    ros::Time last_update_;
    ros::Subscriber odom_sub_;
    ros::Publisher battery_pub_;
    ros::ServiceServer charge_battery_service_;
    ros::ServiceServer empty_battery_service_;
};
