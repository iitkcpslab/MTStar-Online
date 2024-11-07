#include "fake_battery/fake_battery.hpp"
#include <numeric>

FakeBattery::FakeBattery(ros::NodeHandle &nh, ros::NodeHandle &nh_private) 
    : drain_factor_(0.009) {
    battery_state_.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
    nh_private.getParam("max_voltage", battery_state_.capacity);
    nh_private.getParam("drain_factor", drain_factor_);
    battery_state_.voltage = battery_state_.capacity;
    last_update_ = ros::Time::now();

    odom_sub_ = nh.subscribe("odom", 1, &FakeBattery::odomCallback, this);
    battery_pub_ = nh.advertise<sensor_msgs::BatteryState>("battery_status", 1);
    charge_battery_service_ = nh.advertiseService("charge_battery", &FakeBattery::chargeBattery, this);
    empty_battery_service_ = nh.advertiseService("empty_battery", &FakeBattery::emptyBattery, this);
}

void FakeBattery::run() {
    battery_state_.header.stamp = ros::Time::now();
    battery_pub_.publish(battery_state_);
}

bool FakeBattery::chargeBattery(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res) {
    battery_state_.voltage = battery_state_.capacity;
    return true;
}

bool FakeBattery::emptyBattery(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res) {
    battery_state_.voltage = 0.0;
    return true;
}

void FakeBattery::odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    const std::vector<double> vel{msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z};
    battery_state_.voltage -= std::sqrt(std::inner_product(vel.begin(), vel.end(), vel.begin(), 0.0)) * drain_factor_ * (ros::Time::now() - last_update_).toSec();
    last_update_ = ros::Time::now();
}