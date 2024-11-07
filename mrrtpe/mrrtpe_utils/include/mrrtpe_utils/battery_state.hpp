#pragma once

#include <mrrtpe_msgs/BatteryStatus.h>
#include <mrrtpe_utils/conversions.hpp>

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <std_srvs/Empty.h>

namespace mrrtpe::mrrtpe_utils {
class BatteryState {
  private:
    mrrtpe_msgs::BatteryStatus status_;
    double max_voltage_, critical_low_voltage_, low_voltage_;
    ros::Subscriber battery_sub_;
    ros::ServiceClient client_;

    void batteryCallback(const sensor_msgs::BatteryStateConstPtr& msg) {
        if (msg->voltage >= max_voltage_) {
            status_.battery_status = mrrtpe_msgs::BatteryStatus::BATTERY_FULL;
        } else if (msg->voltage > low_voltage_) {
            status_.battery_status = mrrtpe_msgs::BatteryStatus::BATTERY_OPERATIONAL;
        } else if (msg->voltage < low_voltage_ && msg->voltage > critical_low_voltage_) {
            status_.battery_status = mrrtpe_msgs::BatteryStatus::BATTERY_LOW;
        } else {
            status_.battery_status = mrrtpe_msgs::BatteryStatus::BATTERY_CRITICAL_LOW;
        }
    }

  public:
    BatteryState(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
        : max_voltage_(10000)
        , critical_low_voltage_(0.0)
        , low_voltage_(0.0) {
        getParam(nh_private, "max_voltage", max_voltage_);
        getParam(nh_private, "critical_low_voltage", critical_low_voltage_);
        getParam(nh_private, "low_voltage", low_voltage_);

        battery_sub_ = nh.subscribe("battery_status", 1, &BatteryState::batteryCallback, this);
        client_ = nh.serviceClient<std_srvs::Empty>("charge_battery");
    }
    mrrtpe_msgs::BatteryStatus getStatus() const {
        return status_;
    }
    bool chargeBattery() {
        std_srvs::Empty srv;
        if (client_.call(srv)) {
            return true;
        }
        return false;
    }
};

}  // namespace mrrtpe::mrrtpe_utils