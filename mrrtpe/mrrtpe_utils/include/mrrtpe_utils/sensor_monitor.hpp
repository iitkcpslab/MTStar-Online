#pragma once

#include <ros/ros.h>

namespace mrrtpe::mrrtpe_utils {
template<class T> class SensorMonitor {
  public:
    T getMsg() const {
        return msg_;
    }
    ros::Time getLastUpdateTime() const {
        return t_last_update_;
    }
    bool isSensorTimeout() const {
        if ((ros::Time::now() - t_last_update_) > timeout_) {
            return true;
        }
        return false;
    }
    SensorMonitor(ros::NodeHandle& nh,
        const std::string& topic_name,
        const ros::Duration& timeout) {
        sensor_sub_ = nh.subscribe(topic_name, 1, &SensorMonitor::callback, this);
        timeout_ = timeout;
    }

  private:
    void callback(const T& msg) {
        t_last_update_ = ros::Time::now();
        msg_ = msg;
    }
    ros::Time t_last_update_ = ros::Time::now();
    ros::Duration timeout_;
    ros::Subscriber sensor_sub_;
    T msg_;
};
}  // namespace mrrtpe::mrrtpe_utils
