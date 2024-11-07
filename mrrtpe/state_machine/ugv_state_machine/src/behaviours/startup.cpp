#include <ugv_state_machine/behaviours/startup.hpp>

namespace mrrtpe::ugv_state_machine {
StartUp::StartUp(ros::NodeHandle& nh, statePtr& state_ptr)
    : ugv_state_(state_ptr)
    , cmd_vel_zero_(true) {
    cmd_vel_sub_ = nh.subscribe("cmd_vel", 1, &StartUp::cmdVelCallback, this);
}

void StartUp::cmdVelCallback(const geometry_msgs::TwistConstPtr& msg) {
    cmd_vel_zero_ = !(std::fabs(msg->linear.x) > 0.01 || std::fabs(msg->angular.z) > 0.01);
}

bool StartUp::check() const {
    bool is_moving =
        (!cmd_vel_zero_ || (std::fabs(ugv_state_->getOdom()->twist.twist.linear.x) > 0.01) ||
            (std::fabs(ugv_state_->getOdom()->twist.twist.angular.z) > 0.01));

    return ugv_state_->sensorStatus() && !is_moving &&
           (ugv_state_->getRobotStatus().battery_status.battery_status !=
               mrrtpe_msgs::BatteryStatus::BATTERY_CRITICAL_LOW);
}
}  // namespace mrrtpe::ugv_state_machine