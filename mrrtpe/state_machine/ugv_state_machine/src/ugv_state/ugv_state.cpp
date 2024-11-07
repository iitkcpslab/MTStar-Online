#include <ugv_state_machine/ugv_state/ugv_state.hpp>

namespace mrrtpe::ugv_state_machine {

UGVState::UGVState(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : sensor_status_(false)
    , battery_status_(nh, nh_private)
    , move_base_(nh, nh_private)
    , goal_received_(false)
    , anomaly_received_(false)
    , follow_goal_(false)
    , laser_mon_(nh, "scan", ros::Duration(5.0))
    , battery_mon_(nh, "battery_status", ros::Duration(5.0)) {
    utils::getParam(nh_private, "Idle/x", idle_location_.x);
    utils::getParam(nh_private, "Idle/y", idle_location_.y);
    utils::getParam(nh_private, "Idle/yaw", idle_location_.yaw);

    int vehicle_type = 0;
    utils::getParam(nh_private, "vehicle_type", vehicle_type);

    robot_status_msg_.vehicle_type.vehicle_type = uint8_t(vehicle_type);
    robot_status_msg_.path.points.emplace_back(mrrtpe_msgs::Point());
    robot_status_msg_.anomaly_seen = false;

    anomaly_pose_.first = false;
    anomaly_pose_.second = boost::make_shared<nav_msgs::Odometry>();

    odom_sub_ = nh.subscribe("odometry", 1, &UGVState::odometryCallback, this);
    mrrp_sub_ = nh.subscribe("mrrp_plan", 10, &UGVState::mrrpCallback, this);
    object_sub_ = nh.subscribe("object_list", 1, &UGVState::objectDetCallback, this);

    robot_status_pub_ = nh.advertise<mrrtpe_msgs::RobotStatus>("robot_state", 1);

    odom_msg_ptr_ = boost::make_shared<nav_msgs::Odometry>();
}

void UGVState::run() {
    sensor_status_ =
        ((robot_status_msg_.vehicle_type.vehicle_type < mrrtpe_msgs::VehicleType::MAV_END) ||
            !laser_mon_.isSensorTimeout()) &&
        !battery_mon_.isSensorTimeout();
    robot_status_msg_.battery_status = battery_status_.getStatus();
    robot_status_pub_.publish(robot_status_msg_);
}

void UGVState::updateRobotState(const int& state) {
    robot_status_msg_.state.state = uint8_t(state);
    last_state_update_ = ros::Time::now();
}

void UGVState::odometryCallback(const nav_msgs::OdometryPtr& odom) {
    auto pose = odom->pose.pose;
    pose_.x = pose.position.x;
    pose_.y = pose.position.y;
    pose_.z = pose.position.z;
    pose_.yaw = utils::getYaw(pose);
    odom_msg_ptr_ = odom;
    const std::lock_guard<std::mutex> g_l(mutex_);
    robot_status_msg_.path.points[0] = pose_;
}

void UGVState::mrrpCallback(const mrrtpe_msgs::Plan& msg) {
    if (msg.vehicle_type.vehicle_type == robot_status_msg_.vehicle_type.vehicle_type) {
        if (msg.anomaly_received) {
            robot_status_msg_.anomaly_seen = false;
        }
        if (msg.path.points.size() > 1) {
            bool goal = false;
            if (msg.path.goal_type.goal_type == mrrtpe_msgs::GoalType::NORMAL_GOAL) {
                ROS_INFO_STREAM("[UGVState::mrrpCallback]"
                                << "NORMAL_GOAL received");
                goal = true;
            } else if (msg.path.goal_type.goal_type == mrrtpe_msgs::GoalType::FOLLOW_GOAL) {
                ROS_INFO_STREAM("[UGVState::mrrpCallback]"
                                << "FOLLOW_GOAL received");
                goal = true;
                follow_goal_ = true;
            } else if (msg.path.goal_type.goal_type == mrrtpe_msgs::GoalType::IDLE_GOAL) {
                ROS_INFO_STREAM("[UGVState::mrrpCallback]"
                                << "IDLE_GOAL received");
                goal = true;
            }

            if (goal) {
                const std::lock_guard<std::mutex> g_l(mutex_);
                robot_status_msg_.path = msg.path;
                robot_status_msg_.path.points[0] = pose_;
                goal_received_ = true;
            }
        } else {
            move_base_.cancelGoal();
        }
    }
}

void UGVState::objectDetCallback(const nav_msgs::OdometryConstPtr& msg) {
    anomaly_received_ = true;
    anomaly_pose_.first = false;
    anomaly_pose_.second->header.frame_id = "world";
    anomaly_pose_.second->child_frame_id = "world";

    if (msg->header.frame_id != "world") {
        double r = utils::getDistance(msg->pose.pose, geometry_msgs::Pose());
        double yaw = atan2(msg->pose.pose.position.y, msg->pose.pose.position.x);

        anomaly_pose_.second->pose.pose.position.x =
            pose_.x + (msg->pose.pose.position.x * cos(pose_.yaw) -
                          msg->pose.pose.position.y * sin(pose_.yaw));
        anomaly_pose_.second->pose.pose.position.y =
            pose_.y + (msg->pose.pose.position.x * sin(pose_.yaw) +
                          msg->pose.pose.position.y * cos(pose_.yaw));
        anomaly_pose_.second->twist.twist.linear.x =
            cos(pose_.yaw) * msg->twist.twist.linear.x + sin(pose_.yaw) * msg->twist.twist.linear.y;
        anomaly_pose_.second->twist.twist.linear.y =
            cos(pose_.yaw) * msg->twist.twist.linear.y - sin(pose_.yaw) * msg->twist.twist.linear.x;

    } else {
        anomaly_pose_.second->pose = msg->pose;
        anomaly_pose_.second->twist = msg->twist;
    }
    robot_status_msg_.anomaly_seen = true && robot_status_msg_.path.goal_type.goal_type != mrrtpe_msgs::GoalType::FOLLOW_GOAL;
    robot_status_msg_.anomaly = anomaly_pose_.second->pose.pose.position;
    ROS_INFO_STREAM_THROTTLE(5.0,
        "[UGVState::objectDetCallback]"
            << "Anomaly pose received");
}
}  // namespace mrrtpe::ugv_state_machine
