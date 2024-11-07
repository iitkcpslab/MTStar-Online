#pragma once

#include <cmath>
#include <geometry_msgs/Pose.h>
#include <mrrtpe_msgs/Point.h>
#include <numeric>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>

namespace mrrtpe::mrrtpe_utils {

template<typename T>
inline bool getParam(ros::NodeHandle& nh_private, const std::string& name, T& data) {
    if (nh_private.getParam(name, data)) {
        return true;
    } else {
        ROS_ERROR_STREAM(
            "Not able to retrive the param " << name << ", will use default value " << data);
        return false;
    }
}

inline double getDistance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2) {
    std::vector<double> diff{
        pose1.position.x - pose2.position.x, pose1.position.y - pose2.position.y};
    return std::sqrt(std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0));
}

inline geometry_msgs::Quaternion getQuat(const double& yaw) {
    geometry_msgs::Quaternion quat;
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, yaw);
    quat.x = myQuaternion.getX();
    quat.y = myQuaternion.getY();
    quat.z = myQuaternion.getZ();
    quat.w = myQuaternion.getW();
    return quat;
}

inline geometry_msgs::Pose pointToPose(const mrrtpe_msgs::Point& point) {
    geometry_msgs::Pose pose;
    pose.position.x = point.x;
    pose.position.y = point.y;
    pose.position.z = point.z;
    pose.orientation = getQuat(point.yaw);
    return pose;
}

inline double getDistance(const geometry_msgs::Pose& pose1, const mrrtpe_msgs::Point& pose2) {
    return getDistance(pointToPose(pose2), pose1);
}

inline double getYaw(const geometry_msgs::Pose& pose) {
    tf::Quaternion q(
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;

    m.getRPY(roll, pitch, yaw);
    return yaw;
}

inline mrrtpe_msgs::Point poseToPoint(const geometry_msgs::Pose& pose) {
    mrrtpe_msgs::Point point;
    point.x = pose.position.x;
    point.y = pose.position.y;
    point.z = pose.position.z;
    point.yaw = getYaw(pose);
    return point;
}

inline const double getYawFromRelativePose(const geometry_msgs::Pose& head,
    const geometry_msgs::Pose& tail) {
    const double x_diff = head.position.x - tail.position.x;
    const double y_diff = head.position.y - tail.position.y;
    return atan2(y_diff, x_diff);
}

inline const double getYawFromRelativePose(const geometry_msgs::Pose& head,
    const mrrtpe_msgs::Point& tail) {
    return getYawFromRelativePose(head, pointToPose(tail));
}

}  // namespace mrrtpe::mrrtpe_utils