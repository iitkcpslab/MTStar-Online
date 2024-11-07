#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class OdomToTransformNode {
public:
    OdomToTransformNode() : nh_("~"), timer_expired_(false) {
        nh_.param<std::string>("parent_frame", parent_frame_, "map");
        nh_.param<std::string>("child_frame", child_frame_, "odom");
        nh_.param<double>("offset_x", offset_x_, 0.0);
        nh_.param<double>("offset_y", offset_y_, 0.0);

        odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 10, &OdomToTransformNode::odometryCallback, this);
        tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster);
        timer_ = nh_.createTimer(ros::Duration(180.0), &OdomToTransformNode::timerCallback, this, true);  // Wait for 3 minutes
    }

private:
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
        if (!timer_expired_) {
            ROS_INFO("Waiting for 3 minutes before publishing...");
            return;
        }

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = parent_frame_;
        transformStamped.child_frame_id = child_frame_;

        // Apply offset to the odometry position
        transformStamped.transform.translation.x = odom_msg->pose.pose.position.x + offset_x_;
        transformStamped.transform.translation.y = odom_msg->pose.pose.position.y + offset_y_;
        transformStamped.transform.translation.z = odom_msg->pose.pose.position.z;
        transformStamped.transform.rotation = odom_msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(transformStamped);
    }

    void timerCallback(const ros::TimerEvent& event) {
        ROS_INFO("Timer expired. Publishing transform...");
        timer_expired_ = true;
        timer_.stop();
    }

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    ros::Timer timer_;
    std::string parent_frame_;
    std::string child_frame_;
    double offset_x_;
    double offset_y_;
    bool timer_expired_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_to_transform_node");
    OdomToTransformNode node;
    ros::spin();
    return 0;
}

// rosrun your_package_name odom_to_transform_node _parent_frame:=map _child_frame:=odom _offset_x:=16.0 _offset_y:=16.0
