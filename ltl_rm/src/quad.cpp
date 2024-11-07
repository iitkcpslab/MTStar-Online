#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

// Class to handle the offboard control
class OffboardControl {
  public:
    OffboardControl(ros::NodeHandle nh);

    void run();

  private:
    // Callback functions
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    // Publishers and subscribers
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher setpoint_raw_pub_;

    // Service clients
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    // Current state and pose
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;

    // Target pose
    geometry_msgs::PoseStamped target_pose_;

    // Offboard set mode and arming command
    mavros_msgs::SetMode offboard_set_mode_;
    mavros_msgs::CommandBool arm_cmd_;

    // Helper functions
    bool connect();
    void setTargetPose(double x, double y, double z);
    void setOffboardMode();
    void arm();
    void sendSetpoint();

    // Constants
    const double RATE_ = 20.0;  // Hz
};

OffboardControl::OffboardControl(ros::NodeHandle nh) {
    // Subscribe to the state and pose topics
    state_sub_ =
        nh.subscribe<mavros_msgs::State>("mavros/state", 10, &OffboardControl::stateCallback, this);
    pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose", 10, &OffboardControl::poseCallback, this);

    // Publish to the setpoint_raw topic
    setpoint_raw_pub_ = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);

    // Service clients for arming and setting mode
    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // Set the target pose
    setTargetPose(0.0, 0.0, 2.0);

    // Set the offboard mode and arming command
    offboard_set_mode_.request.custom_mode = "OFFBOARD";
    arm_cmd_.request.value = true;
}

void OffboardControl::run() {
    // Wait for connection
    while (ros::ok() && !connect()) {
        ROS_INFO("Waiting for connection...");
        ros::spinOnce();
        ros::Rate(1).sleep();
    }

    // Set the initial mode and arming state
    setOffboardMode();
    arm();

    // Main loop
    while (ros::ok()) {
        // Send the setpoint
        sendSetpoint();

        ros::spinOnce();
        rate_.sleep();
    }
}

void OffboardControl::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state_ = *msg;
}

void OffboardControl::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose_ = *msg;
}

bool OffboardControl::connect() {
    return current_state_.connected;
}
