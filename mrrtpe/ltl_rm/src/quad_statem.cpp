#include <mutex>
#include <thread>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <mrrtpe_msgs/RobotStatus.h>
#include <mrrtpe_msgs/Plan.h>
#include <mrrtpe_msgs/VehicleType.h>
#include <mrrtpe_msgs/State.h>
#include <mrrtpe_msgs/BatteryStatus.h>
#include <mrrtpe_msgs/Path.h>
#include <thread>

#define REQ_INTERVAL 1
#define HEIGHT 5.0
#define START_X 1.0
#define START_Y 9.0
// Global variables
geometry_msgs::PoseStamped current_pose;
ros::Subscriber state_sub,pose_sub;
ros::Publisher local_pos_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
mavros_msgs::State current_state;
mrrtpe_msgs::Path current_path;
int path_no=0;
// For state_publisher
ros::Publisher robot_status_pub;
ros::Subscriber mrrp_sub;
mrrtpe_msgs::VehicleType::_vehicle_type_type vehicle_type;
mrrtpe_msgs::State::_state_type vehicle_state;
ros::Time anomaly_time;
bool v_anomaly_seen;
geometry_msgs::Point v_anomaly_point;
//mrrtpe_msgs::Point goal;
mrrtpe_msgs::RobotStatus data;

//Call back functions

void mrrp_cb(const mrrtpe_msgs::Plan::ConstPtr& msg) {
    if (msg->vehicle_type.vehicle_type == vehicle_type and msg->path.goal_type.goal_type!=mrrtpe_msgs::GoalType::UNKNOWN) {
        ROS_INFO_STREAM("[MRRP_CB]: Received Plan");
        try {
            if (data.anomaly_seen and msg->anomaly_received and (ros::Time::now() - anomaly_time > ros::Duration(10)))
            {
                data.anomaly_seen = !msg->anomaly_received;
            }
            current_path = msg->path;
            // for(auto i=0;i<current_path.points.size();i++){
            //     ROS_INFO_STREAM("Point: "<<i<<" "<<current_path.points[i]);
            // }
            path_no = 1;
            // if(current_path.goal_type.goal_id!=msg->path.goal_type.goal_id){
            //     current_path=msg->path;
            //     path_no=0;
            // }
        } catch (std::bad_alloc& ba) {
            std::cerr << "[MRRP_CB]: bad_alloc caught at: " << ba.what();
        }
       // ROS_INFO_STREAM("[MRRP_CB]: Path Received to: (" << *(current_path.points[].rbegin()).x << "," << *(current_path.points[].rbegin()).y << "," << *(current_path.points[].rbegin()).z << ")");
    }
}

// Callback function for the state subscriber
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}
// calling for pose
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
}


//Functions
double get_yaw(const geometry_msgs::PoseStamped msg) {
    tf::Quaternion q(msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}


bool arm() {
    mavros_msgs::CommandBool arm_cmd;
    ROS_INFO_STREAM("[ARM]: Vehicle Arming");
    arm_cmd.request.value = true;
    return (arming_client.call(arm_cmd) && arm_cmd.response.success);
}

// Function to set the flight mode
bool set_mode(std::string mode) {
    mavros_msgs::SetMode mode_cmd;
    ROS_INFO_STREAM("[SET_MODE]: Flight mode setting to" << mode.c_str());
    mode_cmd.request.custom_mode = mode;
    return (set_mode_client.call(mode_cmd) && mode_cmd.response.mode_sent);
}

void sub_2(const geometry_msgs::PoseStamped msg)
{
    data.anomaly.x = current_pose.pose.position.x - msg.pose.position.y;
    data.anomaly.y = current_pose.pose.position.y - msg.pose.position.x;
    if(!data.anomaly_seen and ((ros::Time::now()-anomaly_time)>ros::Duration(60))){
    data.anomaly_seen=true;
    anomaly_time=ros::Time::now();
    ROS_INFO_STREAM("[ARUCO_CB] Anomaly Seen at ("<<data.anomaly.x<<","<<data.anomaly.y<<").");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "quad_control");
    ros::NodeHandle nh;

    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    mrrp_sub = nh.subscribe<mrrtpe_msgs::Plan>("mrrp_plan", 10, mrrp_cb);
    ros::Subscriber sub_aruco = nh.subscribe("aruco_single/pose", 10, sub_2);
    robot_status_pub = nh.advertise<mrrtpe_msgs::RobotStatus>("robots", 10);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    anomaly_time = ros::Time::now();
    ros::Rate loop_rate(100);
    ROS_INFO_STREAM("[MAIN]: Setting Initial Data");
    vehicle_type = mrrtpe_msgs::VehicleType::MAV_1;
    vehicle_state = mrrtpe_msgs::State::Idle;
    data.anomaly_seen = false;
    data.anomaly.x=0;
    data.anomaly.y=0;
    data.anomaly.z=0;
    data.vehicle_type.vehicle_type = vehicle_type;
    data.battery_status.battery_status = mrrtpe_msgs::BatteryStatus::BATTERY_OPERATIONAL;
    geometry_msgs::PoseStamped pose;

    while (ros::ok() && !current_state.connected) {
    ROS_INFO_STREAM("[MAIN]: Waiting for connection to the quadcopter");
    ros::spinOnce();
    ros::Duration(0.01).sleep();
    }

    // Arm the quadcopter and set the flight mode to offboard
    for (uint i = 0; ros::ok() && i < 100; i++)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        loop_rate.sleep();
    }
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    pose.pose.position.x = START_X;
    pose.pose.position.y = START_Y;

    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {

        if (path_no == 1 and current_path.points.size() > 1 and vehicle_state != mrrtpe_msgs::State::Route)
        {
            ROS_INFO_STREAM("[Main::Loop]: Received Goal");
            ROS_WARN_STREAM("[Main::Loop]: State:ROUTE.");
            vehicle_state = mrrtpe_msgs::State::Route;
        }
        if (current_path.points.size() >1 and path_no >= current_path.points.size() and
            vehicle_state != mrrtpe_msgs::State::PatrolIdle) {
            ROS_WARN_STREAM("[Main::Loop]: Path Succesfully executed");
            ROS_WARN_STREAM("[Main::Loop]: State:PatrolIdle.");
            vehicle_state = mrrtpe_msgs::State::PatrolIdle;
        }
        // if ((v_goal_id_new.goal_type == mrrtpe_msgs::GoalType::NORMAL_GOAL or
        //      v_goal_id_new.goal_type == mrrtpe_msgs::GoalType::FOLLOW_GOAL) and
        //     v_goal_id_new.goal_id != v_goal_id_old.goal_id)
        // {
        //     ROS_INFO_STREAM("[main]: Received Goal");
        //     ROS_WARN_STREAM("[main]: State:ROUTE.");
        //     ROS_WARN_STREAM("[main]: Setting goal to pose.");
        //     vehicle_state = mrrtpe_msgs::State::Route;
        //     pose.pose.position.x = goal.x;
        //     pose.pose.position.y = goal.y;
        //     pose.pose.position.z = HEIGHT;
        // }

        double distance = sqrt(pow(pose.pose.position.x - current_pose.pose.position.x, 2) +
                               pow(pose.pose.position.y - current_pose.pose.position.y, 2) +
                               pow(HEIGHT - current_pose.pose.position.z, 2));
        if (distance <= 0.5 and current_path.points.size() > 1 and
            path_no < current_path.points.size()) {
            ROS_INFO_STREAM("[Main::Loop] Moving to next Point in the Path.");
            pose.pose.position.x = current_path.points[path_no].x;
            pose.pose.position.y = current_path.points[path_no].y;
            ROS_INFO_STREAM("[Main::Loop] goal no "<<path_no<<": " << current_path.points[path_no].x << " "
                                                  << current_path.points[path_no].y);
            path_no=path_no+1;
        }
        // if (current_path.points.size() > 1 and path_no < current_path.points.size()) {
        //     goal = current_path.points[path_no];
        // }
        // ROS_INFO_STREAM("[Main::Loop] goal: " << goal.x << " " << goal.y);
        // pose.pose.position.x = goal.x;
        // pose.pose.position.y = goal.y;
        pose.pose.position.z = HEIGHT;

        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        if (current_state.mode != "OFFBOARD" && ((ros::Time::now() - last_request) > ros::Duration(REQ_INTERVAL)))
            {
            if (set_mode("OFFBOARD")) {
                last_request = ros::Time::now();
                ROS_INFO_STREAM("[Main] Robot set to mode: OFFBOARD");
                }
            }
        else if (!current_state.armed && ((ros::Time::now() - last_request) > ros::Duration(REQ_INTERVAL)))
            {
                if (arm())
                {
                    last_request = ros::Time::now();
                    ROS_INFO_STREAM("[Main] Robot Armed.");
                }
            }

        data.state.state=vehicle_state;
        data.path.goal_type = current_path.goal_type;
        if (current_path.points.size() > 0 and (current_path.points.size() - path_no) >0)
        {
                data.path.points.resize(current_path.points.size() - path_no);
        }
        else {
            data.path.points.resize(1);
        }
        data.path.points[0].x = current_pose.pose.position.x;
        data.path.points[0].y = current_pose.pose.position.y;
        data.path.points[0].z = current_pose.pose.position.z;
        data.path.points[0].yaw = get_yaw(current_pose);
        data.path.points[0].preconditions.clear();
        if (current_path.points.size() > 0 and (current_path.points.size() - path_no) >0) {
            int j = 1;
            for (int i = path_no; i < current_path.points.size() and j<data.path.points.size(); i++) {
                data.path.points[j] = current_path.points[i];
                j++;
            }
        }

        local_pos_pub.publish(pose);
        robot_status_pub.publish(data);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
