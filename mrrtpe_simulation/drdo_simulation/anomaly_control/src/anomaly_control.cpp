#include <anomaly_control/anomaly_control.hpp>
#include <mrrtpe_utils/conversions.hpp>

#include <nav_msgs/Odometry.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>

namespace mrrtpe_simulation {
namespace utils = mrrtpe::mrrtpe_utils;
AnomalyControl::AnomalyControl(ros::NodeHandle& nh, ros::NodeHandle& nh_private, Costmap2DROS* cmap)
    : obj_ctrl_(nh)
    , nh_(nh)
    , waypoint_file_name_("params/waypoints.yaml")
    , move_speed_(1.0)
    , static_obs_timeout_(10.0)
    , anomaly_detection_distance_(3.0)
    , GlobalPlanner("anomaly", cmap->getCostmap(), cmap->getGlobalFrameID())
    , marker_("marker", "world", boost::bind(&AnomalyControl::markerFeedback, this, _1)) {
    if (nh_private.getParam("waypoint_file_name", waypoint_file_name_)) {
        YAML::Node waypoint_conf;

        try {
            waypoint_conf = YAML::LoadFile(waypoint_file_name_);
        } catch (std::exception& e) {
            ROS_ERROR_STREAM("Error loading file " << waypoint_file_name_);
            ROS_DEBUG_STREAM(e.what());
            exit(EXIT_FAILURE);
        }

        int count = 0;
        for (YAML::const_iterator it = waypoint_conf.begin(); it != waypoint_conf.end(); ++it) {
            for (auto point : it->second) {
                rm::Point waypoint;
                waypoint.x = point[0].as<double>();
                waypoint.y = point[1].as<double>();
                waypoints_[count].second.push_back(waypoint);
            }
            waypoints_[count].first = 0;
            ++count;
        }
    }

    nh_private.getParam("move_speed", move_speed_);
    nh_private.getParam("static_obs_timeout", static_obs_timeout_);
    nh_private.getParam("anomaly_detection_distance", anomaly_detection_distance_);

    goal_pose_.pose.position = waypoints_[2].second[0];

    robots_sub_ = nh.subscribe("robots", 10, &AnomalyControl::robotsCallback, this);
    anomaly_toggle_sub_ = nh.subscribe("toggle_anomaly", 10, &AnomalyControl::anomalyToggle, this);

    anomaly_pub_ = nh.advertise<nav_msgs::Odometry>("object_list", 10);

    obstacle_timer_ = nh_private.createTimer(
        ros::Duration(static_obs_timeout_), &AnomalyControl::obstacleControl, this);
    anomaly_timer_ =
        nh_private.createTimer(ros::Duration(0.1), &AnomalyControl::anomalyControl, this);
}

void AnomalyControl::markerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
    ROS_DEBUG_STREAM(feedback->marker_name << " is now at " << feedback->pose.position.x << ", "
                                           << feedback->pose.position.y << ", "
                                           << feedback->pose.position.z);
    goal_pose_.pose = feedback->pose;
}

void AnomalyControl::run() {
    while (ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
}

void AnomalyControl::robotsCallback(const mrrtpe_msgs::RobotStatusConstPtr& msg) {
    if (msg->vehicle_type.vehicle_type != mrrtpe_msgs::VehicleType::UNKNOWN) {
        robots_[msg->vehicle_type.vehicle_type] = std::make_pair(ros::Time::now(), *msg);
        if (anomaly_pubs_.find(msg->vehicle_type.vehicle_type) == anomaly_pubs_.end()) {
            if (msg->vehicle_type.vehicle_type > mrrtpe_msgs::VehicleType::MAV_END) {
                anomaly_pubs_[msg->vehicle_type.vehicle_type] = nh_.advertise<nav_msgs::Odometry>(
                    "/ugv_" +
                        std::to_string(
                            msg->vehicle_type.vehicle_type - mrrtpe_msgs::VehicleType::MAV_END) +
                        "/object_list",
                    10);
            } else {
                anomaly_pubs_[msg->vehicle_type.vehicle_type] = nh_.advertise<nav_msgs::Odometry>(
                    "/mav_" + std::to_string(msg->vehicle_type.vehicle_type) + "/object_list", 10);
            }
        }
    } else {
        ROS_ERROR_STREAM("[AnomalyControl::robotsCallback]"
                         << "Rejected unknown robot ");
    }
}

void AnomalyControl::anomalyToggle(const std_msgs::Empty msg) {
    gazebo_msgs::ModelState model_msg;
    model_msg.model_name = std::string("person_walking_1");
    model_msg.pose.position = waypoints_[2].second[waypoints_[2].first];
    ++waypoints_[2].first;
    waypoints_[2].first %= waypoints_[2].second.size();

    model_msg.reference_frame = "world";
    obj_ctrl_.setObjectPose(boost::make_shared<gazebo_msgs::ModelState>(model_msg));
}

void AnomalyControl::anomalyControl(const ros::TimerEvent&) {
    if (waypoints_[2].first == 0) {
        return;
    }
    rm::PoseStamped start_pose;
    obj_ctrl_.getModelState("person_walking_1", start_pose.pose);

    vector<rm::PoseStamped> path = getPath(start_pose, goal_pose_);

    if (path.size() > 2) {
        auto next_waypoint = std::min(u_long(10.0 * move_speed_), path.size() - 1);

        gazebo_msgs::ModelState msg;
        msg.model_name = std::string("person_walking_1");
        msg.pose.position = path[next_waypoint].pose.position;
        msg.pose.orientation =
            utils::getQuat(utils::getYawFromRelativePose(start_pose.pose, msg.pose) - M_PI_2);
        msg.reference_frame = "world";
        obj_ctrl_.setObjectPose(boost::make_shared<gazebo_msgs::ModelState>(msg));
    }

    for (auto& robot : robots_) {
        double distance =
            utils::getDistance(start_pose.pose, robot.second.second.path.points.front());
        if (distance < anomaly_detection_distance_) {
            double yaw_diff = utils::getYawFromRelativePose(
                start_pose.pose, robot.second.second.path.points.front());
            if ((cos(yaw_diff - robot.second.second.path.points.front().yaw)) > .6) {
                nav_msgs::Odometry anomaly_msg;
                anomaly_msg.header.frame_id = "world";
                anomaly_msg.pose.pose = start_pose.pose;
                anomaly_pubs_[robot.second.second.vehicle_type.vehicle_type].publish(anomaly_msg);
            }
        }
    }
}

vector<PoseStamped> AnomalyControl::getPath(PoseStamped& start_pose, PoseStamped& goal_pose) {
    for (uint8_t i = 3; i < 6; i++) {
        if (robots_.find(i) != robots_.end()) {
            double wx = robots_[i].second.path.points.front().x;
            double wy = robots_[i].second.path.points.front().y;

            markPolygon(wx, wy, 254);
        }
    }
    vector<PoseStamped> path;

    start_pose.header.frame_id = "world";
    goal_pose.header.frame_id = "world";

    makePlan(start_pose, goal_pose, path);

    for (uint8_t i = 3; i < 6; i++) {
        if (robots_.find(i) != robots_.end()) {
            double wx = robots_[i].second.path.points.front().x;
            double wy = robots_[i].second.path.points.front().y;
            markPolygon(wx, wy, 0);
        }
    }

    return path;
}

void AnomalyControl::markPolygon(const double& wx, const double& wy, unsigned char cost) {
    std::vector<geometry_msgs::Point> poly;
    poly.emplace_back();
    poly.back().x = wx - 0.7;
    poly.back().y = wy - 0.7;
    poly.emplace_back();
    poly.back().x = wx + 0.7;
    poly.back().y = wy - 0.7;
    poly.emplace_back();
    poly.back().x = wx + 0.7;
    poly.back().y = wy + 0.7;
    poly.emplace_back();
    poly.back().x = wx - 0.7;
    poly.back().y = wy + 0.7;
    costmap_->setConvexPolygonCost(poly, cost);
}

void AnomalyControl::obstacleControl(const ros::TimerEvent&) {
    setStaticObstacle(waypoints_[0], "person_standing_1");
    setStaticObstacle(waypoints_[1], "person_standing_2");
}

void AnomalyControl::setStaticObstacle(std::pair<int, vector<rm::Point>>& traj,
    const std::string& name) {
    gazebo_msgs::ModelState msg;
    msg.model_name = name;
    msg.pose.position = traj.second[traj.first];
    msg.reference_frame = "world";
    bool blocked_by_robot = false;
    for (auto& robot : robots_) {
        double distance = utils::getDistance(msg.pose, robot.second.second.path.points.front());
        blocked_by_robot = (distance < 0.5);
    }
    if (!blocked_by_robot) {
        obj_ctrl_.setObjectPose(boost::make_shared<gazebo_msgs::ModelState>(msg));
        ++(traj.first);
        traj.first %= traj.second.size();
    }
}
}  // namespace mrrtpe_simulation