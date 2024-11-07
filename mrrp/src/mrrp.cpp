#include <algorithm>
#include <mrrp/mrrp.hpp>
#include <mrrtpe_msgs/Feedback.h>
#include <mrrtpe_utils/conversions.hpp>
#include <unordered_map>
#include <visualization_msgs/Marker.h>
namespace mrrtpe::mrrp {
namespace utils = mrrtpe::mrrtpe_utils;

MRRP::MRRP(ros::NodeHandle& nh, ros::NodeHandle& nh_private, string name, Costmap2DROS* cmap)
    : GlobalPlanner(name, cmap->getCostmap(), cmap->getGlobalFrameID())
    , passive_replan_(false)
    , robot_radius_(0.5)
    , visual_(false)
    , rate_(5.0)
    , robot_timeout_(5.0)
    , waypoint_dist_(1.0)
    , cmap_(cmap) {
    utils::getParam(nh_private, "robot_radius", robot_radius_);
    utils::getParam(nh_private, "visual", visual_);
    utils::getParam(nh_private, "rate", rate_);
    utils::getParam(nh_private, "robot_timeout", robot_timeout_);
    utils::getParam(nh_private, "waypoint_dist", waypoint_dist_);

    planner_req_sub_ = nh.subscribe("planner_req", 1, &MRRP::plannerReqCallback, this);
    robot_sub_ = nh.subscribe("robots", 10, &MRRP::robotStatusCallback, this);
    force_replan_sub_ = nh.subscribe("force_replan", 1, &MRRP::forceReplan, this);
    map_meta_data_sub_ = nh.subscribe("map_metadata", 1, &MRRP::mapMetaDataCallback, this);

    if (visual_) {
        marker_pub_ = nh.advertise<visualization_msgs::Marker>("path_marker", 1);
    }
    plan_pub_ = nh.advertise<mrrtpe_msgs::Plan>("mrrp_plan", 1);
}

void MRRP::robotStatusCallback(const mrrtpe_msgs::RobotStatusConstPtr& msg) {
    if (msg->vehicle_type.vehicle_type != mrrtpe_msgs::VehicleType::UNKNOWN) {
        robots_[msg->vehicle_type.vehicle_type] = std::make_pair(ros::Time::now(), *msg);
    } else {
        ROS_ERROR_STREAM("[MRRP::robotStatusCallback]"
                         << "Rejected unknown robot ");
    }
}

void MRRP::mapMetaDataCallback(const nav_msgs::MapMetaDataConstPtr& msg) {
    map_meta_data_ = msg;
}

void MRRP::plannerReqCallback(const mrrtpe_msgs::PlannerRequestPtr& msg) {
    req_.requests.clear();
    for (auto req : msg->requests) {
        bool goal_type = (req.goal_type.goal_type == mrrtpe_msgs::GoalType::FOLLOW_GOAL) ||
                         (req.goal_type.goal_type == mrrtpe_msgs::GoalType::NORMAL_GOAL) ||
                         (req.goal_type.goal_type == mrrtpe_msgs::GoalType::IDLE_GOAL);

        if (goal_type) {
            req_.requests.push_back(req);
        } else {
            ROS_ERROR_STREAM("[MRRP::plannerReqCallback]"
                             << "Goal rejected for robot " << int(req.vehicle_type.vehicle_type)
                             << ", since goal type was " << int(req.goal_type.goal_type));
        }
    }
    makeMultiPlan();
}

void MRRP::forceReplan(const std_msgs::EmptyConstPtr& msg) {
    ROS_INFO_STREAM("[MRRP::forceReplan]"
                    << " force replanning called");
    makeMultiPlan();
}

void MRRP::run() {
    ros::Rate loop_rate(rate_);
    while (ros::ok()) {
        ros::spinOnce();
        if (passive_replan_ || isReplanningNeeded()) {
            ROS_INFO_STREAM("[MRRP::run]"
                            << " passive replanning called");
            makeMultiPlan();
        }
        loop_rate.sleep();
    }
}

bool MRRP::isValidState(mrrtpe_msgs::RobotStatus& robot) {
    return (robot.state.state == mrrtpe_msgs::State::Idle) ||
           (robot.state.state == mrrtpe_msgs::State::Route) ||
           (robot.state.state == mrrtpe_msgs::State::PatrolIdle) ||
           (robot.state.state == mrrtpe_msgs::State::Follow);
}

bool MRRP::staticRobot(mrrtpe_msgs::RobotStatus& robot) {
    for (auto& req : req_.requests) {
        if ((req.vehicle_type == robot.vehicle_type) &&
            (robot.state.state != mrrtpe_msgs::State::Route ||
                robot.state.state != mrrtpe_msgs::State::Follow)) {
            double distance =
                utils::getDistance(utils::pointToPose(req.goal), robot.path.points.front());
            return distance < 0.6;
        }
    }
    return false;
}

void MRRP::makeMultiPlan() {
    if (!map_meta_data_) {
        ROS_ERROR_STREAM("[MRRP::makeMultiPlan]"
                         << " rejecting planning since there is no map meta data");
        return;
    }
    std::unordered_map<int, int> un_map;
    paths_.clear();

    passive_replan_ = false;
    cmap_->resetLayers();
    ros::Duration(2.0).sleep();

    for (auto& req : req_.requests) {
        if (robots_.find(req.vehicle_type.vehicle_type) != robots_.end()) {
            auto robot = robots_[req.vehicle_type.vehicle_type].second;
            if (!isValidState(robot)) {
                ROS_ERROR_STREAM("[MRRP::makeMultiPlan]"
                                 << " rejecting planning due invalid state for robot "
                                 << int(req.vehicle_type.vehicle_type));
                continue;
            }
            if (robots_[req.vehicle_type.vehicle_type].first - ros::Time::now() >
                ros::Duration(robot_timeout_)) {
                ROS_ERROR_STREAM("[MRRP::checkRobotStatus]"
                                 << "Ignoring plan request for robot "
                                 << int(robot.vehicle_type.vehicle_type)
                                 << " due to connection timeout");
                continue;
            }

            geometry_msgs::PoseStamped start, goal;

            start.pose = utils::pointToPose(robot.path.points[0]);
            goal.pose = utils::pointToPose(req.goal);
            start.header.frame_id = frame_id_;
            goal.header.frame_id = frame_id_;
            paths_[req.vehicle_type.vehicle_type].clear();

            if (req.vehicle_type.vehicle_type > mrrtpe_msgs::VehicleType::MAV_END) {
                // clear area near robot
                std::vector<geometry_msgs::Point> poly;
                poly.clear();
                poly.emplace_back();
                poly.back().x = start.pose.position.x - 1.2;
                poly.back().y = start.pose.position.y - 1.2;
                poly.emplace_back();
                poly.back().x = start.pose.position.x + 1.2;
                poly.back().y = start.pose.position.y - 1.2;
                poly.emplace_back();
                poly.back().x = start.pose.position.x + 1.2;
                poly.back().y = start.pose.position.y + 1.2;
                poly.emplace_back();
                poly.back().x = start.pose.position.x - 1.2;
                poly.back().y = start.pose.position.y + 1.2;
                costmap_->setConvexPolygonCost(poly, 0);

                for (auto& agent : robots_) {
                    if (agent.second.second.vehicle_type.vehicle_type ==
                            req.vehicle_type.vehicle_type ||
                        agent.second.second.vehicle_type.vehicle_type <
                            mrrtpe_msgs::VehicleType::UGV_1) {
                        continue;
                    }
                    double wx = agent.second.second.path.points.front().x;
                    double wy = agent.second.second.path.points.front().y;

                    // map blocked by other robots
                    std::vector<geometry_msgs::Point> poly;
                    poly.clear();
                    poly.emplace_back();
                    poly.back().x = wx - 1.2;
                    poly.back().y = wy - 1.2;
                    poly.emplace_back();
                    poly.back().x = wx + 1.2;
                    poly.back().y = wy - 1.2;
                    poly.emplace_back();
                    poly.back().x = wx + 1.2;
                    poly.back().y = wy + 1.2;
                    poly.emplace_back();
                    poly.back().x = wx - 1.2;
                    poly.back().y = wy + 1.2;
                    costmap_->setConvexPolygonCost(poly, 254);

                    unsigned int mx, my;
                    cmap_->getCostmap()->worldToMap(wx, wy, mx, my);
                    for (int j = -int(20); j < int(20); j++) {
                        for (int i = -int(20); i < int(20); i++) {
                            un_map[cmap_->getCostmap()->getIndex(mx + i, my + j)] =
                                agent.second.second.vehicle_type.vehicle_type;
                        }
                    }
                }

                makePlan(start,
                    goal,
                    0,
                    paths_[req.vehicle_type.vehicle_type],
                    req.vehicle_type.vehicle_type,
                    robot_radius_,
                    un_map);
            } else {
                makePlanMav(start, goal, paths_[req.vehicle_type.vehicle_type]);
            }

            if (!paths_[req.vehicle_type.vehicle_type].empty()) {
                mrrtpe_msgs::Plan plan;
                plan.vehicle_type = req.vehicle_type;
                plan.anomaly_received = req.anomaly_received;
                plan.path.goal_type = req.goal_type;
                computePoints(paths_[req.vehicle_type.vehicle_type], plan, req);
                plan_pub_.publish(plan);

                if (visual_) {
                    visualization_msgs::Marker marker;
                    marker.header.frame_id = frame_id_;
                    marker.header.stamp = ros::Time::now();
                    marker.ns = std::to_string(req.vehicle_type.vehicle_type);
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.orientation.w = 1;

                    for (auto& pose : plan.path.points) {
                        marker.points.push_back(utils::pointToPose(pose).position);
                    }
                    marker.id = 0;
                    marker.type = visualization_msgs::Marker::POINTS;

                    marker.scale.x = 0.1;
                    marker.scale.y = 0.1;
                    marker.color.r = 1.0; 
                    marker.color.g = 1.0;
                    marker.color.b = 1.0;
                    marker.color.a = 1.0;
                    marker_pub_.publish(marker);
                }
                double distance = utils::getDistance(goal.pose,plan.path.points.back());
                if(distance > 0.1) { 
                    passive_replan_ = true;
                }

            } else {
                passive_replan_ = true;
                mrrtpe_msgs::Plan plan;
                plan.vehicle_type = req.vehicle_type;
                plan.anomaly_received = req.anomaly_received;
                plan.path.goal_type = req.goal_type;
                plan.path.points.push_back(
                    robots_[req.vehicle_type.vehicle_type].second.path.points.front());
                plan_pub_.publish(plan);
                ROS_ERROR_STREAM("[MRRP::makeMultiPlan]"
                                 << " not able to create plan for robot "
                                 << int(req.vehicle_type.vehicle_type));
            }
        } else {
            passive_replan_ = true;
            ROS_ERROR_STREAM("[MRRP::makeMultiPlan]"
                             << " rejecting plan since not able to retrive info on robot "
                             << int(req.vehicle_type.vehicle_type));
        }
    }
}

void MRRP::computePoints(const vector<PoseStamped>& path,
    mrrtpe_msgs::Plan& plan,
    const mrrtpe_msgs::Feedback& req) {
    if (req.vehicle_type.vehicle_type > mrrtpe_msgs::VehicleType::MAV_END) {
        mrrtpe_msgs::Point point = utils::poseToPoint(path[0].pose);
        plan.path.points.push_back(point);
        double distance = 0;
        for (auto& pose : path) {
            distance = utils::getDistance(pose.pose, point);
            if (distance > waypoint_dist_) {
                point = utils::poseToPoint(pose.pose);
                plan.path.points.push_back(point);
            }
        }
        if (plan.vehicle_type.vehicle_type != path.back().pose.position.z) {
            passive_replan_ = true;
            mrrtpe_msgs::VehicleType type;
            type.vehicle_type = path.back().pose.position.z;
            plan.path.points.back().preconditions.push_back(type);
        } else {
            plan.path.points.push_back(req.goal);
        }
    } else {
        for (auto& point : path) {
            plan.path.points.push_back(utils::poseToPoint(point.pose));
        }
    }
}

bool MRRP::isReplanningNeeded() {
    bool replan = false;
    for (auto& path : paths_) {
        for (auto& point : path.second) {
            for (auto& robot : robots_) {
                if (path.first == robot.first) {
                    continue;
                }
                double distance =
                    utils::getDistance(point.pose, robot.second.second.path.points.front());
                replan = (distance < (4 * robot_radius_));
                if (replan) {
                    return replan;
                }
            }
        }
    }
    return replan;
}

void MRRP::makePlanMav(const PoseStamped& start,
    const PoseStamped& goal,
    vector<PoseStamped>& plan) {
    plan.push_back(start);
    int count = 1 + (utils::getDistance(start.pose, goal.pose) / 1.0);
    int m = 1;
    int n = count - m;
    while (ros::ok()) {
        if (m == count) {
            break;
        }
        PoseStamped pose;
        pose.pose.position.x = (n * (start.pose.position.x) + m * (goal.pose.position.x)) / (count);
        pose.pose.position.y = (n * (start.pose.position.y) + m * (goal.pose.position.y)) / (count);
        pose.pose.orientation =
            utils::getQuat(utils::getYawFromRelativePose(pose.pose, plan.back().pose));
        plan.push_back(pose);
        ++m;
        --n;
    }
    plan.push_back(goal);
}

}  // namespace mrrtpe::mrrp
