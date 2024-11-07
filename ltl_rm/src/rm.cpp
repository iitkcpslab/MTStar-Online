#include "ltl_rm/MT_star.cpp"
#include "ltl_rm/MTplanner_variables.h"
#include <ltl_rm/rm.hpp>

namespace ltl_rm::rm {
RM::RM(ros::NodeHandle& nh, ros::NodeHandle& nh_private, std::set<std::pair<v_type, std::pair<double, double>>> avail_robots, std::string path_prefix)
    : robots_count_(avail_robots.size())
    , replan0_(true)
    , replan1_(true)
    , replan02_(false)
    , replan12_(false)
    , feedback_(false)
    , path_prefix_(path_prefix)
    , anomaly_seen_(false)
    , anomaly_handled_(false)
    , anomaly_robot_(mrrtpe_msgs::VehicleType::UNKNOWN) {
    robots_.clear();
    phi0_.clear();
    phi1_.clear();
    phi02_.clear();
    phi12_.clear();
    location_.clear();
    received_.clear();
    waiting_.clear();
    goals_.clear();
    refill_loc_phi0_.clear();
    refill_loc_phi1_.clear();
    plan_all_robots.requests.clear();
    for (auto type : avail_robots) {
        if (type.first == mrrtpe_msgs::VehicleType::UNKNOWN) {
            continue;
        }
        waiting_.insert(type.first);
        ROS_INFO("Registering Robot: %d", type.first);
        if ((type.first == mrrtpe_msgs::VehicleType::MAV_1) || (type.first == mrrtpe_msgs::VehicleType::MAV_2) ||
            (type.first == mrrtpe_msgs::VehicleType::MAV_3) || (type.first == mrrtpe_msgs::VehicleType::MAV_4)) {
            phi0_.insert(type.first);
            refill_loc_phi0_.insert(std::make_pair(type.second, mrrtpe_msgs::VehicleType::UNKNOWN));
            ROS_INFO_STREAM("Robot"<< type.first<<" is registered in Phi_0 and its starting point is.("<<type.second.first<<","<<type.second.second<<")");
        } else if ((type.first == mrrtpe_msgs::VehicleType::UGV_1) || (type.first == mrrtpe_msgs::VehicleType::UGV_2) ||
                   (type.first == mrrtpe_msgs::VehicleType::UGV_3)) {
            phi1_.insert(type.first);
            refill_loc_phi1_.insert(std::make_pair(type.second, mrrtpe_msgs::VehicleType::UNKNOWN));
            ROS_INFO_STREAM("Robot"<< type.first<<" is registered in Phi_1 and its starting point is.("<<type.second.first<<","<<type.second.second<<")");
        }
    }
    if(phi0_.size()==0){
        replan0_=false;
    }
    if(phi1_.size()==0){
        replan1_=false;
    }
    robots_count_ = waiting_.size();
    ROS_INFO("%d robots are registered.", robots_count_);
    robot_sub_ = nh.subscribe("robots", 10, &RM::robotStatusCallback, this);
    goal_pub_ = nh.advertise<mrrtpe_msgs::PlannerRequest>("ltl_plan", 10);
}

void RM::robotStatusCallback(const mrrtpe_msgs::RobotStatusConstPtr& msg) {
    if (msg->vehicle_type.vehicle_type != mrrtpe_msgs::VehicleType::UNKNOWN) {
        robots_[msg->vehicle_type.vehicle_type] = *msg;
        if ((msg->anomaly_seen) && (!anomaly_seen_) && (!anomaly_handled_)) {
            anomaly_seen_ = true;
            ROS_INFO("Anamoly Detected at location:(%f,%f)", msg->anomaly.x, msg->anomaly.y);
            anomaly_loc_->x = msg->anomaly.x;
            anomaly_loc_->y = msg->anomaly.y;
            anomaly_loc_->z = 0;
        }
        if (waiting_.find(msg->vehicle_type.vehicle_type) != waiting_.end() && anomaly_robot_ != (mrrtpe_msgs::VehicleType::UNKNOWN)) {
            if (msg->vehicle_type.vehicle_type == anomaly_robot_ && msg->state.state == mrrtpe_msgs::State::Follow) {
                waiting_.erase(msg->vehicle_type.vehicle_type);
                received_.insert(msg->vehicle_type.vehicle_type);
                ROS_INFO("Received new status report for robot: %d", msg->vehicle_type.vehicle_type);
            }
        }
        if (waiting_.find(msg->vehicle_type.vehicle_type) != waiting_.end() &&
            (msg->state.state == mrrtpe_msgs::State::Idle || msg->state.state == mrrtpe_msgs::State::PatrolIdle) &&
            (msg->path.goal_type.goal_id == location_[msg->vehicle_type.vehicle_type])) {
            waiting_.erase(msg->vehicle_type.vehicle_type);
            received_.insert(msg->vehicle_type.vehicle_type);
            ROS_INFO("Received new status report for robot: %d", msg->vehicle_type.vehicle_type);
        }
    } else {
        ROS_ERROR_STREAM("[RM::robotStatusCallback]"
                         << "Rejected unknown robot status");
    }
}

// Mapping Robots to their nearest charging locations.
void RM::map_goals(std::set<std::pair<std::pair<double, double>, v_type>>& location, std::set<v_type>& robots) {
    for (auto i : robots) {
        if (robots_[i].path.goal_type.goal_id == 100) {
            continue;
        }
        double dist = 10000;
        std::pair<std::pair<double, double>, bool> loc;
        for (auto p = location.begin(); p != location.end(); p++) {
            if (p->second!=mrrtpe_msgs::VehicleType::UNKNOWN)
                continue;
            double x = robots_[i].path.points[0].x - p->first.first;
            double y = robots_[i].path.points[0].y - p->first.second;
            x = x * x;
            y = y * y;
            double temp = std::sqrt(x + y);
            if (dist > temp) {
                dist = temp;
                loc = *p;
            }
        }
        std::vector<mrrtpe_msgs::Point> goal1;
        mrrtpe_msgs::Point goal;
        goal.x = loc.first.first+0.5;
        goal.y = loc.first.second+0.5;
        goal.z = 0;
        goal1.push_back(goal);
        location_[i] = 100;
        goals_[i] = goal1;
        goal1.clear();
        location.erase(loc);
        location.insert(std::make_pair(std::make_pair(loc.first.first, loc.first.second), i));
        ROS_INFO("Robot %d charging goal location (%f,%f) is added.", i,goals_[i][0].x,goals_[i][0].y);
    }
}
// Mapping robots to their goals
void RM::map_goals(std::vector<std::vector<std::pair<double, double>>>& trajectories, std::set<v_type>& robots) {
    if (trajectories.size() != robots.size()) {
        ROS_ERROR_STREAM("[RM::map_goals]"
                         << "Rejected mismatch in no of trajectories and number of robots");
    }
    for (auto i : robots) {
        double dist = 10000;
        int loc = -1;
        for (int p = 0; p < trajectories.size(); p++) {
            double x = robots_[i].path.points[0].x - trajectories[p][0].first;
            double y = robots_[i].path.points[0].y - trajectories[p][0].second;
            x = x * x;
            y = y * y;
            double temp = std::sqrt(x + y);
            if (dist > temp) {
                dist = temp;
                loc = p;
            }
        }
        std::vector<mrrtpe_msgs::Point> goal1;
        for (int p = 0; p < trajectories[loc].size(); p++) {
            mrrtpe_msgs::Point goal;
            goal.x = trajectories[loc][p].first+0.5;
            goal.y = trajectories[loc][p].second+0.5;
            goal.z = 0;
            if(p>0 && goal1.front()==goal){
                continue;
            }
            goal1.push_back(goal);
        }
        location_[i] = 0;
        goals_[i] = goal1;
        goal1.clear();
        trajectories.erase(trajectories.begin() + loc);
        ROS_INFO("Robot %d normal goal is added with path length %d.", i,goals_[i].size());
    }
}
// Mapping anomaly location to the nearest UGV.
void RM::map_goals() {
    v_type robot;
    if (anomaly_robot_ == mrrtpe_msgs::VehicleType::UNKNOWN) {
        double dist = 10000;
        for (auto i : phi1_) {
            double x = robots_[i].path.points[0].x - anomaly_loc_->x;
            double y = robots_[i].path.points[0].y - anomaly_loc_->y;
            x = x * x;
            y = y * y;
            double temp = std::sqrt(x + y);
            if (dist > temp) {
                dist = temp;
                robot = i;
            }
        }
        anomaly_robot_ = robot;
        phi1_.erase(robot);
        replan1_=true;
        std::vector<mrrtpe_msgs::Point> goal1;
        mrrtpe_msgs::Point goal;
        goal.x = anomaly_loc_->x;
        goal.y = anomaly_loc_->y;
        goal.z = 0;
        goal1.push_back(goal);
        location_[robot] = 200;
        goals_[robot] = goal1;
        goal1.clear();
        ROS_WARN("Robot %d goal is added for anomaly tracking", robot);
    } 
}

void RM::generatePlan(std::string path_prefix) {
    if (replan0_) {
        int k = phi0_.size();
        if(k==4){
        std::vector<std::vector<std::pair<double, double>>> trajectories={
        {std::make_pair(3,0),std::make_pair(0,3)},
        {std::make_pair(0,16),std::make_pair(3,19)},
        {std::make_pair(16,19),std::make_pair(19,16)},
        {std::make_pair(19,4),std::make_pair(16,0)}};
        map_goals(trajectories, phi0_);
        ROS_INFO("Using pre-generated plan for 4 MAV's");
        }
        else if(k==2){
        std::vector<std::vector<std::pair<double, double>>> trajectories={
        {std::make_pair(2,9),std::make_pair(9,9),std::make_pair(6,8)},
        {std::make_pair(8,3),std::make_pair(4,1),std::make_pair(1,2)}};
        map_goals(trajectories, phi0_);
        ROS_INFO("Using pre-generated plan for 3 MAV's");
        }
        else if(k>0){
        ROS_INFO("Calling mt_star for phi_%d for %d robots with path %s.", 0, k, path_prefix_.c_str());
        std::vector<std::vector<std::pair<double, double>>>trajectories = mt_star(0, k, path_prefix_);
        map_goals(trajectories, phi0_);
        }
        else{
            ROS_WARN_STREAM("No available MAV for replanning:");
        }
        ROS_INFO("Plan generated successfully");
        replan0_ = false;
        ROS_INFO("Replanning done for MAV's");
    }
    if (replan1_) {
        int k = phi1_.size();
        if(k==3){
        std::vector<std::vector<std::pair<double, double>>> trajectories={
        {std::make_pair(18,6),std::make_pair(14,7)},
        {std::make_pair(15,18),std::make_pair(12,18)},
        {std::make_pair(8,12),std::make_pair(4,13)}};
        map_goals(trajectories, phi1_);
        ROS_INFO("Using pre-generated plan for 3 UGV's");
        }
        else if(k>0){
        ROS_INFO("Calling mt_star for phi_%d for %d robots with path %s.", 1, k, path_prefix_.c_str());
        vector<vector<pair<double, double>>> trajectories = mt_star(1, k, path_prefix_);
        map_goals(trajectories, phi1_);
        }
        else{
            ROS_WARN_STREAM("No available UGV for replanning:");
        }
        ROS_INFO("Plan generated successfully");
        replan1_ = false;
        ROS_INFO("Replanning done for UGV's");
    }
    if (replan02_) {
        map_goals(refill_loc_phi0_, phi02_);
        replan02_ = false;
        ROS_INFO("Charging location added for MAV");
    }
    if (replan12_) {
        map_goals(refill_loc_phi1_, phi12_);
        replan12_ = false;
        ROS_INFO("Charging location added for UGV");
    }
    // if (anomaly_seen_ and !(anomaly_handled_)) {
    //     map_goals();
    //     ROS_INFO("UGV goal mapped to anomaly location");
    // }
    ROS_INFO("New Plan Generated for all Robots.");
}

void RM::insertInPhi0(v_type vehicle) {
    phi02_.erase(vehicle);
    phi0_.insert(vehicle);
    std::pair<std::pair<double,double>,v_type> temp;
    for(auto i:refill_loc_phi0_){
        if(i.second==vehicle){
            temp=i;
            break;
        }
    }
    refill_loc_phi0_.erase(temp);
    refill_loc_phi0_.insert(std::make_pair(std::make_pair(temp.first.first,temp.first.second),mrrtpe_msgs::VehicleType::UNKNOWN));
    replan0_ = true;
    ROS_INFO("Robot %d is registered in Phi_0 as its battery is fully charged and the robot is now operational.", vehicle);
}
void RM::insertInPhi1(v_type vehicle) {
    phi12_.erase(vehicle);
    phi1_.insert(vehicle);
    std::pair<std::pair<double,double>,v_type> temp;
    for(auto i:refill_loc_phi1_){
        if(i.second==vehicle){
            temp=i;
            break;
        }
    }
    refill_loc_phi1_.erase(temp);
    refill_loc_phi1_.insert(std::make_pair(std::make_pair(temp.first.first,temp.first.second),mrrtpe_msgs::VehicleType::UNKNOWN));
    replan1_ = true;
    ROS_INFO("Robot %d is registered in Phi_0 as its battery is fully charged and the robot is now operational.", vehicle);
}
void RM::insertInPhi02(v_type vehicle) {
    phi0_.erase(vehicle);
    phi02_.insert(vehicle);
    replan0_ = true;
    replan02_ = true;
    ROS_INFO("Robot %d is registered in Phi_02 as its battery is below the threshold", vehicle);
}
void RM::insertInPhi12(v_type vehicle) {
    phi1_.erase(vehicle);
    phi12_.insert(vehicle);
    replan1_ = true;
    replan12_ = true;
    ROS_INFO("Robot %d is registered in Phi_12 as its battery is below the threshold:", vehicle);
}

void RM::run() {
    ros::spinOnce();
    ros::Duration(1.0).sleep();

    if (received_.size() == robots_count_) {
        feedback_ = true;
        ROS_INFO("Received feedback for all robots.");
    }
    if (feedback_ && anomaly_seen_ && !anomaly_handled_) {
        map_goals();
        ROS_WARN("UGV goal mapped to anomaly location");
    }
    if (feedback_ && anomaly_seen_ && anomaly_handled_ && robots_[anomaly_robot_].state.state == mrrtpe_msgs::State::PatrolIdle) {
        phi1_.insert(anomaly_robot_);
        ROS_WARN("Registering UGV:%d as phi1_ as the anomaly is lost.", anomaly_robot_);
        anomaly_robot_ = mrrtpe_msgs::VehicleType::UNKNOWN;
        anomaly_seen_ = false;
        anomaly_handled_ = false;
        replan1_=true;
    }

    if (feedback_) {
        vector<v_type> temp;
        temp.clear();
        for (auto vehicle : phi0_) {
            if ((robots_[vehicle].battery_status.battery_status == mrrtpe_msgs::BatteryStatus::BATTERY_LOW) or
                (robots_[vehicle].battery_status.battery_status == mrrtpe_msgs::BatteryStatus::BATTERY_CRITICAL_LOW)) {
                //insertInPhi02(vehicle);
                temp.push_back(vehicle);
            }
        }
        for(auto it:temp){
            insertInPhi02(it);
        }
        temp.clear();
        for (auto vehicle : phi1_) {
            if ((robots_[vehicle].battery_status.battery_status == mrrtpe_msgs::BatteryStatus::BATTERY_LOW) or
                (robots_[vehicle].battery_status.battery_status == mrrtpe_msgs::BatteryStatus::BATTERY_CRITICAL_LOW)) {
                //insertInPhi12(vehicle);
                temp.push_back(vehicle);
            }
        }
        for(auto it:temp){
            insertInPhi12(it);
        }
        temp.clear();
        for (auto vehicle : phi02_) {
            if ((robots_[vehicle].battery_status.battery_status == mrrtpe_msgs::BatteryStatus::BATTERY_FULL) or
                (robots_[vehicle].battery_status.battery_status == mrrtpe_msgs::BatteryStatus::BATTERY_OPERATIONAL)) {
                //insertInPhi0(vehicle);
                temp.push_back(vehicle);

            }
        }
        for(auto it:temp){
            insertInPhi0(it);
        }
        temp.clear();
        for (auto vehicle : phi12_) {
            if ((robots_[vehicle].battery_status.battery_status == mrrtpe_msgs::BatteryStatus::BATTERY_FULL) or
                (robots_[vehicle].battery_status.battery_status == mrrtpe_msgs::BatteryStatus::BATTERY_OPERATIONAL)) {
                //insertInPhi1(vehicle);
                temp.push_back(vehicle);
            }
        }
        for(auto it:temp){
            insertInPhi1(it);
        }
        temp.clear();
    }

    while ((replan0_ || replan1_ || replan02_ || replan12_) && feedback_) {
        ROS_INFO("Calling Planner to generate plans as there is a change in Environment.");
        generatePlan(path_prefix_);
        // for (auto& goal : goals_) {
        //     for (auto& point : goal.second) {
        //         ROS_WARN_STREAM(point.x << " " << point.y << " " << goal.second.size());
        //     }
        // }
    }

    if (!(replan0_ || replan1_ || replan02_ || replan12_) && feedback_) {
        ROS_INFO("Printing Robots and their goals:");
        plan_all_robots.requests.clear();
        for (auto it : phi0_) {
            ROS_INFO("Loading next goal location for robot:%d", it);
            auto data = robots_[it];
            mrrtpe_msgs::Feedback plan_1_robot;
            plan_1_robot.anomaly_received = data.anomaly_seen;
            ROS_INFO("Goal Length: %d",goals_[it].size());
            for(auto p:goals_[it]){
                ROS_INFO_STREAM("("<<p.x<<","<<p.y<<")");
            }
            plan_1_robot.goal = goals_[it][location_[it] % goals_[it].size()];
            plan_1_robot.goal_type.goal_type = mrrtpe_msgs::GoalType::NORMAL_GOAL;
            plan_1_robot.goal_type.goal_id = ++location_[it];
            plan_1_robot.vehicle_type.vehicle_type = it;
            plan_all_robots.requests.push_back(plan_1_robot);
            ROS_INFO("Next Goal location for robot %d is (%f,%f) as per plan generated with location id:%d.", it, plan_1_robot.goal.x, plan_1_robot.goal.y,location_[it]);
        }
        for (auto it : phi1_) {
            ROS_INFO("Loading next goal location for robot:%d", it);
            auto data = robots_[it];
            mrrtpe_msgs::Feedback plan_1_robot;
            plan_1_robot.anomaly_received = data.anomaly_seen;
            ROS_INFO("Goal Length: %d",goals_[it].size());
            for(auto p:goals_[it]){
                ROS_INFO_STREAM("("<<p.x<<","<<p.y<<")");
            }
            ROS_INFO("Loaded next goal location for robot:%d", it);
            plan_1_robot.goal = goals_[it][location_[it] % goals_[it].size()];
            plan_1_robot.goal_type.goal_type = mrrtpe_msgs::GoalType::NORMAL_GOAL;
            plan_1_robot.goal_type.goal_id = ++location_[it];
            plan_1_robot.vehicle_type.vehicle_type = it;
            plan_all_robots.requests.push_back(plan_1_robot);
            ROS_INFO("Next Goal location for robot %d is (%f,%f) as per plan generated with location id:%d.", it, plan_1_robot.goal.x, plan_1_robot.goal.y,location_[it]);
        }
        for (auto it : phi02_) {
            auto data = robots_[it];
            if (data.path.goal_type.goal_type == mrrtpe_msgs::GoalType::NORMAL_GOAL) {
                ROS_INFO("Loading next goal location for robot:%d", it);
                mrrtpe_msgs::Feedback plan_1_robot;
                plan_1_robot.anomaly_received = data.anomaly_seen;
                ROS_INFO("Goal Length: %d",goals_[it].size());
                for(auto p:goals_[it]){
                    ROS_INFO_STREAM("("<<p.x<<","<<p.y<<")");
                }
                plan_1_robot.goal = goals_[it][0];
                plan_1_robot.goal_type.goal_type = mrrtpe_msgs::GoalType::IDLE_GOAL;
                plan_1_robot.goal_type.goal_id = location_[it];
                plan_1_robot.vehicle_type.vehicle_type = it;
                ROS_INFO("Sending Robot %d to charging location (%f,%f).", it,goals_[it][0].x,goals_[it][0].y);
                plan_all_robots.requests.push_back(plan_1_robot);
            }
        }
        for (auto it : phi12_) {
            auto data = robots_[it];
            if (data.path.goal_type.goal_type == mrrtpe_msgs::GoalType::NORMAL_GOAL) {
                ROS_INFO("Loading next goal location for robot:%d", it);
                mrrtpe_msgs::Feedback plan_1_robot;
                plan_1_robot.anomaly_received = data.anomaly_seen;
                ROS_INFO("Goal Length: %d",goals_[it].size());
                for(auto p:goals_[it]){
                    ROS_INFO_STREAM("("<<p.x<<","<<p.y<<")");
                }
                plan_1_robot.goal = goals_[it][0];
                plan_1_robot.goal_type.goal_type = mrrtpe_msgs::GoalType::IDLE_GOAL;
                plan_1_robot.goal_type.goal_id = location_[it];
                plan_1_robot.vehicle_type.vehicle_type = it;
                ROS_INFO("Sending Robot %d to charging location (%f,%f).", it,goals_[it][0].x,goals_[it][0].y);
                plan_all_robots.requests.push_back(plan_1_robot);
            }
        }
        if (!(anomaly_handled_) && anomaly_seen_) {
            ROS_INFO("Loading next goal location for robot:%d", anomaly_robot_);
            mrrtpe_msgs::Feedback plan_1_robot;
            plan_1_robot.anomaly_received = robots_[anomaly_robot_].anomaly_seen;
            plan_1_robot.goal = goals_[anomaly_robot_][0];
            plan_1_robot.goal_type.goal_type = mrrtpe_msgs::GoalType::FOLLOW_GOAL;
            plan_1_robot.goal_type.goal_id = location_[anomaly_robot_];
            plan_1_robot.vehicle_type.vehicle_type = anomaly_robot_;
            ROS_INFO("Sending Robot %d to Intrusion location (%f,%f).", anomaly_robot_,goals_[anomaly_robot_][0].x,goals_[anomaly_robot_][0].y);
            plan_all_robots.requests.push_back(plan_1_robot);
            anomaly_handled_ = true;
        }
        ROS_INFO("Dispatching robots to their next goal locations.");
        goal_pub_.publish(plan_all_robots);
        std::swap(received_, waiting_);
        feedback_ = false;
        ros::Duration(3.0).sleep();
        ROS_INFO("Waiting for feedback.");
    }
}
}  // namespace ltl_rm::rm