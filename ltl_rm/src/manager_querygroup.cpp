#include "ltl_rm/MT_star.cpp"
#include "ltl_rm/MTplanner_variables.h"
#include "ltl_rm/manager_querygroup.hpp"
#include <fstream>
#include <string>
#include <thread>

#define GRID_SZ 1  // 1 for use case2 and 2 for usecase1

namespace motion_planning::manager {
Manager::Manager(ros::NodeHandle& nh,
    ros::NodeHandle& nh_private,
    std::vector<mrrtpe_msgs::VehicleType::_vehicle_type_type> robots,
    std::string path_prefix)
    // : _g1(nh, "1", "1", 4, {mrrtpe_msgs::VehicleType::UGV_1, mrrtpe_msgs::VehicleType::UGV_2}, path_prefix),
    // _g2(nh, "2", "1", 2, {mrrtpe_msgs::VehicleType::MAV_1}, path_prefix)
    : _g1(nh,
          "1",
          "2",
          4,
          {mrrtpe_msgs::VehicleType::UGV_1},
          path_prefix)
    , _g2(nh,
          "2",
          "2",
          4,
          {mrrtpe_msgs::VehicleType::UGV_2},
          path_prefix)
    , _g3(nh, "3", "2", 2, {mrrtpe_msgs::VehicleType::MAV_1}, path_prefix)
{
    _robotsData.clear();
    _robotsName.clear();

    _robotSub = nh.subscribe("robots", 5, &Manager::robotStatusCallback, this);
    // _goalPub = nh.advertise<mrrtpe_msgs::PlannerRequest>("dispatcher_plan", 10);
    _envPub = nh.advertise<mrrtpe_msgs::Env>("env_data", 10);

    // service clients
    _service_item1 = nh.advertiseService("item1", &Manager::Serv_item1, this);
    _service_item2 = nh.advertiseService("item2", &Manager::Serv_item2, this);
    _service_item3 = nh.advertiseService("item3", &Manager::Serv_item3, this);
    _service_d1 = nh.advertiseService("d1", &Manager::Serv_d1, this);
    _service_d2 = nh.advertiseService("d2", &Manager::Serv_d2, this);
    _service_d3 = nh.advertiseService("d3", &Manager::Serv_d3, this);

    
    for (int i = 0; i < robots.size(); i++) {
        if (robots[i] != mrrtpe_msgs::VehicleType::UNKNOWN) {
            _robotsName.push_back(robots[i]);
            ROS_INFO_STREAM("[Manager::Manager]"
                            << "Registered Robot: " << unsigned(robots[i]));
        }
    }
    // for (int i = 0; i < robots_2.size(); i++) {
    //     if (robots_2[i] != mrrtpe_msgs::VehicleType::UNKNOWN) {
    //         _robotsName.push_back(robots_2[i]);
    //         ROS_INFO_STREAM("[Manager::Manager]"
    //                         << "Registered Robot: " << unsigned(robots_2[i]));
    //     }
    // }
    ROS_INFO_STREAM("[Manager::Manager]" << _robotsName.size() << " robots registered");
    _envData.battery1 = true;
    _envData.battery2 = true;
    _envData.battery3 = true;
    _envData.intruder_detected = false;
    _envData.item1 = false;
    _envData.item2 = false;
    _envData.item3 = false;
    _envData.d1 = false;
    _envData.d2 = false;
    _envData.d3 = false;
    _intH=false;
}

bool Manager::Serv_item1(mrrtpe_msgs::Environment::Request& req,
    mrrtpe_msgs::Environment::Response& res) {
    _envData.item1 = req.set;
    ROS_INFO_STREAM("[Manager::Sitem1] Setting Environment variable item1 to: " << _envData.item1);
    res.op = _envData.item1;
    return true;
}

bool Manager::Serv_item2(mrrtpe_msgs::Environment::Request& req,
    mrrtpe_msgs::Environment::Response& res) {
    _envData.item2 = req.set;
    ROS_INFO_STREAM("[Manager::Sitem1] Setting Environment variable item2 to: " << _envData.item2);
    res.op = _envData.item2;
    return true;
}

bool Manager::Serv_item3(mrrtpe_msgs::Environment::Request& req,
    mrrtpe_msgs::Environment::Response& res) {
    _envData.item3 = req.set;
    ROS_INFO_STREAM("[Manager::Sitem1] Setting Environment variable item1 to: " << _envData.item3);
    res.op = _envData.item3;
    return true;
}

bool Manager::Serv_d1(mrrtpe_msgs::Environment::Request& req,
    mrrtpe_msgs::Environment::Response& res) {
    _envData.d1 = req.set;
    ROS_INFO_STREAM("[Manager::Sitem1] Setting Environment variable d 1 to: " << _envData.d1);
    res.op = _envData.d1;
    return true;
}

bool Manager::Serv_d2(mrrtpe_msgs::Environment::Request& req,
    mrrtpe_msgs::Environment::Response& res) {
    _envData.d2 = req.set;
    ROS_INFO_STREAM("[Manager::Sitem1] Setting Environment variable d2 to: " << _envData.d2);
    res.op = _envData.d2;
    return true;
}

bool Manager::Serv_d3(mrrtpe_msgs::Environment::Request& req,
    mrrtpe_msgs::Environment::Response& res) {
    _envData.d3 = req.set;
    ROS_INFO_STREAM("[Manager::Sitem1] Setting Environment variable item1 to: " << _envData.d3);
    res.op = _envData.d3;
    return true;
}

void Manager::robotStatusCallback(const mrrtpe_msgs::RobotStatusConstPtr& msg) {
    if (msg->vehicle_type.vehicle_type != mrrtpe_msgs::VehicleType::UNKNOWN) {
        _robotsData[msg->vehicle_type.vehicle_type] = *msg;
        if (msg->anomaly_seen && !_intH) {
            _anomalyLoc->x = msg->anomaly.x;
            _anomalyLoc->y = msg->anomaly.y;
            _anomalyLoc->z = 0;
            if (!_envData.intruder_detected) {
                // _intD=ros::Time::now();
                ROS_INFO_STREAM("[Manager::robotStatusCallback] Intruder Detected at location: ("
                                << msg->anomaly.x << "," << msg->anomaly.y << ")");
            }
            _envData.intruder_detected = true;
        }
        else if(ros::Duration(300)<(ros::Time::now()-_intHT)){
            _intH=false;
        }
        if (msg->vehicle_type.vehicle_type == mrrtpe_msgs::VehicleType::UGV_1) {
            if ((msg->battery_status.battery_status ==
                    mrrtpe_msgs::BatteryStatus::BATTERY_CRITICAL_LOW) or
                (msg->battery_status.battery_status == mrrtpe_msgs::BatteryStatus::BATTERY_LOW)) {
                _envData.battery1 = false;
            } else {
                _envData.battery1 = true;
            }
        }
        if (msg->vehicle_type.vehicle_type == mrrtpe_msgs::VehicleType::UGV_2) {
            if ((msg->battery_status.battery_status ==
                    mrrtpe_msgs::BatteryStatus::BATTERY_CRITICAL_LOW) or
                (msg->battery_status.battery_status == mrrtpe_msgs::BatteryStatus::BATTERY_LOW)) {
                _envData.battery2 = false;
            } else {
                _envData.battery2 = true;
            }
        }
        if (msg->vehicle_type.vehicle_type == mrrtpe_msgs::VehicleType::MAV_1) {
            if ((msg->battery_status.battery_status ==
                    mrrtpe_msgs::BatteryStatus::BATTERY_CRITICAL_LOW) or
                (msg->battery_status.battery_status == mrrtpe_msgs::BatteryStatus::BATTERY_LOW)) {
                _envData.battery3 = false;
            } else {
                _envData.battery3 = true;
            }
        }
    // ROS_WARN_STREAM("I have seen some data on callback of robots");
    } else {
        ROS_ERROR_STREAM("[Manager::robotStatusCallback]"
                         << "Rejected unknown robot status");
    }
    _envPub.publish(_envData);
}
      
void Manager::updateEnv() {  // The Logic here is to be updated for every new problem as the phi_env
                            // may be different for each problem.
    // //USE_CASE_1_START
    // _g1._phiE[0] = ((_envData.battery1 && _envData.battery2) || ((_envData.battery1 or _envData.battery2) and !_envData.intruder_detected));
    // _g1._phiE[1] = ((_envData.battery1 or _envData.battery2) and _envData.intruder_detected);
    // _g1._phiE[2] = !_envData.battery1;
    // _g1._phiE[3] = !_envData.battery2;

    // _g2._phiE[0] = _envData.battery3;
    // _g2._phiE[1] = !_envData.battery3;
    // // USE_CASE_1_END

    // USE_CASE_2_START
    _g1._phiE[0] = (_envData.battery1 and _envData.item1);
    _g1._phiE[1] = (_envData.battery1 and _envData.item2 and !_envData.item1);
    _g1._phiE[2] = (_envData.battery1 and _envData.item3 and !_envData.item2 and !_envData.item1);
    _g1._phiE[3] = !_envData.battery1;

    _g2._phiE[0] = (_envData.battery2 and _envData.d1);
    _g2._phiE[1] = (_envData.battery2 and _envData.d2 and !_envData.d1);
    _g2._phiE[2] = (_envData.battery2 and _envData.d3 and !_envData.d2 and !_envData.d1);
    _g2._phiE[3] = !_envData.battery2;

    _g3._phiE[0] = _envData.battery3;
    _g3._phiE[1] = !_envData.battery3;
    // USE_CASE_2_END

    // //USE_CASE1

    // _phie[0] = ((_envData.battery1 and _envData.battery2) or
    //             ((_envData.battery1 or _envData.battery2) and !_envData.intruder_detected));
    // _phie[1] = _envData.battery3;
    // _phie[2] = ((_envData.battery1 or _envData.battery2) and _envData.intruder_detected);
    // _phie[3] = !_envData.battery1;
    // _phie[4] = !_envData.battery2;
    // _phie[5] = !_envData.battery3;

    // USE_CASE2
    //  _phie[0] = (_envData.battery1 and _envData.item1 and !_envData.item2 and !_envData.item3);
    //  _phie[1] = (_envData.battery1 and _envData.item2 and !_envData.item1 and !_envData.item3);
    //  _phie[2] = (_envData.battery1 and _envData.item3 and !_envData.item2 and !_envData.item1);
    //  _phie[3] = !_envData.battery1;
    //  _phie[4] = (_envData.battery2 and _envData.d1 and !_envData.d2 and !_envData.d3);
    //  _phie[5] = (_envData.battery2 and _envData.d2 and !_envData.d1 and !_envData.d3);
    //  _phie[6] = (_envData.battery2 and _envData.d3 and !_envData.d2 and !_envData.d1);
    //  _phie[7] = !_envData.battery2 and _envData.battery1;
    //  _phie[8] = _envData.battery3;
    //  _phie[9] = !(_envData.battery3);
    ROS_INFO_STREAM("[Manager::updateEnv] _phiE updated for the groups.");
}

void Manager::run() {
    ros::spinOnce();
    // ros::Duration(1.0).sleep();
    updateEnv();
    //USE_CASE_2 Comment next 2 line in case of USE_CASE_1
    std::thread t3(&QueryGroup::QueryGroup::run, &_g3, _robotsData, _envData, _anomalyLoc);
    ros::Duration(2.0).sleep();
    std::thread t1(&QueryGroup::QueryGroup::run, &_g2, _robotsData, _envData, _anomalyLoc);
    ros::Duration(2.0).sleep();
    std::thread t2(&QueryGroup::QueryGroup::run, &_g1, _robotsData, _envData, _anomalyLoc);
    t1.join();
    t2.join();
    t3.join();  // USE_CASE_2 Comment this line in case of USE_CASE_1
    
    //USE_CASE_1 Uncomment the next lines for use-case-1

    if (_g1._intruderH || _g2._intruderH) {
        _envData.intruder_detected = false;
        _g1._intruderH = false;
        _g2._intruderH = false;
        _intHT=ros::Time::now();
        _intH=true;
    }
    
    ROS_INFO_STREAM("[Manager::run] hey there!");
}

}  // namespace motion_planning::manager
