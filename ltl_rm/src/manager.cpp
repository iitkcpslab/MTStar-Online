#include "ltl_rm/manager.hpp"
#include "ltl_rm/MT_star.cpp"
#include "ltl_rm/MTplanner_variables.h"
#include <fstream>
#include <string>


#define GRID_SZ 2  //1 for use case2 and 2 for usecase1

namespace motion_planning::manager {
Manager::Manager(int m, std::string path_prefix)
    : _replan(m)
    , _status(m, false)
    , _queries(m)
    , _feedback(false)
    , _planningRequired(false){
    _queryFile = path_prefix + "query.dat";
    _queriesFile = path_prefix + "queries.dat";
    _mapFile = path_prefix + "map_11.dat";
    std::ifstream fin;
    fin.open(_queriesFile);
    for (int i = 0; i < m; i++) {
        std::string line;
        std::getline(fin, line);
        _queries[i] = line;
        std::cout << "Query " << i << ": " << _queries[i] << " Registered." << std::endl;
    }
    fin.close();
    mergeQueries();
    _trajectory.clear();
    // goal_pub_ = nh.advertise<mrrtpe_msgs::PlannerRequest>("dispatcher_plan", 10);
}
Manager::Manager(ros::NodeHandle& nh, ros::NodeHandle& nh_private,int m,std::vector<mrrtpe_msgs::VehicleType::_vehicle_type_type> robots,std::string path_prefix)
    : _replan(m)
    , _status(m, false)
    , _phie(m,false)
    , _queries(m)
    , _feedback(false)
    , _planningRequired(false) {
    
    _trajectory.clear();
    _robotsData.clear();
    _robotsName.clear();
    _waitingReport.clear();
    _receivedReport.clear();

    _queryFile = path_prefix + "query.dat";

    //Use Case 1
    _queriesFile = path_prefix + "queries_uc1.dat";
    _mapFile = path_prefix + "map_11.dat";

    // // Use Case 2
    // _queriesFile = path_prefix + "queries_uc2.dat";
    // _mapFile = path_prefix + "map_20.dat";

    _robotSub=nh.subscribe("robots",10,&Manager::robotStatusCallback,this);
    _goalPub=nh.advertise<mrrtpe_msgs::PlannerRequest>("dispatcher_plan",10);
    _envPub=nh.advertise<mrrtpe_msgs::Env>("env_data",10);

    //service clients
    _service_item1 = nh.advertiseService("item1",&Manager::Serv_item1,this);
    _service_item2 = nh.advertiseService("item2", &Manager::Serv_item2, this);
    _service_item3 = nh.advertiseService("item3", &Manager::Serv_item3, this);
    _service_d1 = nh.advertiseService("d1", &Manager::Serv_d1, this);
    _service_d2 = nh.advertiseService("d2", &Manager::Serv_d2, this);
    _service_d3 = nh.advertiseService("d3", &Manager::Serv_d3, this);

    std::ifstream fin;
    fin.open(_queriesFile);
    for (int i = 0; i < m; i++) {
        std::string line;
        std::getline(fin, line);
        _queries[i] = line;
        ROS_INFO_STREAM("[Manager::Manager]"<< "Query " << i << ": " << _queries[i] << " Registered.");
    }
    fin.close();
    mergeQueries();

    for (int i=0;i<robots.size();i++) {
        if (robots[i] != mrrtpe_msgs::VehicleType::UNKNOWN) {
            _robotsName.push_back(robots[i]);
            _waitingReport.insert(robots[i]);
            ROS_INFO_STREAM("[Manager::Manager]"
                            << "Registered Robot: " << unsigned(robots[i]));
        }
    }
    ROS_INFO_STREAM("[Manager::Manager]"<< _robotsName.size() << " robots registered");
    _goalId=0;
    //_feedback=true;
    _envData.battery1=true;
    _envData.battery2=true;
    _envData.battery3=true;
    _envData.intruder_detected=false;
    _envData.item1=false;
    _envData.item2=false;
    _envData.item3=false;
    _envData.d1=false;
    _envData.d2=false;
    _envData.d3=false;
    _intRobot = mrrtpe_msgs::VehicleType::UNKNOWN;
}

bool Manager::Serv_item1(mrrtpe_msgs::Environment::Request &req, mrrtpe_msgs::Environment::Response &res){
    _envData.item1=req.set;
    ROS_INFO_STREAM("[Manager::Sitem1] Setting Environment variable item1 to: " << _envData.item1);
    res.op = _envData.item1;
    return true;
}

bool Manager::Serv_item2(mrrtpe_msgs::Environment::Request& req, mrrtpe_msgs::Environment::Response& res) {
    _envData.item2 = req.set;
    ROS_INFO_STREAM("[Manager::Sitem1] Setting Environment variable item2 to: " << _envData.item2);
    res.op = _envData.item2;
    return true;
}

bool Manager::Serv_item3(mrrtpe_msgs::Environment::Request& req, mrrtpe_msgs::Environment::Response& res) {
    _envData.item3 = req.set;
    ROS_INFO_STREAM("[Manager::Sitem1] Setting Environment variable item1 to: " << _envData.item3);
    res.op = _envData.item3;
    return true;
}

bool Manager::Serv_d1(mrrtpe_msgs::Environment::Request& req, mrrtpe_msgs::Environment::Response& res) {
    _envData.d1 = req.set;
    ROS_INFO_STREAM("[Manager::Sitem1] Setting Environment variable d 1 to: " << _envData.d1);
    res.op = _envData.d1;
    return true;
}

bool Manager::Serv_d2(mrrtpe_msgs::Environment::Request& req, mrrtpe_msgs::Environment::Response& res) {
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
        if (msg->anomaly_seen) {
            _anomalyLoc->x = msg->anomaly.x;
            _anomalyLoc->y = msg->anomaly.y;
            _anomalyLoc->z = 0;
            if (!_envData.intruder_detected) {
                ROS_INFO_STREAM("[Manager::robotStatusCallback] Intruder Detected at location: ("
                                << msg->anomaly.x << "," << msg->anomaly.y << ")");
            }
            _envData.intruder_detected = true;
        }
        if (msg->vehicle_type.vehicle_type == mrrtpe_msgs::VehicleType::UGV_1){
            if ((msg->battery_status.battery_status ==
                    mrrtpe_msgs::BatteryStatus::BATTERY_CRITICAL_LOW) or
                (msg->battery_status.battery_status == mrrtpe_msgs::BatteryStatus::BATTERY_LOW)) {
                _envData.battery1=false;
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
        if (_waitingReport.find(msg->vehicle_type.vehicle_type) != _waitingReport.end() &&
            ((((msg->state.state == mrrtpe_msgs::State::Idle ||
                   msg->state.state == mrrtpe_msgs::State::PatrolIdle) &&
                 msg->path.goal_type.goal_id == _goalId)) ||
                msg->path.goal_type.goal_type == mrrtpe_msgs::GoalType::FOLLOW_GOAL)) {
            _waitingReport.erase(msg->vehicle_type.vehicle_type);
            _receivedReport.insert(msg->vehicle_type.vehicle_type);
            for (auto i = _plan4AllRobots.requests.begin(); i != _plan4AllRobots.requests.end();i++) {  //Removing goal request for a particular robot as it has already reached its goal
                if (i->vehicle_type.vehicle_type == msg->vehicle_type.vehicle_type){
                    _plan4AllRobots.requests.erase(i);
                    ROS_INFO_STREAM("[Manager::robotStatusCallback] Removing goal request for robot:"
                                    << unsigned(msg->vehicle_type.vehicle_type));
                    break;
                }
            }
                ROS_INFO_STREAM("[Manager::robotStatusCallback] Received new status for robot:"
                                << unsigned(msg->vehicle_type.vehicle_type));
            if(_receivedReport.size()==_robotsName.size()){
                _feedback=true;
            }
        }
    } else {
        ROS_ERROR_STREAM("[Manager::robotStatusCallback]"
                         << "Rejected unknown robot status");
    }
    //ROS_WARN_STREAM("I have seen some data on callback of robots");
    _envData.feedback = _feedback;
    _envPub.publish(_envData);
}

void Manager::mergeQueries() {
    // this is hard-coded for this example, need to do a string matching on queries to generate
    // logically
    //USE CASE 1
    _replan.merge(0, 2);
    _replan.merge(0, 3);
    _replan.merge(0, 4);
    _replan.merge(1, 5);
    


    //USE CASE 2
    // _replan.merge(0, 1);
    // _replan.merge(0, 2);
    // _replan.merge(0, 3);
    // _replan.merge(3, 7);
    // _replan.merge(4, 5);
    // _replan.merge(4, 6);
    // _replan.merge(4, 7);
    // _replan.merge(8, 9);

    ROS_INFO_STREAM("[Manager::mergeQueries] Queries merged");
}

void Manager::generateAutomata(std::string& phi) {
    ofstream fout;
    fout.open(_queryFile);
    if(fout) {
        fout << "/home/rotor/src/drdo_sim_ws/src/spot-2.6/bin/ltl2tgba --spin '" << phi<<"'" << endl;  // Add the spot tool path
    }
    fout.close();
    ROS_INFO_STREAM("[Manager::generateAutomata] Generated the query: " << phi);
}

int Manager::updateEnv() {//The Logic here is to be updated for every new problem as the phi_env may be different for each problem.
    // for(int i=0;i<_queries.size();i++){
    //     if(phi_env_i=false){//Add the logic to check phi_environment for each i
    //         _status[i]=false;
    //     }
    //     else if(_status[i]=false){
    //         _status[i]=true;
    //         _replan.update(i,1,true);
    //     }
    //     if(_trajectory.empty()){//Add the logic to check the dispatcher status
    //         std::cout<<"No Trajectory available so, setting feedback to true."<<std::endl;
    //         _feedback=true;
    //     }
    // }
    //Some Locha here
    ros::spinOnce();


    // //USE_CASE1
    _phie[0] = ((_envData.battery1 and _envData.battery2) or
                 ((_envData.battery1 or _envData.battery2) and !_envData.intruder_detected));
    _phie[1]=_envData.battery3;
    _phie[2] = ((_envData.battery1 or _envData.battery2) and _envData.intruder_detected);
    _phie[3] = !_envData.battery1;
    _phie[4] = !_envData.battery2;
    _phie[5] = !_envData.battery3;

    //USE_CASE2
    // _phie[0] = (_envData.battery1 and _envData.item1 and !_envData.item2 and !_envData.item3);
    // _phie[1] = (_envData.battery1 and _envData.item2 and !_envData.item1 and !_envData.item3);
    // _phie[2] = (_envData.battery1 and _envData.item3 and !_envData.item2 and !_envData.item1);
    // _phie[3] = !_envData.battery1;
    // _phie[4] = (_envData.battery2 and _envData.d1 and !_envData.d2 and !_envData.d3);
    // _phie[5] = (_envData.battery2 and _envData.d2 and !_envData.d1 and !_envData.d3);
    // _phie[6] = (_envData.battery2 and _envData.d3 and !_envData.d2 and !_envData.d1);
    // _phie[7] = !_envData.battery2 and _envData.battery1;
    // _phie[8] = _envData.battery3;
    // _phie[9] = !(_envData.battery3);

    for (int i = 0; i < _phie.size(); i++) {
        if (_phie[i] == false) {  // Phi_Env_0
            if (_status[i]) {
                _status[i] = false;
                _replan.update(i, 1, true);
            }
            ROS_INFO_STREAM("[Manager::updateEnv] phi[" << i << "]: false");
        } else if (_status[i] == false) {
            ROS_INFO_STREAM("[Manager::updateEnv] phi[" << i << "]: true with replan");
            _status[i] = true;
            _replan.update(i, 1, true);
        }
        ROS_INFO_STREAM("[Manager::updateEnv] phi[" << i << "]: " << _status[i]);
}
    // if (phie0 == false) {  // Phi_Env_0
    //     if(_status[0]){
    //         _status[0] = false;
    //         _replan.update(0,1,true);
    //     }
    //     ROS_INFO_STREAM("[Manager::updateEnv] phi0: false");
    // }
    // else if (_status[0] == false) {
    //     ROS_INFO_STREAM("[Manager::updateEnv] phi0: true with replan");
    //     _status[0] = true;
    //     _replan.update(0, 1, true);
    // }
    // ROS_INFO_STREAM("[Manager::updateEnv] Phi0:" << _status[0]);
    // if(phie1==false) {//Phi_Env_1
    //     if (_status[1]) {
    //         _status[1] = false;
    //         _replan.update(1, 1, true);
    //     }
    //     ROS_INFO_STREAM("[Manager::updateEnv] phi1: false");
    // } else if (_status[1] == false) {
    //     _status[1] = true;
    //     _replan.update(1, 1, true);
    //     ROS_INFO_STREAM("[Manager::updateEnv] phi1: true");
    // }
    // ROS_INFO_STREAM("[Manager::updateEnv] Phi1:" << _status[1]);
    // if(phie2==false) {//Phi_Env_2
    //     if (_status[2]) {
    //         _status[2] = false;
    //         _intRobot = mrrtpe_msgs::VehicleType::UNKNOWN;
    //         _replan.update(2, 1, true);
    //     }
    //     ROS_INFO_STREAM("[Manager::updateEnv] phi2: false");
    // } else if (_status[2] == false) {
    //     _status[2] = true;
    //     _replan.update(2, 1, true);
    //     ROS_INFO_STREAM("[Manager::updateEnv] phi2: true");
    // }
    // ROS_INFO_STREAM("[Manager::updateEnv] Phi2:" << _status[2]);
    // if(phie3==false) {//Phi_Env_3
    //     if (_status[3]) {
    //         _status[3] = false;
    //         _replan.update(3, 1, true);
    //     }
    //     ROS_INFO_STREAM("[Manager::updateEnv] phi3: false");
    // } else if(_status[3]==false){
    //     _status[3]=true;
    //     _replan.update(3,1,true);
    //     ROS_INFO_STREAM("[Manager::updateEnv] phi3: true");
    // }
    // ROS_INFO_STREAM("[Manager::updateEnv] Phi3:" << _status[3]);
    // if (phie4 == false) {  // Phi_Env_4
    //     if (_status[4]) {
    //         _status[4] = false;
    //         _replan.update(4, 1, true);
    //     }
    //     ROS_INFO_STREAM("[Manager::updateEnv] phi4: false");
    // } else if (_status[4] == false) {
    //     _status[4] = true;
    //     _replan.update(4, 1, true);
    //     ROS_INFO_STREAM("[Manager::updateEnv] phi4: true");
    // }
    // ROS_INFO_STREAM("[Manager::updateEnv] Phi4:" << _status[4]);
    // if (phie5 == false) {  // Phi_Env_5
    //     if (_status[5]) {
    //         _status[5] = false;
    //         _replan.update(5, 1, true);
    //     }
    //     ROS_INFO_STREAM("[Manager::updateEnv] phi5: false");
    // } else if (_status[5] == false) {
    //     _status[5] = true;
    //     _replan.update(5, 1, true);
    //     ROS_INFO_STREAM("[Manager::updateEnv] phi5: true");
    // }
     int no=0;
    // for(int i=0;i<6;i++){
    //     if(_status[i]) no=(no+i+1);
    // }
    // ROS_INFO_STREAM("[Manager::updateEnv] Phi5:" << _status[5]);
    ROS_INFO_STREAM("[Manager::updateEnv] Updated Environment");
    if(!_feedback and (ros::Duration(45)<(ros::Time::now()-_lastRequest)) and _plan4AllRobots.requests.size()>0){    //Add to check if feedback is waiting for more than 45 secs then send the goal request again
        _lastRequest=ros::Time::now();
        _goalPub.publish(_plan4AllRobots);
        ROS_INFO_STREAM("[Manager::updateEnv] Dispatching the goal again as feedback waiting time is more than 45 seconds.");
    }
    return no;
}

bool Manager::dispatchTask(std::vector<std::vector<std::pair<double, double>>> trajectory) {
    
    int size=0;
    for (int k = 0; k < _robotsName.size();k++){
        if(size<trajectory[k].size())
            size = trajectory[k].size();
    }
    ROS_INFO_STREAM("[Manager::dispatchTask] Suffix Length:" << size);

    for(int i=0;i<size;i++){
        
        while((!_feedback) and (_robotsData.size()==_robotsName.size())){
            ros::spinOnce();
            ros::Duration(1.0).sleep();
            if (!_feedback and (ros::Duration(45) < (ros::Time::now() - _lastRequest)) and
                _plan4AllRobots.requests.size() >
                    0) {  // Add to check if feedback is waiting for more than 45 secs then send the
                          // goal request again
                _lastRequest = ros::Time::now();
                _goalPub.publish(_plan4AllRobots);
                ROS_INFO_STREAM("[Manager::dispatchTask] Dispatching the goal again as feedback "
                                "waiting time is more than 45 seconds.");
            }
            //_goalPub.publish(_plan4AllRobots);
        };

        ROS_INFO_STREAM("[Manager::dispatchTask] Received Feedback.");
        ROS_INFO_STREAM("[Manager::dispatchTask] Suffix No:" << i);
        _plan4AllRobots.requests.clear();
        
        for(int r=0;r<_robotsName.size();r++){
            if(i>=trajectory[r].size()) continue; //add something so that intruder goal goal is not sent multiple times.
            if (_robotsData.find(_robotsName[r]) == _robotsData.end()) {
                ROS_INFO_STREAM("[Manager::dispatchTask] Robot not found in status");
                continue;
            }
        
            auto data = _robotsData[_robotsName[r]];
            ROS_INFO_STREAM("[Manager::dispatchTask] Generating goal location for Robot: "
                            << unsigned(_robotsName[r]));
            mrrtpe_msgs::Feedback plan41Robot;

            plan41Robot.anomaly_received=data.anomaly_seen;
            plan41Robot.vehicle_type.vehicle_type=_robotsName[r];
            plan41Robot.goal_type.goal_id=_goalId+1;
            auto it = data.path.points.begin();

            ROS_INFO_STREAM("[Manager::dispatchTask] Robot " << unsigned(_robotsName[r])
                                                             << " currently at (" << double(it->x)
                                                             << ", " << double(it->y) << ")");

            if(trajectory[r][i].first==-1 and trajectory[r][i].second==-1){ //Stay at Same Place

                plan41Robot.goal.x = it->x;
                plan41Robot.goal.y = it->y;
                plan41Robot.goal_type.goal_type = mrrtpe_msgs::GoalType::IDLE_GOAL;

                ROS_INFO_STREAM("[Manager::dispatchTask] Next Goal Location for Robot "
                                << unsigned(plan41Robot.vehicle_type.vehicle_type) << " is ("
                                << double(plan41Robot.goal.x) << "," << double(plan41Robot.goal.y)
                                << ") of type: Stay where You are "
                                << "with id: " << unsigned(plan41Robot.goal_type.goal_id));
                _plan4AllRobots.requests.push_back(plan41Robot);

            } else if (trajectory[r][i].first == 0 and
                           trajectory[r][i].second == 10 and _envData.intruder_detected) {  // Intruder Goal check, add something in condition of loop for
                                     // intruder goal

                plan41Robot.goal.x = _anomalyLoc->x;
                plan41Robot.goal.y = _anomalyLoc->y;
                plan41Robot.goal_type.goal_type = mrrtpe_msgs::GoalType::FOLLOW_GOAL;

                _envData.intruder_detected = false;
                _intRobot = plan41Robot.vehicle_type.vehicle_type;
                ROS_INFO_STREAM("[Manager::dispatchTask] Next Goal Location for Robot "
                                << unsigned(plan41Robot.vehicle_type.vehicle_type) << " is ("
                                << double(plan41Robot.goal.x) << "," << double(plan41Robot.goal.y)
                                << ") of type: Follow the intruder "
                                << "with id: " << unsigned(plan41Robot.goal_type.goal_id));
                _plan4AllRobots.requests.push_back(plan41Robot);

            } else {  // Move to next Goal

                plan41Robot.goal.x = (trajectory[r][i].first+0.5)*GRID_SZ;
                plan41Robot.goal.y = (trajectory[r][i].second+0.5)*GRID_SZ;
                plan41Robot.goal_type.goal_type = mrrtpe_msgs::GoalType::NORMAL_GOAL;

                ROS_INFO_STREAM("[Manager::dispatchTask] Next Goal Location for Robot "
                                << unsigned(plan41Robot.vehicle_type.vehicle_type) << " is ("
                                << double(plan41Robot.goal.x) << "," << double(plan41Robot.goal.y)
                                << ") of type: normal "
                                << "with id: " << unsigned(plan41Robot.goal_type.goal_id));
                _plan4AllRobots.requests.push_back(plan41Robot);
            }

            // _plan4AllRobots.requests.push_back(plan41Robot);
            _receivedReport.erase(_robotsName[r]);
            _waitingReport.insert(_robotsName[r]);
        }

        ROS_INFO_STREAM("[Manager::dispatchTask] Dispatching Goals.");

        _goalPub.publish(_plan4AllRobots);
        _lastRequest=ros::Time::now();

        //std::swap(_receivedReport,_waitingReport);

        if (_waitingReport.size()>=1) {
            _feedback = false;
            ros::Duration(15.0).sleep();
            _goalId++;
            ROS_INFO_STREAM(
                "[Manager::dispatchTask] Waiting for Robots to execute their Assigned Tasks");
        }

    }
    // std::cout << "Printing Trajectories for " << trajectory.size() << " robots." << std::endl;
    // for (int i = 0; i < trajectory.size(); i++) {
    //     std::cout << "Robot:" << i << " trajectory length= " << trajectory[i].size() << std::endl;
    //     for (int j = 0; j < trajectory[i].size(); j++) {
    //         std::cout << "(" << trajectory[i][j].first << "," << trajectory[i][j].second << ") ";
    //     }
    //     std::cout << std::endl;
    // }
    ROS_WARN_STREAM("[Manager::dispatchTask] Suffix Successfully Executed on the Robots.");
    return true;
}

void Manager::run() {
    ros::spinOnce();//Find a better place to put
    ros::Duration(1.0).sleep();
    int no=updateEnv();
    if(_feedback){
    ROS_INFO_STREAM("[Manager::run] Checking Validity of the earlier plan");
    std::string phi="True";
    for (int i = 0; i < _queries.size(); i++) {
        phi = phi + " && [](";
        bool bracket=false;
        if (_status[i] and _replan.get(i) == 1) {
            phi = phi + " && " + _queries[i];
            if(i>1 and !bracket) {
                bracket=true;
                phi = phi + ")";}
            ROS_INFO_STREAM("[Manager::run] Added query " << i << ": '" << _queries[i]
                                                              << "' for planning.");
            _replan.update(i, 0, false);
            _planningRequired = true;
            ROS_WARN_STREAM("[Manager::run] Setting Replanning to true");
        }
    }
    if (_planningRequired) {
        ROS_WARN_STREAM("[Manager::run] Replanning Required");
        // updateJts();
        generateAutomata(phi);
        //for use_case 1
        // _trajectory.clear();
        // ROS_WARN_STREAM("[Manager::run] Calling Planner");
        // _trajectory = mt_star(_mapFile, _queryFile,-1);  // Call the planner in the correct manner //no
        // ROS_WARN_STREAM("[Manager::run] Printing Trajectories");
        // for (int i = 0; i < _trajectory.size(); i++) {
        //     cout << "Robot " << i << " Trajectory\n";
        //     for (int j = 0; j < _trajectory[i].size(); j++) {
        //         cout << "(" << _trajectory[i][j].first << "," << _trajectory[i][j].second << ") ";
        //     }
        //     cout << "\n";
        // }

        // for use_case 2
        //_trajectory.clear();

        ROS_WARN_STREAM("[Manager::run] Calling Planner");
        std::vector<std::vector<std::pair<double, double>>> trajectory_new;
        trajectory_new = mt_star(_mapFile, _queryFile, -1);  // Call the planner in the correct manner //no 
        ROS_WARN_STREAM("[Manager::run] Printing Trajectories");
        if (_trajectory.size() < 1)
            _trajectory = trajectory_new;
        for (int i = 0; i < _robotsName.size(); i++) {
            cout << "Robot " << int(_robotsName[i]) << " Trajectory\n";
            if (trajectory_new[i].size()>0){
                cout << "Robot has new Trajectory\n";
                _trajectory[i].clear();
                _trajectory[i] = trajectory_new[i];
            }
            for (int j = 0; j < _trajectory[i].size(); j++) {
                cout << "(" << _trajectory[i][j].first << "," << _trajectory[i][j].second<< ") ";
            }
        }

        ROS_INFO_STREAM("[Manager::run] New Plan generated");
        _planningRequired = false;
        ROS_INFO_STREAM("[Manager::run] Setting Replanning to false");
    }
    if(!_trajectory.empty()) {
        ROS_WARN_STREAM("[Manager::run] Dispatching Suffix");
        dispatchTask(_trajectory);
    }
    }
}

}  // namespace motion_planning::manager
