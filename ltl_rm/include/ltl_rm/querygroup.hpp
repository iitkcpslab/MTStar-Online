#ifndef _QUERYGROUP__
#define _QUERYGROUP__
#include "ltl_rm/MT_star.cpp"
#include "ltl_rm/MTplanner_variables.h"
#include <fstream>
#include <iostream>
#include <mrrtpe_msgs/Env.h>
#include <mrrtpe_msgs/PlannerRequest.h>
#include <mrrtpe_msgs/RobotStatus.h>
#include <ros/ros.h>
#include <unordered_map>
#include <vector>
#include <algorithm>
// USE_CASE_2 1
// USE_CASE_1 2
#define GRID_SZ 1

typedef mrrtpe_msgs::VehicleType::_vehicle_type_type v_type;
typedef std::unordered_map<v_type, mrrtpe_msgs::RobotStatus> robot_map;
namespace motion_planning::QueryGroup {
class QueryGroup {
  public:
    QueryGroup(ros::NodeHandle& nh,std::string id,std::string use_case,
        int m,
        std::vector<mrrtpe_msgs::VehicleType::_vehicle_type_type> robots,
        std::string path_prefix)
        : _id(id)
        , _queries(m)
        , _status(m, false)
        , _phiE(m, false)
        , _intruderH(false)
        , _feedback(false)
        , _planningRequired(false)
        , _suffixDone(true) {
        _trajectory.clear();
        _robotsName.clear();
        _queriesFile = path_prefix+"queries_uc"+use_case+"_"+_id+".dat";
        _mapFile = path_prefix + "map_uc" + use_case + "_" + _id + ".dat";
        _queryFile = path_prefix + "query_uc" + use_case + "_" + _id + ".dat";
        _dispatchTopic="planner_req_"+_id;

        std::ifstream fin;
        ROS_INFO_STREAM("[QueryGroup::QueryGroup] " << _id << ":Query File:" << _queriesFile );
        fin.open(_queriesFile);
        for (int i = 0; i < m; i++) {
            std::string line;
            std::getline(fin, line);
            _queries[i] = line;
            ROS_INFO_STREAM("[QueryGroup::QueryGroup] "<<_id<< ":Query " << i << ": " << _queries[i] << " Registered.");
        }
        fin.close();
        
        for (int i=0;i<robots.size();i++) {
        if (robots[i] != mrrtpe_msgs::VehicleType::UNKNOWN) {
            _robotsName.push_back(robots[i]);
            _waitingReport.insert(robots[i]);
            ROS_INFO_STREAM("[QueryGroup::QueryGroup] "<<_id
                            << " : Registered Robot: " << unsigned(robots[i]));
        }
        }
        ROS_INFO_STREAM("[QueryGroup::QueryGroup] "<<_id<<" :"<< _robotsName.size() << " robots registered");
        _goalId=0;
        _Fid=-1;
        
        _goalPub = nh.advertise<mrrtpe_msgs::PlannerRequest>(_dispatchTopic, 10);
    }

    void generateAutomata(std::string& phi) {
        ofstream fout;
        fout.open(_queryFile);
        if (fout) {
        fout << "/home/rotor/src/drdo_sim_ws/src/spot-2.6/bin/ltl2tgba --spin '" << phi << "'"
             << endl;  // Add the spot tool path
        }
        fout.close();
        ROS_INFO_STREAM("[QueryGroup::generateAutomata] "<<_id<<" :Generated the query: " << phi);
    }

    void checkGoalId(){
      // Increments GoalId by 1. Also checks if goal id is greater than suffix length then sets suffix to true.
      // ROS_INFO_STREAM("[QueryGroup::checkGoalId] "<<_id<<" : incrementing Goal Id");
      // _goalId++;
      if (_goalId > _maxGoalSZ) { // removing the equality check
        _suffixDone=true;
        _goalId=0;
        ROS_WARN_STREAM("[QueryGroup::checkGoalId] " << _id << " : Suffix Complete.");
      }
    }

    bool update(const robot_map& _robotsData){
      // Check for feedback(Add the logic)
      for(int i=0;i<_robotsName.size();i++){
        ROS_INFO_STREAM("[QueryGroup::update] "<<_id<<": Searching for robot status for robot: "<< unsigned(_robotsName[i]));
        if(_waitingReport.find(_robotsName[i])!=_waitingReport.end()){
            ROS_INFO_STREAM("[QueryGroup::update] " << _id << ": Robot in waiting");
            robot_map::const_iterator itt = _robotsData.find(_robotsName[i]);
            if (itt != _robotsData.end()){
                ROS_INFO_STREAM("[QueryGroup::update] "
                                << _id <<" Robot:"<< unsigned(_robotsName[i])
                                << ": Current State: " << unsigned(itt->second.state.state)
                                << " Goal Id:" << itt->second.path.goal_type.goal_id
                                << " Goal Type:" << unsigned(itt->second.path.goal_type.goal_type));
                if (((itt->second.state.state == mrrtpe_msgs::State::Idle ||
                         itt->second.state.state == mrrtpe_msgs::State::PatrolIdle) &&
                        itt->second.path.goal_type.goal_id == _goalId) ||
                    (itt->second.path.goal_type.goal_type == mrrtpe_msgs::GoalType::FOLLOW_GOAL &&
                        itt->second.state.state == mrrtpe_msgs::State::Route)) {
                    ROS_INFO_STREAM("[QueryGroup::update] " << _id << " :Received feedback robot: "
                                                            << unsigned(_robotsName[i]));
                    _waitingReport.erase(_robotsName[i]);
                    for (auto j = _plan.requests.begin(); j != _plan.requests.end();
                         j++)  // Removing goal request for a particular robot as it has already
                               // reached its goal
                    {
                        if (j->vehicle_type.vehicle_type == itt->second.vehicle_type.vehicle_type) {
                            _plan.requests.erase(j);
                            ROS_INFO_STREAM("[QueryGroup::update] "
                                            << _id << " :Removing goal request for robot:"
                                            << unsigned(itt->second.vehicle_type.vehicle_type));
                            break;
                        }
                    }
                }
                // else {
                //     ROS_INFO_STREAM("[QueryGroup::update] "
                //                     << _id
                //                     << ": Current State: " << unsigned(itt->second.state.state)
                //                     << " Goal Id:" << itt->second.path.goal_type.goal_id
                //                     << " Goal Type:" << unsigned(itt->second.path.goal_type.goal_type));
                // }
            }
        else{
          ROS_INFO_STREAM("[QueryGroup::update] "
                        << _id << ": RObot not found in _robotData");  
        }
      }
      else{
          ROS_INFO_STREAM("[QueryGroup::update] " << _id << ": Robot Not in waiting");
      }
      }
      if(_waitingReport.size()==0){
        ROS_INFO_STREAM("[QueryGroup::update] "<<_id<<" :Received feedback for all robots. Setting it to TRUE.");
        _feedback=true;
        checkGoalId();
      }
      else{
        for(auto i:_waitingReport){
        ROS_INFO_STREAM("[QueryGroup::update] " << _id << " :Waiting feedback for robot: "<<unsigned(i));
        }
      }

      bool flag=false;
      for (int i = 0; i < _phiE.size(); i++) {
        if(_phiE[i]) flag=true;
        if (!_phiE[i]) {
            if (_status[i]) {
                _status[i] = false;
                _planningRequired=true;
            }
        } else if (!_status[i]) {
            _status[i] = true;
            _planningRequired=true;
        }
        ROS_INFO_STREAM("[QueryGroup::update] "<<_id <<" :phi[" << i << "]: " << _status[i]);
      }
    return flag;
    }

    void dispatchTask(const robot_map& _robotsData,
        const mrrtpe_msgs::Env& _envData,
        const geometry_msgs::PointPtr& _anomalyLoc) {
      if(_feedback){
        ROS_INFO_STREAM("[QueryGroup::dispatchTask] "<<_id<<" :Goal Id :"<<_goalId);
        _plan.requests.clear();
        for (int r = 0; r < _robotsName.size(); r++) {
            if (_goalId >= _trajectory[r].size())
                continue;  // add something so that intruder goal goal is not sent multiple times.
            if (_robotsData.find(_robotsName[r]) == _robotsData.end()) {
                ROS_INFO_STREAM("[QueryGroup::dispatchTask] " << _id << " :Robot "
                                                              << unsigned(_robotsName[r])
                                                              << "not found in status");
                continue;
            }

            auto data = _robotsData.find(_robotsName[r])->second;
            ROS_INFO_STREAM(
                "[QueryGroup::dispatchTask] "
                << _id << " :Generating goal location for Robot: " << unsigned(_robotsName[r]));
            mrrtpe_msgs::Feedback plan41Robot;

            plan41Robot.anomaly_received = data.anomaly_seen;
            plan41Robot.vehicle_type.vehicle_type = _robotsName[r];
            plan41Robot.goal_type.goal_id = _goalId+1;
            auto it = data.path.points.begin();

            ROS_INFO_STREAM("[QueryGroup::dispatchTask] "
                            << _id << " :Robot " << unsigned(_robotsName[r]) << " currently at ("
                            << double(it->x) << ", " << double(it->y) << ")");

            if (_trajectory[r][_goalId].first == -1 and
                _trajectory[r][_goalId].second == -1) {  // Stay at Same Place

                plan41Robot.goal.x = it->x;
                plan41Robot.goal.y = it->y;
                plan41Robot.goal_type.goal_type = mrrtpe_msgs::GoalType::IDLE_GOAL;

                ROS_INFO_STREAM("[QueryGroup::dispatchTask] "
                                << _id << " :Next Goal Location for Robot "
                                << unsigned(plan41Robot.vehicle_type.vehicle_type) << " is ("
                                << double(plan41Robot.goal.x) << "," << double(plan41Robot.goal.y)
                                << ") of type: Stay where You are "
                                << "with id: " << unsigned(plan41Robot.goal_type.goal_id));
                _plan.requests.push_back(plan41Robot);

            } else if (_trajectory[r][_goalId].first == 0 and _trajectory[r][_goalId].second == 10 and
                       _envData.intruder_detected) {  // Intruder Goal check, add something in
                                                      // condition of loop for
                                                      // intruder goal

                plan41Robot.goal.x = _anomalyLoc->x;
                plan41Robot.goal.y = _anomalyLoc->y;
                plan41Robot.goal_type.goal_type = mrrtpe_msgs::GoalType::FOLLOW_GOAL;
                // _fGoalID=_goalId;
                _intruderH=true;
                plan41Robot.goal_type.goal_id = 499;
                // _envData.intruder_detected = false;
                // _intRobot = plan41Robot.vehicle_type.vehicle_type;
                ROS_INFO_STREAM("[QueryGroup::dispatchTask] "
                                << _id << " :Next Goal Location for Robot "
                                << unsigned(plan41Robot.vehicle_type.vehicle_type) << " is ("
                                << double(plan41Robot.goal.x) << "," << double(plan41Robot.goal.y)
                                << ") of type: Follow the intruder "
                                << "with id: " << unsigned(plan41Robot.goal_type.goal_id));
                _plan.requests.push_back(plan41Robot);

            } else {  // Move to next Goal

                plan41Robot.goal.x = (_trajectory[r][_goalId].first + 0.5) * GRID_SZ;
                plan41Robot.goal.y = (_trajectory[r][_goalId].second + 0.5) * GRID_SZ;
                plan41Robot.goal_type.goal_type = mrrtpe_msgs::GoalType::NORMAL_GOAL;

                ROS_INFO_STREAM("[QueryGroup::dispatchTask] "
                                << _id << " :Next Goal Location for Robot "
                                << unsigned(plan41Robot.vehicle_type.vehicle_type) << " is ("
                                << double(plan41Robot.goal.x) << "," << double(plan41Robot.goal.y)
                                << ") of type: normal "
                                << "with id: " << unsigned(plan41Robot.goal_type.goal_id));
                _plan.requests.push_back(plan41Robot);
            }
            _waitingReport.insert(_robotsName[r]);
        }

        ROS_INFO_STREAM("[QueryGroup::dispatchTask] " << _id << " :Dispatching Goals.");

        _goalPub.publish(_plan);
        ROS_INFO_STREAM("[QueryGroup::dispatchTask] " << _id << " :Setting Feedback to FALSE.");
        _feedback=false;
        ros::Duration(10.0).sleep();
        _goalId++;
        _lastRequest = ros::Time::now();
        _count=0;
      }
      else{
        if((ros::Duration(45)<(ros::Time::now()-_lastRequest)) && _plan.requests.size()>0){
          _count++;
          if(_count>1){
              ROS_INFO_STREAM(
                  "[QueryGroup::dispatchTask] "
                  << _id
                  << " :Unable to reach goal location, so sending a dummy plan");
                  mrrtpe_msgs::PlannerRequest dummyPlan;
                  for (int r = 0; r < _robotsName.size(); r++) {
                      auto data = _robotsData.find(_robotsName[r])->second;
                      ROS_INFO_STREAM("[QueryGroup::dispatchTask] "
                                      << _id << " :Generating dummy goal location for Robot: "
                                      << unsigned(_robotsName[r]));
                      mrrtpe_msgs::Feedback plan41Robot;

                      plan41Robot.anomaly_received = data.anomaly_seen;
                      plan41Robot.vehicle_type.vehicle_type = _robotsName[r];
                      plan41Robot.goal_type.goal_id = 500;
                      auto it = data.path.points.begin();
                      ROS_INFO_STREAM("[QueryGroup::dispatchTask] "
                                      << _id << " :Robot " << unsigned(_robotsName[r])
                                      << " currently at (" << double(it->x) << ", " << double(it->y)
                                      << ")");
                      plan41Robot.goal.x = it->x+2;
                      plan41Robot.goal.y = it->y+2;
                      plan41Robot.goal_type.goal_type = mrrtpe_msgs::GoalType::NORMAL_GOAL;

                      ROS_INFO_STREAM("[QueryGroup::dispatchTask] "
                                      << _id << " :Next Goal Location for Robot "
                                      << unsigned(plan41Robot.vehicle_type.vehicle_type) << " is ("
                                      << double(plan41Robot.goal.x) << ","
                                      << double(plan41Robot.goal.y)
                                      << ") of type: Stay where You are "
                                      << "with id: " << unsigned(plan41Robot.goal_type.goal_id));
                      dummyPlan.requests.push_back(plan41Robot);
                  }
                  _goalPub.publish(dummyPlan);
                  _count=0;
                  ROS_INFO_STREAM("Dummy Goal Sent.");
                  ros::Duration(20.0).sleep();
          }
          _lastRequest=ros::Time::now();
          _goalPub.publish(_plan);
          ROS_INFO_STREAM("[QueryGroup::dispatchTask] " << _id << " :Dispatching Plans as feedback waiting time is more than 45 seconds.");
          ros::Duration(10.0).sleep(); 
        }
      }
    }

    void run(const robot_map& _robotsData,
        const mrrtpe_msgs::Env& _envData,
        const geometry_msgs::PointPtr& _anomalyLoc) {
        if(!update(_robotsData)) return;
        if(_feedback && _suffixDone && _planningRequired){
          std::string phi="True";
          for(int i=0;i<_queries.size();i++){
            if(_status[i]) {
              phi=phi+" && "+_queries[i];
              ROS_INFO_STREAM("[QueryGroup::run] "<<_id<<": Adding Query '"<<_queries[i]<< "' for planning.");
              }
          }
          generateAutomata(phi);
          ROS_INFO_STREAM("[QueryGroup::run] "<<_id<<": Calling Planner with map_file:<"<<_mapFile<<"> and query file:<"<<_queryFile<<">");
          _trajectory=mt_star(_mapFile,_queryFile,-1);
          // ROS_WARN_STREAM("[QueryGroup::run] " << _id << ": Printing Trajectories");
          _maxGoalSZ=0;
          for (int i = 0; i < _robotsName.size(); i++) {
              //  ROS_INFO_STREAM("[QueryGroup::run] " << _id << ": Robot " << int(_robotsName[i])
              //      << " Trajectory\n");
              int sz = _trajectory[i].size(); 
              _maxGoalSZ = std::max(_maxGoalSZ, sz);
              // if (sz > 0) {
              // cout << "[QueryGroup::run] " << _id << ":Robot has new Trajectory\n";
              // }
              // for (int j = 0; j < sz; j++) {
              // cout << "(" << _trajectory[i][j].first << "," << _trajectory[i][j].second << ") ";
              // }
          }
          ROS_WARN_STREAM("[QueryGroup::run] " << _id << ": Trajectory size:"<<_maxGoalSZ);
          _planningRequired=false;
          // _suffixDone=false;
          _goalId=0;
        }
        if(_goalId==0 and !_planningRequired){
          _suffixDone = false;
          ROS_WARN_STREAM("[QueryGroup::run] "<<_id<<" : Setting Suffix to False");
        }
        dispatchTask(_robotsData,_envData,_anomalyLoc);
    }
    // std::thread runThread(const robot_map& _robotsData,
    //     mrrtpe_msgs::Env& _envData,
    //     const geometry_msgs::PointPtr& _anomalyLoc) {
    //     return std::thread([=] { run(_robotsData,_envData,_anomalyLoc); });
    // }

    std::vector<bool> _phiE;
    bool _intruderH;

  private:
    mrrtpe_msgs::PlannerRequest _plan;
    std::string _mapFile, _queryFile, _id, _queriesFile,_dispatchTopic;
    std::vector<std::string> _queries;
    std::vector<bool> _status;
    bool _feedback,_planningRequired,_suffixDone;
    int _goalId,_maxGoalSZ,_Fid;
    std::vector<std::vector<std::pair<double, double>>> _trajectory;
    std::vector<v_type> _robotsName;
    ros::Time _lastRequest;
    std::set<v_type> _waitingReport;
    ros::Publisher _goalPub;
    int _count=0;
    // bool _intruderH;
};
}  // namespace motion_planning::QueryGroup
#endif