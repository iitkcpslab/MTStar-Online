#ifndef __MANAGER_HPP
#define __MANAGER_HPP
#include "ltl_rm/disjointset.hpp"
#include <mrrtpe_msgs/PlannerRequest.h>
#include <mrrtpe_msgs/RobotStatus.h>
#include <mrrtpe_msgs/Env.h>
#include <ros/ros.h>
#include <unordered_map>
#include <vector>
#include <mrrtpe_msgs/Environment.h>
namespace motion_planning::manager {
typedef mrrtpe_msgs::VehicleType::_vehicle_type_type v_type;
typedef std::unordered_map<v_type, mrrtpe_msgs::RobotStatus> robot_map;
class Manager {
  private:
    /* data */
    DisjointSet::DisjointSet _replan;
    std::vector<bool> _phie;
    std::vector<bool> _status;
    std::vector<std::string> _queries;
    bool _feedback, _planningRequired;
    std::string _queriesFile, _mapFile, _queryFile;
    std::vector<std::vector<std::pair<double, double>>> _trajectory;
    int _goalId;
    int updateEnv();
    void mergeQueries();
    // void updateTts();
    void generateAutomata(std::string& phi);
    bool dispatchTask(std::vector<std::vector<std::pair<double, double>>> trajectory);
    //ROS Related Stuff
    void robotStatusCallback(const mrrtpe_msgs::RobotStatusConstPtr& msg);
    bool Serv_item1(mrrtpe_msgs::Environment::Request& req,
        mrrtpe_msgs::Environment::Response& res);
    bool Serv_item2(mrrtpe_msgs::Environment::Request& req,
        mrrtpe_msgs::Environment::Response& res);
    bool Serv_item3(mrrtpe_msgs::Environment::Request& req,
        mrrtpe_msgs::Environment::Response& res);
    bool Serv_d1(mrrtpe_msgs::Environment::Request& req,
        mrrtpe_msgs::Environment::Response& res);
    bool Serv_d2(mrrtpe_msgs::Environment::Request& req,
        mrrtpe_msgs::Environment::Response& res);
    bool Serv_d3(mrrtpe_msgs::Environment::Request& req,
        mrrtpe_msgs::Environment::Response& res);

    mrrtpe_msgs::Env _envData;
    mrrtpe_msgs::PlannerRequest _plan4AllRobots;
    geometry_msgs::PointPtr _anomalyLoc = boost::make_shared<geometry_msgs::Point>();
    robot_map _robotsData;
    v_type _intRobot;
    std::vector<v_type> _robotsName;
    std::set<v_type> _receivedReport, _waitingReport;
    ros::Subscriber _robotSub;
    ros::Publisher _goalPub, _envPub;
    ros::Time _lastRequest;

//Ros service for setting the environment variables other than batteries to set/reset
    ros::ServiceServer _service_item1, _service_item2, _service_item3, _service_d1, _service_d2,
        _service_d3;

  public:
    Manager(ros::NodeHandle& nh,
        ros::NodeHandle& nh_private,
        int m,
        std::vector<v_type> robots,
        std::string path_prefix = "");
    Manager(int m, std::string path_prefix="");
    //~manager();
    void run();
};
}  // namespace motion_planning::manager
#endif