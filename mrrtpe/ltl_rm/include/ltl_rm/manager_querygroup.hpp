#ifndef __MANAGER_QUERYGROUP_HPP
#define __MANAGER_QUERYGROUP_HPP
#include "querygroup.hpp"
#include <mrrtpe_msgs/Env.h>
#include <mrrtpe_msgs/Environment.h>
#include <mrrtpe_msgs/PlannerRequest.h>
#include <mrrtpe_msgs/RobotStatus.h>
#include <ros/ros.h>
#include <unordered_map>
#include <vector>
namespace motion_planning::manager {
typedef mrrtpe_msgs::VehicleType::_vehicle_type_type v_type;
typedef std::unordered_map<v_type, mrrtpe_msgs::RobotStatus> robot_map;
class Manager {
  private:
    /* data */
    
    void updateEnv();
    
    void robotStatusCallback(const mrrtpe_msgs::RobotStatusConstPtr& msg);
    bool Serv_item1(mrrtpe_msgs::Environment::Request& req,
        mrrtpe_msgs::Environment::Response& res);
    bool Serv_item2(mrrtpe_msgs::Environment::Request& req,
        mrrtpe_msgs::Environment::Response& res);
    bool Serv_item3(mrrtpe_msgs::Environment::Request& req,
        mrrtpe_msgs::Environment::Response& res);
    bool Serv_d1(mrrtpe_msgs::Environment::Request& req, mrrtpe_msgs::Environment::Response& res);
    bool Serv_d2(mrrtpe_msgs::Environment::Request& req, mrrtpe_msgs::Environment::Response& res);
    bool Serv_d3(mrrtpe_msgs::Environment::Request& req, mrrtpe_msgs::Environment::Response& res);

    // std::vector<QueryGroup::QueryGroup> _groups;
    QueryGroup::QueryGroup _g1,_g2,_g3; //Add the no of groups based on use_case(g1,g2 for uc1, g1,g2,g3 for uc2)
    mrrtpe_msgs::Env _envData;
    geometry_msgs::PointPtr _anomalyLoc = boost::make_shared<geometry_msgs::Point>();
    robot_map _robotsData;
    std::vector<v_type> _robotsName;
    ros::Subscriber _robotSub;
    ros::Publisher _goalPub, _envPub;
    ros::Time _intHT;
    bool _intH;

    // Ros service for setting the environment variables other than batteries to set/reset
    ros::ServiceServer _service_item1, _service_item2, _service_item3, _service_d1, _service_d2,
        _service_d3;

  public:
    Manager(ros::NodeHandle& nh,
        ros::NodeHandle& nh_private,
        std::vector<v_type> robots, std::string path_prefix ="");
    //~manager();
    void run();
};
}  // namespace motion_planning::manager
#endif