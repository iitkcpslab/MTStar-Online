#include <mrrtpe_msgs/PlannerRequest.h>
#include <mrrtpe_msgs/RobotStatus.h>
#include <ros/ros.h>
#include <unordered_map>

namespace ltl_rm::rm {
typedef mrrtpe_msgs::VehicleType::_vehicle_type_type v_type;  
typedef std::unordered_map<v_type, mrrtpe_msgs::RobotStatus> robot_map;
class RM {
  private:
    void robotStatusCallback(const mrrtpe_msgs::RobotStatusConstPtr& msg);
    void generatePlan(std::string path_prefix);
    //void generatePlan(int phi);
    //void anamoly(geometry_msgs::PointPtr& point);
    void insertInPhi0(v_type vehicle);
    void insertInPhi1(v_type vehicle);
    void insertInPhi02(v_type vehicle);
    void insertInPhi12(v_type vehicle);
    void map_goals(std::vector<std::vector<std::pair<double,double>>> &trajectories,std::set<v_type>& robots);//mapping of goals to free robots
    void map_goals(std::set<std::pair<std::pair<double,double>,v_type>> &location,std::set<v_type>& robots);//mapping of charging locations to discharged robots
    void map_goals();

    robot_map robots_;
    //std::set<std::pair<v_type,std::pair<int,int>>> phi0_,phi1_,phi02_,phi12_;
    std::set<v_type> received_, waiting_,phi0_,phi1_,phi02_,phi12_;
    std::set<std::pair<std::pair<double,double>,v_type>> refill_loc_phi0_,refill_loc_phi1_; //charging location for phi-0 and phi_1 robots along with their availability status.
    std::unordered_map<v_type, std::vector<mrrtpe_msgs::Point>> goals_;
    std::unordered_map<v_type,int> location_;
    mrrtpe_msgs::PlannerRequest plan_all_robots;
    geometry_msgs::PointPtr anomaly_loc_ = boost::make_shared<geometry_msgs::Point>();
    v_type anomaly_robot_;

    bool replan0_,replan1_,replan02_,replan12_,feedback_,anomaly_seen_,anomaly_handled_;
    int robots_count_;
    std::string path_prefix_;

    ros::Subscriber robot_sub_;
    ros::Publisher goal_pub_;

  public:
    RM(ros::NodeHandle& nh, ros::NodeHandle& nh_private, std::set<std::pair<v_type,std::pair<double,double>>> avail_robots,std::string path_prefix);

    void run();
};

}  // namespace ltl_rm::rm
