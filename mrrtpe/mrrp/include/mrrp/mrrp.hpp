#include <costmap_2d/costmap_2d_ros.h>
#include <global_planner/planner_core.h>
#include <mrrtpe_msgs/Plan.h>
#include <mrrtpe_msgs/PlannerRequest.h>
#include <mrrtpe_msgs/RobotStatus.h>
#include <nav_msgs/MapMetaData.h>
#include <std_msgs/Empty.h>

namespace cm = costmap_2d;
namespace rm = geometry_msgs;

using cm::Costmap2D;
using cm::Costmap2DROS;
using rm::PoseStamped;
using std::string;
using std::vector;

namespace mrrtpe::mrrp {
typedef mrrtpe_msgs::VehicleType::_vehicle_type_type v_type;
typedef std::unordered_map<v_type, std::pair<ros::Time, mrrtpe_msgs::RobotStatus>> robot_map;
class MRRP : public global_planner::GlobalPlanner {
  public:
    MRRP(ros::NodeHandle& nh, ros::NodeHandle& nh_private, string name, Costmap2DROS* cmap);
    void run();

  private:
    void plannerReqCallback(const mrrtpe_msgs::PlannerRequestPtr& msg);
    void robotStatusCallback(const mrrtpe_msgs::RobotStatusConstPtr& msg);
    void mapMetaDataCallback(const nav_msgs::MapMetaDataConstPtr& msg);
    void makePlanMav(const PoseStamped& start, const PoseStamped& goal, vector<PoseStamped>& plan);
    void makeMultiPlan();
    void computePoints(const vector<PoseStamped>& path,
        mrrtpe_msgs::Plan& plan,
        const mrrtpe_msgs::Feedback& req);
    void forceReplan(const std_msgs::EmptyConstPtr& msg);
    void checkRobotStatus();
    bool isValidState(mrrtpe_msgs::RobotStatus& robot);
    bool staticRobot(mrrtpe_msgs::RobotStatus& robot);
    bool isReplanningNeeded();

    mrrtpe_msgs::PlannerRequest req_;
    ros::Subscriber planner_req_sub_, robot_sub_, force_replan_sub_, map_meta_data_sub_;
    ros::Publisher marker_pub_, plan_pub_;
    Costmap2DROS* cmap_;

    double robot_radius_, rate_, robot_timeout_, waypoint_dist_;
    bool passive_replan_, visual_;
    nav_msgs::MapMetaDataConstPtr map_meta_data_;
    robot_map robots_;
    std::unordered_map<v_type, vector<PoseStamped>> paths_;
};
}  // namespace mrrtpe::mrrp