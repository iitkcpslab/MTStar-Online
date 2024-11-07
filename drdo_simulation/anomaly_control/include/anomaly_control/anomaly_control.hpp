#include <anomaly_control/intract_marker.hpp>
#include <anomaly_control/object_control.hpp>

#include <unordered_map>

#include <costmap_2d/costmap_2d_ros.h>
#include <global_planner/planner_core.h>
#include <mrrtpe_msgs/Plan.h>
#include <mrrtpe_msgs/RobotStatus.h>
#include <std_msgs/Empty.h>

namespace cm = costmap_2d;
namespace rm = geometry_msgs;

using cm::Costmap2D;
using cm::Costmap2DROS;
using rm::PoseStamped;
using std::string;
using std::vector;

namespace mrrtpe_simulation {
typedef mrrtpe_msgs::VehicleType::_vehicle_type_type v_type;
typedef std::unordered_map<v_type, std::pair<ros::Time, mrrtpe_msgs::RobotStatus>> robot_map;

class AnomalyControl : public global_planner::GlobalPlanner {
  public:
    AnomalyControl(ros::NodeHandle& nh, ros::NodeHandle& nh_private, Costmap2DROS* cmap);
    void run();

  private:
    void robotsCallback(const mrrtpe_msgs::RobotStatusConstPtr& msg);
    void pathCallback(const mrrtpe_msgs::PlanConstPtr& msg);
    void anomalyToggle(const std_msgs::Empty msg);
    void markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    void obstacleControl(const ros::TimerEvent&);
    void anomalyControl(const ros::TimerEvent&);

    void setStaticObstacle(std::pair<int, vector<rm::Point>>& traj, const std::string& name);
    vector<PoseStamped> getPath(PoseStamped& start_pose, PoseStamped& goal_pose);
    void markPolygon(const double& wx, const double& wy, unsigned char cost);

    ObjectControl obj_ctrl_;

    double move_speed_, static_obs_timeout_, anomaly_detection_distance_;
    ros::NodeHandle nh_;

    IntractMarker marker_;

    PoseStamped goal_pose_;

    ros::Subscriber robots_sub_;
    ros::Subscriber anomaly_toggle_sub_;

    ros::Publisher anomaly_pub_;
    ros::Timer obstacle_timer_, anomaly_timer_;

    std::string waypoint_file_name_;

    std::array<std::pair<int, vector<rm::Point>>, 3> waypoints_;
    std::unordered_map<v_type, ros::Publisher> anomaly_pubs_;

    robot_map robots_;
};
}  // namespace mrrtpe_simulation