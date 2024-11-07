#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <ros/ros.h>

namespace mrrtpe_simulation {
class ObjectControl {
  public:
    ObjectControl(ros::NodeHandle& nh);
    ~ObjectControl(){};
    void setObjectPose(const gazebo_msgs::ModelStateConstPtr& msg);
    bool getModelState(const std::string& model_name, geometry_msgs::Pose& pose);

  private:
    ros::Publisher set_model_state_pub_;
    ros::ServiceClient gazebo_get_model_client_;
};
}  // namespace mrrtpe_simulation
