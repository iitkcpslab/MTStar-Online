#include "anomaly_control/object_control.hpp"
namespace mrrtpe_simulation {
ObjectControl::ObjectControl(ros::NodeHandle& nh) {
    set_model_state_pub_ = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
    gazebo_get_model_client_ =
        nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true);
}

void ObjectControl::setObjectPose(const gazebo_msgs::ModelStateConstPtr& msg) {
    set_model_state_pub_.publish(msg);
}

bool ObjectControl::getModelState(const std::string& model_name, geometry_msgs::Pose& pose) {
    gazebo_msgs::GetModelState get_model;
    get_model.request.model_name = model_name;

    if (gazebo_get_model_client_.call(get_model)) {
        pose = get_model.response.pose;
        return true;
    } else {
        ROS_ERROR_STREAM("[ObjectControl::getModelState]"
                         << "Failed to call service for Model " << model_name);
        return false;
    }
}
}  // namespace mrrtpe_simulation