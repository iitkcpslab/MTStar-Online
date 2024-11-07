#include <ltl_rm/manager.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "ltl_rm");
    std::string path_prefix;
    ros::NodeHandle nh("");
    ros::NodeHandle nh_priv("~");
    // DOnt forget to change the no of queries
    int sz_q=6;    //10 for use case2,6 for use case 1
    if (nh_priv.getParam("path_prefix", path_prefix)) {
        ROS_INFO("Got param: %s", path_prefix.c_str());
    } else {
        ROS_ERROR("Failed to get param named path_prefix");
        exit(EXIT_FAILURE);
    }
    std::vector<mrrtpe_msgs::VehicleType::_vehicle_type_type> robots = {
        mrrtpe_msgs::VehicleType::UGV_1,
        mrrtpe_msgs::VehicleType::UGV_2,
        mrrtpe_msgs::VehicleType::MAV_1};
    motion_planning::manager::Manager obj(nh, nh_priv, sz_q, robots, path_prefix);  
    // ltl_rm::rm::RM obj(nh, nh_priv, avail_robots, path_prefix);

    while (ros::ok()) {
        obj.run();
    }
    return 0;
}