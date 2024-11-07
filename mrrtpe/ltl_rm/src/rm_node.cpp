#include <ltl_rm/rm.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "ltl_rm");
    std::string path_prefix;
    ros::NodeHandle nh("");
    ros::NodeHandle nh_priv("~");
    if (nh_priv.getParam("path_prefix", path_prefix)) {
        ROS_INFO("Got param: %s", path_prefix.c_str());
    } else {
        ROS_ERROR("Failed to get param named path_prefix");
        exit(EXIT_FAILURE);
    }
    std::set<std::pair<mrrtpe_msgs::VehicleType::_vehicle_type_type, std::pair<double, double>>>
        // avail_robots = {std::make_pair(mrrtpe_msgs::VehicleType::MAV_1,std::make_pair(0.0,10.0)),
        // std::make_pair(mrrtpe_msgs::VehicleType::MAV_2, std::make_pair(10.0, 0.0)),
        // std::make_pair(mrrtpe_msgs::VehicleType::UGV_1, std::make_pair(4.0, 2.0))};

        avail_robots = {std::make_pair(mrrtpe_msgs::VehicleType::MAV_1, std::make_pair(0.0, 0.0)),
            std::make_pair(mrrtpe_msgs::VehicleType::MAV_2, std::make_pair(0.0, 19.0)),
            std::make_pair(mrrtpe_msgs::VehicleType::MAV_3, std::make_pair(19.0, 19.0)),
            std::make_pair(mrrtpe_msgs::VehicleType::MAV_4, std::make_pair(19.0, 0.0)),
            std::make_pair(mrrtpe_msgs::VehicleType::UGV_1, std::make_pair(2.0, 9.0)),
            std::make_pair(mrrtpe_msgs::VehicleType::UGV_2, std::make_pair(6.0, 10.0)),
            std::make_pair(mrrtpe_msgs::VehicleType::UGV_3, std::make_pair(1.0, 1.0))};


        // avail_robots = {std::make_pair(mrrtpe_msgs::VehicleType::UGV_1, std::make_pair(2.0, 9.0)),
        //     std::make_pair(mrrtpe_msgs::VehicleType::UGV_2, std::make_pair(6.0, 10.0)),
        //     std::make_pair(mrrtpe_msgs::VehicleType::UGV_3, std::make_pair(1.0, 1.0))};
    ltl_rm::rm::RM obj(nh, nh_priv, avail_robots, path_prefix);

    while (ros::ok()) {
        obj.run();
    }
    return 0;
}