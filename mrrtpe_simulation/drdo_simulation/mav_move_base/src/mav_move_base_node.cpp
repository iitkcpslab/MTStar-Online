
#include <mav_move_base/mav_move_base.hpp>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_base_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_priv("~");

    drdo_simulation::mav_move_base::MAVMoveBase move_base(nh, nh_priv);
    ros::spin();

    return (0);
}