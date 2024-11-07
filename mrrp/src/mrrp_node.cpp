#include <mrrp/mrrp.hpp>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_planner");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_priv("~");

    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);

    costmap_2d::Costmap2DROS lcr("costmap", buffer);

    mrrtpe::mrrp::MRRP pppp(nh, nh_priv,"planner", &lcr);

    pppp.run();
    return 0;
}