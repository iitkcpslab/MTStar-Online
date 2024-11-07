#include <anomaly_control/anomaly_control.hpp>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "anomaly_control");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_priv("~");

    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);

    costmap_2d::Costmap2DROS lcr("costmap", buffer);

    mrrtpe_simulation::AnomalyControl pppp(nh, nh_priv, &lcr);

    pppp.run();
    return 0;
}