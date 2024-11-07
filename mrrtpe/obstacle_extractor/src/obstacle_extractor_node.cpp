#include "obstacle_extractor/obstacle_extractor.hpp"

using namespace mrrtpe::ugv_obstacle_extractor;

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_extractor");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ObstacleExtractor node(nh, nh_private);
    ros::spin();
}
