#include "fake_battery/fake_battery.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_battery");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    FakeBattery fake_battery(nh, nh_private);
    ros::Rate loop_rate(20);
    while(ros::ok()) {
        fake_battery.run();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}