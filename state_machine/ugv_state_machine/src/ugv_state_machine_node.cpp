#include <ugv_state_machine/ugv_state_machine.hpp>

using namespace mrrtpe::ugv_state_machine;

int main(int argc, char** argv) {
    ros::init(argc, argv, "ugv_state_machine");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    UGVStateMachine fsm(nh, nh_private);
    fsm.run();
}
