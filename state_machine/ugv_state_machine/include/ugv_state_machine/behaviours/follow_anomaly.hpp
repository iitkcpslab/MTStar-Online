#pragma once

#include <thread>
#include <ugv_state_machine/ugv_state/ugv_state.hpp>
namespace mrrtpe::ugv_state_machine {

class FollowAnomaly {
  public:
    struct Event {};
    FollowAnomaly() = default;
    FollowAnomaly(ros::NodeHandle& nh_private, const statePtr state_ptr);

    void execute(const Event& evt);
    void waitForTermination();

    bool searchAnomaly() const {
        return search_anomaly_;
    }
    bool followAnomaly() const {
        return follow_anomaly_;
    }

  private:
    void follow();
    void inspect();
    void run();

    bool search_anomaly_;
    bool follow_anomaly_;

    int search_anomaly_retry_;
    double lost_anomaly_timeout_;
    double anomaly_poll_time_;

    std::thread follow_th_;
    const statePtr ugv_state_;
};

}  // namespace mrrtpe::ugv_state_machine
