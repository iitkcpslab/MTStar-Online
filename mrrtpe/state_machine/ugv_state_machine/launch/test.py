from ast import While
from cgi import test
from time import sleep
import rospy
from mrrtpe_msgs.msg import Path, Point, Plan, Obstacle, RobotStatus, State, PlannerRequest, Feedback
from nav_msgs.msg import Odometry


class TestBehaviour:
    def __init__(self) -> None:
        self.plan_pub = rospy.Publisher('ugv_1/mrrp_plan', Plan, queue_size=10)
        self.obs_pub = rospy.Publisher(
            'ugv_1/object_list', Odometry, queue_size=10)
        self.req_pub = rospy.Publisher(
            '/planner_req', PlannerRequest, queue_size=10)
        self.robot_sub = rospy.Subscriber(
            "ugv_1/robot_state", RobotStatus, self.statusCb)
        self.robot_status = RobotStatus()

        rospy.init_node('tester', anonymous=True)
        pass

    def statusCb(self, data):
        self.robot_status = data

    def publishPlan(self):
        plan = Plan()
        plan.vehicle_type.vehicle_type = 3
        plan.path.goal_type.goal_type = Path().goal_type.FOLLOW_GOAL
        plan.path.points.append(Point())
        # goal1 = Point()
        # goal1.x = 5
        # goal1.y = 2
        # plan.path.points.append(goal1)
        # goal2 = Point()
        # goal2.x = 9
        # goal2.y = 2
        # plan.path.points.append(goal2)
        # self.plan_pub.publish(plan)
        goal2 = Point()
        goal2.x = 5
        goal2.y = 2
        plan.path.points.append(goal2)
        self.plan_pub.publish(plan)


    def publishFollowPlan(self):
        plan = Plan()
        plan.vehicle_type.vehicle_type = 3
        plan.path.goal_type.goal_type = Path().goal_type.FOLLOW_GOAL
        plan.path.points.append(Point())
        goal1 = Point()
        goal1.x = 5
        goal1.y = 2
        plan.path.points.append(goal1)
        goal2 = Point()
        goal2.x = 2
        goal2.y = 2
        plan.path.points.append(goal2)
        self.plan_pub.publish(plan)

    def publishIdlePlan(self):
        plan = Plan()
        plan.vehicle_type.vehicle_type = 0
        plan.path.goal_type.goal_type = Path().goal_type.IDLE_GOAL
        plan.path.points.append(Point())
        goal1 = Point()
        goal1.x = 5
        goal1.y = 2
        plan.path.points.append(goal1)
        goal2 = Point()
        goal2.x = 2
        goal2.y = 2
        plan.path.points.append(goal2)
        self.plan_pub.publish(plan)

    def publishUnknownObstacle(self):
        obs = Obstacle()
        obs.obs_type = Obstacle().UNKNOWN
        self.obs_pub.publish(obs)

    def publishAnomaly(self):
        obs = Obstacle()
        obs.obs_type = Obstacle().ANOMALY
        self.obs_pub.publish(obs)

    def publishOtherObstacle(self):
        obs = Obstacle()
        obs.obs_type = Obstacle().OTHER
        self.obs_pub.publish(obs)
    
    def pubReq(self):
        reqs = PlannerRequest()
        req1 = Feedback()
        req1.goal_type.goal_type = Path().goal_type.NORMAL_GOAL
        req1.goal.x = 3
        req1.goal.y = 2
        req1.vehicle_type.vehicle_type = Plan().vehicle_type.UGV_1
        reqs.requests.append(req1)
        # req2 = Feedback()
        # req2.goal_type.goal_type = Path().goal_type.NORMAL_GOAL
        # req2.goal.x = 10
        # req2.goal.y = 2
        # req2.vehicle_type.vehicle_type = Plan().vehicle_type.UGV_2
        # reqs.requests.append(req2)
        # req2 = Feedback()
        # req2.goal_type.goal_type = Path().goal_type.NORMAL_GOAL
        # req2.goal.x = 6
        # req2.goal.y = 2
        # req2.vehicle_type.vehicle_type = Plan().vehicle_type.UGV_FOLLOW
        # reqs.requests.append(req2)
        self.req_pub.publish(reqs)
        


if __name__ == '__main__':
    tester = TestBehaviour()
    x = 2.0
    y = 2.0
    while (tester.obs_pub.get_num_connections() == 0 or tester.req_pub.get_num_connections() == 0):
        rospy.sleep(1.0)

    tester.pubReq()
    print("plan")
    # rospy.sleep(5.0)
    # tester.publishUnknownObstacle()
    # print("unknownobs")
    # rospy.sleep(10.0)
    # tester.publishAnomaly()
    # print("anoma")

    # tester.publishFollowPlan()
    # print("follow Plan")

    # while (tester.robot_status.state.state != State().Inspect):
    #     print(tester.robot_status.state)
    #     rospy.sleep(1.0)

    # rospy.sleep(1.0)
    # while (True):
    #     # y += 0.5
    #     print("anomaly pub")
    #     obs = Odometry()
    #     obs.pose.pose.position.x = 1.5
    #     obs.pose.pose.position.y = 0
    #     tester.obs_pub.publish(obs)
    #     rospy.sleep(1.0)
