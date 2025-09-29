#!/usr/bin/env python3
import time
import rospy
import actionlib
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

ARM_JOINTS = [
    "arm_1_joint","arm_2_joint","arm_3_joint","arm_4_joint",
    "arm_5_joint","arm_6_joint","arm_7_joint"
]

def drive_forward(topic="/mobile_base_controller/cmd_vel"):
    """Drive forward ~0.3m (about 1 foot)."""
    pub = rospy.Publisher(topic, Twist, queue_size=1)
    rospy.sleep(1.0)  # wait for pub connection

    cmd = Twist()
    cmd.linear.x = 0.1   # 0.1 m/s speed

    duration = 3.0  # 3s at 0.1 m/s ≈ 0.3m
    rate = rospy.Rate(10)
    t_end = time.time() + duration

    rospy.loginfo(f"[base] Moving forward {duration:.1f}s at {cmd.linear.x:.2f} m/s")
    while not rospy.is_shutdown() and time.time() < t_end:
        pub.publish(cmd)
        rate.sleep()

    pub.publish(Twist())  # stop
    rospy.loginfo("[base] Done moving forward.")

def move_arm(action_name="/arm_controller/follow_joint_trajectory"):
    """Send a simple arm extension trajectory."""
    client = actionlib.SimpleActionClient(action_name, FollowJointTrajectoryAction)
    rospy.loginfo(f"[arm] Waiting for action server {action_name} ...")
    client.wait_for_server()

    traj = JointTrajectory()
    traj.joint_names = ARM_JOINTS

    home  = [0.0,  0.4, -1.0, 1.6, 0.0,  0.0, 0.0]
    extend = [0.2, 0.6, -1.2, 1.8, 0.2, -0.3, 0.0]

    traj.points = [
        JointTrajectoryPoint(positions=home,   time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=extend, time_from_start=rospy.Duration(5.0)),
    ]

    goal = FollowJointTrajectoryGoal(trajectory=traj)
    rospy.loginfo("[arm] Sending extension goal...")
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(10.0))
    rospy.loginfo("[arm] Extension done.")

def main():
    rospy.init_node("tiago_base_then_arm")

    # 1 foot forward
    drive_forward("/mobile_base_controller/cmd_vel")

    # extend arm
    move_arm("/arm_controller/follow_joint_trajectory")

if __name__ == "__main__":
    main()
