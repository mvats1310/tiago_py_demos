#!/usr/bin/env python3
import sys
import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

def main():
    rospy.init_node("play_motion_demo")

    motion = rospy.get_param("~motion", "home")  # e.g. wave, home, prepare_grasp
    skip_planning = rospy.get_param("~skip_planning", False)

    client = actionlib.SimpleActionClient("/play_motion", PlayMotionAction)
    rospy.loginfo("Waiting for /play_motion action server...")
    if not client.wait_for_server(rospy.Duration(15.0)):
        rospy.logerr("No /play_motion server. Is TIAGo launched?")
        sys.exit(1)

    goal = PlayMotionGoal()
    goal.motion_name = motion
    goal.skip_planning = skip_planning  # False → let it plan if needed

    rospy.loginfo("Playing motion: %s (skip_planning=%s)", motion, skip_planning)
    client.send_goal(goal)
    finished = client.wait_for_result(rospy.Duration(60.0))
    if not finished:
        rospy.logwarn("Timeout waiting for motion result")
        sys.exit(2)

    state = client.get_state()
    rospy.loginfo("Motion result state: %s", str(state))

if __name__ == "__main__":
    main()
