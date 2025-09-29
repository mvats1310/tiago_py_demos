#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node("tiago_base_drive")
    topic = rospy.get_param("~cmd_topic", "/mobile_base_controller/cmd_vel")
    pub = rospy.Publisher(topic, Twist, queue_size=1)
    rospy.sleep(1.0)

    rate = rospy.Rate(10)
    t = Twist(); t.linear.x = 0.2
    for _ in range(30):   # ~3 s forward
        pub.publish(t); rate.sleep()
    pub.publish(Twist())

if __name__ == "__main__":
    main()


