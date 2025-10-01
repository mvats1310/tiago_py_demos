#!/usr/bin/env python3
import sys, select, termios, tty
import rospy
from geometry_msgs.msg import Twist

HELP = """
TIAGo Key Teleop
----------------
Move:  w
Back:  s
Left:  a
Right: d
Stop:  space or x
Lin +/-: = / -
Ang +/-: ] / [
Quit:  q

Publishing: {topic}
"""

KEYS = {'w': (1,0), 's': (-1,0), 'a': (0,1), 'd': (0,-1)}

def get_key(timeout=0.05):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r,_,_ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def clamp(v, lo, hi): return max(lo, min(hi, v))

def main():
    rospy.init_node('tiago_key_teleop')
    topic = rospy.get_param('~cmd_topic', '/mobile_base_controller/cmd_vel')
    lin = rospy.get_param('~linear_speed', 0.25)
    ang = rospy.get_param('~angular_speed', 0.7)
    alpha = rospy.get_param('~accel', 0.8)  # smoothing [0..1]

    pub = rospy.Publisher(topic, Twist, queue_size=10)
    v_t, w_t, v, w = 0.0, 0.0, 0.0, 0.0
    rate = rospy.Rate(30)

    print(HELP.format(topic=topic))
    while not rospy.is_shutdown():
        k = get_key()
        if k in KEYS:
            fwd, yaw = KEYS[k]
            v_t = fwd * lin
            w_t = yaw * ang
        elif k in (' ', 'x'):
            v_t = w_t = 0.0
        elif k == '=':
            lin = clamp(lin + 0.05, 0.05, 1.0); print(f"\nlinear -> {lin:.2f}")
        elif k == '-':
            lin = clamp(lin - 0.05, 0.05, 1.0); print(f"\nlinear -> {lin:.2f}")
        elif k == ']':
            ang = clamp(ang + 0.05, 0.1, 2.5); print(f"\nangular -> {ang:.2f}")
        elif k == '[':
            ang = clamp(ang - 0.05, 0.1, 2.5); print(f"\nangular -> {ang:.2f}")
        elif k == 'q':
            break

        v = alpha*v_t + (1-alpha)*v
        w = alpha*w_t + (1-alpha)*w

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        pub.publish(msg)
        rate.sleep()

    pub.publish(Twist())
    print("\nbye!")

if __name__ == '__main__':
    main()
