#!/usr/bin/env python3
import math
import rospy
import cv2
import actionlib
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from control_msgs.msg import PointHeadAction, PointHeadGoal

WINDOW_NAME = "Inside of TIAGo's head"
CAMERA_FRAME = "/xtion_rgb_optical_frame"
IMAGE_TOPIC = "/xtion/rgb/image_raw"
CAM_INFO_TOPIC = "/xtion/rgb/camera_info"

bridge = CvBridge()
latest_image_stamp = None
K_fx = K_fy = K_cx = K_cy = None
img_for_display = None
client = None

def image_cb(msg):
    """ROS callback for camera images -> show image and track latest stamp."""
    global latest_image_stamp, img_for_display
    latest_image_stamp = msg.header.stamp
    try:
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as e:
        rospy.logwarn("cv_bridge error: %s", e)
        return
    img_for_display = cv_img
    cv2.imshow(WINDOW_NAME, img_for_display)
    # small wait so HighGUI processes events (incl. mouse)
    cv2.waitKey(1)

def on_mouse(event, u, v, flags, userdata):
    """OpenCV mouse callback: left-click to send a head pointing goal."""
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    if None in (K_fx, K_fy, K_cx, K_cy) or latest_image_stamp is None:
        rospy.logwarn("Intrinsics or image timestamp not ready yet.")
        return

    rospy.loginfo("Pixel selected (%d, %d) → making TIAGo look there", u, v)

    # normalized pixel coords
    x_norm = (u - K_cx) / K_fx
    y_norm = (v - K_cy) / K_fy

    # pick an arbitrary depth along camera Z (like tutorial uses Z=1.0)
    Z = 1.0
    target = PointStamped()
    target.header.frame_id = CAMERA_FRAME
    target.header.stamp = latest_image_stamp
    target.point.x = x_norm * Z
    target.point.y = y_norm * Z
    target.point.z = Z

    # build and send the PointHead goal
    goal = PointHeadGoal()
    goal.target = target
    # make the camera optical frame Z axis point to the target
    goal.pointing_frame = CAMERA_FRAME
    goal.pointing_axis.x = 0.0
    goal.pointing_axis.y = 0.0
    goal.pointing_axis.z = 1.0
    goal.min_duration = rospy.Duration(1.0)
    goal.max_velocity = 0.25

    client.send_goal(goal)
    # brief pause to avoid flooding
    rospy.sleep(0.5)

def wait_for_intrinsics(timeout=10.0):
    """Get camera intrinsics (fx, fy, cx, cy) from /xtion/rgb/camera_info."""
    global K_fx, K_fy, K_cx, K_cy
    rospy.loginfo("Waiting for camera intrinsics on %s ...", CAM_INFO_TOPIC)
    msg = rospy.wait_for_message(CAM_INFO_TOPIC, CameraInfo, timeout=timeout)
    K_fx = msg.K[0]  # fx
    K_fy = msg.K[4]  # fy
    K_cx = msg.K[2]  # cx
    K_cy = msg.K[5]  # cy
    rospy.loginfo("Camera intrinsics: fx=%.3f fy=%.3f cx=%.3f cy=%.3f", K_fx, K_fy, K_cx, K_cy)

def main():
    global client

    rospy.init_node("look_to_point_py")

    # simulated time may start at 0 — wait until valid
    timeout = rospy.Time.now() + rospy.Duration(10.0)
    while rospy.Time.now().to_sec() == 0:
        if rospy.Time.now() > timeout:
            rospy.logfatal("Timed-out waiting for valid /clock")
            sys.exit(1)
        rospy.sleep(0.1)
    # action client
    rospy.loginfo("Creating action client to /head_controller/point_head_action")
    client = actionlib.SimpleActionClient("/head_controller/point_head_action", PointHeadAction)
    if not client.wait_for_server(rospy.Duration(10.0)):
        rospy.logfatal("Head controller action server not available.")
        return

    # intrinsics
    try:
        wait_for_intrinsics(timeout=10.0)
    except rospy.ROSException:
        rospy.logfatal("Camera intrinsics not received in time.")
        return

    # image subscriber + UI window
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW_NAME, on_mouse)

    rospy.Subscriber(IMAGE_TOPIC, Image, image_cb, queue_size=1)

    rospy.loginfo("Subscribed to %s. Click on the image window to point the head!", IMAGE_TOPIC)
    try:
        # spin while keeping the OpenCV window responsive
        while not rospy.is_shutdown():
            # img drawing is handled in callback; just keep UI alive
            if img_for_display is None:
                cv2.waitKey(10)
            rospy.sleep(0.01)
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

