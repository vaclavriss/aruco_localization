#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rospy.init_node("webcam_republisher", anonymous=True)

    # Topic name can be remapped in launch file
    image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)
    bridge = CvBridge()

    # Open default webcam (0 = built-in, try 1 if you have external USB camera)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        rospy.logerr("Could not open webcam!")
        return

    rate = rospy.Rate(30)  # 30 Hz
    rospy.loginfo("Webcam publisher started. Publishing images on /camera/image_raw")

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to capture frame from webcam")
            continue

        # Convert OpenCV image (BGR) -> ROS Image
        img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = rospy.Time.now()
        image_pub.publish(img_msg)

        rate.sleep()

    cap.release()

if __name__ == "__main__":
    main()

