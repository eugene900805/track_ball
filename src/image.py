 #! /usr/bin/python
import rospy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String, Header
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Instantiate CvBridge
bridge = CvBridge()

def callback(msg):
    try:
        pub = rospy.Publisher('/robotis/images', Image, queue_size=1)
        pub.publish(msg)
    except CvBridgeError, e:
        print(e)

def main():
    rospy.init_node('image_listener')
    image_topic = "/robotis_op3/camera/image_raw"
    rospy.Subscriber(image_topic, Image, callback)
    
    rospy.spin()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
    
'''
roslaunch op3_gazebo my_world.launch
roslaunch op3_manager op3_gazebo.launch
rosrun rqt_image_view rqt_image_view

roscd track_ball/src
python image.py
roscd track_ball/src
python judge.py
'''
    
