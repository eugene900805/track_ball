#! /usr/bin/python
import rospy
# ROS Image message
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import numpy as np
from std_msgs.msg import Header
import time

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    print("Received an image!")
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "rgb8")
        cv2_img = np.array(cv2_img)
    except CvBridgeError, e:
        print(e)
    else:
       
        pan_tilt = [0,0]
        h, w, d = cv2_img.shape

        cv2_img = cv2.resize(cv2_img, (w/10, h/10), interpolation=cv2.INTER_NEAREST)
        h, w, d = cv2_img.shape
        
        # cv2.imwrite('image/camera_image'+str(time.time())+'.png', cv2_img)
        
       
        color_1 = [155,20,20]
        color_range_1 = [106,21,21]
        
        ball_all_x = []
        ball_all_y = []
        for y in range(h):
            for x in range(w):
                if (np.abs(color_1[2]-cv2_img[y,x,2]) < color_range_1[2]):
                    if (np.abs(color_1[1]-cv2_img[y,x,1]) < color_range_1[1]):
                        if(np.abs(color_1[0]-cv2_img[y,x,0]) < color_range_1[0]): 
                            ball_all_x.append(x)
                            ball_all_y.append(y)
        if ball_all_x != []: 
            ball_y = int(sum(ball_all_y) / len(ball_all_y))
            ball_x = int(sum(ball_all_x) / len(ball_all_x))
            
            
            move_angles = 5
            if ball_y == int(h/2):
                pan_tilt[1] = 0
            elif ball_y > int(h/2):
                pan_tilt[1] = -3.1415926/180/2 * move_angles
            else:
                pan_tilt[1] = 3.1415926/180/2 * move_angles
                
            if ball_x == int(w/2):
                pan_tilt[0] = 0
            elif ball_x > int(w/2):
                pan_tilt[0] = -3.1415926/180/2 * move_angles
            else:
                pan_tilt[0] = 3.1415926/180/2 * move_angles
        
      
        pub_m = rospy.Publisher('/robotis/enable_ctrl_module', String, queue_size=1)
        module = "head_control_module"
        pub_m.publish(module)
        
        pub = rospy.Publisher('/robotis/head_control/set_joint_states_offset', JointState, queue_size=1)
 
        print(pan_tilt)
        
        change_angle = JointState()
        change_angle.header = Header()
        # change_angle.header.stamp = rospy.get_rostime()
        change_angle.name = ['head_pan', 'head_tilt']
        change_angle.position = [pan_tilt[0], pan_tilt[1]]
        change_angle.velocity = []
        change_angle.effort = []
        pub.publish(change_angle)


def main():
    rospy.init_node('image_judge')
    

    image_topic = "/robotis/images"
    rospy.Subscriber(image_topic, Image, image_callback)
    
    rospy.spin()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass






