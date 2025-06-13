#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessor(Node):

    def empty(self,a):
        pass


    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Change this if your topic name is different
            self.listener_callback,
            10)
        self.bridge = CvBridge()
            
        cv2.namedWindow("HSV")
        cv2.resizeWindow("HSV", 300, 300)
        cv2.createTrackbar("HUE Min", "HSV", 0, 179, self.empty)
        cv2.createTrackbar("HUE Max", "HSV", 179, 179, self.empty)
        cv2.createTrackbar("SAT Min", "HSV", 0, 255, self.empty)
        cv2.createTrackbar("SAT Max", "HSV", 255, 255, self.empty)
        cv2.createTrackbar("VALUE Min", "HSV", 0, 255, self.empty)
        cv2.createTrackbar("VALUE Max", "HSV", 255, 255, self.empty)
        cv2.createTrackbar("R1 Max", "HSV", 179, 179, self.empty)
        cv2.createTrackbar("R1 Min", "HSV", 0, 179, self.empty)


        cv2.resizeWindow('F', 700,600)



    def resize_final_img(self,x,y,*argv):
        images  = cv2.resize(argv[0], (x, y))
        for i in argv[1:]:
            resize = cv2.resize(i, (x, y))
            images = np.concatenate((images,resize),axis = 1)
        return images

    def listener_callback(self, msg):
        # Convert ROS Image to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h_min = cv2.getTrackbarPos("HUE Min", "HSV")
        h_max = cv2.getTrackbarPos("HUE Max", "HSV")
        s_min = cv2.getTrackbarPos("SAT Min", "HSV")
        s_max = cv2.getTrackbarPos("SAT Max", "HSV")
        v_min = cv2.getTrackbarPos("VALUE Min", "HSV")
        v_max = cv2.getTrackbarPos("VALUE Max", "HSV")
        r1_min = cv2.getTrackbarPos("R1 Min", "HSV")
        r1_max = cv2.getTrackbarPos("R1 Max", "HSV")
        # ----- ✅ Example: Convert to HSV and Threshold Red Color -----
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        r1upper = np.array([r1_max, s_max, v_max])
        r1lower = np.array([r1_min, s_min, v_min])
        mask1 = cv2.inRange(hsv_img, lower, upper)
        mask2 = cv2.inRange(hsv_img, r1lower, r1upper)
        mask = mask1 + mask2
        #mask = mask1
        kernel = np.ones((3,3),'uint8')
       #mask = cv2.
        #mask = cv2.dilate(mask, kernel, iterations=1)
        #mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel,iterations = 2)
        d_img = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel,iterations = 2)

        

        final_img = self.resize_final_img(300,300, d_img)
        # final_img = np.concatenate((mask,d_img,e_img),axis =1)
        

        # ----- ✅ Show results -----
        cv2.imshow("Original", cv_image)
        cv2.imshow("Red Mask", mask)
        #cv2.imshow("Result", result)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

