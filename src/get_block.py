#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessor(Node):

 


    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Change this if your topic name is different
            self.listener_callback,
            10)
        self.pub = self.create_publisher(String, '/color_direction', 10)

        self.bridge = CvBridge()
        
        self.both_flag = False
        self.all_flag = False
        self.only_red = False
        self.only_green = False
        self.only_pink = False
        self.pink_red = False
        self.pink_green = False
        self.green_present = False
        self.red_present = False
        self.pink_present = False

            


        self.kernel = np.ones((3, 3), 'uint8')
        font = cv2.FONT_HERSHEY_SIMPLEX
        org = (0, 20)
        fontScale = 0.6
        color = (0, 0, 255)
        thickness = 2


    def resize_final_img(self,x,y,*argv):
        images  = cv2.resize(argv[0], (x, y))
        for i in argv[1:]:
            resize = cv2.resize(i, (x, y))
            images = np.concatenate((images,resize),axis = 1)
        return images

    def listener_callback(self, msg):
        # Convert ROS Image to OpenCV image
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # green
        hsv_img1 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # red
        hsv_img2 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # pink

        # predefined mask for green colour detection
        # For Green Color
        lower = np.array([56, 0, 105])  # green
        upper = np.array([61, 255, 145])
        mask = cv2.inRange(hsv_img, lower, upper)
        mask = cv2.dilate(mask, self.kernel, iterations=3)
        mask = cv2.erode(mask, self.kernel, iterations=3)

        # For Red Color
        lower1 = np.array([179, 1, 75])  # red
        upper1 = np.array([179, 255, 159])
        lower1_r = np.array([0, 1, 75])
        upper1_r = np.array([0, 255, 159])
        mask1 = cv2.inRange(hsv_img1, lower1, upper1)
        mask1_r = cv2.inRange(hsv_img1, lower1_r, upper1_r)
        mask_red = mask1 + mask1_r
        mask_red = cv2.dilate(mask_red, self.kernel, iterations=3)
        mask_red = cv2.erode(mask_red, self.kernel, iterations=3)
                
        # For Pink Color
        lower2 = np.array([154, 168, 59])  # pink
        upper2 = np.array([171, 217, 192])
        mask2 = cv2.inRange(hsv_img2, lower2, upper2)
        mask2 = cv2.dilate(mask2, self.kernel, iterations=5)
        mask2 = cv2.erode(mask2, self.kernel, iterations=5)

        # Remove Extra garbage from image
        d_img = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=5)  # green
        d_img1 = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, self.kernel, iterations=5)  # red
        d_img2 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, self.kernel, iterations=5)  # pink

        # find the histogram
        cont, hei = cv2.findContours(
            d_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cont = sorted(cont, key=cv2.contourArea, reverse=True)[:1]

        cont1, hei1 = cv2.findContours(
            d_img1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cont1 = sorted(cont1, key=cv2.contourArea, reverse=True)[:1]

        cont2, hei2 = cv2.findContours(
            d_img2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cont2 = sorted(cont2, key=cv2.contourArea, reverse=True)[:1]


        # ----------------------------------------------
        if len(cont) == 0:
            self.green_present = False

        else:
            max_cnt = max(cont, key=cv2.contourArea)
            if cv2.contourArea(max_cnt) > 2000:
	            self.green_present = True
            else:
	            self.green_present = False
        

        # ---------------------------------------
        if len(cont1) == 0:
            self.red_present = False
            #print("No contours present")

        else:
            max_cnt1 = max(cont1, key=cv2.contourArea)
            #print(f"area cont1: {cv2.contourArea(max_cnt1)}")  
            if cv2.contourArea(max_cnt1) > 2000:
	            self.red_present = True
            else:
	            self.red_present = False
        # ----------------------------------------------------
        if len(cont2) == 0:
            self.pink_present = False

        else:
            max_cnt2 = max(cont2, key=cv2.contourArea)
            if cv2.contourArea(max_cnt2) > 2000:
	            self.pink_present = True
            else:
	            self.pink_present = False
        # --------------------------------------------------------
        
        



        #print(f"Green: {cv2.contourArea(max_cnt)} Red:{cv2.contourArea(max_cnt1)} Pink:{cv2.contourArea(max_cnt2)}")
        
        if not self.red_present and not self.green_present and not self.pink_present:

            self.all_flag = False
            self.both_flag = False
            self.pink_red = False
            self.pink_green = False
            self.only_red = False
            self.only_green = False
            self.only_pink = False

        if (self.red_present and self.green_present and self.pink_present):
            self.all_flag = True
            self.both_flag = False
            self.pink_red = False
            self.pink_green = False
            self.only_red = False
            self.only_green = False
            self.only_pink = False
        elif (self.red_present and self.green_present) and not self.pink_present:
            self.both_flag = True
            self.all_flag = False
            self.only_red = False
            self.only_green = False
            self.only_pink = False
            self.pink_red = False
            self.pink_green = False
        elif self.red_present and (not self.pink_present and not self.green_present):
            self.only_red = True
            self.both_flag = False
            self.all_flag = False
            self.only_green = False
            self.only_pink = False
            self.pink_red = False
            self.pink_green = False
        elif self.green_present and (not self.pink_present and not self.red_present):
            self.only_green = True
            self.both_flag = False
            self.all_flag = False
            self.only_red = False
            self.only_pink = False
            self.pink_red = False
            self.pink_green = False
        elif self.pink_present and (not self.green_present and not self.red_present):
            self.only_pink = True
            self.both_flag = False
            self.all_flag = False
            self.only_red = False
            self.only_green = False
            self.pink_red = False
            self.pink_green = False
        elif (self.pink_present and self.green_present) and not self.red_present:
            self.only_pink = False
            self.both_flag = False
            self.all_flag = False
            self.only_red = False
            self.only_green = False
            self.pink_red = False
            self.pink_green = True

        elif (self.pink_present and self.red_present) and not self.green_present:
            self.only_pink = False
            self.both_flag = False
            self.all_flag = False
            self.only_red = False
            self.only_green = False
            self.pink_red = True
            avoid_park.value = True
            self.pink_green = False
            
        if self.all_flag:

            # print("ITS THE FIRST LOOP")
            # FOR GREEN BOX
            if cv2.contourArea(max_cnt) > cv2.contourArea(max_cnt1) and cv2.contourArea(max_cnt) > cv2.contourArea(max_cnt2):
                if cv2.contourArea(max_cnt) > 1000 and cv2.contourArea(max_cnt) < 306000:
                    # Draw a rectange on the contour
                    rect = cv2.minAreaRect(max_cnt)
                    box = cv2.boxPoints(rect)
                    box = np.intp(box)
                    cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
                    (x, y, w, h) = cv2.boundingRect(box)
                    centroid_y = y + h // 2
                    self.pub.publish(String(data="green"))



            # FOR RED BOX
            if cv2.contourArea(max_cnt1) > cv2.contourArea(max_cnt) and cv2.contourArea(max_cnt1) > cv2.contourArea(max_cnt2):
                if (cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000):
                    # Draw a rectange on the contour
                    rect1 = cv2.minAreaRect(max_cnt1)
                    box = cv2.boxPoints(rect1)
                    box = np.intp(box)
                    cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

                    (x, y, w, h) = cv2.boundingRect(box)

                    centroid_y_red = y + h // 2
                    self.pub.publish(String(data="red"))
      
            # FOR PINK BOX
            if cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt) and cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt1):
                if (cv2.contourArea(max_cnt2) > 2000 and cv2.contourArea(max_cnt2) < 306000):
                    # Draw a rectange on the contour
                    rect2 = cv2.minAreaRect(max_cnt2)
                    box = cv2.boxPoints(rect2)
                    box = np.intp(box)
                    cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

                    (x, y, w, h) = cv2.boundingRect(box)

                    centroid_y = y + h // 2

                    self.pub.publish(String(data="pink"))

        # FOR RED BOX
        elif self.only_red:

            if cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000:
                # Draw a rectange on the contour
                rect1 = cv2.minAreaRect(max_cnt1)
                box = cv2.boxPoints(rect1)
                box = np.intp(box)
                cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
                (x, y, w, h) = cv2.boundingRect(box)

                centroid_y_red = y + h // 2
                self.pub.publish(String(data="red"))
        # FOR GREEN BOX
        elif self.only_green:

            # print(cv2.contourArea(max_cnt))
            if cv2.contourArea(max_cnt) > 1000 and cv2.contourArea(max_cnt) < 306000:
                # Draw a rectange on the contour
                rect = cv2.minAreaRect(max_cnt)
                box = cv2.boxPoints(rect)
                box = np.intp(box)
                cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
                (x, y, w, h) = cv2.boundingRect(box)
                self.pub.publish(String(data="green"))
                centroid_y = y + h // 2


        # FOR PINK BOX
        elif self.only_pink:
            if (cv2.contourArea(max_cnt2) > 2000 and cv2.contourArea(max_cnt2) < 306000):
                # Draw a rectange on the contour
                rect2 = cv2.minAreaRect(max_cnt2)
                box = cv2.boxPoints(rect2)
                box = np.intp(box)
                cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

                (x, y, w, h) = cv2.boundingRect(box)

                centroid_y = y + h // 2

        elif self.both_flag:

            # print("BOTH ARE PRESENT...")
            # FOR GREEN BOX
            if cv2.contourArea(max_cnt) > cv2.contourArea(max_cnt1):
                if cv2.contourArea(max_cnt) > 1000 and cv2.contourArea(max_cnt) < 306000:
                    # Draw a rectange on the contour
                    rect = cv2.minAreaRect(max_cnt)
                    box = cv2.boxPoints(rect)
                    box = np.intp(box)
                    cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
                    (x, y, w, h) = cv2.boundingRect(box)
                    centroid_y = y + h // 2
                    self.pub.publish(String(data="green"))

            # FOR RED BOX
            elif cv2.contourArea(max_cnt1) > cv2.contourArea(max_cnt):
                if (cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000):
                    # Draw a rectange on the contour
                    rect1 = cv2.minAreaRect(max_cnt1)
                    box = cv2.boxPoints(rect1)
                    box = np.intp(box)
                    cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

                    (x, y, w, h) = cv2.boundingRect(box)

                    centroid_y_red = y + h // 2
                    self.pub.publish(String(data="red"))


        elif self.pink_red:
            # FOR RED BOX
            if cv2.contourArea(max_cnt1) > cv2.contourArea(max_cnt2):
                if (cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000):
                    # Draw a rectange on the contour
                    rect1 = cv2.minAreaRect(max_cnt1)
                    box = cv2.boxPoints(rect1)
                    box = np.intp(box)
                    cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

                    (x, y, w, h) = cv2.boundingRect(box)
                    self.pub.publish(String(data="red"))
                    centroid_y_red = y + h // 2

            elif cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt1):
                if (cv2.contourArea(max_cnt2) > 1000 and cv2.contourArea(max_cnt2) < 306000):
                    # Draw a rectange on the contour
                    rect2 = cv2.minAreaRect(max_cnt2)
                    box = cv2.boxPoints(rect2)
                    box = np.intp(box)
                    cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

                    (x, y, w, h) = cv2.boundingRect(box)

                    centroid_y = y + h // 2



        elif self.pink_green:
            if cv2.contourArea(max_cnt) > cv2.contourArea(max_cnt2):
                if (cv2.contourArea(max_cnt) > 2000 and cv2.contourArea(max_cnt) < 306000):
                    # Draw a rectange on the contour
                    rect = cv2.minAreaRect(max_cnt)
                    box = cv2.boxPoints(rect)
                    box = np.intp(box)
                    cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
                    self.pub.publish(String(data="green"))
                    (x, y, w, h) = cv2.boundingRect(box)

                    centroid_y = y + h // 2

            if cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt):
                if (cv2.contourArea(max_cnt2) > 2000 and cv2.contourArea(max_cnt2) < 306000):
                    # Draw a rectange on the contour
                    rect2 = cv2.minAreaRect(max_cnt2)
                    box = cv2.boxPoints(rect2)
                    box = np.intp(box)
                    cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
                    (x, y, w, h) = cv2.boundingRect(box)
                    centroid_y = y + h // 2

        
        #print(f"Green:{self.green_present}, red:{self.red_present}, pink:{self.pink_present}")
        # print(f"all:{self.all_flag}, self.only_red:{self.only_red}, self.only_green:{self.only_green}, self.only_pink:{self.only_pink}, self.pink_green:{self.pink_green}, self.pink_red:{self.pink_red}, both:{self.both_flag}")
        # print(f"g_next:{g_next.value}, r_next:{r_next.value}")
        # print(f"green:{green_b.value}  red:{red_b.value}, pink:{pink_b.value}")
        cv2.imshow('Object Dist Measure ', img)

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

