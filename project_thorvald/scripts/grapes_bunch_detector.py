#!/usr/bin/env python

import rospy, roslib, image_geometry, tf
import cv2
import math
import numpy
import sys
import rospy
from cv2 import namedWindow, cvtColor, imshow, inRange
import time
from cv2 import destroyAllWindows, startWindowThread, destroyWindow
from cv2 import COLOR_BGR2GRAY, waitKey, COLOR_BGR2HSV
from cv2 import blur, Canny, resize, INTER_CUBIC,INTER_LINEAR, INTER_NEAREST, INTER_LANCZOS4
from numpy import mean
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

#ros message
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

from math import cos, sin
from tf import TransformListener


import actionlib
from topological_navigation_msgs.msg import GotoNodeAction, GotoNodeGoal

#Declare variables to track the nodes to append objects co-ordinates of the detected grapes
exec_once=0
count_var = []
node = 0

class image_converter:

    global count_var
    camera_model = None
    image_depth_ros = None
    visualisation = True
    global_count = 0
    color2depth_aspect = (84.1/1920) / (70.0/512)
    global node
    global temp


    def __init__(self):

        
        #Subscribe to the required topics- camera, object location, depth sensor, Movement control, robot's position
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect",Image, self.image_callback)

        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_right_camera/hd/camera_info',
            CameraInfo, self.camera_info_callback)


        self.object_location_pub = rospy.Publisher('/thorvald_001/object_location', PoseStamped, queue_size=10)


        rospy.Subscriber("/thorvald_001/kinect2_right_sensor/sd/image_depth_rect",Image, self.image_depth_callback)

        self.tf_listener = tf.TransformListener()

        self.publisher = rospy.Publisher('/thorvald_001/teleop_joy/cmd_vel',Twist, queue_size=1)


    #Calculate the area of a contour 
    def contour_area(self, contours):

        # create an empty list
        cnt_area = []
        # loop through all the contours
        for i in range(0,len(contours),1):
            # for each contour, use OpenCV to calculate the area of the contour
             cnt_area.append(cv2.contourArea(contours[i]))

        # Sort our list of contour areas in descending order
        list.sort(cnt_area, reverse=True)
        return cnt_area
    
    #Get the position of beacons to eliminate the grapes outside the range of two beacons
    def get_beacon_range(self,data,image_color, image_depth):
        map_coord = []
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = resize(cv_image, None, fx=0.2, fy=0.2, interpolation = INTER_CUBIC)
        gray_img = cvtColor(cv_image, COLOR_BGR2HSV)
        h,s,v = cv2.split(gray_img)
        mask = cv2.inRange(gray_img, numpy.array((0, 0, 0)), numpy.array((33, 111, 28)))
        kernel = numpy.ones((5,5),numpy.uint8)
        kernel1 = numpy.ones((4,4),numpy.uint8)
        mask = cv2.erode(mask,kernel1,iterations = 1)
        mask = cv2.dilate(mask,kernel,iterations = 1)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        c=[]

        #Mask of beacons may not be together, time to cluster them
        c = self.agglomerative_cluster(contours, threshold_distance=100.0)
        i =0
        no_c =len(c)
        c_min=0
        c_max=100000
        for c in contours:
            # get the bounding rect
            i = i +1 
            if(i<3):
                x, y, w, h = cv2.boundingRect(c)
                m = cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(cv_image, str(i), (x+2, y+2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
                imshow("Beacon detector", m)
                if(i==1):
                    c_min = x
                if(i==2):
                    c_max = x
            
            #Get the x-axis values of left and right most beacon
            if(c_min > c_max):
                temp = c_max
                c_max = c_min
                c_min = temp

        return c_min, c_max, no_c

    #To calculate the distance of beacon's contour to group them together
    def calculate_contour_distance(self, contour1, contour2): 
        x1, y1, w1, h1 = cv2.boundingRect(contour1)
        c_x1 = x1 + w1/2
        c_y1 = y1 + h1/2

        x2, y2, w2, h2 = cv2.boundingRect(contour2)
        c_x2 = x2 + w2/2
        c_y2 = y2 + h2/2

        return max(abs(c_x1 - c_x2) - (w1 + w2)/2, abs(c_y1 - c_y2) - (h1 + h2)/2)

    #Merging contours only if it is within the threshold
    def merge_contours(self, contour1, contour2):
        return numpy.concatenate((contour1, contour2), axis=0)

    #inspired from https://www.datanovia.com/en/lessons/agglomerative-hierarchical-clustering/
    def agglomerative_cluster(self, contours, threshold_distance=1000.0):
        current_contours = contours
        while len(current_contours) > 1:
            min_distance = None
            min_coordinate = None

            for x in range(len(current_contours)-1):
                for y in range(x+1, len(current_contours)):
                    #Calculate the distance between the contours of the beacons to separate them apart
                    distance = self.calculate_contour_distance(current_contours[x], current_contours[y])
                    if min_distance is None:
                        min_distance = distance
                        min_coordinate = (x, y)
                    elif distance < min_distance:
                        min_distance = distance
                        min_coordinate = (x, y)

            if min_distance < threshold_distance:
                index1, index2 = min_coordinate
                current_contours[index1] = self.merge_contours(current_contours[index1], current_contours[index2])
                del current_contours[index2]
            else: 
                break

        return current_contours

    #Draw a bounding box around the grape bunches
    #https://www.freedomvc.com/index.php/2021/06/26/contours-and-bounding-boxes/
    def draw_bounding_box(self, contours, image, image_color, image_depth, b_min, b_max):
        # Call our function to get the list of contour areas
        global count_var
        cnt_area = self.contour_area(contours)
        j=0
       

        map_coord = []
         
        # Loop through each contour of our image
        for i in range(0,len(contours),1):
            cnt = contours[i]

            #contour extraction
            M = cv2.moments(cnt)

            try:
                cx = int(M["m10"]/M["m00"])
                cy = int(M["m01"]/M["m00"])

            
                #newly added
                # calculate the y,x centroid
                image_coords = (M["m01"] / M["m00"], M["m10"] / M["m00"])
                # "map" from color to depth image
                depth_coords = (image_depth.shape[0]/2 + (image_coords[0] - image_color.shape[0]/2)*self.color2depth_aspect,
                image_depth.shape[1]/2 + (image_coords[1] - image_color.shape[1]/2)*self.color2depth_aspect)
                # get the depth reading at the centroid location
                depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])] # you might need to do some boundary checking first!
                cv2.circle(image,(cx,cy), 1, (255,255,255),-1)
            
            except:
                #print("Error caught")
                pass


            # Draw the bounding box on all the grapes within the x range
            try:
                #print(b_min,b_max)
                if (cx - 10> b_min and cx<b_max+10):
                    j = j+1
                    x,y,w,h = cv2.boundingRect(cnt)
                    image=cv2.rectangle(image,(x,y),(x+w,y+h),(0,0,255),2)
                    cv2.putText(image, str(j) ,(cx, cy), cv2.FONT_HERSHEY_TRIPLEX, 0.2, (255,255,255), 1)
                    #Nan values occurs as sometimes the robot tries to see through the holes of bunches. Let us know check the neighbouring pixels in 'X' Shaped manner
                    for i in range(10):
                        #print('depth value',i, depth_value)
                        if(math.isnan(depth_value)==True):
                            #print('out at first iteration', depth_value, i)
                            depth_coords = (image_depth.shape[0]/2 + (image_coords[0]-i - image_color.shape[0]/2)*self.color2depth_aspect,image_depth.shape[1]/2 + (image_coords[1]+i - image_color.shape[1]/2)*self.color2depth_aspect)
                            depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])] # you might need to do some boundary checking first!
                        if(math.isnan(depth_value)==True):
                            #print('out at second iteration', depth_value, i)
                            depth_coords = (image_depth.shape[0]/2 + (image_coords[0]+i - image_color.shape[0]/2)*self.color2depth_aspect,image_depth.shape[1]/2 + (image_coords[1]-i - image_color.shape[1]/2)*self.color2depth_aspect)
                            depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])] # you might need to do some boundary checking first!
                        if(math.isnan(depth_value)==True):
                            #print('out at third iteration', depth_value, i)
                            depth_coords = (image_depth.shape[0]/2 + (image_coords[0]-i - image_color.shape[0]/2)*self.color2depth_aspect,image_depth.shape[1]/2 + (image_coords[1]-i - image_color.shape[1]/2)*self.color2depth_aspect)
                            depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])] # you might need to do some boundary checking first!
                        if(math.isnan(depth_value)==True):
                            #print('out at fourth iteration', depth_value, i)
                            depth_coords = (image_depth.shape[0]/2 + (image_coords[0]+i - image_color.shape[0]/2)*self.color2depth_aspect,image_depth.shape[1]/2 + (image_coords[1]+i - image_color.shape[1]/2)*self.color2depth_aspect)
                            depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])] # you might need to do some boundary checking first!
                    # calculate object's 3d location in camera coords
                    camera_coords = self.camera_model.projectPixelTo3dRay((image_coords[1], image_coords[0])) #project the image coords (x,y) into 3D ray in camera coords 
                    camera_coords = [x/camera_coords[2] for x in camera_coords] # adjust the resulting vector so that z = 1
                    camera_coords = [x*depth_value for x in camera_coords] # multiply the vector by depth

                    #define a point in camera coordinates
                    object_location = PoseStamped()
                    object_location.header.frame_id = "thorvald_001/kinect2_right_rgb_optical_frame"

                    object_location.pose.orientation.w = 1.0
                    object_location.pose.position.x = round(camera_coords[0],2)
                    object_location.pose.position.y = round(camera_coords[1],2)
                    object_location.pose.position.z = round(camera_coords[2],2)

                    # publish so we can see that in rviz
                    self.object_location_pub.publish(object_location)


                    # print out the coordinates in the map frame
                    p_camera = self.tf_listener.transformPose('map', object_location)

                    map_coord.append(p_camera.pose.position)
                    count_var.append(p_camera.pose.position)

            except:
                #print("Hungry! Looking for grapes")
                pass
        print('grapes here', j)
        self.global_count = self.global_count + j
        print('Total grapes without duplicates', self.global_count)
        return image, self.global_count
    
    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Only subscribe once
    
    def image_depth_callback(self, data):
        self.image_depth_ros = data
            
    #Approach inspired from workshop codes
    def image_callback(self, data):
        global exec_once

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            #print(e)
            pass
        b_cnt = 0 
        cv_image = resize(cv_image, None, fx=0.2, fy=0.2, interpolation = INTER_CUBIC)
        image_depth = resize(image_depth, None, fx=0.2, fy=0.2, interpolation = INTER_CUBIC)

        #Get the min and max beacon x-range
        try:
            c_min, c_max, b_cnt = self.get_beacon_range(data,cv_image, image_depth)
        except:
            pass

        #Convert the actual BGR image to HSV format
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        #Mask out only grapes in the hsv image
        mask = cv2.inRange(hsv_img,numpy.array((70, 0, 0)),numpy.array((158, 255, 255)))

        #Perform appropriate morphological operations to help the robot detect the grape bunches better
        kernel = numpy.ones((5,5),numpy.uint8)
        kernel1 = numpy.ones((6,6),numpy.uint8)
        kernel2 = numpy.ones((15,15),numpy.uint8)
        mask = cv2.dilate(mask,kernel,iterations = 1)
        mask = cv2.erode(mask,kernel1,iterations = 1)

        #Find the contours from the mask of the grapes spotted
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        image = cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 2) 
        #show_image = image
        imshow('Live feed',image)
        contours,hierarchy = cv2.findContours(mask, 1, 2) 
        waitKey(1)
        #Capture the image of the grapes only once at each topological nodes
        if(exec_once==0):
            #Count the grapes only if it is between two beacons
            if(b_cnt==2):
                contours,hierarchy = cv2.findContours(mask, 1, 2) 
                box, global_count = self.draw_bounding_box(contours, image, cv_image, image_depth, c_min , c_max)
                exec_once=1
                thickness =2
                cv2.putText(box, 'Total count -' + str(len(count_var)), (260, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.3,(255,255,255), 1, 1)
                imshow('Captured frame to detect grapes', box)
            waitKey(1)


if __name__ == '__main__':

    #Declare the node for the task
    rospy.init_node('topological_navigation_client')
    client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
    client.wait_for_server()
    #Mention the goals of the robot to navigate
    g = [3,4,6,7,2,5,8,9,1,0]
    #Iterate over the goals mentioned above
    for i in range(len(g)):
        goal = GotoNodeGoal()
        a = "WayPoint"+str(g[i])
        print(a)
        goal.target = a
        rospy.loginfo("going to %s", goal.target)
        client.send_goal(goal)
        status = client.wait_for_result() # wait until the action is complete
        result = client.get_result()
        rospy.loginfo("status is %s", status)
        rospy.loginfo("result is %s", result)
        if(i==0):
            ic = image_converter()
        exec_once=0
        node = node + 1
        time.sleep(5)
        print("came out of node",g[i])

        temp = []

        #Perform the iterations to avoid duplicates of grape bunches with -/+ 0.1 range
        for x in count_var:
            check_1=0
            check_2=0
            check_3=0

            x.x= round(x.x-0.1,1)
            x.y= round(x.y-0.1,1)
            x.z= round(x.z-0.1,1)
            if x not in temp:
                check_1=1
            x.x= round(x.x+0.2,1)
            x.y= round(x.y+0.2,1)
            x.z= round(x.z+0.2,1)
            if x not in temp:
                check_2=1
            x.x= round(x.x-0.1,1)
            x.y= round(x.y-0.1,1)
            x.z= round(x.z-0.1,1)
            if x not in temp:
                check_3=1
            if (check_1 and check_2 and check_3)==1:
                temp.append(x)

        count_var = temp
        time.sleep(5)
        #print(f'Updated List after removing duplicates = {temp}')
        print('updated count avoiding duplicates',len(temp))
