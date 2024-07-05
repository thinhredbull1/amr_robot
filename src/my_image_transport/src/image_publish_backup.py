#!/usr/bin/env python3

import rospy
# from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String
import cv2 as cv
# from cv_bridge import CvBridge
import numpy as np
import math
import pyzbar.pyzbar as pyzbar
from geometry_msgs.msg import Twist
class QRCodeDistanceCalculator:
    def __init__(self):
        self.camID = "v4l2src device=/dev/video0 ! video/x-raw, width=640, height=480 ! videoconvert ! video/x-raw,format=BGR ! appsink "
        self.KNOWN_DISTANCE = 30.1  # cm
        self.KNOWN_WIDTH = 5.5  # cm
        self.Rwidth = 111.07

        # Define the fonts
        self.fonts = cv.FONT_HERSHEY_COMPLEX
        self.Pos = (50, 50)
        # Colors (BGR)
        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.MAGENTA = (255, 0, 255)
        self.GREEN = (0, 255, 0)
        self.BLUE = (255, 0, 0)
        self.RED = (0, 0, 255)
        self.CYAN = (255, 255, 0)
        self.GOLD = (0, 255, 215)
        self.YELLOW = (0, 255, 255)
        self.ORANGE = (0, 165, 230)
        self.focalLength = self.focalLengthFinder(self.KNOWN_DISTANCE, self.KNOWN_WIDTH, self.Rwidth)
        self.tag_stop_precise=["0","1"]
        self.tag_now="0"
        self.navi_data=""
        self.tag_des=""
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rospy_pub_tag=rospy.Publisher('/tag_dest',String,queue_size=1)
        self.pub_pose=rospy.Publisher('/id_init_pose',String,queue_size=1)
        self.rospy_pub_bt=rospy.Publisher('/bt_request',String,queue_size=1)
        rospy.Subscriber('/navi_status',String, self.navi_callback)
        rospy.Subscriber('/current_tag_position',String, self.tag_check)
        rospy.Subscriber('/tag_dest',String, self.store_tag)
    def store_tag(self,data):
        self.tag_des=data.data
        rospy.loginfo("tag_des %s",self.tag_des)
    def navi_callback(self,data):
        self.navi_data=data.data
        rospy.loginfo("NAVI STATUS %s",self.navi_data)
    def tag_check(self,data):
        self.tag_now=data.data
        rospy.loginfo("tag %s",self.tag_now)
    def showText(self, image, text, position, color, animateOnValue=None):
        fonts = cv.FONT_HERSHEY_COMPLEX
        x, y = position
        y = y - 20
        lineThicknes = 30
        offset = int(lineThicknes / 2)
        center = int(offset / 2)
        new_pos = (x, y + center)
        lineLength = 229
        newThickness = int(lineThicknes * 0.7)

        cv.line(image, (x, y), (x + lineLength, y), self.ORANGE, lineThicknes)
        cv.line(image, (x, y), (x + lineLength, y), self.GREEN, newThickness)
        if animateOnValue is not None:
            cv.line(image, (x, y), (x + animateOnValue, y), self.WHITE, newThickness)
        cv.putText(image, text, new_pos, fonts, 0.6, self.BLACK, 2)

    def euclideanDistance(self, x, y, x1, y1):
        return math.sqrt((x1 - x) ** 2 + (y1 - y) ** 2)

    def focalLengthFinder(self, knownDistance, knownWidth, widthInImage):
        return (widthInImage * knownDistance) / knownWidth

    def realHorizontalDistanceFromCenter(self, center_x, frame_width, distance, focalLength):
        # Calculate the real horizontal distance from the center of the image (camera)
        delta_x = center_x - (frame_width / 2)
        # Assuming the width of the image at the given distance
        width_at_distance = (frame_width * distance) / focalLength
        # Calculate the real horizontal distance
        real_horizontal_distance = (delta_x * width_at_distance) / frame_width
        return real_horizontal_distance

    def distanceFinder(self, focalLength, knownWidth, widthInImage):
        return (knownWidth * focalLength) / widthInImage

    def DetectQRcode(self, image):
        if image is None:
            return None, None, None, None
        Gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        objectQRcode = pyzbar.decode(Gray)
        for obDecoded in objectQRcode:
            points = obDecoded.polygon
            if len(points) > 4:
                hull = cv.convexHull(np.array([points for point in points], dtype=np.float32))
                hull = list(map(tuple, np.squeeze(hull)))
            else:
                hull = points

            n = len(hull)
            for j in range(0, n):
                cv.line(image, hull[j], hull[(j + 1) % n], self.ORANGE, 2)

            x, x1 = hull[0][0], hull[1][0]
            y, y1 = hull[0][1], hull[1][1]
            self.Pos = hull[3]
            qr_type = obDecoded.data.decode("utf-8")  # Get the QR code data

            center_x = sum([point[0] for point in hull]) / len(hull)
            center_y = sum([point[1] for point in hull]) / len(hull)
            return self.euclideanDistance(x, y, x1, y1), self.Pos, qr_type, (center_x, center_y)
        return None, None, None, None

    def main(self):
        rospy.init_node('image_process', anonymous=True)
        # pub = rospy.Publisher('camera/image', Image, queue_size=10)
        # bridge = CvBridge()
        camera=None
        counter = 0
        rate = rospy.Rate(10)
        run_camera=0
        twist = Twist()
        twist.linear.y = 0.0  # Vận tốc tuyến tính theo trục y (m/s)
        twist.linear.z = 0.0  # Vận tốc tuyến tính theo trục z (m/s)
        twist.angular.x = 0.0 # Vận tốc góc theo trục x (rad/s)
        twist.angular.y = 0.0 # Vận tốc góc theo trục y (rad/s)
        Distance=0
        Dis_stop=10
        y_stop=4.82
        wait_to_distance=0
        x_vel=0
        turn_vel=0
        end_time=rospy.Time.now()
        count_to_go=0
        stop_now=0
        done_forward=0
        count_stop=0
        camera_calib=0
        wait_to_pub=0
        last_turn=0
        while not rospy.is_shutdown():
            if self.tag_des==self.tag_now and (self.tag_des in self.tag_stop_precise) and run_camera==0:
                if self.navi_data=="SUCCESS":
                    self.navi_data=""
                    rospy.loginfo("start_process camera")
                    camera = cv.VideoCapture(self.camID)
                    if not camera.isOpened():
                        rospy.logerr("Error: Could not open camera.")
                    else:
                        run_camera=1
                        rospy.loginfo("open camera ok")
            if run_camera:
                ret, frame = camera.read()
                if not ret:
                    rospy.logwarn("Failed to grab frame.")
                    break
                codeWidth, Pos, qr_type, center = self.DetectQRcode(frame)
                if codeWidth and self.focalLength:
                    Distance = self.distanceFinder(self.focalLength, self.KNOWN_WIDTH, codeWidth)-1
                    frame_width = frame.shape[1]
                    horizontal_distance = self.realHorizontalDistanceFromCenter(center[0], frame_width, Distance, self.focalLength)
                    print("y:"+str(horizontal_distance))
                    sign_x=Distance-Dis_stop
                    sign_y=y_stop-horizontal_distance
                    turn_vel_abs=(sign_y)*0.02+(sign_y-last_turn)*0.01
                    last_turn=sign_y
                    turn_vel=turn_vel_abs
                    if(abs(sign_y)<0.08):
                        turn_vel=0
                    if wait_to_distance==0:
                        print("x:"+str(Distance))
                        second_to_wait=round(sign_x/6)
                        if(second_to_wait==0 or abs(Distance)<=Dis_stop):
                            x_vel = 0.0  # Vận tốc tuyến tính theo trục x (m/s)
                            turn_vel = 0.0 # Vận tốc góc theo trục z (rad/s)
                            run_camera=0
                            twist.linear.x =x_vel  # Vận tốc tuyến tính theo trục x (m/s
                            twist.angular.z = turn_vel # Vận tốc góc theo trục z (rad/s)
                            print(x_vel)
                            print(turn_vel)
                            self.cmd_vel_pub.publish(twist)
                            self.navi_data=""
                            self.tag_des=""
                            rospy.loginfo("Done1")
                            count_to_go=0
                        else:
                            wait_duration=rospy.Duration(second_to_wait)
                            end_time=rospy.Time.now()+wait_duration
                         
                            wait_to_distance=1
                            if(sign_x>0):
                                x_vel=0.06
                            else:
                                x_vel=-0.06
                          
                else:
                    if(count_to_go>=1):
                        wait_to_distance=0
                        count_to_go=0
                        x_vel = 0.0  # Vận tốc tuyến tính theo trục x (m/s)
                        turn_vel = 0.0 # Vận tốc góc theo trục z (rad/s)
                        run_camera=0
                        self.navi_data=""
                        self.tag_des=""
                        rospy.loginfo("Done2")
                        twist.linear.x =x_vel  # Vận tốc tuyến tính theo trục x (m/s
                        twist.angular.z = turn_vel # Vận tốc góc theo trục z (rad/s)
                        print(turn_vel)
                        self.cmd_vel_pub.publish(twist)
                        
                    # turn_vel=0
                if wait_to_distance:
                    if(abs(Distance)<=Dis_stop):
                        x_vel = 0.0  # Vận tốc tuyến tính theo trục x (m/s)
                        turn_vel = 0.0 # Vận tốc góc theo trục z (rad/s)
                        run_camera=0
                        self.navi_data=""
                        self.tag_des=""
                        rospy.loginfo("Done3")
                        count_to_go=0
                    twist.linear.x =x_vel  # Vận tốc tuyến tính theo trục x (m/s
                    twist.angular.z = turn_vel # Vận tốc góc theo trục z (rad/s)
           
                    print(turn_vel)
                    self.cmd_vel_pub.publish(twist)
                    if(rospy.Time.now()>=end_time):
                        rospy.loginfo("out of time")
                        wait_to_distance=0
                        count_to_go+=1
                if(count_to_go>=3):
                    wait_to_distance=0
                    count_to_go=0
                    x_vel = 0.0  # Vận tốc tuyến tính theo trục x (m/s)
                    turn_vel = 0.0 # Vận tốc góc theo trục z (rad/s)
                    run_camera=0
                    self.navi_data=""
                    self.tag_des=""
                    rospy.loginfo("Done4")
                    twist.linear.x =x_vel  # Vận tốc tuyến tính theo trục x (m/s
                    twist.angular.z = turn_vel # Vận tốc góc theo trục z (rad/s)
        
                    print(turn_vel)
                    self.cmd_vel_pub.publish(twist)
                if(run_camera==1):
                    cv.imshow("frame", frame)
                    key = cv.waitKey(1)
                    if key == ord('q'):
                        break
                else:
                    count_to_go=0
                    camera_calib=1
                    wait_to_distance=0
                    camera.release()
                    cv.destroyAllWindows()
            else:
                if camera_calib==1:
                    # if(self.tag_des != ""):
                    wait_to_pub+=1
                    if(wait_to_pub>=50):
                        camera_calib=0
                        print("go_out")
                        twist.linear.x =-0.06  # Vận tốc tuyến tính theo trục x (m/s
                        self.cmd_vel_pub.publish(twist)
                        stop_now=1
                else:
                    wait_to_pub=0
                if stop_now==1:
                    count_stop+=1
                    if(count_stop>=40):
                        stop_now=0
                        twist.linear.x =0  # Vận tốc tuyến tính theo trục x (m/s
                        self.cmd_vel_pub.publish(twist)
                        if(self.tag_now=="1"):
                            self.rospy_pub_tag.publish("2")
                            rospy.sleep(0.5)
                            self.rospy_pub_bt.publish("STARTSLAM")
                        elif(self.tag_now=="2"):
                            self.rospy_pub_tag.publish("0")
                            rospy.sleep(0.5)
                            self.rospy_pub_bt.publish("STARTSLAM")
                else:
                    count_stop=0
                rate.sleep()
        camera.release()
        cv.destroyAllWindows()

if __name__ == '__main__':
    try:
        qr_code_process=QRCodeDistanceCalculator()
        qr_code_process.main()
    except rospy.ROSInterruptException:
        pass
