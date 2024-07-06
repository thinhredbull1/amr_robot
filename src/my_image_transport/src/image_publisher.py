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

SLAM_MODE=0
CAMERA_MODE=1
WAIT_TO_X=2
ALIGN=3
GO_OUT=3
OUT_OF_TIME=4
class QRCodeDistanceCalculator:
    def __init__(self):
        self.camID = "v4l2src device=/dev/video0 ! video/x-raw, width=640, height=480 ! videoconvert ! video/x-raw,format=BGR ! appsink "
        self.KNOWN_DISTANCE = 30.1  # cm
        self.KNOWN_WIDTH = 5.5  # cm
        self.Rwidth = 111.07
        self.Dis_stop=12
        self.y_stop=4.82
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
        self.state_robot=SLAM_MODE
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
        rate = rospy.Rate(10)
        twist = Twist()
        twist.linear.y = 0.0  # Vận tốc tuyến tính theo trục y (m/s)
        twist.linear.z = 0.0  # Vận tốc tuyến tính theo trục z (m/s)
        twist.angular.x = 0.0 # Vận tốc góc theo trục x (rad/s)
        twist.angular.y = 0.0 # Vận tốc góc theo trục y (rad/s)
        x_dis=0
        x_vel=0
        turn_vel=0
        end_time=rospy.Time.now()
        count_to_go=0
        count_stop=0
        last_turn=0
        go_backward=0
        qr_type=""
        while not rospy.is_shutdown():
            if self.state_robot==SLAM_MODE:
                if self.tag_des==self.tag_now and (self.tag_des in self.tag_stop_precise) and run_camera==0:
                    if self.navi_data=="SUCCESS":
                        self.navi_data=""
                        rospy.loginfo("start_process camera")
                        camera = cv.VideoCapture(self.camID)
                        if not camera.isOpened():
                            rospy.logerr("Error: Could not open camera.")
                        else:
                            self.state_robot=CAMERA_MODE
                            rospy.loginfo("open camera ok")
                rate.sleep()
            elif self.state_robot!=GO_OUT:
                ret, frame = camera.read()
                if not ret:
                    rospy.logwarn("Failed to grab frame.")
                    break
                codeWidth, Pos, qr_type, center = self.DetectQRcode(frame)
                if codeWidth and self.focalLength:
                    x_dis = self.distanceFinder(self.focalLength, self.KNOWN_WIDTH, codeWidth)-1
                    frame_width = frame.shape[1]
                    y_dis = self.realHorizontalDistanceFromCenter(center[0], frame_width, x_dis, self.focalLength)
                    print("y:"+str(x_dis))
                    ####
                    sign_x=x_dis-self.Dis_stop
                    sign_y=self.y_stop-y_dis
                    turn_vel=(sign_y)*0.02+(sign_y-last_turn)*0.01
                    last_turn=sign_y
                    if(abs(sign_y)<0.05):
                        turn_vel=0
                    ###
                    if self.state_robot!=WAIT_TO_X:
                        print("x:"+str(Distance))
                        second_to_wait=round(sign_x/6)
                        if(second_to_wait==0 or abs(Distance)<=self.Dis_stop):             
                            rospy.loginfo("Done1")
                            self.state_robot=GO_OUT
                        else:
                            wait_duration=rospy.Duration(second_to_wait)
                            end_time=rospy.Time.now()+wait_duration
                            self.state_robot=WAIT_TO_X
                            if(sign_x>0):
                                x_vel=0.06
                            else:
                                x_vel=-0.06
                          
                else:
                    if(count_to_go>=1):
                        rospy.loginfo("OUT OF IMAGE")
                    # turn_vel=0
                if self.state_robot==WAIT_TO_X:
                    if(abs(Distance)<=Dis_stop):
                        rospy.loginfo("Done3")
                        self.state_robot=GO_OUT
                        count_to_go=0
                    twist.linear.x =x_vel  # Vận tốc tuyến tính theo trục x (m/s
                    twist.angular.z = turn_vel # Vận tốc góc theo trục z (rad/s)
                    print(turn_vel)
                    self.cmd_vel_pub.publish(twist)
                    if(rospy.Time.now()>=end_time):
                        rospy.loginfo("out of time")
                        self.state_robot=CAMERA_MODE
                        count_to_go+=1
                if(self.state_robot!=GO_OUT):
                    cv.imshow("frame", frame)
                    key = cv.waitKey(1)
                    if key == ord('q'):
                        break
                else:
                    twist.linear.x =0  # Vận tốc tuyến tính theo trục x (m/s
                    twist.angular.z = 0 # Vận tốc góc theo trục z (rad/s)
                    count_to_go=0
                    last_turn=0
                    self.cmd_vel_pub.publish(twist)
                    camera.release()
                    cv.destroyAllWindows()
            elif self.state_robot==GO_OUT:
                if go_backward==0:
                    wait_to_pub+=1
                    if(wait_to_pub>=50):
                        go_backward=1
                        self.pub_pose.publish(qr_type)
                        print("go_out")
                        twist.linear.x =-0.06  # Vận tốc tuyến tính theo trục x (m/s
                        self.cmd_vel_pub.publish(twist)
                else :
                    wait_to_pub=0
                    count_stop+=1
                    if(count_stop>=40):
                        go_backward=0
                        count_stop=0
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
                rate.sleep()
        camera.release()
        cv.destroyAllWindows()

if __name__ == '__main__':
    try:
        qr_code_process=QRCodeDistanceCalculator()
        qr_code_process.main()
    except rospy.ROSInterruptException:
        pass
