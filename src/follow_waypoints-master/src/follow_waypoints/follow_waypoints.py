#!/usr/bin/env python

import threading
import rospy
import actionlib
from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray ,PointStamped
from std_msgs.msg import Empty, String
from tf import TransformListener
import tf
import math
import rospkg
import csv
import time
from geometry_msgs.msg import PoseStamped

# change Pose to the correct frame 
def changePose(waypoint,target_frame):
    if waypoint.header.frame_id == target_frame:
        # already in correct frame
        return waypoint
    if not hasattr(changePose, 'listener'):
        changePose.listener = tf.TransformListener()
    tmp = PoseStamped()
    tmp.header.frame_id = waypoint.header.frame_id
    tmp.pose = waypoint.pose.pose
    try:
        changePose.listener.waitForTransform(
            target_frame, tmp.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
        pose = changePose.listener.transformPose(target_frame, tmp)
        ret = PoseWithCovarianceStamped()
        ret.header.frame_id = target_frame
        ret.pose.pose = pose.pose
        return ret
    except:
        rospy.loginfo("CAN'T TRANSFORM POSE TO {} FRAME".format(target_frame))
        exit()


#Path for saving and retreiving the pose.csv file 
output_file_path = rospkg.RosPack().get_path('follow_waypoints')+"/saved_path/pose.csv"

def convert_PoseWithCovArray_to_PoseArray(waypoints):
    """Used to publish waypoints as pose array so that you can see them in rviz, etc."""
    poses = PoseArray()
    poses.header.frame_id = rospy.get_param('~goal_frame_id','map')
    poses.poses = [pose.pose.pose for pose in waypoints]
    return poses


class FollowPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints','path_collision'])
        global stop_signal
        global cancel_flag
        self.frame_id = rospy.get_param('~goal_frame_id','map')
        self.odom_frame_id = rospy.get_param('~odom_frame_id','odom')
        self.base_frame_id = rospy.get_param('~base_frame_id','base_footprint')
        self.duration = rospy.get_param('~wait_duration', 0.0)
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')
        rospy.loginfo('Starting a tf listner.')
        self.tf = TransformListener()
        self.listener = tf.TransformListener()
        self.distance_tolerance = rospy.get_param('waypoint_distance_tolerance', 0.3)

        def wait_for_cancel_follow():
            """thread worker function"""
            global cancel_flag
            while not rospy.is_shutdown():
                data = rospy.wait_for_message('/cancel_follow', Empty)
                rospy.loginfo('CANCEL FOLLOWING')
                cancel_flag = True
        cancel_follow_thread = threading.Thread(target=wait_for_cancel_follow)
        cancel_follow_thread.start()

        def wait_for_stop_signal():
            """thread worker function"""
            global stop_signal
            while not rospy.is_shutdown():
                data = rospy.wait_for_message('/stop_signal', Empty)
                rospy.loginfo('COLLISION STOP')
                stop_signal = True
        stop_signal_thread = threading.Thread(target=wait_for_stop_signal)
        stop_signal_thread.start()
    

    def execute(self, userdata):
        global waypoints
        global path_collision
        global node_id
        global stop_signal
        stop_signal = False
        global cancel_flag
        cancel_flag = False
        # Execute waypoints each in sequence
        i = 0

        for waypoint in waypoints:
            # Break if preempted
            if (waypoints == []) or (path_collision == []) or (node_id == []) or (stop_signal == True) or (cancel_flag == True):
                rospy.loginfo('BREAK OUT OF FOLLOW PROCESS')
                break
            # Otherwise publish next waypoint as goal
            else:
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = self.frame_id
                goal.target_pose.pose.position = waypoint.pose.pose.position
                goal.target_pose.pose.orientation = waypoint.pose.pose.orientation
                self.client.send_goal(goal)
                rospy.loginfo('MOVING TO NODE: %s' %
                        (node_id[i]))
                distance = 999
                if (path_collision[i] != 'N') and (path_collision[i] != 'O'):
                    result = None
                    log = False
                    while (result == None) and (cancel_flag == False) and (stop_signal == False):
                        result = self.client.get_result()
                        if log == False:
                            rospy.loginfo('MOVING IN COLLISION NODE')
                            log = True
                        rospy.sleep(0.2)
                    i = i + 1
                    if (stop_signal == True) or (cancel_flag == True):
                        continue
                elif path_collision[i] == 'O':
                    log = False
                    while(distance > self.distance_tolerance) and (cancel_flag == False):
                        now = rospy.Time.now()
                        self.listener.waitForTransform(self.frame_id, self.base_frame_id, now, rospy.Duration(0.5))
                        trans,rot = self.listener.lookupTransform(self.frame_id,self.base_frame_id, now)
                        distance = math.sqrt(pow(waypoint.pose.pose.position.x-trans[0],2)+pow(waypoint.pose.pose.position.y-trans[1],2))
                        if log == False:
                            rospy.loginfo('MOVING OUT OF COLLISION ZONE')
                            log = True
                    i = i + 1
                elif path_collision[i] == 'N':
                    #This is the loop which exist when the robot is near a certain GOAL point.
                    
                    while(distance > self.distance_tolerance) and (cancel_flag == False):
                        now = rospy.Time.now()
                        self.listener.waitForTransform(self.frame_id, self.base_frame_id, now, rospy.Duration(0.5))
                        trans,rot = self.listener.lookupTransform(self.frame_id,self.base_frame_id, now)
                        distance = math.sqrt(pow(waypoint.pose.pose.position.x-trans[0],2)+pow(waypoint.pose.pose.position.y-trans[1],2))
                    i = i + 1
                rospy.sleep(1.2)
        if cancel_flag == False:
            self.client.wait_for_result()
            return 'success'
        else:
            return 'success'

############################################################################################################################

class GetPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints','path_collision'], output_keys=['waypoints','path_collision'])
        # Create publsher to publish waypoints as pose array so that you can see them in rviz, etc.
        self.posearray_topic = rospy.get_param('~posearray_topic','/waypoints')
        self.poseArray_publisher = rospy.Publisher(self.posearray_topic, PoseArray, queue_size=1)

        # Start thread to listen for reset messages to clear the waypoint queue
        # def wait_for_path_reset():
        #     """thread worker function"""
        #     global waypoints
        #     global path_collision
        #     global node_id
        #     while not rospy.is_shutdown():
        #         data = rospy.wait_for_message('/path_reset', Empty)
        #         rospy.loginfo('Recieved path RESET message')
        #         self.initialize_path_queue()
        # reset_thread = threading.Thread(target=wait_for_path_reset)
        # reset_thread.start()
        def wait_for_start_journey():
            """thread worker function"""
            while not rospy.is_shutdown():
                data_from_start_journey = rospy.wait_for_message('/start_journey', Empty)
                rospy.loginfo('Recieved path READY start_journey')
                with open(output_file_path, 'r') as file:
                    reader = csv.reader(file, delimiter = ',')
                    for row in reader:
                        print (row)
                        current_pose = PoseWithCovarianceStamped() 
                        current_pose.pose.pose.position.x     =    float(row[0])
                        current_pose.pose.pose.position.y     =    float(row[1])
                        current_pose.pose.pose.position.z     =    float(row[2])
                        current_pose.pose.pose.orientation.x = float(row[3])
                        current_pose.pose.pose.orientation.y = float(row[4])
                        current_pose.pose.pose.orientation.z = float(row[5])
                        current_pose.pose.pose.orientation.w = float(row[6])
                        waypoints.append(current_pose)
                        path_collision.append(str(row[7]))
                        node_id.append(str(row[8]))
                        self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))
                self.start_journey_bool = True
                
                
        start_journey_thread = threading.Thread(target=wait_for_start_journey)
        start_journey_thread.start()

    def initialize_path_queue(self):
        global waypoints
        waypoints = [] # the waypoint queue
        global path_collision
        path_collision = []
        global node_id
        node_id = []
        # publish empty waypoint queue as pose array so that you can see them the change in rviz, etc.
        self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

        # Start thread to listen start_jorney 
        # for loading the saved poses from follow_waypoints/saved_path/poses.csv

    def execute(self, userdata):
        global waypoints
        global path_collision
        global node_id
        self.initialize_path_queue()
        self.path_ready = False
        global cancel_flag
        cancel_flag = False

        self.start_journey_bool = False

        # Start thread to listen for when the path is ready (this function will end then)
        # Also will save the clicked path to pose.csv file


        while (not self.start_journey_bool):
            continue

        # Path is ready! return success and move on to the next state (FOLLOW_PATH)
        return 'success'


############################################################################################################################

# class PathComplete(State):
#     def __init__(self):
#         State.__init__(self, outcomes=['success'])

#     def execute(self, userdata):
#         rospy.loginfo('###############################')
#         rospy.loginfo('##### REACHED FINISH GATE #####')
#         rospy.loginfo('###############################')
#         return 'success'

############################################################################################################################

def main():
    rospy.init_node('follow_waypoints')

    sm = StateMachine(outcomes=['success'])

    with sm:
        StateMachine.add('GET_PATH', GetPath(),
                           transitions={'success':'FOLLOW_PATH'},
                           remapping={'waypoints':'waypoints','path_collision':'path_collision','cancel_flag':'cancel_flag','node_id':'node_id'})
        StateMachine.add('FOLLOW_PATH', FollowPath(),
                           transitions={'success':'GET_PATH'},
                           remapping={'waypoints':'waypoints','path_collision':'path_collision','cancel_flag':'cancel_flag','node_id':'node_id'})
        # StateMachine.add('PATH_COMPLETE', PathComplete(),
        #                    transitions={'success':'GET_PATH'})

    outcome = sm.execute()
