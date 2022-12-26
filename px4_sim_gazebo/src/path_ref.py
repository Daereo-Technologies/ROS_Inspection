#!/usr/bin/env python3
import rospy
import rospkg
from math import sqrt
from geometry_msgs.msg import PoseStamped

current_wp = 0
wps = [[0,0,2,0,0,0,0],
       [-8.5,3.5,2.4,0,0,0,0],
       [-8.5,-12.5,2.4,0,0,0,0],
       [-8.5,-13,3.5,0,0,-0.5,-0.85],
       [5,-20.5,3.5,0,0,-0.5,-0.85],
       [5,-23.5,3.5,0,0,-0.85,0.5],
       [-8,-19.0,3.5,0,0,-0.85,0.5],
       [-8,-19.0,3.5,0,0,0,0],
       [-8,-22.0,3.5,0,0,0,0],
       [-8,-22.0,4.5,0,0,0,0],
       [-8,-10.0,4.5,0,0,0,0],
       [-8,3.5,4.5,0,0,0,0],
       [0,3.5,4.5,0,0,0,0],
       [0,0,4.5,0,0,0,0],
       [0,0,2,0,0,0,0]]

def py_distance(point1, point2):
    return sqrt(((point1[0] - point2[0])**2)+((point1[1] - point2[1])**2)+ ((point1[2] - point2[2])**2))

def get_pose(data):
    global wps
    global current_wp
    current_pose = [round(data.pose.position.x, 2), round(data.pose.position.y, 2), round(data.pose.position.z, 2)]

    #rospy.loginfo(str(py_distance(wps[current_wp], current_pose)))
    if py_distance(wps[current_wp], current_pose) < 1 and current_wp < len(wps)-1: 
        rospy.sleep(1)
        current_wp = current_wp + 1
        rospy.loginfo("Reached goal " + str(current_wp+1))
    if current_wp == len(wps)-1:
        rospy.sleep(3)
        current_wp = 0


def ref_pub():
    rospy.init_node("ref_pub", anonymous = False)
    ref_pub = rospy.Publisher("uav/ref", PoseStamped, queue_size=100)
    pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, get_pose)
    rate = rospy.Rate(100)

    #x = wps[i][0]
    #y = wps[i][1]
    #z = wps[i][2]
    #roll = wps[i][3]
    #pitch = wps[i][4]
    #yaw = wps[i][5]
    #yaw2= wps[i][6]

    rospy.loginfo("Reference node online")
    rospy.loginfo("Publishing waypoints...")
    while not rospy.is_shutdown():
        ref_pose = PoseStamped()
        
        ref_pose.pose.position.x = wps[current_wp][0]
        ref_pose.pose.position.y = wps[current_wp][1]
        ref_pose.pose.position.z = wps[current_wp][2]

        ref_pose.pose.orientation.x = wps[current_wp][3]
        ref_pose.pose.orientation.y = wps[current_wp][4]
        ref_pose.pose.orientation.z = wps[current_wp][5]
        ref_pose.pose.orientation.w = wps[current_wp][6]

        #ref_pose.header.stamp = rospy.Time.now
        ref_pose.header.stamp.nsecs = 0
        ref_pose.header.stamp.secs = 0
        ref_pose.header.frame_id = "uav_ref"

        ref_pub.publish(ref_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        ref_pub()
    except rospy.ROSInterruptException:
        pass
