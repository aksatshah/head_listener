#!/usr/bin/env python
import roslib; roslib.load_manifest('head_listener')
import rospy
from std_msgs.msg import String
from head_pose.msg import head_data
from geometry_msgs.msg import Twist

def converter(pub,data):
	move = Twist()
	if (data.pitch > 0.2) and (data.pitch < 1.0): #looking down
		move.linear.x = 1.0
	if (data.pitch < -0.3) and (data.pitch > -1.0): #looking up
		move.linear.x = -1.0
	if (data.yaw > 0.3) and (data.yaw < 1.0): #looking left
		move.angular.z = 1.0
	if (data.yaw < -0.3) and (data.yaw > -1.0): #looking right
		move.angular.z = -1.0
	rospy.loginfo("Time to move!")
	print(move)
	pub.publish(move)


def callback(data):
	pub = rospy.Publisher('cmd_vel',Twist)
	converter(pub,data)
	print(data)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("Head_Pose_Data", head_data, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()