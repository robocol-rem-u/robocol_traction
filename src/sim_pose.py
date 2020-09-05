#!/usr/bin/env python3
import rospy
import numpy as np
import time
from std_msgs.msg import String, Int32, Float32MultiArray
from geometry_msgs.msg import Twist
import math, roslaunch, time
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped,PoseWithCovariance,Pose
#from leo_control_auto.msg import Obstacle
from std_msgs.msg import Int32


class Pose(object):
	def __init__(self):
		super(Pose, self).__init__()
		print('Init node...')
		rospy.init_node('sim_gazebo_pose', anonymous=True)
		rospy.Subscriber('/gazebo/link_states', LinkStates, self.poseCallback)
		self.pub = rospy.Publisher('/sim_gazebo_pose', Odometry, queue_size=1)

	def poseCallback(self, param):
		msj = Odometry()
		msj.pose.pose.position.x = param.pose[1].position.x
		msj.pose.pose.position.y = param.pose[1].position.y
		msj.pose.pose.position.z = param.pose[1].position.z

		msj.pose.pose.orientation.x = param.pose[1].orientation.x
		msj.pose.pose.orientation.y = param.pose[1].orientation.y
		msj.pose.pose.orientation.z = param.pose[1].orientation.z
		msj.pose.pose.orientation.w = param.pose[1].orientation.w

		print('x: {}, y: {}\r'.format(round(msj.pose.pose.position.x,3),round(msj.pose.pose.position.y,3)), end="")

		self.pub.publish(msj)
		#print(param.pose[1])
		#print('')



def main():
	try:
		pose = Pose()
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():			
			rate.sleep()
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
	main()