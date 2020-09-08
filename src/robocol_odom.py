#!/usr/bin/env python3
import rospy
import sys
from nav_msgs.msg import Odometry

class Odom(object):
	def __init__(self):
		super(Odom, self).__init__()
		rospy.loginfo('Starting odometry Robocol node...')
		rospy.init_node('robocol_odom', anonymous=True)
		rospy.Subscriber('/sim_gazebo_pose', Odometry, self.odometry_callback)
		self.pubVel = rospy.Publisher('/robocol/odometry',Odometry, queue_size=1)

	def odometry_callback(self,param):
		x = round(param.pose.pose.position.x,2)
		y = round(param.pose.pose.position.y,2)
		z = round(param.pose.pose.position.z,2)
		# print('GLOBAL COORDINATES:')
		# print('\r x: {}, y: {}, z: {}\r'.format(x,y,z), end="\r")
		sys.stdout.write('GLOBAL COORDINATES: \r')
		print('')
		sys.stdout.write(str(x) + "% \r")
		# cat('GLOBAL COORDINATES: %s','efe')

def main():
	try:
		odom = Odom()
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			rate.sleep()
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
	main()