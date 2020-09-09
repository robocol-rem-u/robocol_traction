#!/usr/bin/env python3
import rospy,time,math
import numpy as np
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,TwistStamped

# Sim pose
class Sim_Pose(object):
	def __init__(self):
		super(Sim_Pose, self).__init__()
		# Wheel odometry
		self.wheel_x, self.wheel_y, self.wheel_z  = 0.0,0.0,0.0
		self.wheel_rx,self.wheel_ry,self.wheel_rz = 0.0,0.0,0.0
		# Gazebo "odometry"
		self.gaz_x, self.gaz_y, self.gaz_z  = 0.0,0.0,0.0
		self.gaz_rx,self.gaz_ry,self.gaz_rz = 0.0,0.0,0.0
		# Robocol odometry
		self.robocol_x, self.robocol_y, self.robocol_z  = 0.0,0.0,0.0
		self.robocol_rx,self.robocol_ry,self.robocol_rz = 0.0,0.0,0.0
		# Node init
		self.node_is_up = True
		print('Starting robocol_sim_gazebo_pose node...')
		rospy.init_node('robocol_sim_gazebo_pose')
		# Publishers
		print('Publishing in /zed2/odom (Odometry)')
		self.pub_odom = rospy.Publisher('/zed2/odom', Odometry, queue_size=1)
		print('Publishing in /robocol/odom_error (Twist)')
		self.pub_error = rospy.Publisher('/robocol/odom_error', Twist, queue_size=1)
		# Subscribers
		print('Subscribing to /gazebo/link_states (LinkStates)')
		rospy.Subscriber('/gazebo/link_states', LinkStates, self.poseCallback)
		print('Subscribing to /wheel_odom (LinkStates)')
		rospy.Subscriber('/wheel_odom', TwistStamped, self.wheel_odom_callback)
		print('Subscribing to /robocol/odom (Odometry)')
		rospy.Subscriber('/robocol/odom', Odometry, self.robocol_odom_callback)
		# On rospy shutdown
		rospy.on_shutdown(self.kill_node)

	def poseCallback(self, param):
		link_num = param.name.index('leo::base_link')
		# Publish odometry based on Gazebo link states.
		odom_msg = Odometry()
		odom_msg.pose.pose.position.x = param.pose[link_num].position.x
		odom_msg.pose.pose.position.y = param.pose[link_num].position.y
		odom_msg.pose.pose.position.z = param.pose[link_num].position.z
		odom_msg.pose.pose.orientation.y = param.pose[link_num].orientation.y
		odom_msg.pose.pose.orientation.z = param.pose[link_num].orientation.z
		odom_msg.pose.pose.orientation.x = param.pose[link_num].orientation.x
		odom_msg.pose.pose.orientation.w = param.pose[link_num].orientation.w
		# Mix odoms
		odom_msg = self.mix_gazebo_n_wheel_odom(odom_msg)
		self.pub_odom.publish(odom_msg)
		# Print rounded information on screen.
		link = param.name[link_num]
		self.gaz_x = round(odom_msg.pose.pose.position.x,3)
		self.gaz_y = round(odom_msg.pose.pose.position.y,3)
		self.gaz_z = round(odom_msg.pose.pose.position.z,3)
		qx = round(odom_msg.pose.pose.orientation.x,3)
		qy = round(odom_msg.pose.pose.orientation.y,3)
		qz = round(odom_msg.pose.pose.orientation.z,3)
		qw = round(odom_msg.pose.pose.orientation.w,3)
		self.gaz_rx,self.gaz_ry,self.gaz_rz = self.quat_2_euler(qx,qy,qz,qw)
		if self.node_is_up:
			print('  Odometry msg: link: {}   x: {} y: {} z: {}   qx: {} qy: {} qz:: {} qw: {}'.format(link,self.gaz_x,self.gaz_y,self.gaz_z,qx,qy,qz,qw), end="\r")
	
	def mix_gazebo_n_wheel_odom(self,odom_msg):
		# Mix Gazebo with wheels
		qx,qy,qz,qw = self.euler_2_quat(self.wheel_rx,self.wheel_ry,self.wheel_rz)
		a = 0.7
		odom_msg_2 = Odometry()
		odom_msg_2.pose.pose.position.x    = a*odom_msg.pose.pose.position.x    + (1-a)*self.wheel_x 
		odom_msg_2.pose.pose.position.y    = a*odom_msg.pose.pose.position.y    + (1-a)*self.wheel_y 
		odom_msg_2.pose.pose.position.z    = a*odom_msg.pose.pose.position.z    + (1-a)*self.wheel_z
		odom_msg_2.pose.pose.orientation.x = a*odom_msg.pose.pose.orientation.x + (1-a)*qx
		odom_msg_2.pose.pose.orientation.y = a*odom_msg.pose.pose.orientation.y + (1-a)*qy
		odom_msg_2.pose.pose.orientation.z = a*odom_msg.pose.pose.orientation.z + (1-a)*qz
		odom_msg_2.pose.pose.orientation.w = a*odom_msg.pose.pose.orientation.w + (1-a)*qw
		return odom_msg_2

	def wheel_odom_callback(self,param):
		# Read the wheels odometry
		self.wheel_frame_id = param.header.frame_id
		self.wheel_x = param.twist.linear.x
		self.wheel_y = param.twist.linear.y
		self.wheel_z = param.twist.linear.z
		self.wheel_rx = param.twist.angular.x
		self.wheel_ry = param.twist.angular.y
		self.wheel_rz = param.twist.angular.z

	def robocol_odom_callback(self,param):
		# Robocol odometry
		self.robocol_x  = param.pose.pose.position.x
		self.robocol_y  = param.pose.pose.position.y
		self.robocol_z  = param.pose.pose.position.z
		self.robocol_rx = param.pose.pose.orientation.x
		self.robocol_ry = param.pose.pose.orientation.y
		self.robocol_rz = param.pose.pose.orientation.z

	def publish_odometry_error(self):
		# Publish error of odometry
		error_msg = Twist()
		error_msg.linear.x  = self.wheel_x  - self.robocol_x
		error_msg.linear.y  = self.wheel_y  - self.robocol_y
		error_msg.linear.z  = self.wheel_z  - self.robocol_z
		error_msg.angular.x = self.wheel_rx - self.robocol_rx
		error_msg.angular.y = self.wheel_ry - self.robocol_ry
		error_msg.angular.z = self.wheel_rz - self.robocol_rz
		self.pub_error.publish(error_msg)

	def quat_2_euler(self,x,y,z,w):
		# Transform quaternions to Euler (roll,pitch,yaw)
		t0 =  2.0 * (w * x + y * z)
		t1 =  1.0 - 2.0 * (x * x + y * y)
		roll = round(math.atan2(t0, t1),3)
		
		t2 =  2.0 * (w * y - z * x)
		t2 =  1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		pitch = round(math.asin(t2),3)
		
		t3 =  2.0 * (w * z + x * y)
		t4 =  1.0 - 2.0 * (y * y + z * z)
		yaw = round(math.atan2(t3, t4),3)

		return [roll, pitch, yaw]

	def euler_2_quat(self,roll,pitch,yaw):
		# Transform Euler (roll,pitch,yaw) to quaternions
		qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
		qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
		qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
		qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
		return [qx, qy, qz, qw]

	def kill_node(self):
		# Action taken when node is killed.
		self.node_is_up = False
		print("\nKilling node...\n")

def main():
	try:
		sim_pose = Sim_Pose()
		rate = rospy.Rate(2)
		while not rospy.is_shutdown():
			sim_pose.publish_odometry_error()			
			rate.sleep()
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
	main()