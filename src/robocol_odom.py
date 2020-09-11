#!/usr/bin/env python3
import rospy
import sys,math,time
import threading
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class Odom(object):
	def __init__(self):
		super(Odom, self).__init__()
		self.opciones = True
		# Local coordinates
		# self.loc_x, self.loc_y, self.loc_z  = 0.0, 0.0, 0.0
		# self.loc_rx,self.loc_ry,self.loc_rz = 0.0, 0.0, 0.0
		# Global coordinates
		self.glo_x, self.glo_y, self.glo_z  = 0.0, 0.0, 0.0
		self.glo_rx,self.glo_ry,self.glo_rz = 0.0, 0.0, 0.0
		# Our global coordinates
		self.our_glo_x, self.our_glo_y, self.our_glo_z  = 0.0, 0.0, 0.0
		self.our_glo_rx,self.our_glo_ry,self.our_glo_rz = 0.0, 0.0, 0.0
		# Global coordinates correction
		self.corr_glo_x, self.corr_glo_y, self.corr_glo_z  = 0.0, 0.0, 0.0
		self.corr_glo_rx,self.corr_glo_ry,self.corr_glo_rz = 0.0, 0.0, 0.0
		# Global coordinates initial corrections
		self.ini_glo_x, self.ini_glo_y, self.ini_glo_z  = 0.0, 0.0, 0.0
		self.ini_glo_rx,self.ini_glo_ry,self.ini_glo_rz = 0.0, 0.0, 0.0
		# Node init
		print('Starting robocol_odom node...')
		rospy.init_node('robocol_odom', anonymous=True)
		# Publishers
		print('Publishing in /robocol/odom (Odometry)')
		self.pubOdom = rospy.Publisher('/robocol/odom',Odometry, queue_size=1)
		print('Publishing in /robocol/pose (Twist)')
		self.pubPose = rospy.Publisher('/robocol/pose',Twist, queue_size=1)
		# Subscribers
		print('Subscribing to /zed2/imu/data (Imu)')
		rospy.Subscriber('/zed2/imu/data', Imu, self.imu_callback)
		print('Subscribing to /zed2/odom (Odometry)')
		rospy.Subscriber('/zed2/odom', Odometry, self.odometry_callback)
		print('Subscribing to /robocol/vision_correction (Twist)')
		rospy.Subscriber('/robocol/vision_correction', Twist, self.vision_correction_callback)

		time.sleep(3)
		rospy.on_shutdown(self.kill)
		print('')

		x = threading.Thread(target=self.thread_function)
		x.start()

	def kill(self):
		print("\nCerrando nodo...")
		self.opciones = False
		print('Presione Enter para finalizar...')

	def vision_correction_callback(self,param):
		print('\nCorreción de visión -> x: {} y: {}\n'.format(param.linear.x,param.linear.y))
		# self.corr_glo_x  = ( param.linear.x - self.our_glo_x)
		# self.corr_glo_y  = ( param.linear.y - self.our_glo_y)
		# self.corr_glo_z  = ( param.linear.z - self.our_glo_z)
		# self.corr_glo_rx = (param.angular.x - self.our_glo_rx)
		# self.corr_glo_ry = (param.angular.y - self.our_glo_ry)
		# self.corr_glo_rz = (param.angular.z - self.our_glo_rz)

		self.corr_glo_x  = self.corr_glo_x  + ( param.linear.x - self.our_glo_x)
		self.corr_glo_y  = self.corr_glo_y  + ( param.linear.y - self.our_glo_y)
		self.corr_glo_z  = self.corr_glo_z  + ( param.linear.z - self.our_glo_z)
		# self.corr_glo_rx = self.corr_glo_rx + (param.angular.x - self.our_glo_rx)
		# self.corr_glo_ry = self.corr_glo_ry + (param.angular.y - self.our_glo_ry)
		# self.corr_glo_rz = self.corr_glo_rz + (param.angular.z - self.our_glo_rz)

		# self.corr_glo_x  = param.linear.x-self.our_glo_x
		# self.corr_glo_y  = param.linear.y-self.our_glo_y
		# self.corr_glo_z  = param.linear.z-self.our_glo_z
		# self.corr_glo_rx = param.angular.x-self.our_glo_rx
		# self.corr_glo_ry = param.angular.y-self.our_glo_ry
		# self.corr_glo_rz = param.angular.z-self.our_glo_rz



	def odometry_callback(self,param):
		self.glo_x = round(param.pose.pose.position.x,3)
		self.glo_y = round(param.pose.pose.position.y,3)
		self.glo_z = round(param.pose.pose.position.z,3)
		qx = param.pose.pose.orientation.x
		qy = param.pose.pose.orientation.y
		qz = param.pose.pose.orientation.z
		qw = param.pose.pose.orientation.w
		self.glo_rx,self.glo_ry,self.glo_rz = self.quat_2_euler(qx,qy,qz,qw)

		self.our_glo_x  = self.corr_glo_x  + self.glo_x
		self.our_glo_y  = self.corr_glo_y  + self.glo_y
		self.our_glo_z  = self.corr_glo_z  + self.glo_z
		self.our_glo_rx = self.corr_glo_rx + self.glo_rx
		self.our_glo_ry = self.corr_glo_ry + self.glo_ry
		self.our_glo_rz = self.corr_glo_rz + self.glo_rz
		# print('GLOBAL',self.our_glo_rz)
		if self.our_glo_rz >= np.pi:
			# print('th: ',self.our_glo_rz,"  -2pi= ",np.pi)
			self.our_glo_rz = self.our_glo_rz - 2.0*np.pi
		elif self.our_glo_rz < -np.pi:
			# print('th: ',self.our_glo_rz,"  2pi= ",np.pi)
			self.our_glo_rz = self.our_glo_rz + 2.0*np.pi

		pose_msg = Twist()
		pose_msg.linear.x = self.our_glo_x
		pose_msg.linear.y = self.our_glo_y
		pose_msg.angular.z = self.our_glo_rz
		self.pubPose.publish(pose_msg)
		# test = self.our_glo_rz
		# if self.our_glo_rz > np.pi:
		# 	self.our_glo_rz 0 - self.our_glo_rz
		# self.our_glo_rz =  self.our_glo_rz % np.pi
		# self.our_glo_rz = (self.our_glo_rz + 2.0*np.pi) % 2.0*np.pi
		# if (self.our_glo_rz  > np.pi):
		#     self.our_glo_rz -= 2.0*np.pi

		g_coord = 'GLOBAL x {:.3f} y {:.3f} z {:.3f} rx {:.3f} ry {:.3f} rz {:.3f}'
		# l_coord = ' LOCAL x {:.3f} y {:.3f} z {:.3f} rx {:.3f} ry {:.3f} rz {:.3f}'
		msg = '	'+g_coord+'\r'
		print(msg.format(self.our_glo_x,self.our_glo_y,self.our_glo_z,self.our_glo_rx,self.our_glo_ry,self.our_glo_rz), end="")

		# self.our_glo_x  = self.ini_glo_x  + self.corr_glo_x  + self.glo_x
		# self.our_glo_y  = self.ini_glo_y  + self.corr_glo_y  + self.glo_y
		# self.our_glo_z  = self.ini_glo_z  + self.corr_glo_z  + self.glo_z
		# self.our_glo_rx = self.ini_glo_rx + self.corr_glo_rx + self.glo_rx
		# self.our_glo_ry = self.ini_glo_ry + self.corr_glo_ry + self.glo_ry
		# self.our_glo_rz = self.ini_glo_rz + self.corr_glo_rz + self.glo_rz


		odom_msg = Odometry()
		odom_msg.pose.pose.position.x    = self.our_glo_x
		odom_msg.pose.pose.position.y    = self.our_glo_y
		odom_msg.pose.pose.position.z    = self.our_glo_z
		odom_msg.pose.pose.orientation.x = self.our_glo_rx
		odom_msg.pose.pose.orientation.y = self.our_glo_ry
		odom_msg.pose.pose.orientation.z = self.our_glo_rz
		self.pubOdom.publish(odom_msg)

	def imu_callback(self,param):
		# Orientation
		x = param.orientation.x
		y = param.orientation.y
		z = param.orientation.z
		w = param.orientation.w
		self.imu_roll,self.imu_pitch,self.imu_yaw = self.quat_2_euler(x,y,z,w)
		# Angular Velocity
		self.imu_ang_vel_x = param.angular_velocity.x
		self.imu_ang_vel_y = param.angular_velocity.y
		self.imu_ang_vel_z = param.angular_velocity.z
		# Linear Velocity
		self.imu_lin_vel_x = param.linear_acceleration.x
		self.imu_lin_vel_y = param.linear_acceleration.y
		self.imu_lin_vel_z = param.linear_acceleration.z

	def reset_global_coord(self):
		self.corr_glo_x  = -self.glo_x
		self.corr_glo_y  = -self.glo_y
		self.corr_glo_z  = -self.glo_z
		self.corr_glo_rx = -self.glo_rx
		self.corr_glo_rz = -self.glo_rz
		self.corr_glo_ry = -self.glo_ry

	def thread_function(self):
		while self.opciones:
			print("Elija una opción:")
			print(" R: Para resetear coordenadas globales.")
			# print(" - Para ingresar coordenadas globales (G)")
			op = input('>')
			if op == "R":
				print(" Reseteando coordenadas locales...")
				self.reset_global_coord()
			# elif op == "G":
			# 	print(" Cambiando coordenadas globales...")
			# 	print("  Ingrese coordenada en x:")
			# 	x = input('>')
			# 	print("  Ingrese coordenada en y:")
			# 	y = input('>')
			else:
				print(' COMANDO NO RECONOCIDO.')
			print('')
		print('Closing thread...')

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

def main():
	try:
		odom = Odom()
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			# print('')
			rate.sleep()
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
	main()