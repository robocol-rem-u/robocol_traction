#!/usr/bin/env python3
import rospy
import time
import threading
from std_msgs.msg import Header
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose


class Poses_Publish(object):
	def __init__(self):
		super(Poses_Publish, self).__init__()
		
		self.opciones = True
		
		self.glo_x,self.glo_y,self.glo_z = 0.0,0.0,0.0
		self.ini_pose = [self.glo_x,self.glo_y,self.glo_z]
		self.end_pose = [self.glo_x,self.glo_y,self.glo_z]

		self.landmarks= [[7.31, 0], [7.19,7.55], [18.85,-3.59], [33.77,6.41], [13.22,-13.61],[21.01,13.21],[20.96,3.36], [20.40, -19.41], [14.77,6.89],[22.46,-10.36], [31.56, -18.81], [29.92,11.44], [32.79,-6.79], [2.04,-12.02], [7.63,13.24]]
		
		print('Starting robocol_poses node...')
		rospy.init_node('robocol_poses')
		# Publishers
		print('Publishing in /robocol/inicio_destino_no_conversion (Float32MultiArray)')
		self.pubPose = rospy.Publisher('/robocol/inicio_destino_no_conversion',Float32MultiArray,queue_size=1)
		# print('Publishing in /robocol/odom (Odometry)\n')
		# self.pubOdom = rospy.Publisher('/robocol/odom',Odometry, queue_size=1)
		# # Subscribers
		print('Subscribing in /robocol/odom (Odometry)\n')
		rospy.Subscriber('/robocol/odom',Odometry, self.odometry_callback)
		# print('Subscribing to /zed2/imu/data (Imu)')
		# rospy.Subscriber('/zed2/imu/data', Imu, self.imu_callback)
		# print('Subscribing to /robocol/vision_correction (Twist)')
		# rospy.Subscriber('/robocol/vision_correction', Twist, self.vision_correction_callback)

		rospy.on_shutdown(self.kill)
		print('')

		x = threading.Thread(target=self.thread_function)
		x.start()

	def odometry_callback(self,param):
		self.glo_x = param.pose.pose.position.x
		self.glo_y = param.pose.pose.position.y
		self.glo_z = param.pose.pose.position.z
		self.ini_pose = [self.glo_x,self.glo_y,self.glo_z]

	def thread_function(self):
		while self.opciones:
			print("Choose an option:")
			print(" L: To choose landmark.")
			print(" C: To send new coordinate.")
			print(" R: To publish last coordinate again.")
			op = str(input(' > '))
			if op == "L":
				print(' LANDMARKS:')
				for i in range(len(self.landmarks)):
					msg = '  Landmark {:2d} -->   x: {:6.2f}   y: {:6.2f}'
					print(msg.format(i+1,self.landmarks[i][0],self.landmarks[i][1]))
				print('   Choose a landmark:')
				lm = input('   > ')
				try:
					xlm = self.landmarks[int(lm)-1][0]
					ylm = self.landmarks[int(lm)-1][1]
					print(msg.format(int(lm),xlm,ylm))
				except Exception as e:
					print('Not a number')
			elif op == "C":
				print(" Enter desired coordinates:")
				print("  Enter x coordinate:")
				self.x = input('  > ')
				print("  Enter y coordinate:")
				self.y = input('  > ')
				self.pub_coords(self.x,self.y,0.0)
			elif op == "R":
				self.pub_coords(self.x,self.y,0.0)
			else:
				print(' COMMAND NOT RECOGNIZED.')
			print('')
		print('Closing thread...')

	def kill(self):
		print("\nKilling node...")
		self.opciones = False
		print('Press Enter to end...')
		
	def pub_coords(self,x,y,z):
		ini_end = []
		end_x,end_y,end_z = float(x),float(y),float(z)
		ini_end.append(float(self.ini_pose[0]))
		ini_end.append(float(self.ini_pose[1]))
		ini_end.append(float(self.ini_pose[2]))
		ini_end.append(end_x)
		ini_end.append(end_y)
		ini_end.append(end_z)

		print(' Sending:')
		print('  Initial pose: ')
		print('   x: {}  y: {}  z: {}'.format(ini_end[0],ini_end[1],ini_end[2]))
		print('  Final pose: ')
		print('   x: {}  y: {}  z: {}'.format(end_x,end_y,end_z))
		poses_msg = Float32MultiArray()
		poses_msg.data = ini_end
		self.pubPose.publish(poses_msg)

	def pub_test(self):
		print('Printing test')
		test = Float32MultiArray()
		self.pubPose.publish(test)

def main():
	try:
		poses_Publish = Poses_Publish()

		time.sleep(1)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			# poses_Publish.pub_test()
			rate.sleep()
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
	main()
