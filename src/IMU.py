#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np
import math

class IMU():
	def __init__(self):
		super(IMU, self).__init__()
		self.roll,self.pitch,self.yaw = 0.0,0.0,0.0
		print('Init node...')
		rospy.init_node('IMU',anonymous=True)
		rospy.Subscriber('/zed2/imu/data',Imu, self.callback_IMU)
		self.pubVel = rospy.Publisher ('/robocol/IMU_euler', Twist, queue_size=1)

	def callback_IMU(self,param):
		x,y,z,w = param.orientation.x,param.orientation.y,param.orientation.z,param.orientation.w
		self.quat_2_euler(x,y,z,w)
		# print(' r: ',self.roll,' p: ',self.pitch,' y: ',self.yaw)
		IMU_msg = Twist()
		IMU_msg.angular.x = self.roll
		IMU_msg.angular.y = self.pitch
		IMU_msg.angular.z = self.yaw
		self.pubVel.publish(IMU_msg)
		# print(' y: ',self.yaw)


	def quat_2_euler(self,x,y,z,w):
		t0 =  2.0 * (w * x + y * z)
		t1 =  1.0 - 2.0 * (x * x + y * y)
		self.roll = round(math.atan2(t0, t1),4)
		
		t2 =  2.0 * (w * y - z * x)
		t2 =  1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		self.pitch = round(math.asin(t2),4)
		
		t3 =  2.0 * (w * z + x * y)
		t4 =  1.0 - 2.0 * (y * y + z * z)
		self.yaw = round(math.atan2(t3, t4),4)
		
def main():
	try:
		imu = IMU()
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			rate.sleep()
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
	main()