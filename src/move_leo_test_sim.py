#!/usr/bin/env python3
import rospy
import numpy as np
import time
from std_msgs.msg import String, Int32, Float32MultiArray
from geometry_msgs.msg import Twist
import math, roslaunch, time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped,PoseWithCovariance,Pose
#from leo_control_auto.msg import Obstacle
from std_msgs.msg import Int32


class MoveLeo(object):
	def __init__(self):
		super(MoveLeo, self).__init__()
		self.posicionActual = 0.0
		self.pausar=False
		self.hayRuta = False
		self.ruta = []
		rospy.Subscriber('/robocol/ruta', Float32MultiArray, self.setRutaCallback)
		self.x,self.y,self.theta = 0.0,0.0,0.0
		print('Init node...')
		rospy.init_node('leo_move', anonymous=True)
		rospy.Subscriber('/sim_gazebo_pose', Odometry, self.setPositionCallback)
		rospy.Subscriber('/robocol/pause', bool, self.setPausar)
		self.pubVel = rospy.Publisher('/cmd_vel',Twist, queue_size=10)

	def callback_IMU(self,param):
		self.theta = param.angular.z
		# print(param)
	def setRutaCallback(self,pruta):
		self.ruta=pruta.data
		self.hayRuta=True

	def girar(self,param):
		kp = 0.4
		ka = 0.5 + 0.3 * np.exp(-param)
		giro = Twist()
		w = ka * param + kp * np.sin(param) * np.cos(param)
		giro.angular.z = w
		while (self.pausar is True):
			msg = Twist()
			msg.linear.x = 0.0
			msg.angular.z = 0.0
			self.pubVel.publish(msg)
			
		self.pubVel.publish(giro)

	def adelantar(self,rho, alpha):
		kp = 1 + 0.5 * np.exp(-rho)
		ka = 1.5 + 0.5 * np.exp(-alpha)

		vmax = 1.5
		msg = Twist()

		v = kp * rho * np.cos(alpha)
		w = ka * alpha + kp * np.sin(alpha) * np.cos(alpha)

		msg.linear.x = v
		msg.angular.z = w
		while (self.pausar is True):
			msg = Twist()
			msg.linear.x = 0.0
			msg.angular.z = 0.0
			self.pubVel.publish(msg)
			
		self.pubVel.publish(msg)

	def setPositionCallback(self,odom):
		pose = odom.pose.pose
		self.posicionActual = pose
		self.x = round(pose.position.x,4)
		self.y = round(pose.position.y,4)
		# print(' x: ',self.x,' y: ',self.y)
		qx,qy,qz,qw = pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w
		roll,pitch,self.theta = self.quat_2_euler(qx,qy,qz,qw)
		# print(' x: ',pose.orientation.x,' y: ',pose.orientation.y,'z: ',pose.orientation.z,'w: ',pose.orientation.w)
		# print(' roll: ',roll,' pitch: ',pitch,'yaw: ',yaw)

	def setPausar(self,pause):
		self.pausar=pause

	def quat_2_euler(self,x,y,z,w):
		t0 =  2.0 * (w * x + y * z)
		t1 =  1.0 - 2.0 * (x * x + y * y)
		roll = math.atan2(t0, t1)
		
		t2 =  2.0 * (w * y - z * x)
		t2 =  1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		pitch = math.asin(t2)
		
		t3 =  2.0 * (w * z + x * y)
		t4 =  1.0 - 2.0 * (y * y + z * z)
		yaw = math.atan2(t3, t4)
		
		return [roll, pitch, yaw]

	def control(self):
		if self.hayRuta:
			print('Empezó ruta.')
			print(' Ruta: ',self.ruta)
			for i in range(len(self.ruta)):
				rotate,advance = False,False
				print('  # de Ruta ',i,':')
				if i != 0:
					print(i)
					# xf = self.ruta.rutax[i]
					# yf = self.ruta.rutay[i]
					xf = self.ruta[i][0]#.rutax[i]
					yf = self.ruta[i][1]#.rutay[i]
					rho = 100
					alpha = 100

					while alpha > 0.05 or alpha < - 0.05 and not rospy.is_shutdown():
						error = [xf - self.x, yf - self.y]
						angulo = np.arctan2(error[1], error[0])
						# print(' Ángulo: ',angulo)
						# if angulo > 2.5 or angulo < -2.5:
						# alpha = abs(angulo) * np.sign(theta) - theta
						if angulo > 2.8 or angulo < -2.8:
							b = np.pi - abs(angulo)
							b = b * np.sign(angulo)
							c = np.pi - abs(self.theta)
							c = c * np.sign(self.theta)
							alpha = c-b
						else:
							alpha = angulo - self.theta
						self.girar(alpha)
						print('Girando --- rho: {} alpha: {}\r'.format(round(rho,3),round(alpha,3)), end="")
						# print('a: ',alpha,'t: ',self.theta,'ang: ',angulo)

					msg = Twist()
					msg.linear.x = 0.0
					msg.angular.z = 0.0
					
	
						
					self.pubVel.publish(msg)
					time.sleep(1)

					print('------------------------------')

					while rho > 0.04 and not rospy.is_shutdown():
						# print('advance')

						error = [xf - self.x, yf - self.y]
						angulo = np.arctan2(error[1], error[0])
						rho = np.sqrt(np.power(error[0], 2) + np.power(error[1], 2))
						if angulo > 2.8 or angulo < -2.8:
							b = np.pi - abs(angulo)
							b = b * np.sign(angulo)
							c = np.pi - abs(self.theta)
							c = c * np.sign(self.theta)
							alpha = c-b
						else:
							alpha = angulo - self.theta
						# print(' rho:',rho,' angle: ', angulo,' theta: ', self.theta,' alpha: ', alpha)
						print('Avanzando --- rho: {} alpha: {}\r'.format(round(rho,3),round(alpha,3)), end="")
						self.adelantar(rho, alpha)
						# print('a: ',alpha,'t: ',self.theta,'ang: ',angulo)
			msg = Twist()
			msg.linear.x = 0.0
			msg.angular.z = 0.0
			self.pubVel.publish(msg)
			self.hayRuta = False
			print('Terminó ruta.')


def main():
	try:
		moveLeo = MoveLeo()
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			moveLeo.control()			
			rate.sleep()
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
	main()