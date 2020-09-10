#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
import threading
from std_msgs.msg import Float32MultiArray

class MapConversion(object):
	def __init__(self):
		super(MapConversion, self).__init__()
		self.opciones = True
		self.x, self.y = 0.0,0.0
		print('Starting map_conversion node...')
		rospy.init_node('map_conversion')
		# print('Subscribing to /gazebo/link_states (LinkStates)')
		# rospy.Subscriber('/robocol/pose',Twist, self.pose_callaback)
		print('Subscribing in /robocol/inicio_destino_no_conversion (Float32MultiArray)')
		rospy.Subscriber('/robocol/inicio_destino_no_conversion', Float32MultiArray, self.callback_ini_des)

		print('Publishing in /robocol/inicio_destino (Float32MultiArray)')
		self.pubCoords = rospy.Publisher('/robocol/inicio_destino',Float32MultiArray, queue_size=10)
		
		rospy.on_shutdown(self.kill)
		print('')

		x = threading.Thread(target=self.thread_function)
		x.start()

	def kill(self):
		print("\nCerrando nodo...")
		self.opciones = False
		print('Presione Enter para finalizar...')

	def callback_ini_des(self,param):
		print('Coordinate received from poses alt...')
		vector = list(param.data)
		vector[0] += self.x_moved
		vector[3] += self.x_moved
		vector[1] += self.y_moved
		vector[4] += self.y_moved
		print(vector)

		new_vector = Float32MultiArray()
		new_vector.data = vector
		self.pubCoords.publish(new_vector)

	def move_plane(self):
		print(' ')
		print(' YOU ENTER THESE COORDINATES:')
		print('  x: {} y: {}'.format(self.x,self.y))
		self.x_moved = float(self.x)
		self.y_moved = float(self.y)
		print(' THE COORDINATES FOR THE MAP ARE:')
		print('  x: {} y: {}'.format(self.x_moved,self.y_moved))

	def thread_function(self):
		while self.opciones:
			print("Choose an option:")
			print(" I: To enter initial GLOBAL coordinates respect to center of map.")
			# print(" - Para ingresar coordenadas globales (G)")
			op = input('> ')
			if op == "I":
				print(" Enter desired GLOBAL coordinates:")
				print("  Enter x coordinate:")
				self.x = input('  > ')
				print("  Enter y coordinate:")
				self.y = input('  > ')
				print('  Moving coordinates...')
				try:
					self.move_plane()
				except:
					print('Move plane FAILED, try again.')
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

def main():
	try:
		mapConversion = MapConversion()
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			# mapConversion.control()           
			rate.sleep()
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
	main()
