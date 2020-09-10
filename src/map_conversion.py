#!/usr/bin/env python
# -- coding: utf-8 --
import rospy,roslib
import threading
import numpy
from std_msgs.msg import String, Int32, Float32MultiArray, Bool, MultiArrayDimension
from rospy.numpy_msg import numpy_msg
roslib.load_manifest('rospy')

class MapConversion(object):
	def __init__(self):
		super(MapConversion, self).__init__()
		self.opciones = True
		self.x, self.y = 0.0,0.0
		print('Starting map_conversion node...')
		rospy.init_node('map_conversion')
		# print('Subscribing to /gazebo/link_states (LinkStates)')
		# rospy.Subscriber('/robocol/pose',Twist, self.pose_callaback)

		# Subscribers
		print('Subscribing in /robocol/inicio_destino_no_conversion (Float32MultiArray)')
		rospy.Subscriber('/robocol/inicio_destino_no_conversion', Float32MultiArray, self.callback_ini_des)
		# print('Subscribing in /robocol/ruta_no_corregida (numpy_nd_msg(Float32MultiArray))')
		# rospy.Subscriber('/robocol/ruta_no_corregida', numpy_nd_msg(Float32MultiArray),self.ruta_no_corregida_callback)
		# Publishers
		print('Publishing in /robocol/inicio_destino (Float32MultiArray)')
		self.pubCoords = rospy.Publisher('/robocol/inicio_destino',Float32MultiArray, queue_size=1)
		
		
		rospy.on_shutdown(self.kill)
		print('')

		x = threading.Thread(target=self.thread_function)
		x.start()

	def kill(self):
		print("\nCerrando nodo...")
		self.opciones = False
		print('Presione Enter para finalizar...')

	# def ruta_no_corregida_callback(self,param):
	# 	print(param)
	# 	# self.pubRutaCorrect = param


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
			op = raw_input('> ')
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

def _serialize_numpy(self, buff):
    """
    wrapper for factory-generated class that passes numpy module into serialize
    """
    # pass in numpy module reference to prevent import in auto-generated code
    if self.layout.dim == []:
        self.layout.dim = [ MultiArrayDimension('dim%d' %i, self.data.shape[i], self.data.shape[i]*self.data.dtype.itemsize) for i in range(len(self.data.shape))];
    self.data = self.data.reshape([1, -1])[0];
    return self.serialize_numpy(buff, numpy)

def _deserialize_numpy(self, str):
	"""
	wrapper for factory-generated class that passes numpy module into deserialize    
	"""
	# pass in numpy module reference to prevent import in auto-generated code
	self.deserialize_numpy(str, numpy)
	dims=map(lambda x:x.size, self.layout.dim)
	self.data = self.data.reshape(dims)
	return self

## Use this function to generate message instances using numpy array
## types for numerical arrays. 
## @msg_type Message class: call this functioning on the message type that you pass
## into a Publisher or Subscriber call. 
## @returns Message class
def numpy_nd_msg(msg_type):
	print('numpy_nd_msg')
	classdict = { '__slots__': msg_type.__slots__, '_slot_types': msg_type._slot_types,
				'_md5sum': msg_type._md5sum, '_type': msg_type._type,
				'_has_header': msg_type._has_header, '_full_text': msg_type._full_text,
				'serialize': _serialize_numpy, 'deserialize': _deserialize_numpy,
				'serialize_numpy': msg_type.serialize_numpy,
				'deserialize_numpy': msg_type.deserialize_numpy
}
	# create the numpy message type
	msg_type_name = "Numpy_%s"%msg_type._type.replace('/', '__')
	return type(msg_type_name,(msg_type,),classdict)

def ruta_no_corregida_callback(param):
	global mapConversion,pubRutaCorrect
	print(' ')
	nueva_ruta = []
	ruta = param.data
	for i in ruta:
		vec = [i[0]-mapConversion.x_moved,i[1]-mapConversion.y_moved]
		nueva_ruta.append(vec)
		print(i)
	# nueva_ruta	
 # 	a = numpy.array(nueva_ruta, dtype=numpy.float32)
	# print("sending\n", a)
	# pubRutaCorrect.publish(data=a)
	print('Ruta')
	print(ruta)
	print('Nueva ruta')
	print(nueva_ruta)

def main():
	global mapConversion,pubRutaCorrect
	try:
		mapConversion = MapConversion()
		print('Subscribing in /robocol/ruta_no_corregida (numpy_nd_msg(Float32MultiArray))')
		rospy.Subscriber("/robocol/ruta_no_corregida", numpy_nd_msg(Float32MultiArray),ruta_no_corregida_callback)
		print('Publishing in /robocol/ruta (numpy_nd_msg(Float32MultiArray))')
		pubRutaCorrect = rospy.Publisher('/robocol/ruta', numpy_nd_msg(Float32MultiArray), queue_size=1)
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
