#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
import numpy
import time,sys
import roslib
import math, roslaunch, time
from std_msgs.msg import String, Int32, Float32MultiArray, Bool, MultiArrayDimension
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped,PoseWithCovariance,Pose
roslib.load_manifest('rospy')


class MoveLeo(object):
    def __init__(self):
        super(MoveLeo, self).__init__()
        self.posicionActual = 0.0
        self.twistDirection = 0.0
        self.pausar=False
        self.hayRuta = False
        self.detectStation = False
        self.stationCentered = False
        self.ruta = []
        self.landmarks= [[7.31, 0], [7.19,7.55], [18.85,-3.59], [33.77,6.41], [13.22,-13.61],[21.01,13.21],[20.96,3.36], [20.40, -19.41], [14.77,6.89],[22.46,-10.36], [31.56, -18.81], [29.92,11.44], [32.79,-6.79], [2.04,-12.02], [7.63,13.24]]
        #rospy.Subscriber('/robocol/ruta', Float32MultiArray, self.setRutaCallback)
        self.x,self.y,self.theta = 0.0,0.0,0.0
        print('Init node...')
        rospy.init_node('leo_move', anonymous=True)

        rospy.Subscriber('/robocol/pose', Twist, self.setPositionCallback)
        rospy.Subscriber('/robocol/pause', Bool, self.setPausar)
        rospy.Subscriber('/robocol_vision_object_FINAL', String, self.setDetectStationCallback)
        #Crear subscriber de vision
        self.pubVel = rospy.Publisher('/cmd_vel',Twist, queue_size=1)
        self.pubPoseCorr = rospy.Publisher('/robocol/vision_correction',Twist, queue_size=1)

        rospy.on_shutdown(self.kill)
        print('')

    # def cb(self, data):
    #     print ("I heard\n",data.data)
    #     self.hayRuta = True
    #     self.ruta = data.data
    def kill(self):
        print("\nCerrando nodo...")
        self.hayRuta = False
        # sys.exit()
        print('Presione Enter para finalizar...')

    def callback_IMU(self,param):
        pass
        # self.theta = param.angular.z
        # print(param)

    def pose_callaback(self,param):
        # print('Callabck')
        self.x     = param.linear.x
        self.y     = param.linear.y
        self.theta = param.angular.z
    # def setRutaCallback(self,pruta):
    #   self.ruta=pruta.data
    #   self.hayRuta=True

    def girar(self,param):
        kp = 0.4
        ka = 0.5 + 0.3 * numpy.exp(-param)
        giro = Twist()
        w = ka * param + kp * numpy.sin(param) * numpy.cos(param)
        giro.angular.z = w
        # while (self.pausar is True):
        #   msg = Twist()
        #   msg.linear.x = 0.0
        #   msg.angular.z = 0.0
        #   self.pubVel.publish(msg)
        self.pubVel.publish(giro)

    def adelantar(self,rho, alpha):
        kp = 1 + 0.5 * numpy.exp(-rho)
        ka = 1.2 + 0.7 * numpy.exp(-alpha)

        vmax = 1.5
        msg = Twist()

        v = kp * rho * numpy.cos(alpha)
        w = ka * alpha + kp * numpy.sin(alpha) * numpy.cos(alpha)

        msg.linear.x = v
        msg.angular.z = w
        # while (self.pausar is True):
        #   msg = Twist()
        #   msg.linear.x = 0.0
        #   msg.angular.z = 0.0
        #   self.pubVel.publish(msg)
            
        self.pubVel.publish(msg)

    def pointStation(self):
        msg = Twist()
        v = 0
        if not self.stationCentered:
            w = 0.3*self.twistDirection
        else:
            w = 0
            calculatePosition()
        msg.linear.x = v
        msg.angular.z = w
        self.pubVel.publish(msg)

    def calculatePosition(self):
        msj = Twist()
        x_calc = self.landmarks[self.tagNumber][0]-self.depth*numpy.cos(self.theta)
        y_calc = self.landmarks[self.tagNumber][1]-self.depth*numpy.sin(self.theta)
        msj.linear.x = x_calc
        msj.linear.y = y_calc
        delta_x = abs(self.x-x_calc)
        delta_y = abs(self.y-y_calc)

        if delta_x > 2.5 or delta_y > 2.5:
            print('No se acepto correcion posicion(fuera de rango)')
        else:
            print('Se acepto la correcion de posicion')
            print('x: {} y: {}'.format(x_calc, y_calc))
            pubPoseCorr.publish(msj)
            print('Publicado')

        self.detectStation = False


    def setPositionCallback(self,odom):
        # pose = odom.pose.pose
        #self.posicionActual = pose
        self.x = round(odom.linear.x,3)
        self.y = round(odom.linear.y,3)
        self.theta = round(odom.angular.z,3)
        # print(' x: ',self.x,' y: ',self.y)
        #qx,qy,qz,qw = pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w
        #roll,pitch,self.theta = self.quat_2_euler(qx,qy,qz,qw)
        # print(' x: ',pose.orientation.x,' y: ',pose.orientation.y,'z: ',pose.orientation.z,'w: ',pose.orientation.w)
        # print(' roll: ',roll,' pitch: ',pitch,'yaw: ',yaw)

    def setDetectStationCallback(self, param):

        msn=param.msn[1:-1]
        msn=msn.split(',')
        Bandera=int(msn[0])

        if Bandera ==1:
            self.detectStation = True
        else:
            self.detectStation = False

        rango=int(msn[1])

        direccion=int(msn[2])
        if direccion==1:
            self.twistDirection = 1
        elif direccion == 2:
            self.twistDirection = -1
        else:
            self.twistDirection = 0
            self.stationCentered = True


        self.depth=float(msn[3])
        self.tagNumber = int(msn[4])
        #CoorMsgAA = [detectamos,rango,direccion 1 iz 2 der 3 cen,profundidad,tag ]
        #Completar

    def setPausar(self,pause):
        self.pausar=pause.data

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

                    while (alpha > 0.05 or alpha < - 0.05) and not rospy.is_shutdown() and self.hayRuta:
                        error = [xf - self.x, yf - self.y]
                        angulo = numpy.arctan2(error[1], error[0])
                        # print(' Ángulo: ',angulo)
                        # if angulo > 2.5 or angulo < -2.5:
                        # alpha = abs(angulo) * numpy.sign(theta) - theta
                        if angulo > 2.8 or angulo < -2.8:
                            b = numpy.pi - abs(angulo)
                            b = b * numpy.sign(angulo)
                            c = numpy.pi - abs(self.theta)
                            c = c * numpy.sign(self.theta)
                            alpha = c-b
                        else:
                            alpha = angulo - self.theta


                        while (self.pausar is True):
                            print('Pausado')
                            msg = Twist()
                            msg.linear.x = 0.0
                            msg.angular.z = 0.0
                            self.pubVel.publish(msg)
   
                        while (self.detectStation is True):
                           pointStation()
                        
                        self.girar(alpha)
                        print('Girando --- x: {} y: {} rho: {} theta: {} alpha: {}\r'.format(self.x,self.y,round(rho,3),round(self.theta,3),round(alpha,3)))
                        # print('a: ',alpha,'t: ',self.theta,'ang: ',angulo)

                    msg = Twist()
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.pubVel.publish(msg)
                    time.sleep(1)

                    print('------------------------------')

                    while (rho > 0.07) and not rospy.is_shutdown() and self.hayRuta:
                        error = [xf - self.x, yf - self.y]
                        angulo = numpy.arctan2(error[1], error[0])
                        rho = numpy.sqrt(numpy.power(error[0], 2) + numpy.power(error[1], 2))
                        if angulo > 2.8 or angulo < -2.8:
                            b = numpy.pi - abs(angulo)
                            b = b * numpy.sign(angulo)
                            c = numpy.pi - abs(self.theta)
                            c = c * numpy.sign(self.theta)
                            alpha = c-b
                        else:
                            alpha = angulo - self.theta

                        while (self.pausar is True):
                            print('Pausado')
                            msg = Twist()
                            msg.linear.x = 0.0
                            msg.angular.z = 0.0
                            self.pubVel.publish(msg)

                        while (self.detectStation is True):
                            pointStation()

                        # print(' rho:',rho,' angle: ', angulo,' theta: ', self.theta,' alpha: ', alpha)
                        print('Avanzando ---  rho: {} theta: {} alpha: {}\r'.format(round(rho,3),round(self.theta,3),round(alpha,3)))
                        self.adelantar(rho, alpha)
                        # print('a: ',alpha,'t: ',self.theta,'ang: ',angulo)
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.pubVel.publish(msg)
            self.hayRuta = False
            print('Terminó ruta.')

    def empezarRuta(self,ruta):
        print('Ruta aceptada')
        self.ruta = ruta
        self.hayRuta = True
        print('Ejecutando...')



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

def callbackPrueba(param):
    global moveLeo
    ruta = param.data
    print(' Ruta: ')
    print('  ',ruta)
    print('  Desea aceptar la ruta? (s/n)')
    # try:
    inp = str(raw_input('  > '))
    if inp == 's':
        print('Intentando aceptar ruta...')
        moveLeo.empezarRuta(ruta)
    else:
        print('Ruta no aceptada.')


# def setRutaCallback(self,pruta):
    #   self.ruta=pruta.data
    #   self.hayRuta=True

def main():
    global moveLeo
    try:
        moveLeo = MoveLeo()
        rospy.Subscriber("/robocol/ruta", numpy_nd_msg(Float32MultiArray), callbackPrueba)
        #rospy.Subscriber("/robocol/ruta2", String, callbackPrueba)
        print('Subscribing to /robocol/ruta')
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
