#!/usr/bin/env python3
import sys
import time

import numpy as np
import rospy
import cv2
import matplotlib.pyplot as plt
from robocol_traction.srv import Navegacion, NavegacionResponse, GridmapPoints
from nav_msgs.msg import Path
from PIL import Image, ImageDraw
from std_msgs.msg import Float32MultiArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats


def solicitarRuta():
    navegacion = rospy.ServiceProxy('navegacion', Navegacion)
    msj = Navegacion()
    #msj.inicio = [0.0,0.0]
    #msj.destino = [2.0,2.0]
    #resp = navegacion(msj.inicio,msj.destino)
    print(msj)
    #print('rutax: "{}"'.format(resp.rutax))
    #print('rutay: "{}"'.format(resp.rutay))

    #hayRuta = True

def main():

    rospy.init_node('pruebita', anonymous=True)
    print('solicitando ruta')
    solicitarRuta()
    #s = rospy.Service('navegacion', Navegacion, ruta.navegacion)
    #print('========= Waiting for service ========')

    rospy.spin()


if __name__ == '__main__':
    main()