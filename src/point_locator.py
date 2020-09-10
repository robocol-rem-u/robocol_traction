#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from robocol_traction.srv import GridmapPoints, GridmapPointsResponse
import matplotlib.pyplot as plt


# from PIL import Image

class Points:

    def __init__(self):
        self.ruta_imagen = './src/robocol_traction/map/sinBOrde.png'  # ruta de la imagen

        # self.dim = (int(500), int(500))  # Dimensiones del mapa

        self.point = [None, None ]  # puntos de las esquinas
        self.i = 0
        self.a = 0
        self.stop=False

    def click(self,event,x,y,flags,param):
        """
        Determina la posicion x,y de los clicks del boton izquierod del mouse y los almacena en una lista.
        ----------
        x : int
            pixel en la coordenada x de la imagen
        y : int
            pixel en la coordenada y de la imagen
        """

        if event == cv2.EVENT_LBUTTONDOWN:
            rospy.loginfo('click')
            self.point[0]=int(x)

            self.point[1]=(int(y))
            self.i += 1

            numx = 40.0 / 928.0
            numy = 30.0 / 747.0
            print ('punto x ' + str((self.point[1] - 464.5) * numx))
            print ('punto y ' + str((self.point[0] - 373.5) * numy))
        elif event==cv2.EVENT_RBUTTONDOWN:
            self.stop=True
        

    def gridmap_points(self):
        """
        Determina las esquinas del gridmap.
        ----------
        req : request
            estructura del servicio ed ROS
        """

        rospy.loginfo('Corriendo servicio gridmap_points')
        imagen = cv2.imread(self.ruta_imagen)
  

        # cv2.line(imagen, (start_x, start_y), (end_x, end_y), (255, 0, 0), 1, 1)

        dimensions = imagen.shape
        print dimensions

        # dim = (383, 412)
        # imagen = cv2.resize(imagen, dim)

        
        while True:
            cv2.imshow('gridmap', imagen) 
            # show the image
         
            cv2.waitKey(10)
            
            cv2.setMouseCallback('gridmap', self.click)
            
            if self.a!=self.i:
                cv2.rectangle(imagen, (self.point[0]-5, self.point[1]-5), (self.point[0]+5, self.point[1]+5), (0, 255, 0), 2)
                cv2.imshow('gridmap', imagen)  
                self.a=self.i
                #key = cv2.waitKey(1) & 0xFF
                #if key == ord("c"):
                 #   continue
                # if the 'c' key is pressed, break from the loop
                #elif key == ord("q"):
                 #   cv2.destroyWindow('gridmap')
                  #  break
            if self.stop==True:
                cv2.destroyWindow('gridmap')
                break



            #if self.i == 1:
             #   cv2.destroyWindow('gridmap')  # Destruye la imagen
        
       
        
        
        # show the image

        #plt.imshow(cv2.cvtColor(imagen, cv2.COLOR_BGR2RGB))

        #plt.show()


        # response = GridmapPointsResponse()
        # response.point = self.point
        # return response

def main():
    """
    main del servicio gridmap points
    """

    rospy.init_node('gridmap_points', anonymous=True)
    points = Points()                                                                                                              
    points.gridmap_points()

    # s = rospy.Service('gridmap_points', GridmapPoints, points.gridmap_points)
    # print('========= Waiting for service ========')

    rospy.spin()


if __name__ == '__main__':
    main()
