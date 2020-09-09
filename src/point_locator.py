#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from robocol_traction.srv import GridmapPoints, GridmapPointsResponse


class Points:

    def __init__(self):
        self.ruta_imagen = './src/robocol_traction/map/ERC_map.pgm'  # ruta de la imagen
        # self.dim = (int(500), int(500))  # Dimensiones del mapa
        self.point= [] # puntos de las esquinas
        self.i = 0
	

	


    def click(self, event, x, y, flags, param):
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
            self.point.append( int(x))
       
            self.point.append(int(y))
	    numx=40.0/795.0
            numy=30.0/594.0
	    print("punto x " + str ((self.point[1]-398)*numx))
	    print("punto y " + str ((self. point[0]-298)*numy))
 
           
            self.i += 1

    def gridmap_points(self):
        """
        Determina las esquinas del gridmap.
        ----------
        req : request
            estructura del servicio ed ROS
        """
        rospy.loginfo('Corriendo servicio gridmap_points')
        imagen = cv2.imread(self.ruta_imagen)
	dimensions = imagen.shape
	print(dimensions) 
        # dim = (383, 412)
        # imagen = cv2.resize(imagen, dim)
        while self.i < 1:
           
            cv2.imshow('gridmap', imagen)  # show the image

            cv2.waitKey(10)
            cv2.setMouseCallback('gridmap', self.click)
		
            if self.i == 1:
                cv2.destroyWindow('gridmap')  # Destruye la imagen

        #response = GridmapPointsResponse()
        #response.point = self.point
        #return response

 	
def main():
    """
    main del servicio gridmap points
    """
    rospy.init_node('gridmap_points', anonymous=True)
    points = Points()
    points.gridmap_points()
    # s = rospy.Service('gridmap_points', GridmapPoints, points.gridmap_points)
    print('========= Waiting for service ========')
    rospy.spin()


if __name__ == '__main__':
    main()

