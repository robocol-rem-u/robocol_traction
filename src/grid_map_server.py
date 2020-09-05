#!/usr/bin/env python
import rospy
import cv2
from robocol_traction.srv import GridmapPoints, GridmapPointsResponse


class Points:

    def __init__(self):
        self.ruta_imagen = './src/robocol_traction/map/ERC_map.pgm'  # ruta de la imagen
        # self.dim = (int(500), int(500))  # Dimensiones del mapa
        self.points = [None, None, None, None, None, None, None, None]  # puntos de las esquinas
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
            self.points[self.i*2] = int(x)
            self.points[self.i*2+1] = int(y)
            self.i += 1

    def gridmap_points(self, req):
        """
        Determina las esquinas del gridmap.
        ----------
        req : request
            estructura del servicio ed ROS
        """
        rospy.loginfo('Corriendo servicio gridmap_points')
        imagen = cv2.imread(self.ruta_imagen)
        # dim = (383, 412)
        # imagen = cv2.resize(imagen, dim)
        while self.i < 4:
            rospy.loginfo('i "{}"' .format(self.i))
            cv2.imshow('gridmap', imagen)  # show the image
            cv2.waitKey(10)
            cv2.setMouseCallback('gridmap', self.click)

            if self.i == 4:
                cv2.destroyWindow('gridmap')  # Destruye la imagen
                
        response = GridmapPointsResponse()
        response.points = self.points
        return response


def main():
    """
    main del servicio gridmap points
    """
    rospy.init_node('gridmap_points', anonymous=True)
    points = Points()
    s = rospy.Service('gridmap_points', GridmapPoints, points.gridmap_points)
    print('========= Waiting for service ========')
    rospy.spin()


if __name__ == '__main__':
    main()
