#!/usr/bin/env python
import sys
import time

import numpy
import rospy,roslib
import cv2
import matplotlib.pyplot as plt
from robocol_traction.srv import Navegacion, NavegacionResponse, GridmapPoints
from nav_msgs.msg import Path
from PIL import Image, ImageDraw
from std_msgs.msg import Float32MultiArray
from rospy.numpy_msg import numpy_msg
# from rospy_tutorials.msg import Floats
from std_msgs.msg import Float32MultiArray
roslib.load_manifest('rospy')
from std_msgs.msg import MultiArrayDimension
"""
metodos usados en la clase minheap 
"""


def compareTo(this, that):
    return this[0] >= that[0]


def parent(i):
    return int((i - 1) / 2)


"""
Clase que implementa una cola de prioridad.
Los datos ingresados son parejas de datos de la forma [prioridad, data]
"""


class MinHeap:

    # Constructor to initialize a heap
    def __init__(self):
        self.heap = []
        self.values = []

    # ordena el min heap
    def order(self, i):
        if i != 0:
            if not compareTo(self.heap[i], self.heap[parent(i)]):
                self.heap[i], self.heap[parent(i)] = self.heap[parent(i)], self.heap[i]
                self.order(parent(i))

    # Inserts a new key 'k'
    def insert(self, k, val):
        self.heap.append([k, val])
        self.order(len(self.heap) - 1)

    # Decrease value of key at index 'i' to new_val
    # It is assumed that new_val is smaller than heap[i]
    def decreaseKey(self, i, new_val):
        for j in range(len(self.heap)):
            if self.heap[j][1] == i:
                break
        self.heap[j] = [new_val, i]
        self.order(j)

    #  cambia la posicion de 2 nodos en el minheap
    def bajar(self, i, size):
        if i * 2 + 2 >= size:
            pass
        elif compareTo(self.heap[i * 2 + 1], self.heap[i * 2 + 2]):
            if not compareTo(self.heap[i * 2 + 2], self.heap[i]):
                self.heap[i], self.heap[i * 2 + 2] = self.heap[i * 2 + 2], self.heap[i]
                self.bajar(i * 2 + 2, len(self.heap))
        else:
            if not compareTo(self.heap[i * 2 + 1], self.heap[i]):
                self.heap[i], self.heap[i * 2 + 1] = self.heap[i * 2 + 1], self.heap[i]
                self.bajar(i * 2 + 1, len(self.heap))

    # Method to remove minium element from min heap
    def extractMin(self):
        self.heap[0], self.heap[len(self.heap) - 1] = self.heap[len(self.heap) - 1], self.heap[0]
        sacar = self.heap.pop()
        self.bajar(0, len(self.heap))
        return sacar

    # This functon deletes key at index i. It first reduces
    # value to minus infinite and then calls extractMin()
    def deleteKey(self, i):
        self.decreaseKey(i, float("-inf"))
        self.extractMin()

    # Get the minimum element from the heap
    def getMin(self):
        return self.heap[0]

    # Dice si el heap esta vacio
    def is_empty(self):
        return len(self.heap) == 0


class Ruta:

    def __init__(self):
        self.ruta_imagen = './src/robocol_traction/map/ERC_map.pgm'  # ruta de la imagen
        self.ruta_resultados = './src/robocol_traction/results/'
        self.ruta = []  # ruta optima
        self.inicio = None  # nodo inicio
        self.final = None  # nodo final
        self.grafo = None  # grafo de exploracion
        self.heuristica = 'm'
        self.points = []
        self.callback = False
        # Publishers
        
        # self.pub = rospy.Publisher('mytopic', numpy_nd_msg(Float32MultiArray))
        # Subscribers
        rospy.Subscriber('/robocol/inicio_destino', Float32MultiArray, self.callbackPath)

    def euclidiana(self, nodo):
        """
        Calcula la distancia euclidiana entre 2 nodos del gridmap
        Parameters
        ----------
        nodo : [Integer, Integer]
            nodo en el gridmap
        Return
        ----------
        ans: Integer
            distancia euclidiana
        """
        return ((nodo[0] - self.final[0]) ** 2 + (nodo[1] - self.final[1]) ** 2) ** (1 / 2)

    def manhattan(self, nodo):
        """
        Calcula la distancia de manhattan entre 2 nodos del gridmap
        Parameters
        ----------
        nodo : [Integer, Integer]
            nodo en el gridmap
        Return
        ----------
        ans: Integer
            distancia de manhattan
        """
        ans = abs(nodo[0] - self.final[0])
        ans += abs(nodo[1] - self.final[1])
        return float(ans)

    def A(self):
        """
        Implementa el algoritmo A*
        """
        explorados = []  # lista de nodos explorados
        anterior = {}  # diccionario donde el value es el nodo anterior
        priority_queue = MinHeap()  # cola de prioridad
        end = False  # Variable para indicar si se llego al nodo destino
        visitados = {}  # diccionario para indicar que nodos han sido visitados

        for nodo in self.grafo:  # poner todos los nodos como no visitados
            visitados[nodo] = False

        priority_queue.insert(0, [self.inicio, 0])  # agregar el nodo de inicio a la cola de prioridad

        while not priority_queue.is_empty() and not end:  # recorrer hasta llegar al destino o  visitar todos los nodos

            obj = priority_queue.extractMin()[1]  # saca el nodo con menor costo
            nodo = obj[0]
            cn_ant = obj[1]
            ad = self.grafo[nodo]
            explorados.append(nodo)

            if nodo == self.final:  # si llega al nodo destino recorre 'anterior' para retornar la ruta
                end = True
                next = nodo

                while next != self.inicio:
                    self.ruta.append(next)
                    next = anterior[next]
                self.ruta.append(next)  # para guardar el nodo inicial
            else:
                for i in ad:
                    adyacente = i[1]
                    if not visitados[adyacente]:
                        # explorados.append(adyacente)  # agregar nodo a la lista de exploracion
                        visitados[adyacente] = True
                        cn = cn_ant + 0.2  # costo de celdas  de 0.2
                        if self.heuristica == 'm':
                            costo = cn + self.manhattan(adyacente)
                        else:
                            costo = cn + self.euclidiana(adyacente)
                        priority_queue.insert(costo, [adyacente, cn])  # agrega el nodo a la cola de prioridad
                        anterior[adyacente] = nodo

        rospy.loginfo('termino ruta')
        return explorados

    def dijkstra(self):
        """
        Implementa el algoritmo de dijkstra
        """
        anterior = {}  # diccionario en el que se guardara la informacion del nodo anterior
        end = False  # variable para definir si ya termino
        priority_queue = MinHeap()  # queue
        visitados = {}  # diccionario de nodos visitados
        dist = {}  # distancia desde el nodo de inicio hasta el nodo en la key del diccionario

        explorados = []  # nodos explorados
        for nodo in self.grafo:  # inicia los nodos con costo infinito y los asigna como no visitados
            visitados[nodo] = False
            priority_queue.insert(sys.maxsize, nodo)
            dist[nodo] = sys.maxsize

        nodo = self.inicio
        priority_queue.decreaseKey(nodo, 0)
        dist[nodo] = 0  # pone el primer nodo con costo 0 y lo agrega en la cola de prioridad

        while not priority_queue.is_empty() and not end:
            nodo = priority_queue.extractMin()[1]  # saca el nodo con menor costo
            explorados.append(nodo)  # agrega el nodo a la lista de exploracion
            visitados[nodo] = True
            if nodo == self.final:  # si llega al destino recorre la lista 'anterior' para retornar la ruta
                end = True
                next = nodo
                while next != self.inicio:
                    self.ruta.append(next)
                    next = anterior[next]
                self.ruta.append(next)

            else:
                ad = self.grafo[nodo]
                for a in ad:  # recorre los nodos adyacentes
                    costo = dist[nodo] + 0.2  # se asume costo de 0.2
                    # Si el nodo no se ha visitado y el costo calculado es menor al que se tenia previamente
                    if not visitados[a[1]] and costo < dist[a[1]]:
                        anterior[a[1]] = nodo
                        dist[a[1]] = costo
                        priority_queue.decreaseKey(a[1], costo)  # actualiza el costo en la cola de prioridad
        return explorados

    def gridmap2graph(self, gridmap, w, h):
        """
        Genera un grafo a partir del gridmap_resized teniendo en cuenta la vecindad para cada celda, asumiendo que las
        acciones discretas del robot son cuatro: moverse al norte, al sur, al oriente y al este.
        El grafo sera un diccionario donde el key es la coordenada (i,j) de la celda y sus valores son las coordenadas
        de los vecinos + la accion del robot que lleva a cada uno
        Parameters
        ----------
        gridmap : Integer[Integer[]]
            Imagen que representa el gridmap con el preprocesamiento aplicado
        w : Integer
            Ancho del gridmap
        h : Alto
            Alto del gridmap
        """

        maze = numpy.where(gridmap == 255, 0, gridmap)  # celda libre
        maze = numpy.where(gridmap == 0, 1, maze)  # celda ocupada

        graph = {(i, j): [] for j in range(w) for i in range(h) if not maze[i][j]}
        for row, col in graph.keys():
            if row < h - 1 and not maze[row + 1][col]:
                graph[(row, col)].append(("S", (row + 1, col)))
                graph[(row + 1, col)].append(("N", (row, col)))
            if col < w - 1 and not maze[row][col + 1]:
                graph[(row, col)].append(("E", (row, col + 1)))
                graph[(row, col + 1)].append(("W", (row, col)))

        self.grafo = graph

    def four_point_transform(self, image, pts):
        """
        realiza la transformada de la imagen a partir de las 4 esquinas seleccionadas
        """
        rect = []
        for j in range(4):
            rect.append([pts[j * 2], pts[j * 2 + 1]])

        rect = numpy.array(rect, dtype="float32")
        (tl, tr, br, bl) = rect
        # compute the width of the new image, which will be the
        # maximum distance between bottom-right and bottom-left
        # x-coordiates or the top-right and top-left x-coordinates
        widthA = numpy.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
        widthB = numpy.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
        maxWidth = max(int(widthA), int(widthB))
        # compute the height of the new image, which will be the
        # maximum distance between the top-right and bottom-right
        # y-coordinates or the top-left and bottom-left y-coordinates
        heightA = numpy.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
        heightB = numpy.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
        maxHeight = max(int(heightA), int(heightB))
        # now that we have the dimensions of the new image, construct
        # the set of destination points to obtain a "birds eye view",
        # (i.e. top-down view) of the image, again specifying points
        # in the top-left, top-right, bottom-right, and bottom-left
        # order
        dst = numpy.array([
            [0, 0],
            [maxWidth - 1, 0],
            [maxWidth - 1, maxHeight - 1],
            [0, maxHeight - 1]], dtype="float32")
        # compute the perspective transform matrix and then apply it
        M = cv2.getPerspectiveTransform(rect, dst)
        warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
        # return the warped image
        return warped

    def get_graph(self, points):
        """
        Obtiene un grafo a partir del gridmap proporcionado, aplicando un preprocesamiento al gridmap.
        Preprocesamiento de la forma:
        1. Leer gridmap y escalar el gridmap con los puntos seleccionados en el servicio gridmap_points
        2. convertir celdas de las que no se tiene informacion a celdas ocupadas
        3. dilatar los obstaculos
        4. Reescalar la imagen
        Return
        ----------
        gridmap_resized: Integer[Integer[]]
            Imagen del gridmap con el preprocesamiento aplicado
        """

        gridmap = cv2.imread(self.ruta_imagen, -1)

        gridmap = self.four_point_transform(gridmap, points)

        gridmap[(gridmap >= 179) & (gridmap <= 238)] = 0
        gridmap[(gridmap >= 241) & (gridmap <= 255)] = 255

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
        gridmap_dilatated = cv2.dilate(cv2.bitwise_not(gridmap), kernel, iterations=1)
        gridmap_dilatated = cv2.bitwise_not(gridmap_dilatated)


        scale_percent = 25  # percent of original size
        width = int(gridmap_dilatated.shape[1] * scale_percent / 100)
        height = int(gridmap_dilatated.shape[0] * scale_percent / 100)
        dim = (width, height)
        gridmap_resized = cv2.resize(gridmap_dilatated, dim, interpolation=cv2.INTER_NEAREST)

        self.gridmap2graph(gridmap_resized, width, height)

        return gridmap_resized, width, height

    def save_ruta(self, gridmap, explorado):

        images = []
        dim = (500, 500)

        backtorgb = cv2.cvtColor(gridmap, cv2.COLOR_GRAY2RGB)  # pasa la imagen a RGB
        backtorgb[self.inicio[0], self.inicio[1]] = (0, 255, 0)  # pinta el inicio de color verde
        backtorgb[self.final[0], self.final[1]] = (255, 0, 0)  # pinta el final de color rojo

        imagen = cv2.resize(backtorgb, dim)
        im = Image.fromarray(imagen)
        images.append(im)

        for k in range(len(explorado)):  # Pinta los nodos recorridos de color azul
            if explorado[k] == self.inicio:
                backtorgb[explorado[k][0], explorado[k][1]] = (0, 255, 0)
            elif explorado[k] == self.final:
                backtorgb[explorado[k][0], explorado[k][1]] = (255, 0, 0)
            else:
                backtorgb[explorado[k][0], explorado[k][1]] = (40, 112, 255)
                imagen = cv2.resize(backtorgb, dim)
                im = Image.fromarray(imagen)
                images.append(im)

        for h in range(len(self.ruta)):  # pinta los nodos del camnino respuesta de color naranja
            if self.ruta[h] == self.inicio:
                backtorgb[self.ruta[h][0], self.ruta[h][1]] = (0, 255, 0)
            elif self.ruta[h] == self.final:
                backtorgb[self.ruta[h][0], self.ruta[h][1]] = (255, 0, 0)
            else:
                backtorgb[self.ruta[h][0], self.ruta[h][1]] = (255, 112, 40)
                imagen = cv2.resize(backtorgb, dim)
                im = Image.fromarray(imagen)
                images.append(im)

        images[len(images) - 1].save(self.ruta_resultados + 'navegacion.gif', format='GIF', append_images=images[1:],
                                     save_all=True,
                                     duration=50, loop=0)
        print('Termine el gif tututututututu')

    def vrep_to_gridmap(self, ancho, alto):

        relacionX = float(40.0 / alto)
        relacionY = float(30.0 / ancho)

        rospy.loginfo('Nodo inicio vrep "{}"'.format(self.points[0]))
        rospy.loginfo('Nodo destino vrep "{}"'.format(self.points[3]))

        rospy.loginfo('Parametros metodo ancho:"{}"'.format(ancho))
        rospy.loginfo('Parametros metodo alto:"{}"'.format(alto))

        self.inicio = (int((self.points[0] + 20.0) / relacionX), int((self.points[1] + 15.0) / relacionY))
        self.final = (int((self.points[3] + 20.0) / relacionX), int((self.points[4] + 15.0) / relacionY))
        rospy.loginfo('Nodo inicio "{}"'.format(self.inicio))
        rospy.loginfo('Nodo destino "{}"'.format(self.final))

    def gridmap_to_vrep(self, ancho, alto):
        relacionX = 40.0 / alto
        relacionY = 30.0 / ancho
        direccion = None
        for nodo in self.ruta:
            if nodo == self.final:
                x = []
                y = []
            else:
                if nodo[0] == anterior[0]:
                    if direccion != 'x':
                        x.append(anterior[0] * relacionX - 20.0)
                        y.append(anterior[1] * relacionY - 15.0)
                    direccion = 'x'
                elif nodo[1] == anterior[1]:
                    if direccion != 'y':
                        x.append(anterior[0] * relacionX - 20.0)
                        y.append(anterior[1] * relacionY - 15.0)
                    direccion = 'y'
            anterior = nodo
        x.append(self.inicio[0] * relacionX - 20.0)
        y.append(self.inicio[1] * relacionY - 15.0)

        x.reverse()
        y.reverse()

        return x, y


    def buscarCercano(self, punto):
        #print(self.grafo.values()[0])
        #print(self.grafo.keys()[0])

        mini=10000
        nuevo=[]
        for i in self.grafo:
           #print('i: ',i)
           #print('Punto0: ',punto[0], 'i0,', i[0])
           # dist=numpy.sqrt(numpy.power(punto[0] - i[0],2) + numpy.power(punto[1] - i[1],2))
           #dist=((punto[0] - i[0])**2 + (punto[1] - i[1])**2)**(1/2)
           dist=((punto[0] - i[0])**2.0 + (punto[1] - i[1])**2.0)**(1.0/2.0) 
           #print(dist)
           if dist<mini:
               nuevo=i
               mini=dist
        return(nuevo)

    def removeLandmarks(self, landmarks, grafo,ancho,alto):
        listica = []
        for i in landmarks:
            puntos=self.landmarksMat(i,ancho,alto)
            
            #print(puntos)
            for j in grafo:
                if self.inSquare(j,puntos) is True:
                    listica.append(j)
        for i in listica:
            grafo.pop(i)
        self.grafo = grafo

    def inSquare(self,punto,puntos):
        
        x1=puntos[0][0]
        y1=puntos[0][1]
        x2=puntos[1][0]
        y2=puntos[1][1]

        if punto[0]>=x1 and punto[0]<=x2:
            if punto[1]>=y1 and punto[1]<=y2:
                return True
        return False
        
    def landmarksMat(self, punto,ancho,alto):
        lis= []
        lis.append([punto[0]-0.073,punto[1]-0.073])
        lis.append([punto[0]+0.073,punto[1]+0.073])
        lisFinal= []
        relacionX = float(40.0 / alto)
        relacionY = float(30.0 / ancho)
        for i in lis:
            inicio = (int((i[0]+ 20.0) / relacionX), int((i[1] + 15.0) / relacionY))
            lisFinal.append(inicio)
        return lisFinal


    def navegacion(self):
        """
        Determina la ruta optima.
        ----------
        req : request
            estructura del servicio ed ROS
        """

        #gridmap_points = rospy.ServiceProxy('gridmap_points', GridmapPoints)
        gridmap_points = [9, 51, 615, 53, 614, 854, 7, 850]
        landmarks= [[7.31, 0], [7.19,7.55], [18.85,-3.59], [33.77,6.41], [13.22,-13.61],[21.01,13.21],[20.96,3.36], [20.40, -19.41], [14.77,6.89],[22.46,-10.36], [31.56, -18.81], [29.92,11.44], [32.79,-6.79], [2.04,-12.02], [7.63,13.24]]

        #resp = gridmap_points()
        rospy.loginfo('Corriendo servicio gridmap points"{}"'.format(gridmap_points))

        rospy.loginfo('Corriendo servicio navegacion')
        gridmap_preprocesado, w, h = self.get_graph(gridmap_points)
        print('Gridmap Preprocesado')

        self.vrep_to_gridmap(w, h)

        self.ruta = []
        #print(self.inicio)
        #print(self.final)
        print('Quitandito')
        self.removeLandmarks(landmarks, self.grafo,w,h)

        if (self.inicio not in self.grafo):
            new= self.buscarCercano(self.inicio)
            self.inicio=new
        if (self.final not in self.grafo):
            new= self.buscarCercano(self.final)
            print("invalido")
            self.final=new


        explorados = self.A()
        # if req.metodo == 'A':
        #     explorados = self.A()
        # elif req.metodo == 'd':
        #     explorados = self.dijkstra()
        # else:
        #     rospy.logerr('Metodo desconocido "{}"'.format(req.metodo))
        #     return None

        self.save_ruta(gridmap_preprocesado, explorados)

        response = NavegacionResponse()
        response.rutax, response.rutay = self.gridmap_to_vrep(w, h)


        print('aqui')
        print(response)
   
        ans=[]
        for i in range(0,len(response.rutax)):
            act= []
            act.append(response.rutax[i])
            act.append(response.rutay[i])
            ans.append(act)
        
        

        #print('filas: ',len(ans),' columnas: ', len(ans[0]), ' ans: ', ans)
        #a = numpy.array([1.0, 2.1, 3.2, 4.3, 5.4, 6.5], dtype=numpy.float32)
        # msg=Float32MultiArray()
        # print('antes', msg)
        # msg.data=ans
        # print('despues',msg)
        # self.pubRuta.publish(msg)

        return ans



    
    

        # create the numpy message type
        msg_type_name = "Numpy_%s"%msg_type._type.replace('/', '__')
        return type(msg_type_name,(msg_type,),classdict)

    def callbackPath(self, param):
        #print('aaaa')
        self.points = param.data
        #print('callbackPath')
        self.callback = True
        # self.navegacion()
            
        #print(param.poses[0].pose)

## Use this function to generate message instances using numpy array
## types for numerical arrays. 
## @msg_type Message class: call this functioning on the message type that you pass
## into a Publisher or Subscriber call. 
## @returns Message class
# def numpy_nd_msg(msg_type):
#     classdict = { '__slots__': msg_type.__slots__, '_slot_types': msg_type._slot_types,
#                   '_md5sum': msg_type._md5sum, '_type': msg_type._type,
#                   '_has_header': msg_type._has_header, '_full_text': msg_type._full_text,
#                   'serialize': _serialize_numpy, 'deserialize': _deserialize_numpy,
#                   'serialize_numpy': msg_type.serialize_numpy,
#                   'deserialize_numpy': msg_type.deserialize_numpy
#                   }

# def _serialize_numpy(self, buff):
#     """
#     wrapper for factory-generated class that passes numpy module into serialize
#     """
#     # pass in numpy module reference to prevent import in auto-generated code
#     if self.layout.dim == []:
#         self.layout.dim = [ MultiArrayDimension('dim%d' %i, self.data.shape[i], self.data.shape[i]*self.data.dtype.itemsize) for i in range(len(self.data.shape))];
#     self.data = self.data.reshape([1, -1])[0];
#     return self.serialize_numpy(buff, numpy)

# def _deserialize_numpy(self, str):
#     """
#     wrapper for factory-generated class that passes numpy module into deserialize    
#     """
#     # pass in numpy module reference to prevent import in auto-generated code
#     self.deserialize_numpy(str, numpy)
#     dims=map(lambda x:x.size, self.layout.dim)
#     self.data = self.data.reshape(dims)
#     return self     

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

## Use this function to generate message instances using numpy array types for numerical arrays. 
## @msg_type Message class: call this functioning on the message type that you pass
## into a Publisher or Subscriber call. 
## @returns Message class
def numpy_nd_msg(msg_type):
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


def main():
    rospy.init_node('navegacion', anonymous=True)
    ruta = Ruta()
    rate = rospy.Rate(10)
    pub = rospy.Publisher('/robocol/ruta', numpy_nd_msg(Float32MultiArray), queue_size=1)
    print('Waiting')
    while not rospy.is_shutdown():
        if ruta.callback == True:
            ans = ruta.navegacion()
            a = numpy.array(ans, dtype=numpy.float32)
            print("sending\n", a)
            pub.publish(data=a)
            ruta.callback = False
        rate.sleep()

# def main():
#     """
#     main del servicio navegacion
#     """
#     rospy.init_node('navegacion', anonymous=True)

#     ruta = Ruta()
#     s = rospy.Service('navegacion', Navegacion, ruta.navegacion)
#     print('========= Waiting for service ========')
#     rospy.spin()


if __name__ == '__main__':
    main()
