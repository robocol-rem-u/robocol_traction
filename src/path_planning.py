#!/usr/bin/env python3
# Se importan las librerias necesarias junto con los mensajes a utilizar
import rospy, math, roslaunch, time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped,PoseWithCovariance,Pose
import networkx as nx
#from leo_control_auto.msg import Obstacle
from std_msgs.msg import Int32
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray

# Clase que representa una casilla, tiene ubicacion o punto que la define (mitad) y si un objeto la cubre o no.
class Casilla:
    def __init__(self, xP, yP, pE):
        self.x = float(xP)
        self.y = float(yP)
        self.libre = pE

# Clase que hace referencia a una posicion en especifico.
class Posicion:
    def __init__(self, xP, yP, pTeta):
        self.x=float(xP)
        self.y=float(yP)
        self.teta=pTeta

# Iniciar grafico de networkx
g = nx.Graph()
# Distancia entre centro de cuadriculas, 10 debe ser divisible por esta distancia
distanciaCuadricula =  0.8  # .8 #1
# Numero de cuadriculas de la escena seleccionada
n = int(25/distanciaCuadricula)
# Arreglo con la informacion de cada una de las casillas
casillas = []
# Posicion actual del robot, debe actualizarse por el topico
posicionActual = Pose()
# Posicion final del robot, inicialmente se toma como la cuadricula superior derecha con angulo 0
posicionFinal = Pose()

# Numero de obstaculos
nObstacles = 0
# Arreglo con obstaculos
obstacles = []

# Senal para saber si ya se ejecuto
startRecieved = False
endRecieved=False

# En este metodo se inicializa el nodo, se suscribe a los topicos necesarios, se crea la variable para publicar al
# topico de motorsVel y tambien se lanza el nodo encargado de graficar. Ademas es el metodo encargado de realizar
# las acciones de control necesarias segun la ruta dada para llevar el robot a la posicion final.

def route():
    global posicionActual, g, ruta, pubMot, arrivedP, p, umbralP, kp, kb, ka, empezar, pedal, obstacles,startRecieved,endRecieved,casillas
    # Se crean el nodo
    print('Initialiazing node...')
    rospy.init_node('path_node', anonymous=True)
    # Se suscribe a al topico de la informacion de la posicion del pioneer
    rospy.Subscriber ('/controllers/diff_drive/odom', Odometry, setInitialCallback)
    rospy.Subscriber('/robocol/posicionFinal', Pose, setFinalCallBack)
    pubRut = rospy.Publisher ('/robocol/ruta', Float32MultiArray, queue_size=1)

    while not startRecieved and endRecieved:
        pass

    #pos3 = Pose()
    #pos3.position.x =  2
    #pos3.position.y =  0
    #pos3.orientation.w = 0
    #ob=Obstacle()
    #ob.position=pos3
    #ob.radius=0.5
    #obstacles.append(ob)
 
    while (not rospy.is_shutdown()):
        if endRecieved == True:
            # Se crean los vertices y casillas del grafo y arreglo respectivamente
            creadorVerticesCasillas()
            # Se crean los arcos del grafo
            creadorArcos()
            # Se emplea el metodo de la libreria networkx para sacar la ruta de nodos A*, a este metodo es necesario
            # suministrarle el nombre del nodo inicial, final y un parametro que haga referencia a un metodo que calcule la
            # heuristica entre dos nodos
            posicionActual = Pose()
            posicionActual.position.x = 0.0
            posicionActual.position.y = 0.0
            ruta = nx.astar_path(g, numCasillas(posicionActual.position.x, posicionActual.position.y),
                         numCasillas(posicionFinal.position.x, posicionFinal.position.y), heuristic=heuristic)

            visualizacionPrevia(ruta)
            rut=[]
            for i in ruta:
                a=[]
                a.append(round(casillas[i].x,2))
                a.append(round(casillas[i].y,2))
                rut.append(a)
            # rut = rut[-1:1]
            # rut.reverse()
            msg = Float32MultiArray()
            # r = ruta_simple(rut)
            msg.data = rut
            pubRut.publish(msg)
            endRecieved=False

    #print("aca2")

def ruta_simple(rutaA):
	print(rutaA)
	return ruta
# def ruta_simple(rutaA):
#     global posicion
#     direccion = None
#     l = []
#     anterior = ruta[0]
#     for nodo in rutaA:
#         if nodo == posicionFinal:
#             x = []
#             y = []
#         else:
#             if nodo[0] == anterior[0]:
#                 if direccion != 'x':
#                     x.append(anterior[0] * relacionX - 7.5)
#                     y.append(anterior[1] * relacionY - 7.5)
#                 direccion = 'x'
#             elif nodo[1] == anterior[1]:
#                 if direccion != 'y':
#                     x.append(anterior[0] * relacionX - 7.5)
#                     y.append(anterior[1] * relacionY - 7.5)
#                 direccion = 'y'
#         anterior = nodo
#     x.append(self.inicio[0] * relacionX - 7.5)
#     y.append(self.inicio[1] * relacionY - 7.5)
#     x.reverse()
#     y.reverse()
#     return x, y

# Metodo que arroja la distancia euclidiana entre dos casillas segun su numeramiento (no posicion exacta)

def heuristic(i, j):

    global n

    # Se retorna la distacnai relativa entre casillas por distancia euclidiana

    return math.sqrt((i % n-j % n)**2+(i//n-j//n)**2)

 

 

# Busca la casilla mas cercana para dos coordenadas en la distribucion

def numCasillas(x, y):

    global n

    # Inicializa la distancia minima en infinito y el indice de la casilla en -1

    dist = float('Inf')

    indice = -1

    # Se empieza a buscar la casilla mas cercana, en caso de que competidor sea mas cercano modifica la distancia minima

    # y el indice de la casilla

    for i in range(0, n ** 2):

        distancia = math.sqrt((casillas[i].x - x) ** 2 + (casillas[i].y - y) ** 2)

        if distancia < dist:

            indice = i

            dist = distancia

    return indice

 

 

# Metodo asociedo a topico para actualizar la posicon del pioneer

def setInitialCallback(odom):

    global posicionActual, startRecieved

    pos=odom.pose

    point= pos.pose

    posicionActual=point

    startRecieved=True

 

 

# Metodo asociedo a topico para actualizar la posicon del pioneer

def setFinalCallBack(pos):

    global posicionFinal, endRecieved

    posicionFinal=pos

    endRecieved=True

 

 

 

 

# Metodo que crea los vertices y casillas del arreglo y del grafo no dirigido

def creadorVerticesCasillas():

    global distanciaCuadricula, n

    # Las variables a continuacion definen desde que posicion se empieza a discretizar la escena

    xInic = distanciaCuadricula/2

    yInic = 25 - distanciaCuadricula/2

    # El ciclo empieza a generar la matriz de casillas donde se guarda la discretizacion y anade el nodo al grafo.

    for i in range(0, n*n):

        g.add_node(i)

        mod = i % n

        div = i//n

        nC = Casilla(xInic+mod*distanciaCuadricula, yInic-div*distanciaCuadricula, libre(xInic+mod*distanciaCuadricula, yInic-div*distanciaCuadricula))

        casillas.append(nC)

 

 

 

# Crea los arcos entre los vertices creados en el grafo, solo es necesario revisar los nodos que pueden ser vecinos, en

# de que ambos esten vacios genera el arco.

def creadorArcos():

    global n

    # El ciclo revisa cada uno de los nodos del grafo

    for i in range(0, n**2):

        # Se revisa primero si la casilla actual esta libre de obstaculos, de lo contrario no es necesario generar arcos

        if casillas[i].libre:

            c = i % n

            f = i//n

            # El ciclo revisa los posibles ocho vecinos de la casilla actual en cada una de las iteraciones

            for j in range(1, 9):

                if j == 1:

                    cP = c-1

                    fP = f

                elif j == 2:

                    cP = c-1

                    fP = f-1

                elif j == 3:

                    cP = c

                    fP = f-1

                elif j == 4:

                    cP = c+1

                    fP = f-1

                elif j == 5:

                    cP = c+1

                    fP = f

                elif j == 6:

                    cP = c+1

                    fP = f+1

                elif j == 7:

                    cP = c

                    fP = f+1
                elif j == 8:

                    cP = c-1
                    fP = f+1
                # En caso de que la casilla si pertenezca a los rangos de discretizacion y este libre se crea un arco
                # entre la casilla actual y el vecino revisado
                if cP in range(0, n) and fP in range(0, n) and casillas[fP*n+cP].libre:
                    g.add_edge(i, fP*n+cP)

 

# Metodo que indica si cierta posicion en la escena esta ocupada por alguno de los obstaculos, devuelve True en caso de

# que no haya obstaculo en ese punto y False de lo contrario

# preguntar distRef pero ya corregido

def libre(xCas, yCas):  # Si se encuentra un obstaculo en ella

    distanciaCarro = 0.42

    distBool = True

    for i in obstacles:

        rospy.loginfo("i:")

        rospy.loginfo(i)

        distObs = math.sqrt((i.position.position.x - xCas)**2 + (i.position.position.y - yCas)**2)

        distRef = i.radius + distanciaCarro

        distBool = (distObs >= distRef)

        if not distBool:

            break

    return distBool

    # En caso de que alguna distancia de referencia sea mayor que la distancia al obstaculo se considera que ese punto

    # esta ocupado por un obstaculo

    # return (dist0>=distRef0) and (dist1>=distRef1) and (dist2>=distRef2) and (dist3>=distRef3) and (dist4>=distRef4)


# Metodo que grafica el camino generado por el RTT previo a que se realice la accion de control
def visualizacionPrevia(path):
    global g
    cordX = []
    cordY = []
    cordXObs = []
    cordYObs = []
    xPath = []
    yPath = []

    for i in range(0, len(casillas)):
        casillaActual = casillas[i]
        if casillaActual.libre:
            if i in path:
                xPath.append(casillaActual.x)
                yPath.append(casillaActual.y)
            else:
                cordX.append(casillaActual.x)
                cordY.append(casillaActual.y)
        else:
            if i not in path:
                cordXObs.append (casillaActual.x)
                cordYObs.append (casillaActual.y)
            else:
                print("Error casilla ocupada pertenece a ruta")
    plt.plot(cordX, cordY, 'bo')
    plt.plot(cordXObs, cordYObs, 'ro')
    plt.plot(xPath, yPath, 'go')
    plt.show()


# Metodo main, mira si existen parametros para la posicion final deseada y ejecuta el metodo principal
if __name__ == '__main__':
    try:
        # En caso de que se pasen tres parametros tipo numero se ajusta la nueva posicion final deseada
        route()
    except rospy.ROSInterruptException:
        pass