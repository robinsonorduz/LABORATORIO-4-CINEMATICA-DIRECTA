# LABORATORIO-4-CINEMATICA-DIRECTA
# Robinson Jair Orduz Gomez
# introducción:
en este laboratorio se hace cinematica directa de un manipulador phantom x pincher, primero se tomaron las dimensiones de longitud de los eslabones y a partir de estas, con el algoritmo Denavit-Hartenberg se obtuvo el modelo matematico que permite saber el punto en el espacio en el que se encuentra el tcp del manipulador a partir de los angulos de las articulaciones y longitud de los eslabones.
# Método DH:
https://github.com/robinsonorduz/LABORATORIO-4-CINEMATICA-DIRECTA/blob/main/Captura.JPG
# Metodologia:
## instalar catkin build:
Es un complemento de ROS que permite comunicarlo con los motores Dinamixel del manipulador.
## instalar el paquete pxROBOT:
Este paquete crea los nodos para acceder a los servicios de los motores dinamixel
## instalar libreria de Peter Corke:
se instala para pyton, y permite dibujar las posiciones articulares del manipulador en una grafica que se ve en el PC.

## Crear el manipulador en el entorno virtual:
se importan las librerias que van a dibujar el manipulador y configurar las variables a partir de la tabla DH para generar el grafico que copia la posición del manipulador.
``` python
  
    import roboticstoolbox as rtb
    import numpy as np
    import time as t
    robot= rtb.DHRobot([
      rtb.RevoluteDH(d=0.045, alpha= -np.pi/2, offset= 0),
      rtb.RevoluteDH(a= 0.105, offset= -np.pi/2),
      rtb.RevoluteDH(a=0.105, offset= np.pi/2),
      rtb.RevoluteDH(a=0.110, offset= np.pi/2)
    ], name= "Pincher")
robot.plot([np.pi/6, -np.pi/3, 0, 0]).hold()
```
https://github.com/robinsonorduz/LABORATORIO-4-CINEMATICA-DIRECTA/blob/main/img1.png

## Suscripción a las variables de articulación del manipulador:
para ver el angulo de articulacion de los motores, se ejecuta px_robot desde catkin y luego se ejecuta el archivo Suscriptor.py
``` 
  
   catkin build px_robot
   source devel/setup.bash
   roslaunch px_robot px_controllers.launch
    
```
```  python
  
  import rospy
  import numpy as np
  import roboticstoolbox as rtb
  from std_msgs.msg import String 
  from sensor_msgs.msg import JointState
  import time as t
    
```
## Verificación del home del manipulador
https://github.com/robinsonorduz/LABORATORIO-4-CINEMATICA-DIRECTA/blob/main/home.png
https://github.com/robinsonorduz/LABORATORIO-4-CINEMATICA-DIRECTA/blob/main/home_consola.png

Para este programa tambien se requiere inicializar el paquete px_robot. A continuación se importan las librerías necesarias:

```  python
  import rospy
  import numpy as np
  import time
  import roboticstoolbox as rtb
  from std_msgs.msg import String
  from sensor_msgs.msg import JointState
  from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    
```
## publicación de posiciones del manipulador
Luego de crear el manipulador se llama joint_publisher() asi:

- Se crea un publicador con rospy.Publisher y se ingresa como argumento un tópico y un tipo de mensaje.
- Se inia un nodo publicador.
- Se crea el arreglo posicionActual, el cual contiene inicialmente las posiciones de HOME.
- Se crea un ciclo infinito con  while.

Dentro de while se crea un mensaje de tipo JointTrajectory, se le pone el tiempo actual el encabezado del mensaje en el atributo header.stamp y se nombra cada articulacion con el atributo joint_names. Se crea un nuevo mensaje de tipo JointTrajectoryPoint. A continuación el programa  pedirá escribir la articulación que se quiere modificar y su nueva posición. Para la articulación escogida, se harán los cáculos necesarios para modificarla teniendo en cuenta los rangos articulares y convirtiendo la posición a radianes. Si la posición se sale del rango se irá a la posición mas cercana;luego de la conversión, la nueva posición se añade al arreglo posicionActual. 

Finalmente, se grafica la nueva posición mediante el toolbox. Se agrega el arreglo de posiciones al mensaje de puntos y luego este al mensaje de estado. Finalmente se publica el mensaje y en consola se muestra cual es la articulación editada con su nueva posición:

```  python
 def joint_publisher():
    pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=0)
    rospy.init_node('joint_publisher', anonymous=False)


    posicionActual=[0, 0, -np.pi/2, -np.pi/2, 0]

    while True:


        state = JointTrajectory()
        state.header.stamp = rospy.Time.now()
        state.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        point = JointTrajectoryPoint()
        actual=int(input("Ingrese la articulación que desea manipular: "))
        if actual<1 or actual>5:
            break
        else:
            angDes=int(input("Ingrese el angulo deseado para la articulacion: "))

            if actual==1:
                ang=angDes+0
                if ang>-180 and ang<180:
                    posicionActual[0]=ang*np.pi/180
                elif ang<=-180:
                    angDes=-179
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[0]=-179*np.pi/180
                else:
                    angDes=179
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[0]=179*np.pi/180
            elif actual==2:
                ang=angDes+0
                if ang>-100 and ang<100:
                    posicionActual[1]=-ang*np.pi/180
                elif ang<=-100:
                    angDes=-99
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[1]=99*np.pi/180
                else:
                    angDes=100
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[1]=-99*np.pi/180
            elif actual==3:
                ang=angDes+90
                if ang>-130 and ang<130:
                    posicionActual[2]=-ang*np.pi/180
                elif ang<=-130:
                    angDes=-219
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[2]=129*np.pi/180
                else:
                    angDes=40
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[2]=-129*np.pi/180
            elif actual==4:
                ang=angDes+90
                if ang>-100 and ang<100:
                    posicionActual[3]=-ang*np.pi/180
                elif ang<=-100:
                    angDes=-190
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[3]=89*np.pi/180
                else:
                    angDes=10
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[3]=-99*np.pi/180
            elif actual==5:
                if ang>-90 and ang<90:
                    posicionActual[4]=-ang*np.pi/180
                elif ang<=-90:
                    ang=-89
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[4]=89*np.pi/180
                else:
                    ang=89
                    print("Posicion no admitida, moviendo a la mas cercana posible.")
                    posicionActual[4]=-89*np.pi/180


            robot.plot([posicionActual[0],-np.pi/2+posicionActual[1],posicionActual[2],posicionActual[3]])
            point.positions = posicionActual  
            point.time_from_start = rospy.Duration(0.5)
            state.points.append(point)
            pub.publish(state)
            print('Articulacion editada: ', actual, ", ", "Nuevo angulo: ", angDes)
            rospy.sleep(1)

    
```
Este es el resultado:
https://github.com/robinsonorduz/LABORATORIO-4-CINEMATICA-DIRECTA/blob/main/posicion1.JPG




## Calcular posiciones del robot con servicios:

Se inicia px_robot, se crea el manipulador, y dos metodos que calculan el valor de articulación que se necesita.


Se crea el robot en el Toolbox como se explicó anteriormente. Se crean dos métodos que permitirán utilizar el servicio y calcular el valor que debe ser enviado.

El primer método es jointCommand(), que recibe como argumentos un tipo de comando, un identificador de articulación, una dirección, un valor y un tiempo. Este espera a que haya disponibilidad del servicio, a continuación lo utiliza para enviar el nuevo comando con los nuevos parámetros y comprueba si funcionó correctamente:


```  python
  import rospy
  import time
  import roboticstoolbox as rtb
  import numpy as np
  from dynamixel_workbench_msgs.srv import DynamixelCommand
  def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))
    
```

El otro método es calculaAngulo() se utiliza para calcuular el valor que debe ser enviado a cada motor, esto debido a que mediante el servicio se envía una señal de entre 0 y 1023  en vez de un ángulo en radianes, se determina la posición de HOME para cada articulación en estos nuevos valores. A continuación, con el ángulo en grados que se desea se hace la conversión a la nueva señal:


```  python
def calculaAngulo(angulo, ID):
    if ID==1:
        salidaA=int(512+angulo/180*512)
    elif ID==2:
        salidaA=int(512-angulo/130*512)
    elif ID==3:
        salidaA=int(220-angulo/130*512)
    elif ID==4:
        salidaA=int(230-angulo/1301*512)
        if salidaA <200:
            salidaA=200
    return salidaA
    
```
Finalmente se tiene una porción de código, que límita el torque en los motores:

```  python
        jointCommand('', 1, 'Torque_Limit', 500, 0)
        jointCommand('', 2, 'Torque_Limit', 500, 0)
        jointCommand('', 3, 'Torque_Limit', 400, 0)
        jointCommand('', 4, 'Torque_Limit', 400, 0)
```

A continuación se tiene un bucle while infinito el cual solicita la posición deseada. A partir de esta utiliza el servicio para enviar la nueva posición de cada articulación y calcula el valor de señal a enviar mediante el método calculaAngulo. Finalmente grafica la posición mediante el Toolbox. El código es:

```  python
       while(True):
            posicion=int(input("Ingrese un numero de posicion: "))
            if(posicion== 1):
                jointCommand('',1, 'Goal_Position', calculaAngulo(0,1), 0)
                jointCommand('',2, 'Goal_Position', calculaAngulo(0,2), 0)
                jointCommand('',3, 'Goal_Position', calculaAngulo(0,3), 0)
                jointCommand('',4, 'Goal_Position', calculaAngulo(0,4), 0)
                robot.plot([0, 0, 0, 0])
            elif(posicion==2):
                jointCommand('',1, 'Goal_Position', calculaAngulo(-20,1), 0)
                jointCommand('',2, 'Goal_Position', calculaAngulo(20,2), 0)
                jointCommand('',3, 'Goal_Position', calculaAngulo(-20,3), 0)
                jointCommand('',4, 'Goal_Position', calculaAngulo(20,4), 0)
                robot.plot([np.pi/180*-20, np.pi/180*20, np.pi/180*-20, np.pi/180*20])
            elif(posicion==3):
                jointCommand('',1, 'Goal_Position', calculaAngulo(30,1), 0)
                jointCommand('',2, 'Goal_Position', calculaAngulo(-30,2), 0)
                jointCommand('',3, 'Goal_Position', calculaAngulo(30,3), 0)
                jointCommand('',4, 'Goal_Position', calculaAngulo(-30,4), 0)
                robot.plot([np.pi/180*30, np.pi/180*-30, np.pi/180*30, np.pi/180*-30])
            elif(posicion==4):
                jointCommand('',1, 'Goal_Position', calculaAngulo(-90,1), 0)
                jointCommand('',2, 'Goal_Position', calculaAngulo(15,2), 0)
                jointCommand('',3, 'Goal_Position', calculaAngulo(-55,3), 0)
                jointCommand('',4, 'Goal_Position', calculaAngulo(10,4), 0)
                robot.plot([np.pi/180*-90, np.pi/180*15, np.pi/180*-55, np.pi/180*10])
            elif(posicion==5):
                jointCommand('',1, 'Goal_Position', calculaAngulo(-90,1), 0)
                jointCommand('',2, 'Goal_Position', calculaAngulo(45,2), 0)
                jointCommand('',3, 'Goal_Position', calculaAngulo(-55,3), 0)
                jointCommand('',4, 'Goal_Position', calculaAngulo(10,4), 0)
                robot.plot([np.pi/180*-90, np.pi/180*45, np.pi/180*-55, np.pi/180*10])
```
![image](https://github.com/robinsonorduz/LABORATORIO-4-CINEMATICA-DIRECTA/assets/20913010/949c6d72-2f05-41ea-9763-46b78d2175eb)

## conclusiones
-El entorno de ROS al principio puede ser confuso debido a que es un método nuevo de comunicacion entre software y hardware, ademas personalmente no habia trabajado mucho con Linux y hay cosas a las que cuesta acostumbrarse en este sistema operativo.
-El mètodo de convertir un robot en un modelo matemàtico matricial de Denavit-Hartenberg ofrece un algoritmo facil de seguir resumiendolo en rotaciones y traslaciones para pasar de una articulaciòn a otra; necesitando una curva de aprendizaje por parte del estudiante resulta un mètodo muy intuitivo.
