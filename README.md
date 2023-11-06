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
