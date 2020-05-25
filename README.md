# Challenge Robótica
## Tabla de Contenidos
- [1. Introducción](#1introducción)
  * [1.1. Objetivos](#11-objetivos)
  * [1.2. Materiales](#12-materiales)
- [2. Instrucciones de Ejecución](#2-instrucciones-de-ejecución)
  * [2.1. Challenge 1](#21-challenge-1)
  * [2.2. Challenge 2](#22-challenge-2)
  * [2.3. Challenge 3](#23-challenge-3)
  * [2.4. Challenge 4](#24-challenge-4)
  * [2.5. Challenge 5](#25-challenge-5)
- [3. Estructura](#3-estructura)
  * [3.1. General](#31-general)
  * [3.2. Nodo Planificador](#32-nodo-planificador)
  * [3.3. Nodo Control](#33-nodo-control)
  * [3.4. Nodo Orca](#34-nodo-orca)
- [4. Componentes](#4-componentes)
  * [4.1. Scripts](#41-scripts)
  * [4.2. Lanzadores](#42-lanzadores)
  * [4.3. Ficheros de Configuración](#43-ficheros-de-configuración)

## 1.	Introducción
### 1.1. Objetivos
  - Crear un programa para controlar un robot (Turtlebot) en un escenario con obstáculos y alcanzar un destino siguiendo un camino.
  - Usar los sensores de distancia integrados en el robot para detectar obstáculos y comandar al robot hasta el destino siguiendo un camino.
### 1.2. Materiales
  
## 2. Instrucciones de Ejecución
Hemos optado por usar dos paquetes de ros distintos para mantener separado el código que carga el mapa del código que mueve al robot.  
Para cada challenge se proporcionan dos formas de ejecutarlo.
### 2.1. Challenge 1
Challenge 1 tal como se definió en el enunciado.
Se deberá ejecutar en un terminal el siguiente comando:  
`roslaunch robotics_challenge solution_robotics_challenge1.launch`  

También se puede lanzar la simulación y el programa por separado usando en dos terminales distintas los siguientes comandos:  
```
roslaunch robotics_challenge robotics_challenge1.launch  
roslaunch challenge challenge1.launch
```
### 2.1. Challenge 2
Challenge 2 tal como se definió en el enunciado.
Se deberá ejecutar en un terminal el siguiente comando:  
`roslaunch robotics_challenge solution_robotics_challenge2.launch`  

También se puede lanzar la simulación y el programa por separado usando en dos terminales distintas los siguientes comandos:  
```
roslaunch robotics_challenge robotics_challenge2.launch  
roslaunch challenge challenge2.launch
```

### 2.3. Challenge 3
Challenge 3 tal como se definió en el enunciado.
Se deberá ejecutar en un terminal el siguiente comando:  
`roslaunch robotics_challenge solution_robotics_challenge3.launch`  

También se puede lanzar la simulación y el programa por separado usando en dos terminales distintas los siguientes comandos:  
```
roslaunch robotics_challenge robotics_challenge3.launch  
roslaunch challenge challenge3.launch
```

### 2.4. Challenge 4
Modificación del challenge 2 que añade una estantería en el recorrido al destino, no visible para el robot al comienzo de la simulación y obliga al robot a replanificar la ruta.
Se deberá ejecutar en un terminal el siguiente comando:  
`roslaunch robotics_challenge solution_robotics_challenge4.launch`  

También se puede lanzar la simulación y el programa por separado usando en dos terminales distintas los siguientes comandos:  
```
roslaunch robotics_challenge robotics_challenge4.launch  
roslaunch challenge challenge2.launch
```

### 2.5. Challenge 5
Modificación del challenge 3 que añade una caja y un cilindro en el recorrido al destino, no visible para el robot al comienzo de la simulación y que obliga al robot a replanificar la ruta en varias ocasiones.
Se deberá ejecutar en un terminal el siguiente comando:  
`roslaunch robotics_challenge solution_robotics_challenge5.launch`  

También se puede lanzar la simulación y el programa por separado usando en dos terminales distintas los siguientes comandos:  
```
roslaunch robotics_challenge robotics_challenge5.launch  
roslaunch challenge challenge3.launch
```

## 3. Estructura
### 3.1. General
A continuación, se puede observar las conexiones por tópicos que utilizan los nodos Ros lanzados durante la ejecución del programa.

![Alt text](estructura_general.png?raw=true "Estructura General")  

Las relaciones entre los nodos creados para nuestro programa son las mostradas a continuación. 

![Alt text](estructura_especifica.png?raw=true "Estructura Específica") 

### 3.2. Nodo Planificador

### 3.3. Nodo Control
En este nodo de ROS vamos a realizar el control del movimiento que debe realizar el robot para seguir el camino calculado por el planificador, así como un “suavizado” para que no pegue acelerones muy bruscos. El camino lo recibe del nodo planner a través del tópico ```/planner/path``` y publica el movimiento que debe realizar al tópico ```/cmd_vel/tracker```.  

### 3.4. Nodo Orca
El nodo Orca tiene la responsabilidad de esquivar los obstáculos que no se hayan tenido en cuenta en el planificador de caminos porque no hayan sido mapeados. Recibe la velocidad a través del tópico ```/cmd_vel/tracker```, la transforma en función de las lecturas que reciba del laser del robot, y publica la velocidad en el tópico ```/cmd_vel_mux/input/navi```, donde será recibido por el robot para realizar el movimiento. Además, en caso de encontrarse en una situación que no pueda resolver para continuar el camino, publicará un booleano en en el tópico ```/planner/stuck```.  

## 4. Componentes
### 4.1. Scripts
#### 4.1.1. Planner_node
Este script se encarga de gestionar la planificación del camino óptimo para alcanzar la meta indicada desde la posición actual del robot utilizando el algoritmo A*. Este script realiza las siguientes funciones:
-	Planificar la ruta óptima a la meta indicada cuando se reciba información del mapa de costes.
-	Replanificar la ruta cuando el nodo orca detecte que el robot no puede seguir avanzando en la ruta actual debido a algún obstáculo no contenido en el mapa de costes al calcularla. Cuando esto ocurra, utilizará la información del mapa de costes dinámico para tener en cuenta los obstáculos detectados y generar una nueva ruta.
-	Publicar el camino calculado al tópico correspondiente.
-	Publicar un marcador para representar el camino calculado en Rviz.

#### 4.1.2. AStar
El algoritmo que utiliza el robot para encontrar el camino que debe seguir. A* es un algoritmo heurístico, es decir, que toma información acerca del propio entorno del problema (permitiéndole ignorar restricciones), que, en base a una función heurística, encuentra ruta más que aceptable hasta la meta.  
El funcionamiento de A* es bastante simple. En primer lugar, partiendo de un punto, calcula todos los movimientos simples que puede realizar (en el caso del robot, se han tenido en cuenta los 4 ejes cardinales y movimientos diagonales con 45º de inclinación). A cada posible punto, le asocia un coste que será igual al coste acumulado de desplazarse a ese punto (todos los costes que lleva ya más el coste de la función heurística) y así sucesivamente.  
```C=f(nodo)+h(nodo)```  
Donde C es el coste total del movimiento, f() es el coste acumulado y h() es el coste de la función heurística.
Para este problema hemos decidido utilizar como función heurística la distancia euclídea desde el punto al que nos desplazaríamos y el punto objetivo.  
``h(nodo)= √((p.x-nodo.x)^2+(p.y-nodo.y)^2 )``  
Sin embargo, esta técnica por si sola y dados los movimientos del robot, devuelve una ruta, aunque corta, bastante lenta, puesto que solo puede moverse en 2 direcciones (hacia delante y hacia atrás) además de girar sobre su propio eje. Por ello hemos optado por aplicarle un “suavizado” en el momento de transformar la ruta para devolverla al planificador que consiste en que si realiza varios cambios de direcciones muy rápidamente, no los añada al camino, sino que salte ese escalado y vaya directamente del principio al final de la escalera. Además, el robot desacelera cuando se está acercando al punto que debe llegar, así que solo se envían los puntos de inicio y fin de una línea, sin puntos intermedios.  

#### 4.1.3. Node
Clase nodo utilizada para controlar las posiciones y el coste de cada punto a contemplar en la solución del problema. Almacena las coordenadas x y del nodo, así como el nodo que le precede en el camino y su coste.  

#### 4.1.4. Control
Script que se encarga de comandar la velocidad lineal y angular que debe adquirir el robot en cada instante para alcanzar la siguiente meta del camino generado por el planificador.

#### 4.1.5. Pyorca
Script que, basándose en el algoritmo *orca 2D* (Optimal Reciprocal Collision Avoidance <http://gamma.cs.unc.edu/ORCA/>) descrito por J. van der Berg, adapta las velocidad lineal y angular exigidas por el script de control teniendo en cuenta las caracteristicas del robot y las lecturas del laser ya adaptadas desde el script de orcaGazebo, todo ello en forma de *'agentes'*, además de el tiempo estimado de colisión inminente. 
#### 4.1.6. Orca gazebo
Este script hace las veces de fachada entre el control y el pyorca, adaptando los datos e interpretando la adaptación calculada por el nodo de pyorca. En primer lugar, transforma la velocidad angular (medida en rad/s) a la velocidad que necesita el algoritmo de pyorca (componente de la velocidad lineal en el eje *y*) 
#### 4.1.7. Halfplaneintersect

### 4.2. Lanzadores

### 4.3. Ficheros de Configuración
