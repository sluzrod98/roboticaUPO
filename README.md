# Challenge Robótica

## Realizado por:
   - Carlos Aunión Domínguez
   - Sergio Luzuriaga Rodríguez

## Tabla de Contenidos
- [1. Introducción](#1introducción)
  * [1.1. Objetivos](#11-objetivos)
  * [1.2. Materiales](#12-materiales)
- [2. Instrucciones de Ejecución](#2-instrucciones-de-ejecución)
  * [2.1. Prerrequisitos](#21-Prerrequisitos)
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
  - Usar los sensores de distancia integrados en el robot para detectar los obstáculos en el mapa y registrarlos en un mapa de costes local dinámico, que se usará para replanificar el camino si este ha sido bloqueado por un obstáculo y comandar al robot hasta el destino.
### 1.2. Materiales
Se usarán las siguientes herramientas para la implementación del programa y la realización del challenge:
  * El motor de simulación 3D Gazebo.
  * La herramienta de visualización Rviz.
  * Un sensor de distancia simulado por Gazebo para detectar obstáculos.
  * Planificación y re-planificación de caminos usando la API ros::Costmap2D
  * Datos y código obtenido de anteriores EPDs.
  
## 2. Instrucciones de Ejecución
Hemos optado por usar dos paquetes de ros distintos para mantener separado el código que carga el mapa del código que mueve al robot.  
Para cada challenge se proporcionan dos formas de ejecutarlo.
### 2.1. Prerrequisitos
La librería OpenCV de por si, contiene una gran variedad de algoritmos especializados en trackear objetos entre los que se encuentra CSRT (Channel and Spatial Reliability Tracker), que solo está disponible a partir de su versión 3.4.2. La distribución de ROS Kinectic no permite utilizar una versión posterior a la 3.3.0-deb, por lo que hemos optado por utilizar la distribución Melodic.

Los algoritmos de tracker aportados por opencv se encuentran en el paquete "contrib", no en el base. Para actualizar la versión, es necesario desinstalar primero opencv para después instalarlo en su versión contrib.

```
sudo pip uninstall opencv-python
sudo pip uninstall opencv-contrib-python
pip install opencv-contrib-python
```
### 2.2. Challenge 2
Challenge 2 tal como se definió en el enunciado.  
Se deberá ejecutar en un terminal el siguiente comando:  
```
roslaunch robotics_challenge solution_robotics_challenge2.launch
```  

También se puede lanzar la simulación y el programa por separado usando en dos terminales distintas los siguientes comandos:  
```
roslaunch robotics_challenge robotics_challenge2.launch  
roslaunch challenge challenge2.launch
```

### 2.3. Challenge 3
Challenge 3 tal como se definió en el enunciado.  
Se deberá ejecutar en un terminal el siguiente comando:  
```
roslaunch robotics_challenge solution_robotics_challenge3.launch
```  

También se puede lanzar la simulación y el programa por separado usando en dos terminales distintas los siguientes comandos:  
```
roslaunch robotics_challenge robotics_challenge3.launch  
roslaunch challenge challenge3.launch
```

### 2.4. Challenge 4
Modificación del challenge 2 que añade una estantería en el recorrido al destino, no visible para el robot al comienzo de la simulación y obliga al robot a replanificar la ruta.  
Se deberá ejecutar en un terminal el siguiente comando:  
```
roslaunch robotics_challenge solution_robotics_challenge4.launch
```  

También se puede lanzar la simulación y el programa por separado usando en dos terminales distintas los siguientes comandos:  
```
roslaunch robotics_challenge robotics_challenge4.launch  
roslaunch challenge challenge2.launch
```

### 2.5. Challenge 5
Modificación del challenge 3 que añade una caja y un cilindro en el recorrido al destino, no visible para el robot al comienzo de la simulación y que obliga al robot a replanificar la ruta en varias ocasiones.  
Se deberá ejecutar en un terminal el siguiente comando:  
```
roslaunch robotics_challenge solution_robotics_challenge5.launch
```  

También se puede lanzar la simulación y el programa por separado usando en dos terminales distintas los siguientes comandos:  
```
roslaunch robotics_challenge robotics_challenge5.launch  
roslaunch challenge challenge3.launch
```


## 3. Estructura
### 3.1. General
A continuación, se puede observar las conexiones por tópicos que utilizan los nodos Ros lanzados durante la ejecución del programa.

![Alt text](img/estructura_general.png?raw=true "Estructura General")  

Las relaciones entre los nodos creados para nuestro programa son las mostradas a continuación. 

![Alt text](img/estructura_especifica.png?raw=true "Estructura Específica") 

### 3.2. Nodo Planificador
Nodo ROS encargado de recibir los datos del mapa de costes del nodo `costmap_2d` a través del tópico `/costmap_2d/costmap/costmap`, calcular el camino óptimo hasta el destino especificado y publicar ese camino al nodo `control` a través del tópico `path`. Recibe también del nodo `orca`, a través del tópico `/planner/stuck`, un flag que indica que el robot ha qudado atascado y debe realizarse una replanificación del camino.  

Además de su funcionalidad principal, el nodo publica un marcador con el camino calculado al tópico `/planner/path_marker` que puede ser visualizado desde Rviz (pre-configurado).

### 3.3. Nodo Control
En este nodo de ROS se calculará el movimiento que debe realizar el robot para seguir el camino calculado por el planificador. El camino lo recibe del nodo `planner` a través del tópico `/planner/path` y publica el movimiento que debe realizar al tópico `/cmd_vel/tracker`, del que hará uso en nodo `orca`.  

### 3.4. Nodo Orca
El nodo Orca tiene la responsabilidad modificar la velocidad recibida del nodo `control` para evitar los obstáculos que no se hayan tenido en cuenta en el planificador de caminos porque no han sido registrados en el mapa anteriormente. Recibe la velocidad a través del tópico `/cmd_vel/tracker`, la transforma en función de las lecturas que reciba del laser del robot, y publica la velocidad al nodo `/mobile_base_nodelet_manager` en el tópico `/cmd_vel_mux/input/navi`, donde será recibido por el robot para realizar el movimiento. Además, en caso de encontrarse en una situación que no pueda resolver para continuar el camino, publicará un booleano a modo de flag al nodo `planner` en en el tópico `/planner/stuck`.  


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

![Alt text](img/A*_formula.svg?raw=true "Fórmula A*") 

Donde *f(x)* es el coste total del movimiento, *g(x)* es el coste acumulado para alcanzar ese punto y *h(x)* es el coste de la función heurística.

Para este problema hemos decidido utilizar como función heurística la distancia euclídea desde el punto al que nos desplazaríamos y el punto objetivo.  

![Alt text](img/Manhattan.svg?raw=true "Distancia de Manhattan") 

Sin embargo, esta técnica por si sola y dados los movimientos del robot, devuelve una ruta, aunque corta, bastante lenta, puesto que solo puede moverse en 2 direcciones (hacia delante y hacia atrás) además de girar sobre su propio eje. Por ello hemos optado por aplicarle un “suavizado” en el momento de transformar la ruta para devolverla al planificador que consiste en que si realiza varios cambios de direcciones muy rápidamente, no los añada al camino, sino que salte ese escalado y vaya directamente del principio al final de la escalera. Además, el robot desacelera cuando se está acercando al punto que debe llegar, así que solo se envían los puntos de inicio y fin de una línea, sin puntos intermedios.  

#### 4.1.3. Node
Clase nodo utilizada para controlar las posiciones y el coste de cada punto a contemplar en la solución del problema. Almacena las coordenadas *(x,y)* del nodo, así como el nodo que le precede en el camino y su coste.  

#### 4.1.4. Control
Script que se encarga de calcular la velocidad lineal y angular que debe adquirir el robot en cada instante para alcanzar la siguiente meta del camino generado por el planificador (sin tener en cuenta obstáculos en el camino) y detectar cuándo se ha llegado a esa meta, dada una distancia límite. Una vez calculadas estas velocidades usando la orientación del robot y su distancia al destino, el script publica un tópico con los datos de velocidad, de los que hará uso el script `orcaGazebo` para implementar la evasión de obstaculos.  

El script comenzará su funcionamiento cuando reciba un camino generado por el planificador y tendrá en cuenta posibles replanificaciones, actualizando el camino recibido para evitar los obstáculos registrados en el mapa de costes dinámico.  

Recibe diversos valores del servidor de parámetros que permiten ajustar el funcionamiento del script para adaptarse a nuevos escenarios si fuera necesario.

#### 4.1.5. Pyorca
Script que, basándose en el algoritmo *ORCA 2D* ([Optimal Reciprocal Collision Avoidance](http://gamma.cs.unc.edu/ORCA/)) descrito por J. van der Berg, adapta las velocidad lineal y angular exigidas por el script de control teniendo en cuenta las caracteristicas del robot y las lecturas del laser ya adaptadas desde el script de orcaGazebo, todo ello en forma de `Agent`, además de el tiempo estimado de colisión inminente. 

#### 4.1.6. Orca gazebo
Este script hace las veces de fachada entre el control y el pyorca, adaptando los datos e interpretando la adaptación calculada por el nodo de pyorca. En primer lugar, transforma la velocidad angular (medida en rad/s) a la velocidad que necesita el algoritmo de pyorca (componente de la velocidad lineal en el eje *y*). Posteriormente transformará a la inversa la velocidad lineal a la angular que recibe el robot. 

Para calcular la velocidad final, el script genera una serie de obstáculos a tener en cuenta según los datos recibidos por el escáner de distancia del robot en el tópico `/scan`. Estas lecturas se separan en sectores para generar un número específico de obstáculos (el más cercano en cada región). Una vez definidos estos obstáculos, son transformados en objetos de la clase `Agent`, contenida en el script `pyorca` para poder ser usados en la llamada al método que calcula la nueva velocidad. Como extra, los obstaculos son publicados como marcadores en el tópico `/visualization_marker` para permitir su visualización en Rviz.

Además, interpreta la salida del orca para decidir si el robot se ha quedado atascado o no, para esto se hace uso de una variable llamada `stuck_count` que determinará que el robot debe replanificar la ruta al destino cuando el valor de esta variable exceda una cifra determinada. 
Esta variable se incrementará cuando el script recibe una velocidad de avance positiva del tópico `/cmd_vel/tracker` y el script `pyorca` calcule que es necesaria una velocidad de retroceso para evitar los obstáculos cercanos, lo que significa que el camino en dirección al destino está bloqueado. Cuando esto ocurra, se publicará un flag en el tópico `/planner/stuck`, que será recibido por el nodo planificador para iniciar una replanificación del camino.
Si la variable no ha alcanzado el limite establecido en un tiempo de 25 segundos, se reiniciará el valor de la variable para evitar una replanificación cuando no sea necesario.

Este script se encargará además de asegurar que la velocidad publicada al robot no excede los límites establecidos en el servidor de parámetros y que no se realicen aceleraciones bruscas que puedan desconfigurar el sistema de referencia interno del robot.

#### 4.1.7. Halfplaneintersect
Script usado por `pyorca` para calcular las trayectorias y velocidades futuras de todos los obstáculos proporcionados al script y calcular así las nuevas velocidades necesarias para cada agente de tal forma que se evite una colisión. En nuestro caso, solo el agente del robot es capaz de moverse y todos los obstáculos se consideran estáticos, por lo que solo se calculará la nueva velocidad para el robot.

### 4.2. Lanzadores
En el proyecto existen 3 lanzadores (1 por cada challenge planteado) cuya función es desplegar en ROS cada nodo. Los nodos que componen la solución son [Planner](#32-nodo-planificador), [Control](#33-nodo-control) y [Orca](#34-nodo-orca).

### 4.3. Ficheros de Configuración
En los distintos ficheros de configuración se recogen todos los parámetros más importantes para el correcto funcionamiento de las solciones. A continuación se presenta una lista con los parámetros recogidos en cada lista, así como su funcionalidad durante la ejecución:

- challengeX
  * init: Coordenadas del punto de inicio
  * goal: Coordenadas del punto final
  * obstacle:threshold: Límite que se toma al cargar el mapa de coste para entender las regiones como visitables o no. El valor de este parámetro es clave, puesto que si es demasiado alto, podría planificar un camino a través de una pared o si es demasiado bajo, no encontrar solución al problema.
  
 - globalParams
   * ang_threshold: límite para que el robot rote en lugar de avanzar.
   * goal_tolerance: Distancia a la que el robot debe considerar que ha alcanzado el punto asignado.
   * linear_max: Máxima velocidad lineal que puede tomar el robot. Si es demasiado alta puede haber problemas con las referencias de la simulación en m/s.
   * speed_parameter: Regula la velocidad lineal que calcula el nodo de control.
   * angular_max: Velocidad de rotación máxima que puede tomar el robot en rad/s.
   * angular_parameter: Regula la velocidad de giro que calcula el nodo de control.
  
- orcaParams
  * radius: Radio del robot.
  * sections: Numero de secciones tomadas por el escaner en orca.
  * obstacle_distance: Distancia a la que el robot detecta obstáculos en su campo de visión.
  * linear_max: Máxima velocidad lineal que puede tomar el robot. Si es demasiado alta puede haber problemas con las referencias de la simulación en m/s.
  * angular_max: Velocidad de rotación máxima que puede tomar el robot en rad/s.
  * smooth_param: Suaviza la aceleración del robot para evitar acelerones bruscos.
  * frecuency: Frecuencia a la que opera el robot.
