# Challenge Robótica
## 1.	Introducción
### 1.1. Objetivos
  - Crear un programa para controlar un robot (Turtlebot) en un escenario con obstáculos y alcanzar un destino siguiendo un camino.
  - Usar los sensores de distancia integrados en el robot para detectar obstáculos y comandar al robot hasta el destino siguiendo un camino.
### 1.2. Materiales
  
## 2. Instrucciones de Ejecución
### 2.1. Challenge 1

### 2.1. Challenge 2

### 2.3. Challenge 3

### 2.4. Challenge 4

### 2.5. Challenge 5


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
#### 4.1.1. planner_node
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

#### 4.1.4. control
Script que se encarga de comandar la velocidad lineal y angular que debe adquirir el robot en cada instante para alcanzar la siguiente meta del camino generado por el planificador.  

#### 4.1.5. pyorca
#### 4.1.6. gazebo
#### 4.1.7. halfplaneintersect

### 4.2. Lanzadores

### 4.3. Ficheros de Configuración
