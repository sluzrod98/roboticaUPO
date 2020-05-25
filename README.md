# Challenge Robótica
## 1.	Introducción

## 3. Estructura
### 3.1. General
A continuación, se puede observar las conexiones por tópicos que utilizan los nodos Ros lanzados durante la ejecución del programa.
![Alt text](estructura_general.png?raw=true "Image 1")

### 3.2. Nodo Planificador

### 3.3. Nodo Control

### 3.4. Nodo Orca
  El nodo Orca tiene la responsabilidad de esquivar los obstáculos que no se hayan tenido en cuenta en el planificador de caminos porque no hayan sido mapeados. Recibe la velocidad a través del tópico /cmd_vel/tracker, la transforma en función de las lecturas que reciba del laser del robot, y publica la velocidad en el tópico ´/cmd_vel_mux/input/navi´, donde será recibido por el robot para realizar el movimiento. Además, en caso de encontrarse en una situación que no pueda resolver para continuar el camino, publicará un booleano en en el tópico ´/planner/stuck´.
