# Lectura-de-fuerzas-y-pares-sensor-OnRobot
Repositorio orientado a la lectura en tiempo real de fuerzas y torques mediante el sensor OnRobot. Incluye los nodos y scripts necesarios para adquirir, procesar y publicar los datos del sensor dentro del entorno ROS.

## Estructura del Proyecto
El repositorio contiene el paquete de ROS **`force_sensor_node`**, cuya organización es la siguiente:
    
- `src/`: Carpeta destinada al código fuente en C++. Contiene el nodo para publicar las fuerzas y pares del sensor OnRobot en el siguiente archivo:
  - `force_sensor_node.cpp`

- `package.xml`: Archivo de metadatos del paquete, donde se especifican las dependencias (`roscpp`, `std_msgs`).  

- `CMakeLists.txt`: Archivo de configuración utilizado por catkin para la compilación e instalación del paquete. 

## Requisitos Previos
Antes de ejecutar el sistema, asegúrese de cumplir con los siguientes requisitos:
- Encender y configurar los siguientes equipos: **'Gauss'** y **'Master'**
- Conectar el sensor de fuerzas **OnRobot**.

# Pasos de Ejecución
1. Iniciar `roscore` en el equipo **'Master'**:
   ```bash
   roscore
   ```
   
3. Desde el equipo **'Gauss'**, lanzar el nodo `/force_sensor_node`.
   ```bash
   cd catkin_ws
   source devel/setup.bash
   rosrun force_sensor_node force_sensor_node 192.168.0.103
   ```
   *(donde `192.168.0.103` corresponde a la IP del bote de fuerzas)*

4. Una vez lanzado el nodo, se comprueba que el tópico `force_torque_sensor` está publicando correctamente la información sobre las fuerzas y pares del sensor OnRobot.
   ```bash
   rostopic echo /force_torque_sensor
   ```
