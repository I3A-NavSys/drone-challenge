![Drone Challenge Logo](https://blog.uclm.es/esiidronechallenge/files/2017/03/cropped-dc.logo_.png)

Este repositorio almacena los ficheros necesarios para la configuración del entorno necesario para ejecutar el simulador para la Drone Challenge. El simulador está formado por distintas piezas software, que deben ser instaladas y configuradas en una máquina real o máquina virtual. Podemos dividir el simulador en dos grandes partes:

- **Controlador del dron:** Es el encargado de decidir qué acciones (moverse, ascender, girar, etc.) debe realizar el drone. Recibe la visualización de la cámara incorporada por el drone y toma la decisión de qué realizar en cada momento. Este controlador ha sido creado en **Matlab**, de _Mathworks_, con algunas herramientas necesarias que son distribuidas por la propia Mathworks. Comentaremos más tarde el proceso de instalación y las dependencias necesarias.
- **Simulador 3D**: Si bien el controlador decide qué realiza el drone, necesitamos un entorno de simulación 3D donde el drone vuele y podamos ver la respuesta del mismo a los comandos del controlador. Para ello se hace uso de **ROS (_Robot Operating System_) y Gazebo**. Gazebo nos permitirá la visualización de drone a través de un simulador 3D, mientras que ROS permitirá implementar el modelo dinámico y la respuesta del drone a los comandos del controlador.



# 1. Configuración del entorno de trabajo

Para configurar el entorno de trabajo necesitamos instalar:

- En Ubuntu 20.04 LTS [^ 1][^2]:
  - ROS Noetic (_Robotic Operating System_)
  - Gazebo 11

[^1]: La configuración del entorno puede realizarse en Ubuntu 18.04 o 20.04 LTS. Sin embargo, si se decide realizar en Ubuntu 18.04, la versión de ROS a instalar será "melodic", y los archivos de configuración a instalar serán los propios de dicha versión.
[^2]: ROS y Gazebo disponen de versiones experimentales para Windows, sin embargo, el funcionamiento del simulador no ha sido probado en Windows.

- En Windows o Ubuntu [^3] :
  - MATLAB R2022a con las siguientes dependencias, que pueden instalarse durante la instalación de Matlab:
    - MATLAB Coder
    - Simulink
    - Simulink Coder
    - Stateflow
    - Aerospace Blockset
    - Aerospace Toolbox
    - Robotics System Toolbox
    - ROS Toolbox

[^3]: La instalación de MATLAB puede realizarse en la misma máquina donde ROS y Gazebo están instalados, o hacerlo en máquinas separadas. Ambas piezas de software son conectadas a través de una red, por lo que deben poder comunicarse entre ellas. Si se hace en la misma máquina, se reducirá la latencia de red entre MATLAB, ROS y Gazebo.  

## 1.1. Configuración de ROS y Gazebo
La instalación de ROS y Gazebo puede realizarse siguiendo la documentación del propio proyecto, disponible en http://wiki.ros.org/noetic/Installation/Ubuntu. Con esta guía podrás:
1. Configurar los repositorios de Ubuntu para poder descargar e instalar ROS y Gazebo.
2. Instalar ROS y Gazebo (versión "Desktop-Full Install")
3. Configurar el entorno 
4. Instalar las dependencias y el gestor de dependencias, que se encarga de mantenerlas actualizadas.

Una vez finalizada la instalación con los pasos mencionados anteriormente dispondremos de ROS y Gazebo instalados en el equipo. Para verificar su correcto funcionamiento, abre una terminal (`Ctrl + Alt + T`) y escribe el siguiente comando:
``
roscore &
``
A continuación, en otra terminal diferente, ejecuta:
``
rosrun gazebo_ros gazebo
``
Si ROS y Gazebo han sido instalados correctamente, observarás como Gazebo despliega un visor con un entorno 3D completamente vacio. Si no es así, prueba a ejecutar los dos comandos anteriores de nuevo. Para cerrar el simulador, usa `Ctrl + C` en la segunda terminal abierta. Si tras varios intentos no funciona, la instalación de ROS y Gazebo no se ha realizado correctamente.

## 1.2. Configuración del plugin de Drone Challenge
Una vez instalado ROS y Gazebo dispondremos del entorno 3D necesario, pero necesitamos instalar un plugin diseñado para la Drone Challenge. Los ficheros del plugin se encuentran en este repositorio, dentro de la carpeta "drone_ros". Dentro de ella encontrarás dos directorios: "melodic" o "neotic". Dependiendo de la versión de ROS que hayas instalado necesitas usar unos ficheros u otros. En caso de haber seguido los pasos anteriores, la versión de ROS instalada es "noetic", por lo que necesitas usar los ficheros del directorio "drone_ros/noetic".
Copia los ficheros de repositorio en la máquina donde has realizado la instalación de ROS y Gazebo, por ejemplo, en `/home/<usuario>/dronechallenge_files`, y, a continuación, sigue los siguientes pasos:
