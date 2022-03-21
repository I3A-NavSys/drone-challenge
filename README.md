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

## 1.1. Instalación de ROS y Gazebo
La instalación de ROS y Gazebo puede realizarse siguiendo la documentación del propio proyecto, disponible en http://wiki.ros.org/noetic/Installation/Ubuntu. Con esta guía podrás:
1. Configurar los repositorios de Ubuntu para poder descargar e instalar ROS y Gazebo.
2. Instalar ROS y Gazebo (versión "Desktop-Full Install")
3. Configurar el entorno 
4. Instalar las dependencias y el gestor de dependencias, que se encarga de mantenerlas actualizadas.

Una vez finalizada la instalación con los pasos mencionados anteriormente dispondremos de ROS y Gazebo instalados en el equipo. Para verificar su correcto funcionamiento, abre una terminal (`Ctrl + Alt + T`) y escribe el siguiente comando:
```
roscore &
```
A continuación, en otra terminal diferente, ejecuta:
```
rosrun gazebo_ros gazebo
```
Si ROS y Gazebo han sido instalados correctamente, observarás como Gazebo despliega un visor con un entorno 3D completamente vacio. Si no es así, prueba a ejecutar los dos comandos anteriores de nuevo. Para cerrar el simulador, usa `Ctrl + C` en la segunda terminal abierta. Si tras varios intentos no funciona, la instalación de ROS y Gazebo no se ha realizado correctamente.

## 1.2. Configuración del plugin de Drone Challenge
Una vez instalado ROS y Gazebo dispondremos del entorno 3D necesario, pero necesitamos instalar un plugin diseñado para la Drone Challenge. Los ficheros del plugin se encuentran en este repositorio, dentro de la carpeta "drone_ros". Dentro de ella encontrarás dos directorios: "melodic" o "neotic". Dependiendo de la versión de ROS que hayas instalado necesitas usar unos ficheros u otros. En caso de haber seguido los pasos anteriores, la versión de ROS instalada es "noetic", por lo que necesitas usar los ficheros del directorio "drone_ros/noetic".
Copia los ficheros de repositorio en la máquina donde has realizado la instalación de ROS y Gazebo, por ejemplo, en `/home/<usuario>/dronechallenge_files`, y, a continuación, sigue los siguientes pasos:

1. Copia los contenidos de `drone_ros` del directorio que acabas de crear en el directorio `/opt/ros/noetic/share`[^1]. Necesitarás permisos de root. Reemplaza _<usuario>_ por el nombre de tu usuario
```shell
sudo cp /home/<usuario>/dronechallenge_files/drone_ros/noetic /opt/ros/noetic/share
```
2.	Los ficheros anteriormente copiados serán ejecutados con el usuario actual (no el root). Por ello, necesitamos modificar los permisos para que el usuario pueda editarlos y ejecuarlos. Para ello, cambiaremos la propiedad de dichos ficheros al usuario y grupo del usuario en cuestión.
```shell
cd /opt/ros/noetic/share
sudo chown <usuario> -R drone_ros
sudo chgrp <usuario> -R drone_ros
```
3. Finalmente, debemos modificar la tarjeta de red que escuchará el plugin para recibir los mensajes de control. Para ello, seguiremos los siguientes pasos:
   3.1.	Si no dispones del paquete `net-tools`, instalalo con el siguiente comando:
   ```shell
   sudo apt install net-tools
   ```
   3.2.	Una vez instalado, ejecuta el siguiente comando:
   ```shell
   ifconfig
   ```
   La salida del comando mostrará las interfaces de red instaladas en tu máquina. A 	continuación mostramos un ejemplo: esta máquina dispone de 2 interfaces de red: _enp0s3_ y _lo_. La interfaz llamada _lo_ es la interfaz interna del ordenador, por lo que esa solo la debemos usar si tanto MATLAB como ROS y Gazebo se encuentran en el mismo equipo e instalación. Apunta el nombre de la interfaz a usar.
   ![Salida del comando ifconfig](https://i.imgur.com/A97i0zu.png)
   
   3.3.	Ahora, edita el fichero `ros_ip_set.sh` localizado en el directorio `/opt/ros/noetic/share/drone_ros/launch`. Para ello, usa estos comandos:
   ```shell
   cd /opt/ros/noetic/share/drone_ros/launch
   gedit ros_ip_set.sh
   ```
   En la primera línea, tras la palabra _ifconfig_, y antes del caracter |, escribe el nombre de tú interfaz. En el caso de esta guía, usaremos _enp0s3_.  Recuerda guardar el fichero antes de salir.
   ![Configuración de la interfaz en el fichero ros_ip_set.sh](https://i.imgur.com/D8lMIg0_d.webp?maxwidth=760&fidelity=grand)

## 1.3. Añadir los modelos 3D a Gazebo
Gazebo dispone de multitud de recursos 3D. Sin embargo, el modelo del drone y los marcos necesarios deben ser instalados de forma independiente. Para ello, simplemente copia el directorio `models` proporcionado dentro de la carpeta oculta `.gazebo`, dentro de la carpeta de tu usuario:
```shell
cd /home/<usuario> 
cp -R /dronechallenge_files/models /.gazebo
```

## 1.4. Instalación de MATLAB y dependencias
Para instalar MATLAB, en este caso la versión 2022a, solo tienes que dirigirte a su página web y descargar el fichero ejecutable (https://es.mathworks.com/products/matlab/whatsnew.html). En esta guía se seguirá los pasos de instalación para Windows, pero puede ser instalado en Ubuntu de forma muy similar.
Una vez descargado el fichero ejecutable, ejecutalo. Sigue el proceso de instalación hasta la ventana que indique "_Select products to install_". Es aquí donde tenemos que marcar las siguientes dependencias:
- MATLAB Coder
- Simulink
- Simulink Coder
- Stateflow
- Aerospace Blockset
- Aerospace Toolbox
- Robotics System Toolbox
- ROS Toolbox

MATLAB empezará a descargar todos los ficheros necesarios y comenzará la instalación. Cuando termine, puede que tengas que activar tu licencia de MATLAB con algunos de los métodos ofrecidos. Sigue los pasos de validación y tendrás MATLAB listo para comenzar.

# 2. Ejecución del simulador
Finalmente tenemos todo instalado. Ya comentamos anteriormente que el simulador se divide en dos partes: el control y el entorno de simulación 3D. Ambos se encuentran comunicados por una red (esto es así porque el control y el entorno de simulación pueden estar en máquinas diferentes. De ahí que anteriormente hayamos tenido que configurar una interfaz de red como medio de comunicación). Ahora, solo tenemos que ejecutar ambos entornos:
## 2.1. Iniciar la simulación en ROS y Gazebo
Primero iniciaremos ROS y Gazebo, ya que es MATLAB quien debe conectarse a ellos. Para ejecutar la simulación, ejecuta los siguientes comandos:
```
roscd drone_ros
cd launch
./round1.sh
```
Verás como la terminal se llena de multitud de lineas con información del entorno de simulación. Entre toda esa información solo tienes que fijarte en la primera línea, donde te indica la dirección IP de ROS.
Tras unos segundos, verás que Gazebo se habrá abierto, y verás que hay un escenario con un drone sobre una caja, y 3 marcos de color rojo, verde y azul flotando. 

![Entorno de simulación 3D de la Drone Challenge](https://i.imgur.com/yIRiAMG.png)

## 2.2. Iniciar el controlador en MATLAB

Ahora, vamos a iniciar el controlador. Para ello, necesitas disponer de los ficheros almacenados en el directorio `drone_matlab` de este repositorio, y en el directorio correspondiente a la versión de MATLAB que dispongas (para esta guía, 2022a o 22a). Almacénalos en un directorio de tu máquina donde puedas acceder fácilmente a ellos. Tras esto, iniciar MATLAB 2022a. 

Ahora, desde MATLAB, verás que en el panel lateral izquierdo dispones de un navegador de archivos. Navega desde el hasta el directorio que acabas de crear, de forma que veas los ficheros en el. Dicho directorio dispone de un fichero `drone_control_R22a.slx` que sirve para probar el simulador. Abre dicho fichero (puede tardar un poco, dado que es un fichero Simulink y debe lanzarse el entorno, que es pesado).

Una vez abierto, puedes ver lo siguiente:
![Controlador en Simulink](https://i.imgur.com/b56IKMq.png)

Verás que hay diferentes bloques:
- El bloque `autopilot` es el pioto automático del drone. La parte a desarrollar en Drone Challenge.
- El bloque `pilot` puede usarse para controlar el drone con un joystick. Si dispones de uno, puedes probarlo. Sin embargo, para que el piloto automático funcione, debe estar comentado. Por defecto viene comentado.
- El bloque `drone simulator` se encarga de la comunicación con el drone, enviandole los comandos y recibiendo la telemetría del mismo.

Para comenzar la simulación, primero, desde la barra de herramientas superior, pulsa sobre "Simulation". Despues, en el panel "PREPARE", pulsa sobre "ROS Network", y en la ventana que se abre, escribe la dirección IP que apareció en la consola cuando ejecutamos ROS y Gazebo, en el apartado que indica "ROS MAster (ROS)". Deberás seleccionar "Custom" en "Netword Address" para escribir la dirección IP. Pulsa sobre "Test" para comprobar que tienes conexión con ROS y Gazebo.
![Conexión MATLAB con ROS](https://i.imgur.com/IxcIj4E_d.webp?maxwidth=760&fidelity=grand)

Finalmente, si la conexión es correcta, puedes pulsar sobre "Run", en el apartado "SIMULATE", de la pestaña "SIMULATION" de la barra de herramientas superior de MATLAB. El programa empezará a compilar y, cuando haya finalizado, verás como el drone se alza en el aire. Verás tambien que desde MATLAB se abren algunas ventanas: una con los comandos enviados al drone, otra con la respuesta del mismo y otra con la vista de la cámara _onboard_ del drone. Ya tienes todo lo necesario para comenzar el reto **Drone Challenge**.

![Entorno de simulación 3D donde el drone está recibiendo los comandos por parte de MATLAB](https://i.imgur.com/mgkWoH4.png)

![](https://i.imgur.com/eU1DWCP.png)
