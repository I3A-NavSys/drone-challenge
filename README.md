![Drone Challenge Logo](https://blog.uclm.es/esiidronechallenge/files/2017/03/cropped-dc.logo_.png)

Este repositorio almacena los ficheros necesarios para la configuración del entorno necesario para ejecutar el simulador para la Drone Challenge. El simulador está formado por distintas piezas software, que deben ser instaladas y configuradas en una máquina real o máquina virtual. Podemos dividir el simulador en dos grandes partes:

- **Controlador del dron:** Es el encargado de decidir qué acciones (moverse, ascender, girar, etc.) debe realizar el drone. Recibe la visualización de la cámara incorporada por el drone y toma la decisión de qué realizar en cada momento. Este controlador ha sido creado en **Matlab**, de _Mathworks_, con algunas herramientas necesarias que son distribuidas por la propia Mathworks. Comentaremos más tarde el proceso de instalación y las dependencias necesarias.
- **Simulador 3D**: Si bien el controlador decide qué realiza el drone, necesitamos un entorno de simulación 3D donde el drone vuele y podamos ver la respuesta del mismo a los comandos del controlador. Para ello se hace uso de **ROS (_Robot Operating System_) y Gazebo**. Gazebo nos permitirá la visualización de drone a través de un simulador 3D, mientras que ROS permitirá implementar el modelo dinámico y la respuesta del drone a los comandos del controlador.





