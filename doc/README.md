El objetivo de este documento es la explicación, con alto nivel de detalle, del funcionamiento del simulador, originalmente creado para la Drone Challenge. Este simulador está compuesto por partes de diferentes sistemas, lo que, sin una documentación especifica, dificulta su funcionamiento.

Los componentes que lo componen son los siguientes:
- ROS (Robot Operating System) provee la capa de transporte de mensajes entre el software de control y el software de simulación.
- Gazebo es usado para mostrar el estado de los UAS, es decir, recibe la información de velocidad comandada por el control a través de la capa de transporte de información de ROS y modifica el estado el modelo del UAS simulado.

Ambas partes, ROS y Gazebo, no se comunican por defecto entre ellos, lo que obliga a usar un plugin creado específicamente para llevar a cabo esta tarea. Sendo plugin se encarga de recibir la información enviada por el software de control a través de un tópico de ROS y almacenar dicha información en el modelo.
A la vez, el plugin dispone del software necesario para traducir los comandos del software de control en fuerzas aplicadas en los rotores del UAS. El resto de físicas son ejecutadas y simuladas por Gazebo de forma automática.

# Despliegue del entorno
Para ejecutar la simulación es necesario que ambos componentes estén en ejecución. Para simplificar esta tarea, todo el entorno de simulación se puede ejecutar de forma automatizada con un par de comandos, con la ayuda de ficheros de lanzamiento y ejecución. Vemos cómo se lleva a cabo este proceso:

1. Para levantar el entorno se usa el comando `./lauch/round1.sh` (asumiendo que estamos en el directorio drone_ros, accesible con `roscd drone_ros`). Este fichero ejecutado es, simplemente, un fichero que contiene comandos shell que obtienen la dirección IP de la máquina actual y lanza ROS con el paquete `drone-ros`y el fichero de lanzamiento de ROS `launch/round1.launch`.

2. ROS carga el paquete comentado, y lee el fichero de lanzamiento introducido. Dicho documento indica qué otros ficheros, paquetes o plugins hay que ejecutar para lanzar el entorno. Este fichero `launch/launch1.launch`, a su vez, incluye otro fichero de lanzamiento `/opt/ros/noetic/gazebo_ros/launch/empty_world.launch`. Este fichero se encarga de lanzar tanto el servidor como el cliente de Gazebo, así como de cargar el fichero de modelos del entorno simulado, que se ha pasado como argumento. Este fichero es `world/round1.world`, que detalla, en XML, la posición y propiedades de cada elemento.

3. Con ROS, Gazebo Server y Gazebo Client en ejecución, y el modelo del entorno cargado, con loos objetos en él, solo ha de cargarse el plugin descrito anteriormente. Este plugin es cargado a través del documento `world/round1.world`. En uno de los modelos, se establece como dependencia dicho plugin, disponible en `plugins/model_push.cc` como código fuente y `plugins/build/libmodel_push.so` como código compilado.

[![](https://mermaid.ink/img/pako:eNpdz7kKwzAMANBfCZpz0NVDoUn6Be0Wh2JsNzb1hQ9KCfn3ujmWapLEE5JmoJZxQDB54kRx77EpclwGRZKhovE2GXaqgxiLqjoX7V9_q8ZtqF1JN7ytV-wQa7GDbgX94FSapAmNzqvVw6UgahqygRI095pIlg-afzMYouCaY0A5ZcS_MGCzZJccI5FfmYzWA3oSFXgJJEV7-xgKKPrED9RLkp_Tu1q-arNQzg)](https://mermaid-js.github.io/mermaid-live-editor/edit#pako:eNpdz7kKwzAMANBfCZpz0NVDoUn6Be0Wh2JsNzb1hQ9KCfn3ujmWapLEE5JmoJZxQDB54kRx77EpclwGRZKhovE2GXaqgxiLqjoX7V9_q8ZtqF1JN7ytV-wQa7GDbgX94FSapAmNzqvVw6UgahqygRI095pIlg-afzMYouCaY0A5ZcS_MGCzZJccI5FfmYzWA3oSFXgJJEV7-xgKKPrED9RLkp_Tu1q-arNQzg)