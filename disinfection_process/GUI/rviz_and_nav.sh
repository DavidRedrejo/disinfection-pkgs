#!/bin/bash

# Se ejecuta el lanzador del RVIZ desde el portatil para poder visualizarlo
gnome-terminal --geometry 67x10+0-0 -e "bash -c 'roslaunch disinfection_process rviz_comp.launch'"

# Se lanzan todos los procesos para la navegacion: mapa, localizacion y navegacion
gnome-terminal --geometry 67x10+0-0 -e "bash -c 'sshpass -p R0b0tn1K ssh rbkairos@192.168.1.156 roslaunch disinfection_process nav_complete.launch'"

exit 0