#!/bin/bash

# Se conecta al robot por SSH para iniciar los programas 
# que activan el brazo y el controlador de MoveIt
gnome-terminal --geometry 67x10+0-0 -e "bash -c 'sshpass -p R0b0tn1K ssh rbkairos@192.168.1.156 roslaunch disinfection_process arm_control_activation.launch'"

sleep 10

# Se ejecuta el lanzador del proceso completo: 
gnome-terminal --geometry 67x10-50-0 -e "bash -c 'roslaunch disinfection_process start_process.launch'"

exit 0