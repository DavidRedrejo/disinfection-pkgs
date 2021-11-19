#!/bin/bash

# Se conecta al robot por SSH para iniciar los programas 
# que activan el brazo y el controlador de MoveIt
gnome-terminal --geometry 67x10+0-0 -e "bash -c 'sshpass -p R0b0tn1K ssh rbkairos@192.168.1.156 roslaunch disinfection_process arm_control_activation.launch'"

sleep 10

# Se ejecuta el lanzador del proceso completo: 
gnome-terminal --geometry 67x10-50-0 -e "bash -c 'roslaunch disinfection_process start_process.launch'"



# msj_1="Se ha iniciado el proceso para seleccionar un punto desde el mapa."
# msj_2="Por favor, comprueba las coordenadas y presiona la tecla [ESPACIO] para guardar las últimas"

# gnome-terminal --geometry 67x10+0+0 -e "bash -c 'echo $msj_1;
# echo $msj_2;
# rosrun disinfection_process set_point_map.py'"
#exec $SHELL

# echo "Se ha iniciado el proceso para seleccionar un punto desde el mapa"
# echo "Por favor, comprueba las coordenadas y presiona la tecla [ESPACIO] para guardar las últimas"

# ls -la
# x-terminal-emulator -geometry 60x15+0+0 -e rviz

# x-terminal-emulator -geometry 60x15+0+0 -e 
# rosrun disinfection_process set_point_map.py #&
# processID=$!

# echo "PID: $processID"

# wait $processID


exit 0