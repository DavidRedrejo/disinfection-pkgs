#!/bin/bash

msj_1="Se ha iniciado el proceso para seleccionar un punto desde el mapa."
msj_2="Doble Click con el botón izquierdo sobre el punto deseado."
msj_3="Por favor, comprueba las coordenadas y presiona [ESPACIO] para guardar las últimas."

gnome-terminal --geometry 67x10+0+0 -e "bash -c 'echo $msj_1;
echo $msj_2;
echo $msj_3;
./set_point_map.py'"

exit 0