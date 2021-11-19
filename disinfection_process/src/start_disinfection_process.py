#! /usr/bin/env python

""" ESTE ARCHIVO COMIENZA EL PROCESO DE DESINFECION COMPLETO """

import os
import rospy
import sys
import yaml
import rospkg

from disinfection_class import DisinfectionProcess


rospy.init_node('disinfection_process_node', log_level=rospy.DEBUG)

# Ruta del fichero que contiene los parametros iniciales:
# path = "/home/david/robot_ws/src/disinfection_process/initial_data/"
rospack = rospkg.RosPack()
folder_path = os.path.join(rospack.get_path('disinfection_process'), "initial_data")
data_file_path = os.path.join(folder_path, "data.yaml")


# Lectura de los datos del fichero para poder usarlos en el constructor del DisinfectionProcess
coord = None
gap = None
radius = None

with open(data_file_path, "r") as file:
    
    data = yaml.safe_load(file)
    
    for item, value in data.items():
        
        if item == "coord":
            coord = value

        if item == "gap_robot_object":
            gap = value
        
        if item == "radius_circular_traj":
            radius = value   
    
    # print(coord, type(coord))
    # print(radius, type(radius))
    # print(gap, type(gap))

desinfeccion = DisinfectionProcess(coord, gap, radius)
desinfeccion.start_process()

