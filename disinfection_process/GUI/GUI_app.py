#!/usr/bin/env python3

import sys
import yaml
import time

from os import system
from PyQt5 import uic
from PyQt5.QtWidgets import QMainWindow, QApplication

import subprocess

class ejemplo(QMainWindow):

    def __init__(self):
        super().__init__()
        uic.loadUi("visual_form.ui", self)

        system('clear')

        subprocess.check_call('./rviz_and_nav.sh')
        print("--    Se inician los nodos para la navegación y RVIZ para visualizar el proceso")
        print("--    Comprueba en RVIZ la localización del robot sobre el mapa.\n"+
        "Si es necesario corregir la localización y la posición estimada usa la herramienta '2D Pose Estimate'")

        self.setWindowTitle("PROCESO DE DESINFECCIÓN")

        self.line_coord_x.setEnabled(False)
        self.line_coord_y.setEnabled(False)
        self.boton_mapa.setEnabled(False)
        self.boton_desinfecion.setEnabled(False)
        self.boton_actualizar_coord.setEnabled(False)
        self.radio_manual.setChecked(False)
        self.radio_mapa.setChecked(False)

        self.boton_mapa.clicked.connect(self.fn_lanzar_mapa)
        self.boton_desinfecion.clicked.connect(self.fn_lanzar_proceso)
        self.boton_actualizar_coord.clicked.connect(self.cargar_coordenadas)

        self.radio_manual.clicked.connect(self.seleccion_manual)
        self.radio_mapa.clicked.connect(self.seleccion_mapa)


    def fn_lanzar_mapa(self):
        system('clear')
        
        subprocess.check_call('./map_point.sh')
        print("--    Se ha abierto otra ventana para seleccionar el punto sobre el mapa.")
        print("Por favor, selecciona el punto sobre el mapa haciendo DOBLE CLICK con el botón izquierdo" +
        "y presiona [ESPACIO] para guardar las coordenadas.")
        print("\n**** IMPORTANTE! **** \n  Al terminar, actualiza las coordenadas pulsando sobre el botón con el mismo nombre.")
        

    def seleccion_manual(self):
        system('clear')

        self.boton_mapa.setEnabled(False)
        self.boton_actualizar_coord.setEnabled(False)
        self.boton_actualizar_coord.setVisible(False)

        self.line_coord_x.setEnabled(True)
        self.line_coord_y.setEnabled(True)

        temp_x = self.line_coord_x.text()
        temp_y = self.line_coord_y.text()

        if temp_x=="" or temp_y=="":
            self.boton_desinfecion.setEnabled(False)
        else:
            self.boton_desinfecion.setEnabled(True)

        print("--    [introducir coordenadas manualmente]    --" +
        "\n  Introduce las coordenadas del punto o modifica las mostradas en los campos X Y Z")

        
    def seleccion_mapa(self):
        system('clear')

        self.boton_mapa.setEnabled(True)
        self.boton_actualizar_coord.setEnabled(True)
        self.boton_actualizar_coord.setVisible(True)

        self.line_coord_x.setEnabled(False)
        self.line_coord_y.setEnabled(False)

        print("--    [seleccionar coordenadas en el mapa]    --" +
        "\n  1)  Pulsa sobre el boton para abrir el mapa y seleccionar el punto " +
        "\n  2)  Pulsa sobre el botón para actualiza las coordenadas")

        # self.cargar_coordenadas()

    def leer_archivo(self):
        with open(folder_path + coord_file, "r") as archivo_coord:
            for linea in archivo_coord:
                self.x_obj,self.y_obj = linea.split()
                print("--    Se han leido las coordenadas del fichero    --")
                print("x: " + self.x_obj)
                print("y: " + self.y_obj)

    def escribir_datos(self):
        obj_coord = [float(self.x_obj), float(self.y_obj), float(self.z_obj)]

        data = { 
        "coord": obj_coord, 
        "gap_robot_object": self.gap,
        "radius_circular_traj": self.radius }

        print("Se escriben los datos en el fichero")

        with open(folder_path + data_file, "w") as file:
            yaml.dump(data, file)



    def cargar_coordenadas(self):
        self.leer_archivo()
        self.line_coord_x.setText(self.x_obj)
        self.line_coord_y.setText(self.y_obj)

        self.boton_desinfecion.setEnabled(True)
        

    def fn_lanzar_proceso(self):

        # comprobar si los valores x e y son distintos de "" para poder seguir, si no, la funcion salta
        temp_x = self.line_coord_x.text()
        temp_y = self.line_coord_y.text()

        # comprobar si X o Y sin cadenas vacias:
        if temp_x=="" or temp_y=="":
            print("--    ERROR.  Los campos de X e Y no pueden estar vacíos.")
            print("  Escribe el valor deseado en el modo manual, o actualiza los valores si se selecciona el mapa.")
            return
        
        # comprobar si X o Y tienen , en lugar de .
        if "," in temp_x or "," in temp_y:
            print("--    ERROR.  La separación de decimales se debe escribir con '.', no ','")
            return

        # comprobar si X o Y contienen letras
        if temp_x.upper().isupper() or temp_y.upper().isupper():
            print("--    ERROR.  Los campos de X e Y no pueden contener caracteres del alfabeto.")
            return


        self.x_obj = float(self.line_coord_x.text())
        self.y_obj = float(self.line_coord_y.text())
        self.z_obj = float(self.line_coord_z.value())

        self.gap = round(self.line_gap.value(),2)
        self.radius = round(self.line_radius.value(),2)

        self.escribir_datos()

        self.boton_desinfecion.setEnabled(False)
        time.sleep(3)

        subprocess.check_call('./start_process.sh')


        


if __name__=='__main__':

    folder_path = "/home/david/robot_ws/src/disinfection_process/initial_data/"
    coord_file = "map_object_point.txt"
    data_file = "data.yaml"

    app = QApplication(sys.argv)
    gui = ejemplo()
    gui.show()
    sys.exit(app.exec_())