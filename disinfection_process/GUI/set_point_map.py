#!/usr/bin/env python

import sys
import cv2
import yaml

# Se inicializan las variables
origen = [0, 0, 0]
grid_size = 0
height = 0
width = 0
channels = 0
init_point = (0,0)
clk_object_point = (0,0)
init_point_map = (0,0)
clk_object_point_map = (0,0)


# Ruta y nombre del mapa que se quiere usar
path_map = "/home/david/robot_ws/src/summit_xl_common/summit_xl_localization/maps/"
file_map_yaml = "test_aula2.yaml"

# Ruta y nombre del archivo en el que se guardan las coordenadas
path_obj_point = "/home/david/robot_ws/src/disinfection_process/initial_data/"
filename_obj_point = "map_object_point.txt"
f = open(path_obj_point + filename_obj_point, "w")


# Funcion que se ejecuta cuando se hace click sobre la imagen
def on_click(event, x, y, p1, p2):
    global init_point, clk_object_point, init_point_map, clk_object_point_map, height, width
    # x,y son las coordendas de la posicion del raton, pero se necesita referenciar esa posicion respecto al origen del mapa (el origen desde el que se contruye el mapa)
    
    # Calcular las coordenadas x,y respecto al origen del mapa
    x_map = round(x*grid_size + origen[0],2)
    y_map = round((height - y)*grid_size + origen[1],2)

    # Calcular las coordenadas x,y del origen del mapa
    x_origen = int(round(-origen[0]/grid_size,3))
    y_origen = int(round(height-(-origen[1]/grid_size),3))
    
    # Al hacer DOBLE CLICK con el boton izquierdo se marca la coordenada deseada
    if event == cv2.EVENT_LBUTTONDBLCLK:
        print("Coordenadas del punto",x_map, y_map)
        clk_object_point = (x, y)
        clk_object_point_map = (x_map, y_map)

        img = cv2.imread(path)  # se vuelve a leer la imagen original para asi borrar los puntos anteriores
        # se dibujan los ejes X,Y del origen del mapa
        cv2.arrowedLine(img, (x_origen,y_origen), (x_origen+10, y_origen), (200,0,0),1)
        cv2.arrowedLine(img, (x_origen,y_origen), (x_origen, y_origen-10), (200,0,0),1)
        
        # se dibuja el punto sobre el que se ha hecho doble click, 
        # que es el punto en el que se encuentra el objeto a desinfectar
        cv2.circle(img, clk_object_point, 3, (0,0,200),2)
        cv2.imshow("mapa", img)


if __name__ == '__main__':    
    try:
        # Se obtiene la informacion del yaml del mapa
        with open(path_map + file_map_yaml) as file:
            datos = yaml.safe_load(file)

            for item, doc in datos.items():
                if item == 'origin':
                    origen = doc
                if item == 'resolution':
                    grid_size = doc
                if item == 'image':
                    path =  doc

        img = cv2.imread(path)
        
        if img is None:
            sys.exit("No se ha podido leer la imagen")

        height, width, channels = img.shape

        # Se muestra el mapa y se indica el callback del click del raton
        cv2.namedWindow("mapa", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('mapa', on_click)
        cv2.imshow("mapa", img)


        # Bucle hasta que el usuario presione ESPACIO para salir y escribir las coordenadas en el archivo
        while(1):
            k = cv2.waitKey(33)
            if k==32:   # tacla ESPACIO
                # Se guardan las coordenadas en el archivo
                f.write("%.3f  \t%.3f\n" % (clk_object_point_map[0], clk_object_point_map[1]))
                break
            else:
                continue
        
        cv2.destroyAllWindows()
        f.close()
        pass
    except:
        pass
