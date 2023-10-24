#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 17 14:35:08 2022

@author: lino
"""

# MODIFICACION NUBES ARTIFICIALES

import numpy as np
import open3d as o3d

# Lo que voy a hacer es recorrer todas las nubes de un dataset y modificar al-
# gunas cosas:
    
# 1) Poner barreras quitamiedos en los laterales de las medianas.
# 2) Eliminar las barreras tipo jersey.
# 3) Poner algún árbol en la mediana.
# 4) Escoger una de las dos calzadas y hacer downsamplings para emular el paso
#    del escáner en la calzada opuesta.


#------------------------------------------------------------------------------
# Paso 0: Primero poner bien PATHS, coger archivos, etc, etc...

import os
import shutil

ruta_DATASET = '/home/lino/Documentos/programas_pruebas_varias/segmentacion_python/segmentacion_bosques/aumentacion_de_datos/Nubes_artificiales_generadas/nubes_buenas/autopista_bermas_senhales'
ruta_TRAIN = ruta_DATASET+'/TRAIN'
ruta_TEST = ruta_DATASET+'/TEST'

numero_nubes_TRAIN = len(os.listdir(ruta_TRAIN))
numero_nubes_TEST = len(os.listdir(ruta_TEST))

os.chdir(ruta_DATASET)

for archivo in os.listdir(os.getcwd()):
    if archivo.startswith('Nube_artificial_'):
        os.chdir(archivo)
        ruta_nube_i_esima = os.getcwd()
        os.chdir('numpy_arrays')
        os.mkdir('nube_modificada')
        
        for numpy_array in os.listdir(os.getcwd()):
            if numpy_array.endswith('.npy'):
                shutil.copyfile(os.getcwd()+'/'+numpy_array,
                                os.getcwd()+'/nube_modificada/'+numpy_array)
                
            
            else:
                pass
        
        
        
        
    else:
        pass















