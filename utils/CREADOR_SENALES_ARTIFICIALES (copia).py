#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 18 14:27:36 2021

@author: lino
"""

import copy
import os
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import trimesh
from scipy.spatial import Delaunay
import itertools
from matplotlib import cm
from matplotlib.colors import LightSource
import math

import sys
sys.path.insert(1, '/home/lino/Documentos/programas_pruebas_varias/modulo_visualizacion') # Ruta a mi visor customizado y módulo de lecturas
import visualizaciones_customizadas_open3d as visor
import lecturas_nubes_de_puntos as lectura


SENHAL = {}

Radio_poste = 0.15
Radio_larguero = 0.1

altura_poste = 7
altura_carretera = 0
altura_larguero = 4.5

mitad = True




alturas = np.linspace(altura_carretera,altura_poste,50)
angulos = np.linspace(0,2*np.pi,100)

POSTE_1 = []

for i in range(len(alturas)):
    for j in range(len(angulos)):
        x = Radio_poste*np.cos(angulos[j])
        y = Radio_poste*np.sin(angulos[j])
        POSTE_1.append([x,y,alturas[i]])
    
POSTE_1 = np.array(POSTE_1)    

# Ahora le añadimos algo de ruido gaussiano a cada punto para hacerlo más real:
ruido_x = np.random.normal(0,0.005,len(POSTE_1.take(0,1)))
ruido_y = np.random.normal(0,0.005,len(POSTE_1.take(0,1)))
ruido_z = np.random.normal(0,0.08,len(POSTE_1.take(0,1)[1:-2]))

POSTE_1[:, 0] = np.reshape(POSTE_1.take(0,1) + ruido_x, -1)
POSTE_1[:, 1] = np.reshape(POSTE_1.take(1,1) + ruido_y, -1)
POSTE_1[1:-2, 2] = np.reshape(POSTE_1.take(2,1)[1:-2] + ruido_z, -1)

poste_1 = o3d.geometry.PointCloud()
poste_1.points = o3d.utility.Vector3dVector(POSTE_1)
poste_1.paint_uniform_color(np.array([160/255.,160/255.,160/255.]))

SENHAL[0] = poste_1

#------------------------------------------------------------------------------
# PARÓN PARA VISUALIZAR:
# visor.custom_draw_geometry_with_key_callback(SENHAL)
#------------------------------------------------------------------------------

anchura_larguero = 10

poste_2 = copy.deepcopy(poste_1).translate((anchura_larguero, 0, 0))

SENHAL[1] = poste_2

#------------------------------------------------------------------------------
# PARÓN PARA VISUALIZAR:
# visor.custom_draw_geometry_with_key_callback(SENHAL,segmentado=True,pcd2=SENHAL[0])
#------------------------------------------------------------------------------

longitudes = np.linspace(altura_carretera,anchura_larguero,50)
angulos = np.linspace(0,2*np.pi,100)

POSTE_3 = []

for i in range(len(longitudes)):
    for j in range(len(angulos)):
        y = Radio_larguero*np.cos(angulos[j])
        z = Radio_larguero*np.sin(angulos[j])
        POSTE_3.append([longitudes[i],y,z+altura_larguero])
    
POSTE_3 = np.array(POSTE_3)    

# Ahora le añadimos algo de ruido gaussiano a cada punto para hacerlo más real:
ruido_x = np.random.normal(0,0.05,len(POSTE_3.take(0,1)))
ruido_y = np.random.normal(0,0.005,len(POSTE_3.take(0,1)))
ruido_z = np.random.normal(0,0.005,len(POSTE_3.take(0,1)[1:-2]))

POSTE_3[:, 0] = np.reshape(POSTE_3.take(0,1) + ruido_x, -1)
POSTE_3[:, 1] = np.reshape(POSTE_3.take(1,1) + ruido_y, -1)
POSTE_3[1:-2, 2] = np.reshape(POSTE_3.take(2,1)[1:-2] + ruido_z, -1)

poste_3 = o3d.geometry.PointCloud()
poste_3.points = o3d.utility.Vector3dVector(POSTE_3)
poste_3.paint_uniform_color(np.array([160/255.,160/255.,160/255.]))

SENHAL[2] = poste_3


#------------------------------------------------------------------------------
# PARÓN PARA VISUALIZAR:
# visor.custom_draw_geometry_with_key_callback(SENHAL,segmentado=True,pcd2=SENHAL[0])
#------------------------------------------------------------------------------

poste_4 = copy.deepcopy(poste_3).translate((0, 0, altura_larguero/2.))

SENHAL[3] = poste_4

#------------------------------------------------------------------------------
# PARÓN PARA VISUALIZAR:
# visor.custom_draw_geometry_with_key_callback(SENHAL,segmentado=True,pcd2=SENHAL[0])
#------------------------------------------------------------------------------


if mitad == False:
    eje_x = np.linspace(anchura_larguero/10.,anchura_larguero-(anchura_larguero/10.),70)
    eje_y = np.full((1,len(eje_x)),np.mean(POSTE_3.take(1,1))+Radio_larguero)[0]
    eje_z = np.linspace(altura_larguero-(altura_larguero/5.),altura_larguero+(altura_larguero/1.5),50)

    CARTEL = []

    for i in range(len(eje_x)):
        for j in range(len(eje_y)):
            for k in range(len(eje_z)):
                punto = [eje_x[i],eje_y[j],eje_z[k]]
                
                # Ahora le añadimos algo de ruido gaussiano a cada punto para hacerlo más real:
                ruido_x = np.random.normal(0,0.05,1)[0]
                ruido_y = np.random.normal(0,0.005,1)[0]
                ruido_z = np.random.normal(0,0.01,1)[0]
                
                punto[0] = punto[0] + ruido_x
                punto[1] = punto[1] + ruido_y
                punto[2] = punto[2] + ruido_z
                
                CARTEL.append(punto)

    CARTEL = np.array(CARTEL)

    cartel = o3d.geometry.PointCloud()
    cartel.points = o3d.utility.Vector3dVector(CARTEL)
    cartel.paint_uniform_color(np.array([0,0,1]))

    cartel_aux = copy.deepcopy(cartel).translate((0, 0.005, 0))
    cartel = cartel + cartel_aux
    
    cartel_aux = copy.deepcopy(cartel).translate((0, 0.005, 0))
    cartel = cartel + cartel_aux

    SENHAL[4] = cartel
    
else:
    eje_x_1 = np.linspace(anchura_larguero/20.,(anchura_larguero-(anchura_larguero/10.))*0.6,70)
    eje_y_1 = np.full((1,len(eje_x_1)),np.mean(POSTE_3.take(1,1))+Radio_larguero)[0]
    eje_z_1 = np.linspace(altura_larguero-(altura_larguero/5.),altura_larguero+(altura_larguero/1.5),50)

    CARTEL_1 = []

    for i in range(len(eje_x_1)):
        for j in range(len(eje_y_1)):
            for k in range(len(eje_z_1)):
                punto = [eje_x_1[i],eje_y_1[j],eje_z_1[k]]
                
                # Ahora le añadimos algo de ruido gaussiano a cada punto para hacerlo más real:
                ruido_x = np.random.normal(0,0.04,1)[0]
                ruido_y = np.random.normal(0,0.005,1)[0]
                ruido_z = np.random.normal(0,0.02,1)[0]
                
                punto[0] = punto[0] + ruido_x
                punto[1] = punto[1] + ruido_y
                punto[2] = punto[2] + ruido_z
                
                CARTEL_1.append(punto)

    CARTEL_1 = np.array(CARTEL_1)

    cartel_1 = o3d.geometry.PointCloud()
    cartel_1.points = o3d.utility.Vector3dVector(CARTEL_1)
    cartel_1.paint_uniform_color(np.array([0,0,1]))    

    cartel_aux = copy.deepcopy(cartel_1).translate((0, 0.005, 0))
    cartel_1 = cartel_1 + cartel_aux

    SENHAL[4] = cartel_1


    
    cartel_2 = copy.deepcopy(cartel_1).translate(((anchura_larguero-(anchura_larguero/20.))*0.6, 0, 0))

    CARTEL_2 = np.array(cartel_2.points)
    indices_eliminar = np.where(CARTEL_2.take(0,1) < (anchura_larguero-(anchura_larguero/20.)))[0]
    CARTEL_2 = CARTEL_2[indices_eliminar]
    cartel_2 = o3d.geometry.PointCloud()
    cartel_2.points = o3d.utility.Vector3dVector(CARTEL_2)
    cartel_2.paint_uniform_color(np.array([0,0,204/255.]))

    cartel_aux = copy.deepcopy(cartel_2).translate((0, 0.005, 0))
    cartel_2 = cartel_2 + cartel_aux
    cartel_aux = copy.deepcopy(cartel_2).translate((0, 0.005, 0))
    cartel_2 = cartel_2 + cartel_aux
    cartel_aux = copy.deepcopy(cartel_2).translate((0, 0.005, 0))
    cartel_2 = cartel_2 + cartel_aux

    SENHAL[5] = cartel_2

#------------------------------------------------------------------------------
# PARÓN PARA VISUALIZAR:
visor.custom_draw_geometry_with_key_callback(SENHAL,segmentado=True,pcd2=SENHAL[0])
#------------------------------------------------------------------------------












