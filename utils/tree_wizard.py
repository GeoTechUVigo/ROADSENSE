#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 20 10:58:25 2021

@author: lino
"""

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




def tree_generator(SEGMENTOS,numero_de_transformaciones_por_arbol,visualizacion=False,voxel_downsampling_size=1000000):


# #==========================LECTURA SEGMENTACIONES==============================

# # Primero leemos las segmentaciones hechas a través del algoritmo de voxeliza-
# # ción cilíndrica:
    
# ruta_segmentaciones_cilindricas = '/home/lino/Documentos/TESTEO_ZEBGO_Nubes/NDP_zebRevo_xures_julio_21/prueba/arboles_manuales_troncos_eliminados/automatizacion/PLANO_TRONCOS_1.000000_0.250000_DBSCAN_TRONCOS_0.100000_1_DBSCAN_CILINDRO_0.900000_10_UMBRAL_0.026263'

# SEGMENTOS = lectura_segmentaciones(ruta_segmentaciones_cilindricas)
# #==============================================================================
# #------------------------------------------------------------------------------

# #------------------------------------------------------------------------------
# #============================CREACIÓN ÁRBOLES==================================

# # Una vez leídas las segmentaciones, vamos a coger y crear nuevos árboles a
# # partir de ellas. Para eso iremos iterando sobre cada segmentación y le apli-
# # caremos algunas transformaciones, como añadir ruído y transformaciones euclí-
# # deas afines (rotaciones, espejos, etc...).

    def rotar_punto(origen, punto, angulo):
    
        # Ángulo en radianes
        
        ox, oy = origen
        px, py = punto[0],punto[1]
    
        qx = ox + math.cos(angulo) * (px - ox) - math.sin(angulo) * (py - oy)
        qy = oy + math.sin(angulo) * (px - ox) + math.cos(angulo) * (py - oy)
        return np.array([qx, qy,punto[2]])
    
    SEGMENTOS_ARTIFICIALES = {}
    
    
    contador_arboles = 0
    for j in range(numero_de_transformaciones_por_arbol):
        for i in range(len(SEGMENTOS)):
        
            segmento = SEGMENTOS[i]
            
            
            
            # Todos los árboles que vayamos a crear se verán sometidos a, como mínimo, una
            # rotación en el plano XY. Para ello calculamos primero su eje de simetría:
                
                
            cgx = (1/len(segmento.points))*(np.sum(np.array(segmento.points).take(0,1)))
            cgy = (1/len(segmento.points))*(np.sum(np.array(segmento.points).take(1,1)))
            #cgz = (1/len(segmento.points))*(np.sum(np.array(segmento.points).take(2,1)))
            
            centro_de_masas = [cgx,cgy]
            
            puntos_rotados = []
            
            angulo = np.random.random() # En radianes lo cogemos
            
            for punto in segmento.points:
                puntos_rotados.append(rotar_punto(centro_de_masas, punto, angulo))
            
            puntos_rotados = np.array(puntos_rotados)
            
            segmento_rotado = o3d.geometry.PointCloud()
            segmento_rotado.points = o3d.utility.Vector3dVector(puntos_rotados)
            
            
            # Ahora le vamos a meter algo de ruido (pero sólo a los puntos que
            # estén por encima de una determinada altura).
            
            dispersion = np.mean(segmento_rotado.compute_point_cloud_distance(segmento))
            
            
            altura_media = np.mean(puntos_rotados[:,2])
            
            indices = np.where(puntos_rotados[:,2]>altura_media)[0]
        
            
            ruido_x = np.random.normal(0,dispersion,len(indices))
            ruido_y = np.random.normal(0,dispersion,len(indices))
            ruido_z = np.random.normal(0,0.5,len(indices))
            
            # print(indices)
            # print()
            # print(puntos_rotados)
            
            puntos_rotados[indices, 0] = np.reshape(puntos_rotados[indices, 0] + ruido_x, -1)
            puntos_rotados[indices, 1] = np.reshape(puntos_rotados[indices, 1] + ruido_y, -1)
            puntos_rotados[indices, 2] = np.reshape(puntos_rotados[indices, 2] + ruido_z, -1)
            
            
            # Volvemos a definir el nuevo árbol:
            arbol_artificial = o3d.geometry.PointCloud()
            arbol_artificial.points = o3d.utility.Vector3dVector(puntos_rotados)
            
            # Lo pintamos con un color:
            lista_colores_arbol = [np.array([0/255,51/255,0/255]),
                             np.array([0/255,102/255,0/255]),
                             np.array([0/255,153/255,0/255]),
                             np.array([0/255,204/255,0/255]),
                             np.array([0/255,255/255,0/255]),
                             np.array([51/255,255/255,51/255]),
                             np.array([102/255,255/255,102/255])]
            
            colores_arbol = np.zeros((len(puntos_rotados),3))
            for e in range(len(colores_arbol)):
                colores_arbol[e] = lista_colores_arbol[np.random.choice(len(lista_colores_arbol))]
            
            arbol_artificial.colors = o3d.utility.Vector3dVector(colores_arbol)
            
            
            
            
            # El False se lo pongo para designar que no ha sido seleccionado to-
            # davía:
            SEGMENTOS_ARTIFICIALES[contador_arboles] = [arbol_artificial,False]
            contador_arboles += 1
    
            if visualizacion:
            
                # PARÓN PARA VISUALIZAR -------------------------------------------------------
                
                visor.custom_draw_geometry_with_key_callback(arbol_artificial)
                
                #------------------------------------------------------------------------------
    
    return SEGMENTOS_ARTIFICIALES


