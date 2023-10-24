#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 16 15:08:05 2021

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


import sys
sys.path.insert(1, '/home/lino/Documentos/programas_pruebas_varias/modulo_visualizacion') # Ruta a mi visor customizado y módulo de lecturas
import visualizaciones_customizadas_open3d as visor
import lecturas_nubes_de_puntos as lectura


from LECTURAS_segmentacion_cilindrica import lectura_segmentaciones



def creador_superficies(SEGMENTOS,numero_de_superficies_a_crear,factor_repeticion,Numero_puntos_superficie,montana=False,visualizacion=False,voxel_downsampling_size=0):


    # #==========================LECTURA SEGMENTACIONES==============================
    
    # # Primero leemos las segmentaciones hechas a través del algoritmo de voxeliza-
    # # ción cilíndrica:
        
    # ruta_segmentaciones_cilindricas = '/home/lino/Documentos/TESTEO_ZEBGO_Nubes/NDP_zebRevo_xures_julio_21/prueba/arboles_manuales_troncos_eliminados/automatizacion/PLANO_TRONCOS_1.000000_0.250000_DBSCAN_TRONCOS_0.100000_1_DBSCAN_CILINDRO_0.900000_10_UMBRAL_0.026263'
    
    # SEGMENTOS = lectura_segmentaciones(ruta_segmentaciones_cilindricas)
    #==============================================================================
    #------------------------------------------------------------------------------
    
    #------------------------------------------------------------------------------
    #============================CREACIÓN SUPERFICIES==============================
    
    # Vamos a crear ahora un pack de superficies simuladas. Para ello definiremos
    # primero un plano y luego le daremos una rugosidad.
    
    # Para crear el plano nos basaremos en los segmentos que tenemos ya leídos.Ve-
    # remos primero cuál es la dimensión media en el plano XY de la colección de 
    # segmentos y luego tomaremos ese valor como referencia para montar una super-
    # ficie en la que quepan al menos unos cuantos árboles de esas proporciones.
    
    media_x_global = 0
    media_y_global = 0
    
    densidad_media_puntos_global = 0
    
    for i in range(len(SEGMENTOS)):
        
        segmento = SEGMENTOS[i]
    
        puntos_segmento = np.array(segmento.points)
        
        coordenadas_x_segmento = puntos_segmento.take(0,1)
        coordenadas_y_segmento = puntos_segmento.take(1,1)
        
        maxima_distancia_x = np.abs(np.max(coordenadas_x_segmento)-np.min(coordenadas_x_segmento))
        maxima_distancia_y = np.abs(np.max(coordenadas_y_segmento)-np.min(coordenadas_y_segmento))
    
        media_x_global += maxima_distancia_x
        media_y_global += maxima_distancia_y
        
        # Densidad de puntos por metro cuadrado:
        densidad_media_puntos_global += len(puntos_segmento)/(maxima_distancia_x*maxima_distancia_y)
    
    media_x_global = media_x_global/len(SEGMENTOS)
    media_y_global = media_y_global/len(SEGMENTOS)
    
    densidad_media_puntos_global = densidad_media_puntos_global/len(SEGMENTOS)
    
    # Tenemos definido un cuadrado pequeño de (media_x_global x media_y_global).
    # Repetimos ese cuadrado unas cuantas veces y definimos la superficie.
    
    
    eje_x = np.linspace(0,media_x_global*factor_repeticion,int(densidad_media_puntos_global/5))
    eje_y = np.linspace(0,media_y_global*factor_repeticion,int(densidad_media_puntos_global/5))
    eje_z = np.zeros(len(eje_x))
    
    
    # Ya tenemos definido el plano que va a dar pie a todas las superficies artifi-
    # ciales. Ahora lo que hacemos es proporcionarle una rugosidad a cada punto del
    # espacio siguiendo alguna distribución 2D conocida.
    
    # SE ME OCURRE:
    # Puedo escoger varios puntos al azar en el plano XY y en ellos aplicar una
    # altura random para hacer un poco de elevaciones. El resto del espacio lo re-
    # construyo por triangulación de Delaunay o haciendo un fit.
    
    SUPERFICIES = {}
    
    for s in range(numero_de_superficies_a_crear):
    
        N = 400
        
        mesh_x, mesh_y = np.meshgrid(eje_x,eje_y)
        
        xyz = np.zeros((np.size(mesh_x), 3))
        xyz[:, 0] = np.reshape(mesh_x, -1)
        xyz[:, 1] = np.reshape(mesh_y, -1)
        
        
        # PARÓN PARA VISUALIZAR -------------------------------------------------------
        
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(xyz)
        # visor.custom_draw_geometry_with_key_callback(pcd)
        
        #------------------------------------------------------------------------------
        
        
        
        altura_maxima = 10*factor_repeticion
        
        for i in range(N):
            
            indice = np.random.choice(range(len(xyz)))
            
            if montana:
                xyz[indice][2] = np.random.randint(altura_maxima*0.99,altura_maxima)
            else:
                # xyz[indice][2] = np.random.random(1)[0]
                xyz[indice][2] = np.random.randint(altura_maxima)
        
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        # o3d.io.write_point_cloud("../../test_data/sync.ply", pcd)
        
        # PARÓN PARA VISUALIZAR -------------------------------------------------------
        
        # visor.custom_draw_geometry_with_key_callback(pcd)
        
        #------------------------------------------------------------------------------
        
        # Vamos a hacer un fit a esos puntos, y la superficie que reconstruyamos será
        # el suelo que consideraremos.
        # El fit que haremos será del mismo tipo que aquellos que usamos en el paper de
        # rugosidades: polinomial de orden 3
        
        def exponential_cov(x, y, params):
        
            return params[0] * np.exp( -0.5 * params[1] * np.subtract.outer(x, y)**2)
        
        def conditional(x_new, x, y, params):
        
            B = exponential_cov(x_new, x, params)
            C = exponential_cov(x, x, params)
            A = exponential_cov(x_new, x_new, params)
            mu = np.linalg.inv(C).dot(B.T).T.dot(y)
            sigma = A - B.dot(np.linalg.inv(C).dot(B.T))
            return(mu.squeeze(), sigma.squeeze())
           
        ordr = 4  # Orden del polinomio al que quiero ajustar en cada "frame"
        
        def matriz_minimos_cuadrados(x, y, order=ordr):
            """ generate Matrix use with lstsq """
            # Genero una matriz con los valores obtenidos por mínimos cuadrados
            ncolumnas = (order + 1)**2
            G = np.zeros((x.size, ncolumnas))
            ij = itertools.product(range(order+1), range(order+1))
            for k, (i, j) in enumerate(ij):
                G[:, k] = x**i * y**j
            return G
        
        coordenadas_z = xyz.take(0,1)
        coordenadas_x = xyz.take(1,1)
        coordenadas_y = xyz.take(2,1)
        
        
        # Calculo los valores x e y mínimos (los necesitaré más adelante por temas de
        # dimensiones de plots y así):
        x_min = np.min(coordenadas_x)
        x_max = np.max(coordenadas_x)
        y_min = np.min(coordenadas_y)
        y_max = np.max(coordenadas_y)
        
        
        
        
        puntos = []
        for punto in xyz:
            puntos.append(punto[0:3])
            
        puntos = np.array(puntos)    
        
        x, y, z = puntos.T # Hago la traspuesta
        x, y = x - x[0], y - y[0]  # Para mejorar la eficacia
        
        # Creamos la matriz que contiene las regresiones por punto:
        G = matriz_minimos_cuadrados(x, y, ordr)
        # Solve for np.dot(G, m) = z:
        # Quiero saber qué valores de m hacen que G·m = z (es decir, np.dot(G,m)=z)
        m = np.linalg.lstsq(G, z)[0]
        
        
        # Evaluamos en una grid el ajuste que acabamos de hacer...
        nx, ny = Numero_puntos_superficie, Numero_puntos_superficie
        xx, yy = np.meshgrid(np.linspace(x.min(), x.max(), nx),
                              np.linspace(y.min(), y.max(), ny))
        
        
        GG = matriz_minimos_cuadrados(xx.ravel(), yy.ravel(), ordr)
        zz = np.reshape(np.dot(GG, m), xx.shape)
        
        
        
        
        
        # Ploteamos y cruzamos dedos
        # fg, ax = plt.subplots(subplot_kw=dict(projection='3d'))
        # ls = LightSource(270, 45)
        # rgb = ls.shade(zz, cmap=cm.gist_earth, vert_exag=0.1, blend_mode='soft')
        # superficie = ax.plot_surface(xx, yy, zz, rstride=1, cstride=1, facecolors=rgb,
        #                         linewidth=0, antialiased=False, shade=False)
        # ax.plot3D(x, y, z, "o",color='red')
        
        # fg.canvas.draw()
        # plt.show()
        
        
        # Convierto la superficie del fit a una nube de puntos:
        superficie_reconstruida = np.zeros((np.size(xx), 3))
        superficie_reconstruida[:, 0] = np.reshape(xx, -1)
        superficie_reconstruida[:, 1] = np.reshape(yy, -1)
        superficie_reconstruida[:, 2] = np.reshape(zz, -1)
        
        desnivel_maximo = np.max(superficie_reconstruida.take(2,1))-np.min(superficie_reconstruida.take(2,1))
        print('Desnivel máximo: ',desnivel_maximo,' (m)')
        
        
        # Ya tenemos la superficie creada, voy a repetirla unas cuantas veces hacia
        # abajo para generar algo de profundidad:
        
        numero_capas = 3
        for i in range(numero_capas):
            aux = np.copy(superficie_reconstruida)
            profundidad = np.zeros(3)
            profundidad[2] = 0.05
            aux = aux + profundidad
            superficie_reconstruida = np.concatenate((superficie_reconstruida,aux))
        
        if voxel_downsampling_size < 1000:
            # Vamos a hacer voxel_downsampling:
            pcd2 = o3d.geometry.PointCloud()
            pcd2.points = o3d.utility.Vector3dVector(superficie_reconstruida)
            pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
            superficie_reconstruida = np.array(pcd2.points)
        
        # Ahora le añadimos algo de ruido gaussiano a cada punto para hacerlo más real:
        ruido_x = np.random.normal(0,eje_x[2]-eje_x[1],len(superficie_reconstruida))
        ruido_y = np.random.normal(0,eje_y[2]-eje_y[1],len(superficie_reconstruida))
        ruido_z = np.random.normal(0,0.2,len(superficie_reconstruida))
        
        superficie_reconstruida[:, 0] = np.reshape(superficie_reconstruida.take(0,1) + ruido_x, -1)
        superficie_reconstruida[:, 1] = np.reshape(superficie_reconstruida.take(1,1) + ruido_y, -1)
        superficie_reconstruida[:, 2] = np.reshape(superficie_reconstruida.take(2,1) + ruido_z, -1)
        
        
        
        
        
        
        
        
        
        
        
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(superficie_reconstruida)
        # o3d.io.write_point_cloud("../../test_data/sync.ply", pcd)
        
        # Vamos a pintar el suelo con algunos colores:
        lista_colores_suelo = [np.array([51/255,51/255,0/255]),
                         np.array([102/255,102/255,0/255]),
                         np.array([153/255,153/255,0/255]),
                         np.array([204/255,204/255,0/255]),
                         np.array([255/255,255/255,0/255]),
                         np.array([102/255,51/255,0/255]),
                         np.array([153/255,76/255,0/255]),
                         np.array([204/255,102/255,0/255])]
        
        colores_suelo = np.zeros((len(superficie_reconstruida),3))
        for e in range(len(colores_suelo)):
            colores_suelo[e] = lista_colores_suelo[np.random.choice(len(lista_colores_suelo))]
        
        pcd2.colors = o3d.utility.Vector3dVector(colores_suelo)
        
        if visualizacion:
        
            # PARÓN PARA VISUALIZAR -------------------------------------------------------
            
            visor.custom_draw_geometry_with_key_callback(pcd2)
            
            #------------------------------------------------------------------------------



        if voxel_downsampling_size < 1000000:
            # nube_SEGMENTOS_ARTIFICIALES = o3d.geometry.PointCloud()
            # for item in SEGMENTOS_ARTIFICIALES:
            #     nube_SEGMENTOS_ARTIFICIALES += item
            puntos_superficie = np.array(pcd2.points)
            colores_superficie = np.array(pcd2.colors)
            try:
                indices_downsampling = np.random.choice(puntos_superficie.shape[0], 20000, replace=False)
            except ValueError:
                indices_downsampling = np.random.choice(puntos_superficie.shape[0], 20000, replace=True)
            puntos_superficie = puntos_superficie[indices_downsampling]
            colores_superficie = colores_superficie[indices_downsampling]
            pcd2 = o3d.geometry.PointCloud()
            pcd2.points = o3d.utility.Vector3dVector(puntos_superficie)
            pcd2.colors = o3d.utility.Vector3dVector(colores_superficie)







        SUPERFICIES[s] = pcd2

    return SUPERFICIES

















