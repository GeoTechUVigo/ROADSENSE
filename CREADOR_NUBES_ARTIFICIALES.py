#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 20 10:43:27 2021

@author: lino
"""

def main():

    import os
    import open3d as o3d
    import numpy as np
    import matplotlib.pyplot as plt
    import trimesh
    from scipy.spatial import Delaunay
    import itertools
    from matplotlib import cm
    from matplotlib.colors import LightSource
    import time
    import copy
    import laspy
    import pathlib
    
    # Relevant paths and files:
    MAIN_PATH = str(pathlib.Path(__file__).parent.resolve())
    CONFIG_PATH = MAIN_PATH+'/config'
    config_file = 'forest_road_backbone.txt'
    UTILS_PATH = MAIN_PATH+'/utils'
    DATA_PATH = MAIN_PATH+'/data'
    
    import sys
    sys.path.insert(1, CONFIG_PATH)
    sys.path.insert(1, UTILS_PATH)
    sys.path.insert(1, DATA_PATH)
    
    
    from reading_trees import read_segments
    from tree_wizard import tree_generator
    from DTM_road_wizard import DTM_road_generator
    from cross_section import highway_vertical_pumping, national_vertical_pumping
    
    # from CREADOR_SUPERFICIES_ARTIFICIALES import creador_superficies





    # Time stamp:
    initial_instant = time.time()
    
    
    # ========================== READING PARAMETERS ===============================
    
    # Reading the config file, where all parameters and other important settings
    # are placed:
    
    with open(CONFIG_PATH+'/'+config_file) as f:
        config_file_lines = f.readlines()[8:] # Parameter definition starts at line 8
        for i in range(len(config_file_lines)):
            config_file_lines[i] = config_file_lines[i].split('\t')
    
    try:
        seed = int(config_file_lines[0][2])
    except:
        seed = int(time.time())
    
    
    
    road = bool(config_file_lines[1][2])
    specral_mode = bool(config_file_lines[2][2])
    road_type = config_file_lines[3][2]
    tree_path = config_file_lines[4][2]
    number_of_clouds = int(config_file_lines[5][2])
    scale = int(config_file_lines[6][2])
    number_of_trees = int(config_file_lines[7][2])
    number_of_transformations = int(config_file_lines[8][2])
    road_buffer = float(config_file_lines[9][2])
    shoulder_buffer = float(config_file_lines[10][2])
    berm_buffer = float(config_file_lines[11][2])
    try:
        slope_buffer = float(config_file_lines[12][2])
    except:
        slope_buffer = np.random.randint(2,5)
    noise_DTM = list(np.float_(config_file_lines[13][2].replace('(','').replace(')','').split(',')))
    noise_road = list(np.float_(config_file_lines[14][2].replace('(','').replace(')','').split(',')))
    noise_shoulder = list(np.float_(config_file_lines[15][2].replace('(','').replace(')','').split(',')))
    noise_slope = list(np.float_(config_file_lines[16][2].replace('(','').replace(')','').split(',')))
    noise_berm = list(np.float_(config_file_lines[17][2].replace('(','').replace(')','').split(',')))
    noise_refugee_island = list(np.float_(config_file_lines[18][2].replace('(','').replace(')','').split(',')))
    number_trees_refugee_island = int(config_file_lines[19][2])
    number_points_DTM = int(config_file_lines[20][2])
    vertical_pumping = bool(config_file_lines[21][2])
    
    
    
    
    # ================================ SIMULATION =================================
     
    #contador == cloud_counter
    cloud_counter = 0    
    for i in range(number_of_clouds):
        #========================== READING DATA ==============================
        
        # First, we read all the tree segments:            
        ORIGINAL_SEGMENTS = read_segments(tree_path)
        if ORIGINAL_SEGMENTS == 0:
            print('The path must be filled with all .pcd segments')
            return 'Exit code: %i'%ORIGINAL_SEGMENTS

        end_reading = time.time()
        print('Finished reading of tree segments: %f s'%(end_reading-initial_instant))
            
        #======================== DTM AND ROAD SIMULATION =========================
            
        if road:
            if road_type in ['highway','national']:
                
    
                #**********************************************************************
                # DEBUG PRINCIPAL                             (donde estoy traduciendo)
                #**********************************************************************        
    
                
                SURFACE,ROAD,SLOPE,SHOULDER,SIGNALS,BERMS,REFUGEE_ISLAND,pcd_barriers,pcd_barrier_1,pcd_barrier_2 = DTM_road_generator(
                    road_type = road_type,
                    ORIGINAL_SEGMENTS = ORIGINAL_SEGMENTS,
                    scale = scale,
                    number_points_DTM = number_points_DTM,
                    road_buffer = road_buffer,
                    slope_buffer = slope_buffer,
                    shoulder_buffer = shoulder_buffer,
                    berm_buffer = berm_buffer,
                    noise_DTM = noise_DTM,
                    noise_road = noise_road,
                    noise_slope = noise_slope,
                    noise_shoulder = noise_shoulder,
                    noise_berm = noise_berm,
                    noise_refugee_island = noise_refugee_island
                    )
                
                # Cross section modification (if required on config file):
                if vertical_pumping and road_type == 'highway':
                    SURFACE,ROAD,SLOPE,SHOULDER,SIGNALS,BERMS,REFUGEE_ISLAND,pcd_barriers,pcd_barrier_1,pcd_barrier_2 = highway_vertical_pumping(
                        SURFACE = SURFACE,
                        ROAD = ROAD,
                        SLOPE = SLOPE,
                        SHOULDER = SHOULDER,
                        SIGNALS = SIGNALS,
                        BERMS = BERMS,
                        REFUGEE_ISLAND = REFUGEE_ISLAND,
                        pcd_barriers = pcd_barriers,
                        pcd_barrier_1 = pcd_barrier_1,
                        pcd_barrier_2 = pcd_barrier_2)
                elif vertical_pumping and road_type == 'national':
                    SURFACE,ROAD,SLOPE,SHOULDER,SIGNALS,BERMS,REFUGEE_ISLAND,pcd_barriers,pcd_barrier_1,pcd_barrier_2,SIGNALS = national_vertical_pumping(
                        SURFACE = SURFACE,
                        ROAD = ROAD,
                        SLOPE = SLOPE,
                        SHOULDER = SHOULDER,
                        SIGNALS = SIGNALS,
                        BERMS = BERMS,
                        REFUGEE_ISLAND = REFUGEE_ISLAND,
                        pcd_barriers = pcd_barriers,
                        pcd_barrier_1 = pcd_barrier_1,
                        pcd_barrier_2 = pcd_barrier_2)
                    
                                    

                elif road_type == 'local':
                    SURFACE,ROAD,SLOPE,SHOULDER,SIGNALS,pcd_barrier_1,pcd_barrier_2 = DTM_road_generator(
                        road_type = road_type,
                        ORIGINAL_SEGMENTS = ORIGINAL_SEGMENTS,
                        scale = scale,
                        number_points_DTM = number_points_DTM,
                        road_buffer = road_buffer,
                        slope_buffer = slope_buffer,
                        shoulder_buffer = shoulder_buffer,
                        noise_DTM = noise_DTM,
                        noise_road = noise_road,
                        noise_slope = noise_slope,
                        noise_shoulder = noise_shoulder,
                        noise_berm = noise_berm,
                        noise_refugee_island = noise_refugee_island
                        )
    
                end_surface = time.time()
                print('Finished simulation of DTM and road environment: %f s'%(end_surface-initial_instant))
    
    
        else:
            # Point clouds with just DTM and trees:
            SURFACE = creador_superficies(ORIGINAL_SEGMENTS,
                                          scale = scale,
                                          number_points_DTM = number_points_DTM)
                                          # montana=montana,
                                          # visualizacion=False,
                                          # voxel_downsampling_size=voxel_downsampling_size)
                                          
            end_surface = time.time()
            
            print('Finished simulation of DTM: %f s'%(end_surface-initial_instant))
            
            
            #========================CREACIÓN ÁRBOLES ARTIFICIALES=========================
            
            # Vamos a generar árboles nuevos basándonos en los que ya hemos leído por ahora:
                
            SEGMENTOS_ARTIFICIALES = creador_arboles_artificiales(SEGMENTOS_ORIG,
                                                                  Numero_transformaciones_por_arbol)
            
            
            # print(SEGMENTOS_ARTIFICIALES_aux[0])
            # print(SEGMENTOS_ARTIFICIALES[0])
            
            
            fin_arboles_artificiales = time.time()
            
            print('Duración de la creación de árboles artificiales: %f s'%(fin_arboles_artificiales-fin_superficies))
    
    #==============================FUSIÓN DE DATOS=================================
    
    # Ya tenemos los siguientes packs de datos:
    # · SEGMENTOS_ORIG
    # · SUPERFICIES
    # · SEGMENTOS_ARTIFICIALES
    
    # Lo que nos queda es fusionarlos, es decir, colocar todos los árboles que po-
    # damos en cada superficie. Para ello seguiremos la siguiente estrategia:
    # 1) Seleccionamos una superficie.
    # 2) Seleccionamos un árbol y lo colocamos en una posición arbitraria de la su-
    #    perficie siempre y cuando no posea puntos fuera de la misma.
    # 3) Seleccionamos otro árbol y le buscamos otra posición aleatoria. En esa po-
    #    sición buscamos el árbol más próximo y hacemos 2 fits: uno a la copa de
    #    cada árbol.
    # 4) Si los dos fits no poseen puntos en común (no intersecan), damos por váli-
    #    da la posición y saltamos al siguiente paso. En caso contrario, repetimos
    #    el paso 3 hasta que no tenga problemas.
    # 5) Si en una cantidad de iteraciones (llamémosla N_it) no se encuentra una
    #    buena posición para el árbol, paramos y damos por terminada la nueva nube.
    
    # NOTA: AL FINAL DE TODO LAS NUBES DE PUNTOS ARTIFICIALES DEBE ESTAR CLASIFICA-
    #       DAS, ES DECIR, VER CÓMO GUARDAR EN .laz Ó .las CADA PUNTO COMO:
    #                  (x_punto,  y_punto,  z_punto,  clasificacion)
    
    
    # Esto es algo temporal:
    # ejemplo = True
    
    
    # if ejemplo:
    
        # Comportamiento que debe obedecer: asignar una label a cada punto tipo ár-
        # bol y guardar la nube final como .las o .laz.
        
        # contador = 0
        # for i in range(len(SUPERFICIES)):
            print('Generando nube artificial %i/%i'%(i+1,Numero_superficies))
            ARBOLES = []
            NUBES_ARTIFICIALES = {}
            superficie = SUPERFICIES[0]
            ARBOLES.append([])
            if carretera:
                carretera_i = CARRETERAS[0]
                talud_i = TALUDES[0]
                if tipo_carretera != 'local':
                    arcen_i = ARCENES[0]
                    berma_i = BERMAS[0]
                if tipo_carretera in ['autovia','autopista']:
                    mediana_i = MEDIANAS[0]
                # talud_2_i = TALUDES_2[i]
                # senhal_i = SENHALES[i]
        # superficie = SUPERFICIES[2] # Ejemplo
        
            
        
            puntos_superficie = np.array(superficie.points)
            
            if tipo_carretera in ['autovia','autopista']:
                puntos_mediana = np.array(mediana_i.points)            
        
            SEGMENTOS_ARTIFICIALES_aux = copy.deepcopy(SEGMENTOS_ARTIFICIALES)
    
            cuntudur = 0
            for j in range(Numero_arboles_por_nube):
                
                if j < Numero_arboles_por_nube-Numero_arboles_mediana:
                
                    # Si ya hemos terminado de poner todos los árboles que teníamos
                    # creados volvemos a empezar:
                    if cuntudur == len(SEGMENTOS_ARTIFICIALES_aux):
                        # print(SEGMENTOS_ARTIFICIALES[0])
                        # SEGMENTOS_ARTIFICIALES_aux.clear()                    
                        
                        
                        SEGMENTOS_ARTIFICIALES_aux = copy.deepcopy(SEGMENTOS_ARTIFICIALES)
                        # print(SEGMENTOS_ARTIFICIALES[0])
                        cuntudur = 0
        
                    posible_ubicacion = puntos_superficie[np.random.choice(len(puntos_superficie))]
                    
                    indice = np.random.choice(len(SEGMENTOS_ARTIFICIALES_aux))
                    # Cojo un árbol al azar:
                    arbol = SEGMENTOS_ARTIFICIALES_aux[indice]
                    # arbol = SEGMENTOS_ARTIFICIALES[10] # Ejemplo
            
                    while arbol[1] == True:
                        indice = np.random.choice(len(SEGMENTOS_ARTIFICIALES_aux))
                        arbol = SEGMENTOS_ARTIFICIALES_aux[indice]
                    if arbol[1] == False:
                        cuntudur += 1
                        
                    arbol = arbol[0]
                    # Le ponemosla etiqueta True porque ya ha sido seleccionado:
                    SEGMENTOS_ARTIFICIALES_aux[indice][1] = True
                    SEGMENTOS_ARTIFICIALES[indice][1] = False
                    # if cuntudur == 1:
                    #     print(SEGMENTOS_ARTIFICIALES_aux[indice][1])
                    #     print(SEGMENTOS_ARTIFICIALES[indice][1])
                    arbol.translate((posible_ubicacion),relative=False)
                    
                    puntos_arbol = np.array(arbol.points)
                    minima_altura = puntos_arbol.take(2,1).min()
                    
                    color_arbol = np.array(arbol.colors)
                    
                    desfase_vertical = posible_ubicacion-minima_altura
                    
                    puntos_arbol[:, 2] = puntos_arbol.take(2,1) + desfase_vertical[2]
                    arbol = o3d.geometry.PointCloud()
                    arbol.points = o3d.utility.Vector3dVector(puntos_arbol)
                    arbol.colors = o3d.utility.Vector3dVector(color_arbol)
                    
                    #-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
                    # PARA LA PARTE DE INTENSIDADES ESTO ES IMPORTANTE!!!:           
                    if calcular_normales:   
                        arbol.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=100, max_nn=1000))
                    #-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
                    
                    
                    if propuesta_IGN:
                        # nube_SEGMENTOS_ARTIFICIALES = o3d.geometry.PointCloud()
                        # for item in SEGMENTOS_ARTIFICIALES:
                        #     nube_SEGMENTOS_ARTIFICIALES += item
                        puntos_arbol = np.array(arbol.points)
                        colores_arbol = np.array(arbol.colors)
                        if carretera:
                            # Para que la nube final sea de ~50.000 puntos necesitaría
                            # que tooodos los árboles sumen unos ~5.000 puntos
                            N_puntos_por_arbol = int(5000/Numero_arboles_por_nube)
                        else:
                            # Para que la nube final sea de ~50.000 puntos necesitaría
                            # que tooodos los árboles sumen unos ~20.000 puntos
                            N_puntos_por_arbol = int(30000/Numero_arboles_por_nube)
                        try:
                            indices_downsampling = np.random.choice(puntos_arbol.shape[0], N_puntos_por_arbol, replace=False)
                        except ValueError:
                            indices_downsampling = np.random.choice(puntos_arbol.shape[0], N_puntos_por_arbol, replace=True)
                        puntos_arbol = puntos_arbol[indices_downsampling]
                        colores_arbol = colores_arbol[indices_downsampling]
                        arbol = o3d.geometry.PointCloud()
                        arbol.points = o3d.utility.Vector3dVector(puntos_arbol)
                        arbol.colors = o3d.utility.Vector3dVector(colores_arbol)
                    
                    
                    ARBOLES[-1].append(arbol)
                    
                    if contador == 0:
            
                        nube_artificial = arbol+superficie
                        contador += 1
            
                    else:
                        nube_artificial += arbol
                
                
                
                
                
                elif not propuesta_IGN and j >= Numero_arboles_por_nube-Numero_arboles_mediana and nubes_modificadas == 'Santarem': # Los últimos 20 árboles los meto en la mediana
                    # Si ya hemos terminado de poner todos los árboles que teníamos
                    # creados volvemos a empezar:
                    if cuntudur == len(SEGMENTOS_ARTIFICIALES_aux):
                        # print(SEGMENTOS_ARTIFICIALES[0])
                        # SEGMENTOS_ARTIFICIALES_aux.clear()
                        SEGMENTOS_ARTIFICIALES_aux = copy.deepcopy(SEGMENTOS_ARTIFICIALES)
                        # print(SEGMENTOS_ARTIFICIALES[0])
                        cuntudur = 0
        
                    posible_ubicacion = puntos_mediana[np.random.choice(len(puntos_mediana))]
                    
                    indice = np.random.choice(len(SEGMENTOS_ARTIFICIALES_aux))
                    # Cojo un árbol al azar:
                    arbol = SEGMENTOS_ARTIFICIALES_aux[indice]
                    # arbol = SEGMENTOS_ARTIFICIALES[10] # Ejemplo
            
                    while arbol[1] == True:
                        indice = np.random.choice(len(SEGMENTOS_ARTIFICIALES_aux))
                        arbol = SEGMENTOS_ARTIFICIALES_aux[indice]
                    if arbol[1] == False:
                        cuntudur += 1
                        
                    arbol = arbol[0]
                    # Le ponemosla etiqueta True porque ya ha sido seleccionado:
                    SEGMENTOS_ARTIFICIALES_aux[indice][1] = True
                    SEGMENTOS_ARTIFICIALES[indice][1] = False
                    # if cuntudur == 1:
                    #     print(SEGMENTOS_ARTIFICIALES_aux[indice][1])
                    #     print(SEGMENTOS_ARTIFICIALES[indice][1])
                    arbol.translate((posible_ubicacion),relative=False)
                    
                    puntos_arbol = np.array(arbol.points)
                    minima_altura = puntos_arbol.take(2,1).min()
                    
                    color_arbol = np.array(arbol.colors)
                    
                    desfase_vertical = posible_ubicacion-minima_altura
                    
                    puntos_arbol[:, 2] = puntos_arbol.take(2,1) + desfase_vertical[2]
                    arbol = o3d.geometry.PointCloud()
                    arbol.points = o3d.utility.Vector3dVector(puntos_arbol)
                    arbol.colors = o3d.utility.Vector3dVector(color_arbol)
                    
                    #-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
                    # PARA LA PARTE DE INTENSIDADES ESTO ES IMPORTANTE!!!:           
                    if calcular_normales:   
                        arbol.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=100, max_nn=1000))
                    #-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
                    
                    
                    
                    if propuesta_IGN:
                        # nube_SEGMENTOS_ARTIFICIALES = o3d.geometry.PointCloud()
                        # for item in SEGMENTOS_ARTIFICIALES:
                        #     nube_SEGMENTOS_ARTIFICIALES += item
                        puntos_arbol = np.array(arbol.points)
                        colores_arbol = np.array(arbol.colors)
                        # Para que la nube final sea de ~50.000 puntos necesitaría
                        # que tooodos los árboles sumen unos ~5.000 puntos
                        N_puntos_por_arbol = int(5000/Numero_arboles_por_nube)
                        try:
                            indices_downsampling = np.random.choice(puntos_arbol.shape[0], N_puntos_por_arbol, replace=False)
                        except ValueError:
                            indices_downsampling = np.random.choice(puntos_arbol.shape[0], N_puntos_por_arbol, replace=True)
                        puntos_arbol = puntos_arbol[indices_downsampling]
                        colores_arbol = colores_arbol[indices_downsampling]
                        arbol = o3d.geometry.PointCloud()
                        arbol.points = o3d.utility.Vector3dVector(puntos_arbol)
                        arbol.colors = o3d.utility.Vector3dVector(colores_arbol)
                    
                    
                    
                    ARBOLES[-1].append(arbol)
                    
                    if contador == 0:
            
                        nube_artificial = arbol+superficie
                        contador += 1
            
                    else:
                        nube_artificial += arbol
    
    
    
                elif j >= Numero_arboles_por_nube-20 and nubes_modificadas == 'defecto':            
                    # Si ya hemos terminado de poner todos los árboles que teníamos
                    # creados volvemos a empezar:
                    if cuntudur == len(SEGMENTOS_ARTIFICIALES_aux):
                        # print(SEGMENTOS_ARTIFICIALES[0])
                        # SEGMENTOS_ARTIFICIALES_aux.clear()
                        SEGMENTOS_ARTIFICIALES_aux = copy.deepcopy(SEGMENTOS_ARTIFICIALES)
                        # print(SEGMENTOS_ARTIFICIALES[0])
                        cuntudur = 0
        
                    posible_ubicacion = puntos_superficie[np.random.choice(len(puntos_superficie))]
                    
                    indice = np.random.choice(len(SEGMENTOS_ARTIFICIALES_aux))
                    # Cojo un árbol al azar:
                    arbol = SEGMENTOS_ARTIFICIALES_aux[indice]
                    # arbol = SEGMENTOS_ARTIFICIALES[10] # Ejemplo
            
                    while arbol[1] == True:
                        indice = np.random.choice(len(SEGMENTOS_ARTIFICIALES_aux))
                        arbol = SEGMENTOS_ARTIFICIALES_aux[indice]
                    if arbol[1] == False:
                        cuntudur += 1
                        
                    arbol = arbol[0]
                    # Le ponemosla etiqueta True porque ya ha sido seleccionado:
                    SEGMENTOS_ARTIFICIALES_aux[indice][1] = True
                    SEGMENTOS_ARTIFICIALES[indice][1] = False
                    # if cuntudur == 1:
                    #     print(SEGMENTOS_ARTIFICIALES_aux[indice][1])
                    #     print(SEGMENTOS_ARTIFICIALES[indice][1])
                    arbol.translate((posible_ubicacion),relative=False)
                    
                    puntos_arbol = np.array(arbol.points)
                    minima_altura = puntos_arbol.take(2,1).min()
                    
                    color_arbol = np.array(arbol.colors)
                    
                    desfase_vertical = posible_ubicacion-minima_altura
                    
                    puntos_arbol[:, 2] = puntos_arbol.take(2,1) + desfase_vertical[2]
                    arbol = o3d.geometry.PointCloud()
                    arbol.points = o3d.utility.Vector3dVector(puntos_arbol)
                    arbol.colors = o3d.utility.Vector3dVector(color_arbol)
                    
                    #-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
                    # PARA LA PARTE DE INTENSIDADES ESTO ES IMPORTANTE!!!:           
                    if calcular_normales:   
                        arbol.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=100, max_nn=1000))
                    #-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
                    
                    
                    if propuesta_IGN:
                        # nube_SEGMENTOS_ARTIFICIALES = o3d.geometry.PointCloud()
                        # for item in SEGMENTOS_ARTIFICIALES:
                        #     nube_SEGMENTOS_ARTIFICIALES += item
                        puntos_arbol = np.array(arbol.points)
                        colores_arbol = np.array(arbol.colors)
                        # Para que la nube final sea de ~50.000 puntos necesitaría
                        # que tooodos los árboles sumen unos ~5.000 puntos
                        N_puntos_por_arbol = int(5000/Numero_arboles_por_nube)
                        try:
                            indices_downsampling = np.random.choice(puntos_arbol.shape[0], N_puntos_por_arbol, replace=False)
                        except ValueError:
                            indices_downsampling = np.random.choice(puntos_arbol.shape[0], N_puntos_por_arbol, replace=True)
                        puntos_arbol = puntos_arbol[indices_downsampling]
                        colores_arbol = colores_arbol[indices_downsampling]
                        arbol = o3d.geometry.PointCloud()
                        arbol.points = o3d.utility.Vector3dVector(puntos_arbol)
                        arbol.colors = o3d.utility.Vector3dVector(colores_arbol)
                    
                    
                    
                    ARBOLES[-1].append(arbol)
                    
                    if contador == 0:
            
                        nube_artificial = arbol+superficie
                        contador += 1
            
                    else:
                        nube_artificial += arbol
                
                
                
                
            # Metemos todos los árboles que coleccionamos en un sólo elemento:
            arbol = o3d.geometry.PointCloud()
            
            for arbol_i in ARBOLES[-1]:
                arbol += arbol_i
            
            # Cambiamos tooodos los elementos por uno sólo que serán todos los ár-
            # boles:
            ARBOLES = arbol
            
            if voxel_downsampling_size < 1000:
                # Vamos a hacer voxel_downsampling:
                ARBOLES = ARBOLES.voxel_down_sample(voxel_size=voxel_downsampling_size)
    
            
            if carretera:
                # Descomentar para visualizaciones algo más bonitas (ralentiza un poco)
                # nube_artificial.estimate_normals()
                
                todas_las_senhales = o3d.geometry.PointCloud()
                
                for LL in range(len(SENHALES)):
                    todas_las_senhales += SENHALES[LL]
            
            if carretera:
                if tipo_carretera in ['autovia','autopista']:
                    if nubes_modificadas == 'defecto':
                        NUBES_ARTIFICIALES = nube_artificial + carretera_i + talud_i + arcen_i + berma_i + mediana_i + pcd_barrera + barrera_quitamiedos_1 + barrera_quitamiedos_2 + todas_las_senhales
                    if nubes_modificadas == 'Santarem':
                        NUBES_ARTIFICIALES = nube_artificial + carretera_i + talud_i + arcen_i + berma_i + mediana_i + barrera_quitamiedos_1 + barrera_quitamiedos_2 + barrera_quitamiedos_1_mediana + barrera_quitamiedos_2_mediana + todas_las_senhales
                if tipo_carretera == 'nacional':
                    NUBES_ARTIFICIALES = nube_artificial + carretera_i + talud_i + arcen_i + berma_i + barrera_quitamiedos_1 + barrera_quitamiedos_2 + todas_las_senhales
                if tipo_carretera == 'local':
                    NUBES_ARTIFICIALES = nube_artificial + carretera_i + talud_i + todas_las_senhales
                
                
            else:
                NUBES_ARTIFICIALES = nube_artificial
            contador = 0
    
    # else:
    #     print('Guardar clasificación en formato .las o .laz')
    
            instante_final = time.time()
            
            print('Duración creación de la nube: %f s'%(instante_final-instante_inicial))
            # instante_inicial = time.time()
        
            
            if visualizacion_por_nube:
                o3d.visualization.draw(NUBES_ARTIFICIALES)
        
            # if downsampling_densidad != 0 and  voxel_downsampling_size > 0:
                
            #     o3d.visualization.draw(NUBES_ARTIFICIALES)
            #     prueba = NUBES_ARTIFICIALES.voxel_down_sample(voxel_size=0.5)
            #     o3d.visualization.draw(prueba)
                
            #     # Primero voy a downsamplear la vegetación y hacer que tenga la mis-
            #     # ma densidad de puntos que el DTM:
            #     puntos_arboles = np.array(ARBOLES.points)
            #     colores_arboles = np.array(ARBOLES.colors)
            #     indices_downsampling = np.random.choice(puntos_arboles.shape[0], len(np.array(superficie.points)), replace=False)
            #     puntos_arboles = puntos_arboles[indices_downsampling]
            #     colores_arboles = colores_arboles[indices_downsampling]
            #     NUBES_ARTIFICIALES = superficie+ARBOLES
                
                
            #     # Ahora downsampleamos la nube completa:
            #     # import pdb
            #     # pdb.set_trace()
            #     Densidad_final = downsampling_densidad
            #     puntos_nube = np.array(NUBES_ARTIFICIALES.points)
            #     colores_nube = np.array(NUBES_ARTIFICIALES.colors)
            #     # Vamos a hacer que la nube tenga una densidad de puntos concreta
            #     Numero_puntos_nube = len(puntos_nube)
            #     longitud_x_nube = np.abs(puntos_nube[:,0].max()-puntos_nube[:,0].min())
            #     longitud_y_nube = np.abs(puntos_nube[:,1].max()-puntos_nube[:,1].min())
            #     Superficie_inicial = longitud_x_nube * longitud_y_nube
            #     Densidad_inicial = Numero_puntos_nube / Superficie_inicial
            #     factor_downsampling = Densidad_inicial/Densidad_final
                
            #     # Ahora hacemos el downsampling:
            #     indices_downsampling = np.random.choice(puntos_nube.shape[0], int(Numero_puntos_nube/factor_downsampling), replace=False)
            #     puntos_nube_downsampleada = puntos_nube[indices_downsampling]
            #     colores_nube_downsampleada = colores_nube[indices_downsampling]
                
            #     NUBES_ARTIFICIALES = o3d.geometry.PointCloud()
            #     NUBES_ARTIFICIALES.points = o3d.utility.Vector3dVector(puntos_nube_downsampleada)
            #     NUBES_ARTIFICIALES.colors = o3d.utility.Vector3dVector(colores_nube_downsampleada)
                
                # if i == 0:
                #     visor.custom_draw_geometry_with_key_callback(NUBES_ARTIFICIALES)
                    
            #=============================ALMACÉN DE NUBES=================================
            
            if not USB_conectado:
                os.chdir(ruta_ejecutable)
            else:
                os.chdir(ruta_guardado_USB)    
            
            try:
                os.mkdir('Nubes_artificiales_generadas')
            except FileExistsError:
                pass
            
            os.chdir('Nubes_artificiales_generadas')
            
            fecha_actual = time.localtime()
            
            
            if i == 0:
                # Genero una carpeta con esta nube artificial concreta:
                if nubes_por_defecto:
                    carpeta = 'Nubes_artificiales_defecto_%i_%i_%i___%i_%i_%i'%(fecha_actual[0],
                                                                     fecha_actual[1],
                                                                     fecha_actual[2],
                                                                     fecha_actual[3],
                                                                     fecha_actual[4],
                                                                     fecha_actual[5])
                else:
                    carpeta = 'Nubes_artificiales_personalizadas_%i_%i_%i___%i_%i_%i'%(fecha_actual[0],
                                                                     fecha_actual[1],
                                                                     fecha_actual[2],
                                                                     fecha_actual[3],
                                                                     fecha_actual[4],
                                                                     fecha_actual[5])
                
                os.mkdir(carpeta)
            os.chdir(carpeta)
            # print(carpeta)
            
            
            if not propuesta_IGN:
            
                # Para temas de entrenamiento de mis redes voy a crear un pequeño .txt
                # con algo de información útil:
                    
                with open("informacion_dataset.txt", "w") as f:
                    f.writelines('INFORMACION DATASET:\n')
                    if not carretera:
                        f.writelines('Tipo de nubes: bosque\n')
                        f.writelines('Numero de clases: 2\n')
                        f.writelines('--- ETIQUETAS ---\n')
                        f.writelines('label    clase\n')
                        f.writelines('0    arbol\n')
                        f.writelines('1    DTM\n')
                    else:
                        f.writelines('Tipo de nubes: %s\n'%tipo_carretera)
                        if tipo_carretera == 'local':
                            f.writelines('Numero de clases: 5\n')
                            f.writelines('--- ETIQUETAS ---\n')
                            f.writelines('label    clase\n')
                            f.writelines('0    arbol\n')
                            f.writelines('1    DTM\n')
                            f.writelines('2    carretera\n')
                            f.writelines('3    talud\n')
                            f.writelines('4    señal de tráfico\n')
                        if tipo_carretera == 'nacional':
                            f.writelines('Numero de clases: 8\n')
                            f.writelines('--- ETIQUETAS ---\n')
                            f.writelines('label    clase\n')
                            f.writelines('0    arbol\n')
                            f.writelines('1    DTM\n')
                            f.writelines('2    carretera\n')
                            f.writelines('3    talud\n')
                            f.writelines('4    señal de tráfico\n')
                            f.writelines('5    barrera_quitamiedos_1\n')
                            f.writelines('6    barrera_quitamiedos_2\n')
                            f.writelines('7    arcén\n')
                        if tipo_carretera in ['autovia','autopista']:
                            if nubes_modificadas == 'Santarem':
                                f.writelines('Numero de clases: 8\n')
                                f.writelines('--- ETIQUETAS ---\n')
                                f.writelines('label    clase\n')
                                f.writelines('0    arbol\n')
                                f.writelines('1    DTM\n')
                                # f.writelines('2    carretera\n')
                                f.writelines('2    talud\n')
                                f.writelines('3    señal de tráfico\n')
                                f.writelines('4    barrera_quitamiedos\n')
                                # f.writelines('6    arcén\n')
                                f.writelines('5    mediana\n')
                                f.writelines('6    berma\n')
                                f.writelines('7    puntos_via_circulacion\n')
                            
                            if nubes_modificadas == 'defecto':
                                
                                f.writelines('Numero de clases: 11\n')
                                f.writelines('--- ETIQUETAS ---\n')
                                f.writelines('label    clase\n')
                                f.writelines('0    arbol\n')
                                f.writelines('1    DTM\n')
                                f.writelines('2    carretera\n')
                                f.writelines('3    talud\n')
                                f.writelines('4    señal de tráfico\n')
                                f.writelines('5    barrera_quitamiedos_1\n')
                                f.writelines('6    barrera_quitamiedos_2\n')
                                f.writelines('7    arcén\n')
                                f.writelines('8    mediana\n')
                                f.writelines('9    barrera_jersey\n')
                                f.writelines('10    berma\n')
                f.close()
            
            else:
                
            # Para temas de entrenamiento de mis redes voy a crear un pequeño .txt
            # con algo de información útil:
                
                with open("informacion_dataset.txt", "w") as f:
                    f.writelines('INFORMACION DATASET:\n')
                    if not carretera:
                        f.writelines('Tipo de nubes: bosque\n')
                        f.writelines('Numero de clases: 2\n')
                        f.writelines('--- ETIQUETAS ---\n')
                        f.writelines('label    clase\n')
                        f.writelines('0    arbol\n')
                        f.writelines('1    DTM\n')
                    else:
                        f.writelines('Tipo de nubes: %s\n'%tipo_carretera)
                        if tipo_carretera == 'local':
                            f.writelines('Numero de clases: 4\n')
                            f.writelines('--- ETIQUETAS ---\n')
                            f.writelines('label    clase\n')
                            f.writelines('0    arbol\n')
                            f.writelines('1    DTM\n')
                            f.writelines('2    infraestructura de transporte\n')
                            f.writelines('3    talud\n')
                        if tipo_carretera == 'nacional':
                            f.writelines('Numero de clases: 4\n')
                            f.writelines('--- ETIQUETAS ---\n')
                            f.writelines('label    clase\n')
                            f.writelines('0    arbol\n')
                            f.writelines('1    DTM\n')
                            f.writelines('2    infraestructura de transporte\n')
                            f.writelines('3    talud\n')
                        if tipo_carretera in ['autovia','autopista']:
                            if nubes_modificadas == 'Santarem':
                                f.writelines('Numero de clases: 4\n')
                                f.writelines('--- ETIQUETAS ---\n')
                                f.writelines('label    clase\n')
                                f.writelines('0    arbol\n')
                                f.writelines('1    DTM\n')
                                f.writelines('2    infraestructura de transporte\n')
                                f.writelines('3    talud\n')
                            
                            if nubes_modificadas == 'defecto':
                                
                                f.writelines('Numero de clases: 4\n')
                                f.writelines('--- ETIQUETAS ---\n')
                                f.writelines('label    clase\n')
                                f.writelines('0    arbol\n')
                                f.writelines('1    DTM\n')
                                f.writelines('2    infraestructura de transporte\n')
                                f.writelines('3    talud\n')
                f.close()
            
            
            # OJO!!! Luego hay que mover manualmente las nubes que queramos a las 
            # carpetas TEST y TRAIN (las cuales se deben crear, manualmente tb).
            
            
            # Vamos a guardar las nubes generadas:
            
            os.mkdir("Nube_artificial_%i"%i)    
            os.chdir("Nube_artificial_%i"%i)
            ruta_simulaciones = os.getcwd()
            
            pcd = NUBES_ARTIFICIALES
            puntos_nube = np.array(pcd.points)
            
            o3d.io.write_point_cloud("Nube_artificial_%i.pcd"%i, pcd)
        
            # Escribimos un .las también:
        
                
            # 1. Create a new header
            header = laspy.LasHeader(point_format=3, version="1.2")
            # header.add_extra_dim(laspy.ExtraBytesParams(name="intensidad_faro", type=np.float32))
            header.offsets = np.min(puntos_nube, axis=0)
            header.scales = np.array([0.1, 0.1, 0.1])
            
            # 2. Create a Las
            las = laspy.LasData(header)
            
            las.x = puntos_nube[:, 0]
            las.y = puntos_nube[:, 1]
            las.z = puntos_nube[:, 2]
            # las.intensidad_faro = intensidades_FARO_comunes
            
            las.write("Nube_artificial_%i.las"%i)
        
            
            '''
            # FORMA ANTIGUA DE ESCRIBIR .las CON LASPY:
            hdr = laspy.header.Header(point_format=2)
            
            outfile = laspy.file.File("Nube_artificial_%i.las"%i, mode="w", header=hdr)
            allx = np.array(pcd.points).take(0,1)
            ally = np.array(pcd.points).take(1,1)
            allz = np.array(pcd.points).take(2,1)
            
            xmin = np.floor(np.min(allx))
            ymin = np.floor(np.min(ally))
            zmin = np.floor(np.min(allz))
            
            outfile.header.offset = [xmin,ymin,zmin]
            outfile.header.scale = [0.001,0.001,0.001]
            
            outfile.x = allx
            outfile.y = ally
            outfile.z = allz
            
            outfile.red = np.array(pcd.colors).take(0,1)*255
            outfile.geen = np.array(pcd.colors).take(1,1)*255
            outfile.blue = np.array(pcd.colors).take(2,1)*255
            
            outfile.close()
            '''
            
            
            # Finalmente, vamos a guardar un .txt con los parámetros usados en esta genera-
            # ción:
                
            with open("parametros.txt", "w") as f:
                f.writelines('--------------------------------------------------------\n')
                f.writelines('FECHA: %i/%i/%i    HORA: %i/%i/%i\n'%(time.gmtime()[2],time.gmtime()[1],time.gmtime()[0],time.gmtime()[3]+2,time.gmtime()[4],time.gmtime()[5]))
                f.writelines('--------------------------------------------------------\n')
                f.writelines('Tiempo_lectura (s): %f\n'%(fin_lectura-instante_inicial))
                f.writelines('Tiempo creación superficies (s): %f\n'%(fin_superficies-fin_lectura))
                f.writelines('Tiempo creación árboles artificiales (s): %f\n'%(fin_arboles_artificiales-fin_superficies))
                f.writelines('Tiempo total (s): %f\n'%(instante_final-instante_inicial))
                f.writelines('--------------------------------------------------------\n')
                f.writelines('Parámetros empleados en esta generación:\n')
                f.writelines('Ruta nubes de árboles originales: %s\n'%(ruta_segmentaciones_cilindricas))
                f.writelines('Numero_superficies: %i\n'%(Numero_superficies))
                f.writelines('factor_repeticion: %f\n'%(factor_repeticion))
                f.writelines('Numero_puntos_superficie: %i\n'%(Numero_puntos_superficie))
                f.writelines('Numero_transformaciones_por_arbol: %f\n'%(Numero_transformaciones_por_arbol))
                f.writelines('Numero_arboles_por_nube: %i\n'%(Numero_arboles_por_nube))
                f.writelines('Bosque de montaña grande: %s\n'%(montana))
                f.writelines('Carretera: %s\n'%(carretera))
                f.writelines('--------------------------------------------------------\n')
                f.writelines('CÁLCULO DE INTENSIDAD\n')
                if calcular_intensidades:
                    f.writelines('Sí se calcula intensidad\n')
                else:
                    f.writelines('No se calcula intensidad\n')
                f.close()
            
            # COUSIÑAS A MAIORES:
            # 1) Podemos generar maleza/arbustos también!
            # 2) En zonas de pendiente abrupta no poner árboles.
            # 3) En zonas de pendiente moderada hacer que los árboles crezcan en la di-
            #    rección perpendicular.
            
            
            
            
            
            # De cara a meter estas nubes en el GAN cunde guardar cada una como un único
            # array en el que tengamos algo así: [x,y,z,clasificación].
            # Vamos a intentarlo:
                
            # Tenemos lo siguiente:
            
            # · SUPERFICIES (diccionario con los puntos tipo suelo/sin clasificación)
            # · ARBOLES (lista con los puntos tipo árbol en cada nube)
            # · CARRETERAS (diccionario con todos los puntos tipo carretera en cada nube)
            # · ARCENES (diccionario con todos los puntos tipo arcén en cada nube)
            # · TALUDES (diccionario con todos los puntos tipo talud en cada nube)
            
            # OJO! Las señales no las voy a incluir aquí porque aún tengo que mejorar todo
            # lo relativo a ellas.
            
            # Vamos a establecer las siguientes etiquetas/label para cada punto:
            
            # · Puntos en SUPERFICIES ---> label = 0
            # · Puntos en ARBOLES ---> label = 1
            # · Puntos en CARRETERAS ---> label = 2
            # · Puntos en ARCENES ---> label = 3
            # · Puntos en TALUDES ---> label = 4
            
            
            # if not arboles_por_separado:
            
            #     puntos_superficie = np.array(SUPERFICIES[0].points)
            #     labels_superficie = np.full((1,len(puntos_superficie)),0)[0]
                
            #     puntos_arboles = np.array(ARBOLES.points)
            #     labels_arboles = np.full((1,len(puntos_arboles)),1)[0]
                
            #     puntos_carretera = np.array(CARRETERAS[0].points)
            #     labels_carretera = np.full((1,len(puntos_carretera)),2)[0]
                
            #     puntos_arcen = np.array(ARCENES[0].points)
            #     labels_arcen = np.full((1,len(puntos_arcen)),3)[0]
                
            #     puntos_talud = np.array(TALUDES[0].points)
            #     labels_talud = np.full((1,len(puntos_talud)),4)[0]
                
            #     puntos_nube = np.concatenate((puntos_superficie,puntos_arboles,
            #                                   puntos_carretera,puntos_arcen,puntos_talud))
            
            #     labels_nube = np.concatenate((labels_superficie,labels_arboles,
            #                              labels_carretera,labels_arcen,labels_talud))
            
            
            #     with open("Nube_artificial_%i.npy"%i, 'wb') as f:    
            #         np.save(f, puntos_nube)
            #         np.save(f,labels_nube)
                    
            #         # Para cargar esos arrays haríamos así:
                        
            #         # with open("Nube_artificial_%i.npy"%s, 'rb') as f:
            #         #     puntos_nube = np.load(f)
            #         #     labels_nube = np.load(f)
            
            
            
            # else:
                
                
                
                
                
                
            os.mkdir('numpy_arrays')
            os.chdir('numpy_arrays')
                
                
            # Guardo los puntos de cada clase como arrays de numpy:
            
            puntos_superficie = np.array(SUPERFICIES[0].points)
            colores_superficie = np.array(SUPERFICIES[0].colors)
            
            # labels_superficie = np.full((1,len(puntos_superficie)),0)[0]
            with open("Nube_artificial_%i_DTM.npy"%i, 'wb') as f:    
                np.save(f, puntos_superficie)
                
            puntos_arboles = np.array(ARBOLES.points)
            colores_arboles = np.array(ARBOLES.colors)
            # labels_arboles = np.full((1,len(puntos_arboles)),1)[0]
            with open("Nube_artificial_%i_arboles.npy"%i, 'wb') as f:    
                np.save(f, puntos_arboles)
            
            if not propuesta_IGN:
                if carretera:
                
                    puntos_carretera = np.array(CARRETERAS[0].points)
                    # labels_carretera = np.full((1,len(puntos_carretera)),2)[0]
                    with open("Nube_artificial_%i_carretera.npy"%i, 'wb') as f:    
                        np.save(f, puntos_carretera)
                    puntos_talud = np.array(TALUDES[0].points)
                    # labels_talud = np.full((1,len(puntos_talud)),4)[0]
                    with open("Nube_artificial_%i_talud.npy"%i, 'wb') as f:    
                        np.save(f, puntos_talud)
                    todas_las_senhales = np.array(todas_las_senhales.points)
                    # labels_talud = np.full((1,len(puntos_talud)),4)[0]
                    with open("Nube_artificial_%i_senhales.npy"%i, 'wb') as f:    
                        np.save(f, todas_las_senhales)
                    
                    
                    if tipo_carretera == 'local':
                        
                        puntos_nube = np.concatenate((puntos_superficie,puntos_arboles,
                                                      puntos_carretera,puntos_talud,
                                                      todas_las_senhales))
                
                        with open("Nube_artificial_%i.npy"%i, 'wb') as f:    
                            np.save(f, puntos_nube)
                    
                    
                    
                    
                    if tipo_carretera != 'local':
                        
                        puntos_barrera_quitamiedos_1 = np.array(barrera_quitamiedos_1.points)
                        with open("Nube_artificial_%i_barrera_quitamiedos_1.npy"%i, 'wb') as f:    
                            np.save(f, puntos_barrera_quitamiedos_1)
                        puntos_barrera_quitamiedos_2 = np.array(barrera_quitamiedos_2.points)
                        with open("Nube_artificial_%i_barrera_quitamiedos_2.npy"%i, 'wb') as f:    
                            np.save(f, puntos_barrera_quitamiedos_2)
                        puntos_arcen = np.array(ARCENES[0].points)
                        # labels_arcen = np.full((1,len(puntos_arcen)),3)[0]
                        with open("Nube_artificial_%i_arcen.npy"%i, 'wb') as f:    
                            np.save(f, puntos_arcen)
                        puntos_berma = np.array(berma_i.points)
                        with open("Nube_artificial_%i_berma.npy"%i, 'wb') as f:    
                            np.save(f, puntos_berma)
                            
                    
                    
                    if tipo_carretera == 'nacional':
                        
                        puntos_nube = np.concatenate((puntos_superficie,puntos_arboles,
                                                      puntos_carretera,puntos_talud,
                                                      puntos_arcen,puntos_barrera_quitamiedos_1,
                                                      puntos_barrera_quitamiedos_2,
                                                      todas_las_senhales))
                
                        with open("Nube_artificial_%i.npy"%i, 'wb') as f:    
                            np.save(f, puntos_nube)
                        
                    
                    if tipo_carretera in ['autovia','autopista']:
                        if nubes_modificadas == 'defecto':
                            puntos_barrera = np.array(pcd_barrera.points)
                            with open("Nube_artificial_%i_barrera_jersey.npy"%i, 'wb') as f:    
                                np.save(f, puntos_barrera)
                            puntos_mediana = np.array(mediana_i.points)
                            with open("Nube_artificial_%i_mediana.npy"%i, 'wb') as f:    
                                np.save(f, puntos_mediana)
                        
                            puntos_nube = np.concatenate((puntos_superficie,puntos_arboles,
                                                          puntos_carretera,puntos_talud,
                                                          puntos_arcen,puntos_barrera_quitamiedos_1,
                                                          puntos_barrera_quitamiedos_2,
                                                          puntos_barrera,todas_las_senhales,
                                                          puntos_mediana))
                
                        if nubes_modificadas == 'Santarem':
                            puntos_mediana = np.array(mediana_i.points)
                            with open("Nube_artificial_%i_mediana.npy"%i, 'wb') as f:    
                                np.save(f, puntos_mediana)
                        
                            puntos_barrera_quitamiedos_1_mediana = np.array(barrera_quitamiedos_1_mediana.points)
                            with open("Nube_artificial_%i_barrera_quitamiedos_1_mediana.npy"%i, 'wb') as f:    
                                np.save(f, puntos_barrera_quitamiedos_1_mediana)
                            
                            puntos_barrera_quitamiedos_2_mediana = np.array(barrera_quitamiedos_2_mediana.points)
                            with open("Nube_artificial_%i_barrera_quitamiedos_2_mediana.npy"%i, 'wb') as f:    
                                np.save(f, puntos_barrera_quitamiedos_2_mediana)
        
        
                            todos_los_quitamiedos = barrera_quitamiedos_1+barrera_quitamiedos_1_mediana+barrera_quitamiedos_2+barrera_quitamiedos_2_mediana
                            puntos_todos_los_quitamiedos = np.array(todos_los_quitamiedos.points)
                            with open("Nube_artificial_%i_barreras_quitamiedos.npy"%i, 'wb') as f:    
                                np.save(f, puntos_todos_los_quitamiedos)
        
        
                            puntos_nube = np.concatenate((puntos_superficie,puntos_arboles,
                                                          puntos_carretera,puntos_talud,
                                                          puntos_arcen,puntos_barrera_quitamiedos_1,
                                                          puntos_barrera_quitamiedos_2,
                                                          puntos_barrera_quitamiedos_1_mediana,
                                                          puntos_barrera_quitamiedos_2_mediana,
                                                          todas_las_senhales,puntos_mediana))
                       
                        
                        with open("Nube_artificial_%i.npy"%i, 'wb') as f:    
                            np.save(f, puntos_nube)
                
                    # labels_nube = np.concatenate((labels_superficie,labels_carretera,
                    #                               labels_arcen,labels_talud))
            
            else:
                
                # Sólo guardamos la infraestructura de transporte y los taludes:
                if carretera:
                
                
                    puntos_carretera = np.array(CARRETERAS[0].points)
                    colores_carretera = np.array(CARRETERAS[0].colors)
                    
                    puntos_talud = np.array(TALUDES[0].points)
                    colores_talud = np.array(TALUDES[0].colors)
                    
                    nube_copia_senhales = copy.deepcopy(todas_las_senhales)
                    todas_las_senhales = np.array(nube_copia_senhales.points)
                    colores_todas_las_senhales = np.array(nube_copia_senhales.colors)
    
    
                    puntos_infraestructura_transporte = np.vstack((puntos_carretera,
                                                                   todas_las_senhales))
                    
                    colores_infraestructura_transporte = np.vstack((colores_carretera,
                                                                   colores_todas_las_senhales))
                    
                    if tipo_carretera != 'local':
                        
                        puntos_barrera_quitamiedos_1 = np.array(barrera_quitamiedos_1.points)
                        colores_barrera_quitamiedos_1 = np.array(barrera_quitamiedos_1.colors)
    
                        puntos_barrera_quitamiedos_2 = np.array(barrera_quitamiedos_2.points)
                        colores_barrera_quitamiedos_2 = np.array(barrera_quitamiedos_2.colors)
    
                        puntos_arcen = np.array(ARCENES[0].points)
                        colores_arcen = np.array(ARCENES[0].colors)
    
                        
                        puntos_berma = np.array(berma_i.points)
                        colores_berma = np.array(berma_i.colors)
                            
                        puntos_infraestructura_transporte = np.vstack((puntos_infraestructura_transporte,
                                                                       puntos_barrera_quitamiedos_1,
                                                                       puntos_barrera_quitamiedos_2,
                                                                       puntos_arcen,
                                                                       puntos_berma))
                        colores_infraestructura_transporte = np.vstack((colores_infraestructura_transporte,
                                                                       colores_barrera_quitamiedos_1,
                                                                       colores_barrera_quitamiedos_2,
                                                                       colores_arcen,
                                                                       colores_berma))
    
                    
                    
                    if tipo_carretera in ['autovia','autopista']:
                        if nubes_modificadas == 'defecto':
                            puntos_barrera = np.array(pcd_barrera.points)
                            colores_barrera = np.array(pcd_barrera.colors)
    
                            puntos_mediana = np.array(mediana_i.points)
                            colores_mediana = np.array(mediana_i.colors)
                            
                            puntos_infraestructura_transporte = np.vstack((puntos_infraestructura_transporte,
                                                                           puntos_barrera,
                                                                           puntos_mediana))
                            
                            colores_infraestructura_transporte = np.vstack((colores_infraestructura_transporte,
                                                                           colores_barrera,
                                                                           colores_mediana))
    
                        if nubes_modificadas == 'Santarem':
                            puntos_mediana = np.array(mediana_i.points)                    
                            puntos_barrera_quitamiedos_1_mediana = np.array(barrera_quitamiedos_1_mediana.points)
                            puntos_barrera_quitamiedos_2_mediana = np.array(barrera_quitamiedos_2_mediana.points)    
                            todos_los_quitamiedos = barrera_quitamiedos_1+barrera_quitamiedos_1_mediana+barrera_quitamiedos_2+barrera_quitamiedos_2_mediana
                            puntos_todos_los_quitamiedos = np.array(todos_los_quitamiedos.points)
                            
                            puntos_infraestructura_transporte = np.vstack((puntos_infraestructura_transporte,
                                                                           puntos_mediana,
                                                                           puntos_todos_los_quitamiedos))
    
                            colores_mediana = np.array(mediana_i.colors)                    
                            colores_barrera_quitamiedos_1_mediana = np.array(barrera_quitamiedos_1_mediana.colors)
                            colores_barrera_quitamiedos_2_mediana = np.array(barrera_quitamiedos_2_mediana.colors)    
                            todos_los_quitamiedos = barrera_quitamiedos_1+barrera_quitamiedos_1_mediana+barrera_quitamiedos_2+barrera_quitamiedos_2_mediana
                            colores_todos_los_quitamiedos = np.array(todos_los_quitamiedos.colors)
                            
                            colores_infraestructura_transporte = np.vstack((colores_infraestructura_transporte,
                                                                           colores_mediana,
                                                                           colores_todos_los_quitamiedos))
                        
                        
                    
                    with open("Nube_artificial_%i_infraestructura_transporte.npy"%i, 'wb') as f:    
                        np.save(f, puntos_nube)
                        
                        
                    puntos_nube = np.concatenate((puntos_superficie,puntos_arboles,
                                                  puntos_infraestructura_transporte))
                    colores_nube = np.concatenate((colores_superficie,colores_arboles,
                                                  colores_infraestructura_transporte))
                    
                    
                    with open("Nube_artificial_%i.npy"%i, 'wb') as f:    
                        np.save(f, puntos_nube)
            
                    # labels_nube = np.concatenate((labels_superficie,labels_carretera,
                    #                               labels_arcen,labels_talud))
        
                
    
            # with open("Nube_artificial_%i_arboles.npy"%i, 'wb') as f:    
            #     np.save(f, puntos_arboles)
                # np.save(f,labels_arboles)
            
                # Para cargar esos arrays haríamos así:
                    
                # with open("Nube_artificial_%i.npy"%s, 'rb') as f:
                #     puntos_nube = np.load(f)
                #     labels_nube = np.load(f)
            
            
            
            if not propuesta_IGN:
            
                os.chdir(ruta_simulaciones)
                os.mkdir('nubes_open3d')
                os.chdir('nubes_open3d')
                
                # Finalmente guardamos cada nube individual para la parte de intensidades:        
                
                o3d.io.write_point_cloud("superficie.pcd", superficie)
                o3d.io.write_point_cloud("ARBOLES.pcd", ARBOLES)
                if carretera:
                    o3d.io.write_point_cloud("talud_i.pcd", talud_i)
                    if tipo_carretera != 'local':
                        o3d.io.write_point_cloud("arcen_i.pcd", arcen_i)
                        o3d.io.write_point_cloud("berma_i.pcd", berma_i)
                    o3d.io.write_point_cloud("carretera_i.pcd", carretera_i)
                    if tipo_carretera in ['autovia','autopista']:
                        o3d.io.write_point_cloud("mediana_i.pcd", mediana_i)
                        o3d.io.write_point_cloud("pcd_barrera.pcd", pcd_barrera)
                    try:
                        o3d.io.write_point_cloud("barrera.pcd", pcd_barrera)
                    except:
                        pass
                    try:
                        o3d.io.write_point_cloud("barrera_quitamiedos_1.pcd", barrera_quitamiedos_1)
                        o3d.io.write_point_cloud("barrera_quitamiedos_2.pcd", barrera_quitamiedos_2)
                    except:
                        pass
                    try:
                        for sss in range(len(SENHALES)):
                            o3d.io.write_point_cloud("senhal_%i.pcd"%sss, SENHALES[sss])
                        o3d.io.write_point_cloud("todas_las_senhales.pcd", todas_las_senhales)
                    except:
                        pass
                
                
            else:    
                os.chdir(ruta_simulaciones)
                os.mkdir('nubes_open3d')
                os.chdir('nubes_open3d')
                
                # Finalmente guardamos cada nube individual para la parte de intensidades:        
                
                o3d.io.write_point_cloud("superficie.pcd", superficie)
                o3d.io.write_point_cloud("ARBOLES.pcd", ARBOLES)
                if carretera:
                    o3d.io.write_point_cloud("talud_i.pcd", talud_i)
                    try:
                        nube_infraestructura_transporte = o3d.geometry.PointCloud()
                        nube_infraestructura_transporte.points = o3d.utility.Vector3dVector(puntos_infraestructura_transporte)
                        nube_infraestructura_transporte.colors = o3d.utility.Vector3dVector(colores_infraestructura_transporte)
                        o3d.io.write_point_cloud("infraestructura_transporte.pcd", nube_infraestructura_transporte)
                    except:
                        pass
                
                
                
            print('----------Pasamos a siguiente nube-----------')
            
    print('----------FIN DE LA CREACIÓN DE LAS NUBES-----------')
    
            
    if calcular_intensidades:
        
        # Volcamos la memoria para no petar el kernel:
        sys.modules[__name__].__dict__.clear()
        
        # Recargamos todos los módulos que necesito:
        import open3d as o3d
        import sys
        import os
        import numpy as np
        import time
        
        sys.path.insert(1, '/home/lino/Documentos/programas_pruebas_varias/simulacion_intensidades')
        import SIMULACION_TRAYECTORIA_INTENSIDAD
        
        ruta_nubes = '/home/lino/Documentos/programas_pruebas_varias/segmentacion_python/segmentacion_bosques/aumentacion_de_datos/Nubes_artificiales_generadas/'
        os.chdir(ruta_nubes)
    
        # Buscamos la carpeta más reciente:
        all_subdirs = [d for d in os.listdir('.') if os.path.isdir(d)]
        latest_subdir = max(all_subdirs, key=os.path.getmtime)
        os.chdir(latest_subdir)
    
        ruta_nubes += latest_subdir
    
        #############
        #lambda_laser=900
        #############
        
        lambda_laser=900
    
        calcular_intensidad_espectral(ruta_nubes,lambda_laser=lambda_laser,visualizacion=False)
    
    
    
    
    
    
    instante_finalisimo = time.time()
    duracion_total = (instante_finalisimo - instante_inicialisimo)/60.
    
    print()
    print()
    
    print('DURACIÓN TOTAL: %f min'%duracion_total)
    
    # PARÓN PARA VISUALIZAR -------------------------------------------------------
    
    # NUBES_ARTIFICIALES.estimate_normals()
    # NUBES_ARTIFICIALES.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=50))
    # VECTOR_LASER_SUPERFICIE = np.array([0,0,1])
    # NUBES_ARTIFICIALES.orient_normals_to_align_with_direction(VECTOR_LASER_SUPERFICIE)
    # visor.custom_draw_geometry_with_key_callback(NUBES_ARTIFICIALES)
    
    '''
    a = 5
    NUBES_ARTIFICIALES[a].estimate_normals()
    visor.custom_draw_geometry_with_key_callback(NUBES_ARTIFICIALES[a])
    '''
    
    
    #------------------------------------------------------------------------------

# Main loop:
if __name__ == "__main__":
    main()
    
