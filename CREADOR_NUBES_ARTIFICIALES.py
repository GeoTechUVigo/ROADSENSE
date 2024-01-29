#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 20 10:43:27 2021

@author: lino
"""

import traceback, sys, code

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
    import shutil
    
    # Relevant paths and files:
    MAIN_PATH = str(pathlib.Path(__file__).parent.resolve())
    CONFIG_PATH = MAIN_PATH+'/config'
    config_file = 'forest_road_backbone.txt'
    UTILS_PATH = MAIN_PATH+'/utils'
    DATA_PATH = MAIN_PATH+'/data'
    OUTPUT_PATH = MAIN_PATH+'/output'

    import sys
    sys.path.insert(1, CONFIG_PATH)
    sys.path.insert(1, UTILS_PATH)
    sys.path.insert(1, DATA_PATH)
    sys.path.insert(1, OUTPUT_PATH)

    
    from reading_trees import read_segments
    from tree_wizard import tree_generator
    from DTM_road_wizard import DTM_road_generator
    from cross_section import highway_vertical_pumping, national_vertical_pumping
    
    # from CREADOR_SURFACE_ARTIFICIALES import creador_SURFACE





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
    spectral_mode = bool(config_file_lines[2][2])
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
     
    #cloud_counter == cloud_counter
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
                
                SURFACE,ROAD,SLOPE,SHOULDER,Signals_list,BERMS,REFUGEE_ISLAND,pcd_central_barrier,pcd_barrier_1,pcd_barrier_2 = DTM_road_generator(
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
                    # SURFACE2,ROAD2,SLOPE2,SHOULDER2,SIGNALS2,BERMS2,REFUGEE_ISLAND2,pcd_barrier_12,pcd_barrier_22 = highway_vertical_pumping(
                    SURFACE,ROAD,SLOPE,SHOULDER,SIGNALS,BERMS,REFUGEE_ISLAND,pcd_central_barrier,pcd_barrier_1,pcd_barrier_2 = highway_vertical_pumping(
                        SURFACE = SURFACE,
                        ROAD = ROAD,
                        SLOPE = SLOPE,
                        SHOULDER = SHOULDER,
                        Signals_list = Signals_list,
                        BERMS = BERMS,
                        REFUGEE_ISLAND = REFUGEE_ISLAND,
                        pcd_central_barrier = pcd_central_barrier,
                        pcd_barrier_1 = pcd_barrier_1,
                        pcd_barrier_2 = pcd_barrier_2)
                    
                    #**********************************************************************
                    # DEBUG PRINCIPAL                             (donde estoy traduciendo)
                    #**********************************************************************        
                    print('LINO! COMPARA LAS VARIABLES TIPO 2 CON LAS ANTERIORES')
                    import pdb
                    pdb.set_trace()

                    
                    
                elif vertical_pumping and road_type == 'national':
                    SURFACE,ROAD,SLOPE,SHOULDER,SIGNALS,BERMS,REFUGEE_ISLAND,pcd_central_barrier,pcd_barrier_1,pcd_barrier_2,SIGNALS = national_vertical_pumping(
                        SURFACE = SURFACE,
                        ROAD = ROAD,
                        SLOPE = SLOPE,
                        SHOULDER = SHOULDER,
                        Signals_list = Signals_list,
                        BERMS = BERMS,
                        REFUGEE_ISLAND = REFUGEE_ISLAND,
                        pcd_central_barrier = pcd_central_barrier,
                        pcd_barrier_1 = pcd_barrier_1,
                        pcd_barrier_2 = pcd_barrier_2)
                    
                                    

                elif road_type == 'local':
                    SURFACE,ROAD,SLOPE,SHOULDER,Signals_list,pcd_barrier_1,pcd_barrier_2 = DTM_road_generator(
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
            SURFACE = creador_SURFACE(ORIGINAL_SEGMENTS,
                                          scale = scale,
                                          number_points_DTM = number_points_DTM)
                                          # montana=montana,
                                          # visualizacion=False,
                                          # voxel_downsampling_size=voxel_downsampling_size)
                                          
            end_surface = time.time()
            
            print('Finished simulation of DTM: %f s'%(end_surface-initial_instant))
            
            
        #========================CREACIÓN ÁRBOLES ARTIFICIALES=========================
        
        # Vamos a generar árboles nuevos basándonos en los que ya hemos leído por aHOUR:
            
        SYNTHETIC_SEGMENTS = tree_generator(ORIGINAL_SEGMENTS,
                                                number_of_transformations)
        
        
        end_synthetic_trees = time.time()
        
        print('Creation of synthetic trees completed: %f s'%(end_synthetic_trees-end_surface))
    
        #==============================FUSIÓN DE DATOS=================================
        
        # Ya tenemos los siguientes packs de datos:
        # · ORIGINAL_SEGMENTS
        # · SURFACE
        # · SYNTHETIC_SEGMENTS
        
        # Lo que nos queda es fusionarlos, es decir, colocar todos los árboles que po-
        # damos en cada surface. Para ello seguiremos la siguiente estrategia:
        # 1) Seleccionamos una surface.
        # 2) Seleccionamos un árbol y lo colocamos en una posición arbitraria de la su-
        #    perficie siempre y cuando no posea points fuera de la misma.
        # 3) Seleccionamos otro árbol y le buscamos otra posición aleatoria. En esa po-
        #    sición buscamos el árbol más próximo y hacemos 2 fits: uno a la copa de
        #    cada árbol.
        # 4) Si los dos fits no poseen points en común (no intersecan), damos por váli-
        #    da la posición y saltamos al siguiente paso. En caso contrario, repetimos
        #    el paso 3 hasta que no tenga problemas.
        # 5) Si en una cantidad de iteraciones (llamémosla N_it) no se encuentra una
        #    buena posición para el árbol, paramos y damos por terminada la nueva nube.
        
        # NOTA: AL FINAL DE TODO LAS NUBES DE points ARTIFICIALES DEBE ESTAR CLASIFICA-
        #       DAS, ES DECIR, VER CÓMO GUARDAR EN .laz Ó .las CADA PUNTO COMO:
        #                  (x_punto,  y_punto,  z_punto,  clasificacion)
    
    
    
        # Comportamiento que debe obedecer: asignar una label a cada punto tipo ár-
        # bol y guardar la nube final como .las o .laz.
        
        # cloud_counter = 0
        # for i in range(len(SURFACE)):
        print('Generating synthetic point cloud %i/%i'%(i+1,number_of_clouds))
        TREES = []
        SYNTHETIC_PCDS = {}
        surface = SURFACE
        TREES.append([])
        if road:
            road_i = ROAD
            slope_i = SLOPE
            if road_type != 'local':
                shoulder_i = SHOULDER
                berm_i = BERMS
            if road_type in ['highway','mixed']:
                refugee_island_i = REFUGEE_ISLAND
    
        surface_points = np.array(surface.points)
        
        if road_type in ['highway','mixed']:
            refugee_island_points = np.array(refugee_island_i.points)            
    
        SYNTHETIC_SEGMENTS_aux = copy.deepcopy(SYNTHETIC_SEGMENTS)

        counter_aux = 0
        for j in range(number_of_trees):
            
            if j < number_of_trees-number_trees_refugee_island:
            
                # If we finished placing all created trees we start again:
                if counter_aux == len(SYNTHETIC_SEGMENTS_aux):
                    SYNTHETIC_SEGMENTS_aux = copy.deepcopy(SYNTHETIC_SEGMENTS)
                    counter_aux = 0
    
                possible_location = surface_points[np.random.choice(len(surface_points))]
                
                index = np.random.choice(len(SYNTHETIC_SEGMENTS_aux))
                # Let's choose randomly a tree:
                tree = SYNTHETIC_SEGMENTS_aux[index]
        
                while tree[1] == True:
                    index = np.random.choice(len(SYNTHETIC_SEGMENTS_aux))
                    tree = SYNTHETIC_SEGMENTS_aux[index]
                if tree[1] == False:
                    counter_aux += 1
                    
                tree = tree[0]
                # We add a label regarding "True" value since it was already
                # selected:
                SYNTHETIC_SEGMENTS_aux[index][1] = True
                SYNTHETIC_SEGMENTS[index][1] = False
                tree.translate((possible_location),relative=False)
                
                tree_points = np.array(tree.points)
                minimum_height = tree_points.take(2,1).min()
                
                color_tree = np.array(tree.colors)
                
                vertical_shift = possible_location-minimum_height
                
                tree_points[:, 2] = tree_points.take(2,1) + vertical_shift[2]
                tree = o3d.geometry.PointCloud()
                tree.points = o3d.utility.Vector3dVector(tree_points)
                tree.colors = o3d.utility.Vector3dVector(color_tree)
                
                #-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
                # THIS IS IMPORTANT FOR THE SPECTRAL PART!!!:           
                if spectral_mode:   
                    tree.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=100, max_nn=1000))
                #-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
          
                TREES[-1].append(tree)
                
                if cloud_counter == 0:
        
                    synthetic_cloud = tree+surface
                    cloud_counter += 1
        
                else:
                    synthetic_cloud += tree
            
            elif j >= number_of_trees-number_trees_refugee_island:
                # We start again:
                if counter_aux == len(SYNTHETIC_SEGMENTS_aux):
                    SYNTHETIC_SEGMENTS_aux = copy.deepcopy(SYNTHETIC_SEGMENTS)
                    counter_aux = 0
    
                possible_location = refugee_island_points[np.random.choice(len(refugee_island_points))]
                
                index = np.random.choice(len(SYNTHETIC_SEGMENTS_aux))
                # Let's choose randomly a tree:
                tree = SYNTHETIC_SEGMENTS_aux[index]
        
                while tree[1] == True:
                    index = np.random.choice(len(SYNTHETIC_SEGMENTS_aux))
                    tree = SYNTHETIC_SEGMENTS_aux[index]
                if tree[1] == False:
                    counter_aux += 1
                    
                tree = tree[0]
                # We add a label regarding "True" value since it was already
                # selected:
                SYNTHETIC_SEGMENTS_aux[index][1] = True
                SYNTHETIC_SEGMENTS[index][1] = False
                tree.translate((possible_location),relative=False)
                
                tree_points = np.array(tree.points)
                minimum_height = tree_points.take(2,1).min()
                
                color_tree = np.array(tree.colors)
                
                vertical_shift = possible_location-minimum_height
                
                tree_points[:, 2] = tree_points.take(2,1) + vertical_shift[2]
                tree = o3d.geometry.PointCloud()
                tree.points = o3d.utility.Vector3dVector(tree_points)
                tree.colors = o3d.utility.Vector3dVector(color_tree)
                
                #-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
                # THIS IS IMPORTANT FOR THE SPECTRAL PART!!!:           
                if spectral_mode:   
                    tree.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=100, max_nn=1000))
                #-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
                
                
                TREES[-1].append(tree)
                
                if cloud_counter == 0:
        
                    synthetic_cloud = tree+surface
                    cloud_counter += 1
        
                else:
                    synthetic_cloud += tree



            elif j >= number_of_trees-20:            
                # We start again:
                if counter_aux == len(SYNTHETIC_SEGMENTS_aux):
                    SYNTHETIC_SEGMENTS_aux = copy.deepcopy(SYNTHETIC_SEGMENTS)
                    counter_aux = 0
    
                possible_location = surface_points[np.random.choice(len(surface_points))]
                
                index = np.random.choice(len(SYNTHETIC_SEGMENTS_aux))
                # Let's choose randomly a tree:
                tree = SYNTHETIC_SEGMENTS_aux[index]

                while tree[1] == True:
                    index = np.random.choice(len(SYNTHETIC_SEGMENTS_aux))
                    tree = SYNTHETIC_SEGMENTS_aux[index]
                if tree[1] == False:
                    counter_aux += 1
                    
                tree = tree[0]
                # We add a label regarding "True" value since it was already
                # selected:
                SYNTHETIC_SEGMENTS_aux[index][1] = True
                SYNTHETIC_SEGMENTS[index][1] = False
                tree.translate((possible_location),relative=False)
                
                tree_points = np.array(tree.points)
                minimum_height = tree_points.take(2,1).min()
                
                color_tree = np.array(tree.colors)
                
                vertical_shift = possible_location-minimum_height
                
                tree_points[:, 2] = tree_points.take(2,1) + vertical_shift[2]
                tree = o3d.geometry.PointCloud()
                tree.points = o3d.utility.Vector3dVector(tree_points)
                tree.colors = o3d.utility.Vector3dVector(color_tree)
                
                #-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
                # THIS IS IMPORTANT FOR THE SPECTRAL PART!!!:           
                if spectral_mode:   
                    tree.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=100, max_nn=1000))
                #-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
                
                
                TREES[-1].append(tree)
                
                if cloud_counter == 0:
        
                    synthetic_cloud = tree+surface
                    cloud_counter += 1
        
                else:
                    synthetic_cloud += tree
            
            
            
        # We merge all the collected trees in one single element:
        tree = o3d.geometry.PointCloud()
        
        for tree_i in TREES[-1]:
            tree += tree_i
        
        TREES = tree
        
        # TODO:
        # if voxel_downsampling_size < 1000:
        #     # Vamos a hacer voxel_downsampling:
        #     TREES = TREES.voxel_down_sample(voxel_size=voxel_downsampling_size)

        
        if road:
            if road_type in ['highway','mixed']:
                SYNTHETIC_PCDS = synthetic_cloud + road_i + slope_i + shoulder_i + berm_i + refugee_island_i + pcd_central_barrier + pcd_barrier_1 + pcd_barrier_2 + SIGNALS
            if road_type == 'national':
                SYNTHETIC_PCDS = synthetic_cloud + road_i + slope_i + shoulder_i + berm_i + pcd_barrier_1 + pcd_barrier_2 + SIGNALS
            if road_type == 'local':
                SYNTHETIC_PCDS = synthetic_cloud + road_i + slope_i + SIGNALS
            
            
        else:
            SYNTHETIC_PCDS = synthetic_cloud
        cloud_counter = 0

        final_time_cloud_generation = time.time()
        
        print('Syntehtic cloud generated in: %f s'%(final_time_cloud_generation-initial_instant))
        # instante_inicial = time.time()
    
        
        o3d.visualization.draw(SYNTHETIC_PCDS)
    
                
        #=============================PCD STORE=================================
        
        os.chdir(OUTPUT_PATH)    
                
        current_date = time.localtime()
        
        
        if i == 0:
            folder = 'SYNTHETIC_PCDS_%i_%i_%i___%i_%i_%i'%(current_date[0],
                                                            current_date[1],
                                                            current_date[2],
                                                            current_date[3],
                                                            current_date[4],
                                                            current_date[5])
            
            os.mkdir(folder)
        os.chdir(folder)
        
        
        # We will create a .txt file containing useful info:
            
        with open("dataset_info.txt", "w") as f:
            f.writelines('DATASET INFORMATION:\n')
            if not road:
                f.writelines('Type of clouds: forest\n')
                f.writelines('Number of classes: 2\n')
                f.writelines('--- LABELS ---\n')
                f.writelines('label    class\n')
                f.writelines('0    tree\n')
                f.writelines('1    DTM\n')
            else:
                f.writelines('Type of clouds %s\n'%road_type)
                if road_type == 'local':
                    f.writelines('Number of classes: 5\n')
                    f.writelines('--- LABELS ---\n')
                    f.writelines('label    class\n')
                    f.writelines('0    tree\n')
                    f.writelines('1    DTM\n')
                    f.writelines('2    road\n')
                    f.writelines('3    slope\n')
                    f.writelines('4    traffic signal\n')
                if road_type == 'national':
                    f.writelines('Number of classes: 8\n')
                    f.writelines('--- LABELS ---\n')
                    f.writelines('label    class\n')
                    f.writelines('0    tree\n')
                    f.writelines('1    DTM\n')
                    f.writelines('2    road\n')
                    f.writelines('3    slope\n')
                    f.writelines('4    traffic signal\n')
                    f.writelines('5    pcd_barrier_1\n')
                    f.writelines('6    pcd_barrier_2\n')
                    f.writelines('7    shoulder\n')
                if road_type in ['highway','mixed']:
                    f.writelines('Number of classes: 11\n')
                    f.writelines('--- LABELS ---\n')
                    f.writelines('label    class\n')
                    f.writelines('0    tree\n')
                    f.writelines('1    DTM\n')
                    f.writelines('2    road\n')
                    f.writelines('3    slope\n')
                    f.writelines('4    traffic signal\n')
                    f.writelines('5    pcd_barrier_1\n')
                    f.writelines('6    pcd_barrier_2\n')
                    f.writelines('7    shoulder\n')
                    f.writelines('8    refugee island\n')
                    f.writelines('9    jersey barrier\n')
                    f.writelines('10    berm\n')
        f.close()
    
        
        
        # Saving data:
        
        os.mkdir("synthetic_cloud_%i"%i)    
        os.chdir("synthetic_cloud_%i"%i)
        path_simulations = os.getcwd()
        
        pcd = SYNTHETIC_PCDS
        cloud_points = np.array(pcd.points)
        
        o3d.io.write_point_cloud("synthetic_cloud_%i.pcd"%i, pcd)
    
        # We also write a .las file:
    
            
        # 1. Create a new header
        header = laspy.LasHeader(point_format=3, version="1.2")
        # header.add_extra_dim(laspy.ExtraBytesParams(name="intensidad_faro", type=np.float32))
        header.offsets = np.min(cloud_points, axis=0)
        header.scales = np.array([0.1, 0.1, 0.1])
        
        # 2. Create a Las
        las = laspy.LasData(header)
        
        las.x = cloud_points[:, 0]
        las.y = cloud_points[:, 1]
        las.z = cloud_points[:, 2]
        # las.intensidad_faro = intensidades_FARO_comunes
        
        las.write("synthetic_cloud_%i.las"%i)
    
        
        '''
        # OLD WAY TO WRITE A LAS FILE:
        hdr = laspy.header.Header(point_format=2)
        
        outfile = laspy.file.File("synthetic_cloud_%i.las"%i, mode="w", header=hdr)
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
        

        shutil.copyfile(CONFIG_PATH+'/'+config_file,
                        path_simulations+'/'+config_file)
            
        with open("parametros.txt", "w") as f:
            f.writelines('--------------------------------------------------------\n')
            f.writelines('DATE: %i/%i/%i    HOUR: %i/%i/%i\n'%(time.gmtime()[2],time.gmtime()[1],time.gmtime()[0],time.gmtime()[3]+2,time.gmtime()[4],time.gmtime()[5]))
            f.writelines('--------------------------------------------------------\n')
            f.writelines('Time reading (s): %f\n'%(end_reading-initial_instant))
            f.writelines('Time surface creation (s): %f\n'%(end_surface-end_reading))
            f.writelines('Time tree generation (s): %f\n'%(end_synthetic_trees-end_surface))
            f.writelines('Total time (s): %f\n'%(final_time_cloud_generation-initial_instant))
            f.writelines('--------------------------------------------------------\n')
            f.close()
            
        os.mkdir('numpy_arrays')
        os.chdir('numpy_arrays')
                    
        surface_points = np.array(SURFACE.points)
        
        # labels_surface = np.full((1,len(surface_points)),0)[0]
        with open("synthetic_cloud_%i_DTM.npy"%i, 'wb') as f:    
            np.save(f, surface_points)
            
        tree_pointsS = np.array(TREES.points)
        # labels_TREES = np.full((1,len(tree_pointsS)),1)[0]
        with open("synthetic_cloud_%i_TREES.npy"%i, 'wb') as f:    
            np.save(f, tree_pointsS)
        
        if road:
            points_road = np.array(ROAD.points)
            # labels_road = np.full((1,len(points_road)),2)[0]
            with open("synthetic_cloud_%i_road.npy"%i, 'wb') as f:    
                np.save(f, points_road)
            points_slope = np.array(SLOPE.points)
            # labels_slope = np.full((1,len(points_slope)),4)[0]
            with open("synthetic_cloud_%i_slope.npy"%i, 'wb') as f:    
                np.save(f, points_slope)
            all_signals = np.array(SIGNALS.points)
            # labels_slope = np.full((1,len(points_slope)),4)[0]
            with open("synthetic_cloud_%i_senhales.npy"%i, 'wb') as f:    
                np.save(f, all_signals)
            
            
            if road_type == 'local':
                
                cloud_points = np.concatenate((surface_points,tree_pointsS,
                                              points_road,points_slope,
                                              all_signals))
        
                with open("synthetic_cloud_%i.npy"%i, 'wb') as f:    
                    np.save(f, cloud_points)
            
            
            
            
            if road_type != 'local':
                
                points_pcd_barrier_1 = np.array(pcd_barrier_1.points)
                with open("synthetic_cloud_%i_pcd_barrier_1.npy"%i, 'wb') as f:    
                    np.save(f, points_pcd_barrier_1)
                points_pcd_barrier_2 = np.array(pcd_barrier_2.points)
                with open("synthetic_cloud_%i_pcd_barrier_2.npy"%i, 'wb') as f:    
                    np.save(f, points_pcd_barrier_2)
                points_shoulder = np.array(SHOULDER[0].points)
                # labels_arcen = np.full((1,len(points_shoulder)),3)[0]
                with open("synthetic_cloud_%i_arcen.npy"%i, 'wb') as f:    
                    np.save(f, points_shoulder)
                points_berm = np.array(berm_i.points)
                with open("synthetic_cloud_%i_berm.npy"%i, 'wb') as f:    
                    np.save(f, points_berm)
                    
            
            
            if road_type == 'national':
                
                cloud_points = np.concatenate((surface_points,tree_pointsS,
                                              points_road,points_slope,
                                              points_shoulder,points_pcd_barrier_1,
                                              points_pcd_barrier_2,
                                              all_signals))
        
                with open("synthetic_cloud_%i.npy"%i, 'wb') as f:    
                    np.save(f, cloud_points)
                
            
            if road_type in ['highway','mixed']:
                points_central_barrier = np.array(pcd_central_barrier.points)
                with open("synthetic_cloud_%i_jersey_barrier.npy"%i, 'wb') as f:    
                    np.save(f, points_central_barrier)
                refugee_island_points = np.array(refugee_island_i.points)
                with open("synthetic_cloud_%i_refugee island.npy"%i, 'wb') as f:    
                    np.save(f, refugee_island_points)
            
                cloud_points = np.concatenate((surface_points,tree_pointsS,
                                              points_road,points_slope,
                                              points_shoulder,points_pcd_barrier_1,
                                              points_pcd_barrier_2,
                                              points_central_barrier,all_signals,
                                              refugee_island_points))
        
               
                
                with open("synthetic_cloud_%i.npy"%i, 'wb') as f:    
                    np.save(f, cloud_points)
        
            # labels_nube = np.concatenate((labels_surface,labels_road,
            #                               labels_arcen,labels_slope))
    
        

        with open("synthetic_cloud_%i_TREES.npy"%i, 'wb') as f:    
            np.save(f, tree_pointsS)
            
    
    
    
        os.chdir(path_simulations)
        os.mkdir('open3d_clouds')
        os.chdir('open3d_clouds')
        
        # This is important for the spectral part:        
        
        o3d.io.write_point_cloud("surface.pcd", surface)
        o3d.io.write_point_cloud("TREES.pcd", TREES)
        if road:
            o3d.io.write_point_cloud("slope_i.pcd", slope_i)
            if road_type != 'local':
                o3d.io.write_point_cloud("shoulder_i.pcd", shoulder_i)
                o3d.io.write_point_cloud("berm_i.pcd", berm_i)
            o3d.io.write_point_cloud("road_i.pcd", road_i)
            if road_type in ['highway','mixed']:
                o3d.io.write_point_cloud("refugee_island_i.pcd", refugee_island_i)
                o3d.io.write_point_cloud("pcd_central_barrier.pcd", pcd_central_barrier)
            try:
                o3d.io.write_point_cloud("barrera.pcd", pcd_central_barrier)
            except:
                pass
            try:
                o3d.io.write_point_cloud("pcd_barrier_1.pcd", pcd_barrier_1)
                o3d.io.write_point_cloud("pcd_barrier_2.pcd", pcd_barrier_2)
            except:
                pass
            try:
                for sss in range(len(Signals_list)):
                    o3d.io.write_point_cloud("signal_%i.pcd"%sss, Signals_list[sss])
                o3d.io.write_point_cloud("all_signals.pcd", all_signals)
            except:
                pass
        
            
            
            
        print('----------Jumping to next cloud-----------')
            
    print('----------END OF POINT CLOUD GENERATION-----------')
    
            
    if spectral_mode:
        
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
        
        ruta_nubes = '/home/lino/Documentos/programas_pruebas_varias/segmentacion_python/segmentacion_bosques/aumentacion_de_datos/SYNTHETIC_PCDS_generadas/'
        os.chdir(ruta_nubes)
    
        # Buscamos la folder más reciente:
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
    
    # SYNTHETIC_PCDS.estimate_normals()
    # SYNTHETIC_PCDS.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=50))
    # VECTOR_LASER_surface = np.array([0,0,1])
    # SYNTHETIC_PCDS.orient_normals_to_align_with_direction(VECTOR_LASER_surface)
    # visor.custom_draw_geometry_with_key_callback(SYNTHETIC_PCDS)
    
    '''
    a = 5
    SYNTHETIC_PCDS[a].estimate_normals()
    visor.custom_draw_geometry_with_key_callback(SYNTHETIC_PCDS[a])
    '''
    
    
    #------------------------------------------------------------------------------

# Main loop:
if __name__ == "__main__":
    main()
    
    
    # try:
    #     main()
    # except:
    #     # Debug on errors:
    #     type, value, tb = sys.exc_info()
    #     traceback.print_exc()
    #     last_frame = lambda tb=tb: last_frame(tb.tb_next) if tb.tb_next else tb
    #     frame = last_frame().tb_frame
    #     ns = dict(frame.f_globals)
    #     ns.update(frame.f_locals)
    #     code.interact(local=ns)