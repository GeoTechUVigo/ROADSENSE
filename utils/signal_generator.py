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
# import matplotlib.pyplot as plt
# import trimesh
# from scipy.spatial import Delaunay
# import itertools
# from matplotlib import cm
# from matplotlib.colors import LightSource
# import math


SIGNALS = {}

def create_elevated_signal(road_height,middle=True,visualization=False,center_first_pole=[0,0,0],voxel_downsampling_size=1000000):
    """
    Function to generate elevated big traffic signals (highway and mixed roads)

    :param road_height: Height of the road where the simulator starts to generate the signal pole. Must be an int or float value
    :param middle: Boolean that indicates whether the signal must be splitted into multiple squares or not
    :param visualization: Boolean that indicates if debug visualizations are required or not
    :param center_first_pole: List with [X, Y, Z] coordinates of the main pole
    :param voxel_downsampling_size: [Experimental] Size of the cell within the downsampling
    :return: Point cloud of the full signal and coordinates of each pole center
    """ 

    # Only valid for highway and mixed types of road.
    pole_radius = 0.15
    crossbar_radius = 0.1
    
    pole_height = 7
    crossbar_height = 4.5    
    
    
    heightS = np.linspace(road_height,pole_height,50)
    angleS = np.linspace(0,2*np.pi,100)
    
    pole_1 = []
    
    for i in range(len(heightS)):
        for j in range(len(angleS)):
            x = pole_radius*np.cos(angleS[j])
            y = pole_radius*np.sin(angleS[j])
            pole_1.append([x,y,heightS[i]])
        
    pole_1 = np.array(pole_1)    
    
    if voxel_downsampling_size < 1000:
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(pole_1)
        pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
        pole_1 = np.array(pcd2.points)

    # Now we add some gaussian noise:
    noise_x_axis = np.random.normal(center_first_pole[0],0.005,len(pole_1.take(0,1)))
    noise_y_axis = np.random.normal(center_first_pole[1],0.005,len(pole_1.take(0,1)))
    noise_z_axis = np.random.normal(center_first_pole[2],0.08,len(pole_1.take(0,1)[1:-2]))
    
    pole_1[:, 0] = np.reshape(pole_1.take(0,1) + noise_x_axis, -1)
    pole_1[:, 1] = np.reshape(pole_1.take(1,1) + noise_y_axis, -1)
    pole_1[1:-2, 2] = np.reshape(pole_1.take(2,1)[1:-2] + noise_z_axis, -1)
    
    pcd_pole_1 = o3d.geometry.PointCloud()
    pcd_pole_1.points = o3d.utility.Vector3dVector(pole_1)
    pcd_pole_1.paint_uniform_color(np.array([160/255.,160/255.,160/255.]))
    
    SIGNALS[len(SIGNALS)] = pcd_pole_1
    
    #------------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # visor.custom_draw_geometry_with_key_callback(SIGNAL)
    #------------------------------------------------------------------------------
    
    # crossbar_width = 10
    crossbar_width = 13.5
    
    pcd_pole_2 = copy.deepcopy(pcd_pole_1).translate((crossbar_width, center_first_pole[1], center_first_pole[2]))
    
    SIGNALS[len(SIGNALS)] = pcd_pole_2
    
    center_second_pole = [crossbar_width,center_first_pole[1],center_first_pole[2]]
    
    #------------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # visor.custom_draw_geometry_with_key_callback(SIGNAL,segmentado=True,pcd2=SIGNAL[0])
    #------------------------------------------------------------------------------
    
    lenghts = np.linspace(center_first_pole[0],crossbar_width,50)
    angleS = np.linspace(0,2*np.pi,100)
    
    pole_3 = []
    
    for i in range(len(lenghts)):
        for j in range(len(angleS)):
            y = crossbar_radius*np.cos(angleS[j])
            z = crossbar_radius*np.sin(angleS[j])
            pole_3.append([lenghts[i],y,z+crossbar_height])
        
    pole_3 = np.array(pole_3)    
    
    if voxel_downsampling_size < 1000:
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(pole_3)
        pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
        pole_3 = np.array(pcd2.points)
    
    # Now we add some gaussian noise:
    noise_x_axis = np.random.normal(center_first_pole[0],0.05,len(pole_3.take(0,1)))
    noise_y_axis = np.random.normal(center_first_pole[1],0.005,len(pole_3.take(0,1)))
    noise_z_axis = np.random.normal(center_first_pole[2],0.005,len(pole_3.take(0,1)[1:-2]))
    
    pole_3[:, 0] = np.reshape(pole_3.take(0,1) + noise_x_axis, -1)
    pole_3[:, 1] = np.reshape(pole_3.take(1,1) + noise_y_axis, -1)
    pole_3[1:-2, 2] = np.reshape(pole_3.take(2,1)[1:-2] + noise_z_axis, -1)
    
    pcd_pole_3 = o3d.geometry.PointCloud()
    pcd_pole_3.points = o3d.utility.Vector3dVector(pole_3)
    pcd_pole_3.paint_uniform_color(np.array([160/255.,160/255.,160/255.]))
    
    SIGNALS[len(SIGNALS)] = pcd_pole_3
    
    
    #------------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # visor.custom_draw_geometry_with_key_callback(SIGNAL,segmentado=True,pcd2=SIGNAL[0])
    #------------------------------------------------------------------------------
    
    pcd_pole_4 = copy.deepcopy(pcd_pole_3).translate((center_first_pole[0], center_first_pole[1], crossbar_height/2.))
    
    SIGNALS[len(SIGNALS)] = pcd_pole_4
    
    #------------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # visor.custom_draw_geometry_with_key_callback(SIGNAL,segmentado=True,pcd2=SIGNAL[0])
    #------------------------------------------------------------------------------
    
    
    if middle == False:
        x_axis = np.linspace(crossbar_width/10.,crossbar_width-(crossbar_width/10.),70)
        y_axis = np.full((1,len(x_axis)),np.mean(pole_3.take(1,1))+crossbar_radius)[0]
        z_axis = np.linspace(crossbar_height-(crossbar_height/5.),crossbar_height+(crossbar_height/1.5),50)
    
        frame = []
    
        for i in range(len(x_axis)):
            for j in range(len(y_axis)):
                for k in range(len(z_axis)):
                    point = [x_axis[i],y_axis[j],z_axis[k]]
                    
                    # Now we add some gaussian noise:
                    noise_x_axis = np.random.normal(center_first_pole[0],0.05,1)[0]
                    noise_y_axis = np.random.normal(center_first_pole[1],0.005,1)[0]
                    noise_z_axis = np.random.normal(center_first_pole[2],0.01,1)[0]
                    
                    point[0] = point[0] + noise_x_axis
                    point[1] = point[1] + noise_y_axis
                    point[2] = point[2] + noise_z_axis
                    
                    frame.append(point)
    
        frame = np.array(frame)
    
        if voxel_downsampling_size < 1000:
            pcd2 = o3d.geometry.PointCloud()
            pcd2.points = o3d.utility.Vector3dVector(frame)
            pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
            frame = np.array(pcd2.points)
    
        pcd_frame = o3d.geometry.PointCloud()
        pcd_frame.points = o3d.utility.Vector3dVector(frame)
        pcd_frame.paint_uniform_color(np.array([0,0,1]))
    
        pcd_frame_aux = copy.deepcopy(pcd_frame).translate((center_first_pole[0], 0.005, center_first_pole[2]))
        pcd_frame = pcd_frame + pcd_frame_aux
        
        pcd_frame_aux = copy.deepcopy(pcd_frame).translate((center_first_pole[0], 0.005, center_first_pole[2]))
        pcd_frame = pcd_frame + pcd_frame_aux
    
        SIGNALS[len(SIGNALS)] = pcd_frame
        
    else:
        x_axis_1 = np.linspace(crossbar_width/20.,(crossbar_width-(crossbar_width/10.))*0.6,70)
        y_axis_1 = np.full((1,len(x_axis_1)),np.mean(pole_3.take(1,1))+crossbar_radius)[0]
        z_axis_1 = np.linspace(crossbar_height-(crossbar_height/5.),crossbar_height+(crossbar_height/1.5),50)
    
        frame_1 = []
    
        for i in range(len(x_axis_1)):
            for j in range(len(y_axis_1)):
                for k in range(len(z_axis_1)):
                    point = [x_axis_1[i],y_axis_1[j],z_axis_1[k]]
                    
                    # Now we add some gaussian noise:
                    noise_x_axis = np.random.normal(center_first_pole[0],0.04,1)[0]
                    noise_y_axis = np.random.normal(center_first_pole[1],0.005,1)[0]
                    noise_z_axis = np.random.normal(center_first_pole[2],0.02,1)[0]
                    
                    point[0] = point[0] + noise_x_axis
                    point[1] = point[1] + noise_y_axis
                    point[2] = point[2] + noise_z_axis
                    
                    frame_1.append(point)
    
        if voxel_downsampling_size < 1000:
            pcd2 = o3d.geometry.PointCloud()
            pcd2.points = o3d.utility.Vector3dVector(frame_1)
            pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
            frame_1 = np.array(pcd2.points)

    
        frame_1 = np.array(frame_1)
    
        pcd_frame_1 = o3d.geometry.PointCloud()
        pcd_frame_1.points = o3d.utility.Vector3dVector(frame_1)
    
        pcd_frame_aux = copy.deepcopy(pcd_frame_1).translate((center_first_pole[0], 0.005, center_first_pole[2]))
        pcd_frame_1 = pcd_frame_1 + pcd_frame_aux
        
        # We paint the frame in blue:
        pcd_frame_1.paint_uniform_color(np.array([0,0,1]))    
        
        SIGNALS[len(SIGNALS)] = pcd_frame_1
    
        pcd_frame_2 = copy.deepcopy(pcd_frame_1).translate(((crossbar_width-(crossbar_width/20.))*0.6, center_first_pole[1], center_first_pole[2]))
    
        frame_2 = np.array(pcd_frame_2.points)
        indexes_delete = np.where(frame_2.take(0,1) < (crossbar_width-(crossbar_width/20.)))[0]
        frame_2 = frame_2[indexes_delete]
        pcd_frame_2 = o3d.geometry.PointCloud()
        pcd_frame_2.points = o3d.utility.Vector3dVector(frame_2)
    
        pcd_frame_aux = copy.deepcopy(pcd_frame_2).translate((center_first_pole[0], 0.005, center_first_pole[2]))
        pcd_frame_2 = pcd_frame_2 + pcd_frame_aux
        pcd_frame_aux = copy.deepcopy(pcd_frame_2).translate((center_first_pole[0], 0.005, center_first_pole[2]))
        pcd_frame_2 = pcd_frame_2 + pcd_frame_aux
        pcd_frame_aux = copy.deepcopy(pcd_frame_2).translate((center_first_pole[0], 0.005, center_first_pole[2]))
        pcd_frame_2 = pcd_frame_2 + pcd_frame_aux
    
        # We paint the frame in a darker blue:
        pcd_frame_2.paint_uniform_color(np.array([0,0,204/255.]))
        
        SIGNALS[len(SIGNALS)] = pcd_frame_2
    
    
    individual_signal = SIGNALS[len(SIGNALS)-5]
    for a in range(len(SIGNALS)-4,len(SIGNALS)):
        individual_signal = individual_signal + SIGNALS[a]

    # Just notation:
    SIGNAL = individual_signal    

    
    signal_center = SIGNAL.get_center()
    
    #------------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # if visualization:
        # visor.custom_draw_geometry_with_key_callback(SIGNALS,segmentado=True,pcd2=SIGNAL)
    #------------------------------------------------------------------------------

    return SIGNAL,signal_center,center_first_pole,center_second_pole



#**********************************************************************
#??? DEBUG SECUNDARIO
#**********************************************************************        



def create_triangular_signal(final_position,road_type,voxel_downsampling_size=1000000):
    """
    Function to generate vertical-triangular signals

    :param final_position: List with [X, Y, Z] coordinates of the main pole position
    :param road_type: String with the type of road
    :param voxel_downsampling_size: [Experimental] Size of the cell within the downsampling
    :return: Point cloud of the full signal and coordinates of each pole center
    """ 
    position = [0,0,0]
    
    
    if road_type in ['autopista','autovia']:
        
        lado = 1.750 # Cada lado del triángulo mide 1.75 por ley
        altura_maxima_poste = 3
        pole_radius = 0.03
        
    elif road_type in ['nacional','comarcal']:
        
        lado = 1.350 # Cada lado del triángulo mide 1.35 por ley
        altura_maxima_poste = 3
        pole_radius = 0.03
        
    elif road_type == 'local':
        
        lado = 0.900 # Cada lado del triángulo mide 1.35 por ley
        altura_maxima_poste = 3
        pole_radius = 0.03
    

    heightS_poste = np.linspace(position[2],altura_maxima_poste,50)       
    angleS = np.linspace(0,2*np.pi,100)
    
    POSTE = []
    
    for i in range(len(heightS_poste)):
        for j in range(len(angleS)):
            x = pole_radius*np.cos(angleS[j])
            y = pole_radius*np.sin(angleS[j])
            POSTE.append([x,y,heightS_poste[i]])
        
    POSTE = np.array(POSTE)    
    
    if voxel_downsampling_size < 1000:
        # Vamos a hacer voxel_downsampling:
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(POSTE)
        pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
        POSTE = np.array(pcd2.points)    
    
    # Ahora le añadimos algo de ruido gaussiano a cada point para hacerlo más real:
    noise_x_axis = np.random.normal(position[0],0.005,len(POSTE.take(0,1)))
    noise_y_axis = np.random.normal(position[1],0.005,len(POSTE.take(0,1)))
    noise_z_axis = np.random.normal(position[2],0.08,len(POSTE.take(0,1)[1:-2]))
    
    POSTE[:, 0] = np.reshape(POSTE.take(0,1) + noise_x_axis, -1)
    POSTE[:, 1] = np.reshape(POSTE.take(1,1) + noise_y_axis, -1)
    POSTE[1:-2, 2] = np.reshape(POSTE.take(2,1)[1:-2] + noise_z_axis, -1)
    
    poste = o3d.geometry.PointCloud()
    poste.points = o3d.utility.Vector3dVector(POSTE)
    poste.paint_uniform_color(np.array([160/255.,160/255.,160/255.]))
    
    SIGNALS[len(SIGNALS)] = poste

    #------------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # visor.custom_draw_geometry_with_key_callback(poste)
    #------------------------------------------------------------------------------


    # Vamos a crear un triángulo.
    # El primer point será el vértice superior:
    
    N_points = int(300/2)
    x_axis_1 = np.linspace(0,lado/2.,N_points)
    x_axis_2 = np.linspace(lado/2.,lado,len(x_axis_1))

    altura_triangulo_maxima = np.sqrt(lado**2 - (lado/2.)**2)
    heightS_triangulo = np.linspace(0,altura_triangulo_maxima,len(x_axis_1))

    TRIANGULO = []

    # Calculo la ecuación de la recta que pasa por las middlees del
    # triángulo (z = mx + n):
    m1 = altura_triangulo_maxima/(lado/2.)
    n1 = 0
    
    m2 = -2*altura_triangulo_maxima/lado
    n2 = 2*altura_triangulo_maxima

    for i in range(len(heightS_triangulo)):
        for j in range(len(x_axis_1)):
            x1 = x_axis_1[j]
            x2 = x_axis_2[j]
            y = position[1]
            z = heightS_triangulo[i]

            if 0 < z <= (m1*x1) + n1 and position[0] < x1 <= lado/2.:
                TRIANGULO.append([x1,y,z])
            if 0 < z <= (m2*x2) + n2 and x2 >= lado/2.:
                TRIANGULO.append([x2,y,z])

    TRIANGULO = np.array(TRIANGULO)
    

    if voxel_downsampling_size < 1000:
        # Vamos a hacer voxel_downsampling:
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(TRIANGULO)
        pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
        TRIANGULO = np.array(pcd2.points)    

    # Ahora le añadimos algo de ruido gaussiano a cada point para hacerlo más real:
    noise_x_axis = np.random.normal(position[0],0.008,len(TRIANGULO.take(0,1)))
    noise_y_axis = np.random.normal(position[1],0.008,len(TRIANGULO.take(0,1)))
    noise_z_axis = np.random.normal(position[2],0.008,len(TRIANGULO.take(0,1)[1:-2]))
    
    TRIANGULO[:, 0] = np.reshape(TRIANGULO.take(0,1) + noise_x_axis, -1)
    TRIANGULO[:, 1] = np.reshape(TRIANGULO.take(1,1) + noise_y_axis, -1)
    TRIANGULO[1:-2, 2] = np.reshape(TRIANGULO.take(2,1)[1:-2] + noise_z_axis, -1)
    
    triangulo = o3d.geometry.PointCloud()
    triangulo.points = o3d.utility.Vector3dVector(TRIANGULO)
    triangulo.paint_uniform_color(np.array([255/255.,0/255.,0/255.]))
    
    y_individual_signal = position[0]
    
    # pcd_pole_2 = copy.deepcopy(pcd_pole_1).translate((crossbar_width, center_first_pole[1], center_first_pole[2]))
    triangulo = copy.deepcopy(triangulo).translate((position[0],y_individual_signal+pole_radius*1.5,(altura_maxima_poste-0.4)+(altura_triangulo_maxima/2.3)),relative=False)
    triangulo_orig = copy.deepcopy(triangulo)
    N_copias = 10
    for k in range(N_copias):
        aux = copy.deepcopy(triangulo_orig).translate((position[0],(y_individual_signal+pole_radius*1.5)+0.002*k,(altura_maxima_poste-0.4)+(altura_triangulo_maxima/2.3)),relative=False)
        triangulo += aux
    
    # IGUAL habría que redondear un poco las esquinas, pero bueno...
    SIGNALS[len(SIGNALS)] = triangulo
    #------------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # visor.custom_draw_geometry_with_key_callback(triangulo)
    #------------------------------------------------------------------------------

    SIGNAL = triangulo + poste

    #------------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # visor.custom_draw_geometry_with_key_callback(SIGNAL)
    #------------------------------------------------------------------------------


    # # Quito de forma random points para que no me quede sumamente densa la nube:
    points_SIGNAL = np.array(SIGNAL.points)
    colors_SIGNAL = np.array(SIGNAL.colors)
    # lista_auxiliar = np.arange(0,len(points_SIGNAL),1)
    # indices = np.random.choice(lista_auxiliar,size=50000,replace=False)
    
    # print(indices)
    
    # points_SIGNAL = points_SIGNAL[indices]
    # colors_SIGNAL = colors_SIGNAL[indices]
    SIGNAL = o3d.geometry.PointCloud()
    SIGNAL.points = o3d.utility.Vector3dVector(points_SIGNAL)
    SIGNAL.colors = o3d.utility.Vector3dVector(colors_SIGNAL)


    SIGNAL.translate(([final_position[0],final_position[1],final_position[2]]))



    # #------------------------------------------------------------------------------
    # # PARÓN PARA VISUALIZAR:
        
    # # Voy a crear una nube de points auxiliar que me señale la posición que le
    # # pedí para comprobar que todo salió bien:
        
    # N_points_esfera = 1000
    # R_esfera = 0.1
    # points_esfera = []
    # for g in range(N_points_esfera):
    #     points_esfera.append([np.random.uniform(final_position[0]-R_esfera,final_position[0]+R_esfera),
    #                           np.random.uniform(final_position[1]-R_esfera,final_position[1]+R_esfera),
    #                           np.random.uniform(final_position[2]-R_esfera,final_position[2]+R_esfera)])

    # points_esfera = np.array(points_esfera)

    # nube_esfera = o3d.geometry.PointCloud()
    # nube_esfera.points = o3d.utility.Vector3dVector(points_esfera)
    # nube_esfera.paint_uniform_color([0,0,0])
        
    # visor.custom_draw_geometry_with_key_callback(SIGNAL+nube_esfera)
    # #------------------------------------------------------------------------------

    return SIGNAL
        


def create_circle_signal(final_position,road_type,voxel_downsampling_size=1000000):
    
    position=[0,0,0]
    
    if road_type in ['autopista','autovia']:
        
        diametro = 1.20 # El diámetro del círculo mide 1.2 por ley
        altura_maxima_poste = 3
        pole_radius = 0.03
        
    elif road_type in ['nacional','comarcal']:
        
        diametro = 0.90 # El diámetro del círculo mide 0.9 por ley
        altura_maxima_poste = 3
        pole_radius = 0.03
        
    elif road_type == 'local':
        
        diametro = 0.60 # El diámetro del círculo mide 0.6 por ley
        altura_maxima_poste = 3
        pole_radius = 0.03
    

    heightS_poste = np.linspace(position[2],altura_maxima_poste,50)       
    angleS = np.linspace(0,2*np.pi,100)
    
    POSTE = []
    
    for i in range(len(heightS_poste)):
        for j in range(len(angleS)):
            x = pole_radius*np.cos(angleS[j])
            y = pole_radius*np.sin(angleS[j])
            POSTE.append([x,y,heightS_poste[i]])
        
    POSTE = np.array(POSTE)    
    
    if voxel_downsampling_size < 1000:
        # Vamos a hacer voxel_downsampling:
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(POSTE)
        pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
        POSTE = np.array(pcd2.points)    

    
    # Ahora le añadimos algo de ruido gaussiano a cada point para hacerlo más real:
    noise_x_axis = np.random.normal(position[0],0.005,len(POSTE.take(0,1)))
    noise_y_axis = np.random.normal(position[1],0.005,len(POSTE.take(0,1)))
    noise_z_axis = np.random.normal(position[2],0.08,len(POSTE.take(0,1)[1:-2]))
    
    POSTE[:, 0] = np.reshape(POSTE.take(0,1) + noise_x_axis, -1)
    POSTE[:, 1] = np.reshape(POSTE.take(1,1) + noise_y_axis, -1)
    POSTE[1:-2, 2] = np.reshape(POSTE.take(2,1)[1:-2] + noise_z_axis, -1)
    
    poste = o3d.geometry.PointCloud()
    poste.points = o3d.utility.Vector3dVector(POSTE)
    poste.paint_uniform_color(np.array([160/255.,160/255.,160/255.]))
    
    SIGNALS[len(SIGNALS)] = poste

    #------------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # visor.custom_draw_geometry_with_key_callback(poste)
    #------------------------------------------------------------------------------


    # Vamos a crear un círculo.
    N_radios = 500
    
    angleS = np.linspace(0, 2*np.pi, 50)
    radios = np.linspace(0,(diametro/2.),N_radios)
    
    # Generamos los points    
    
    CIRCULO = []
    
    for i in range(len(radios)):
        
        x, z = radios[i] * np.cos(angleS), radios[i] * np.sin(angleS) 
        aux = np.stack((x,x,z),axis=1)
        # print(len(aux[0]))

        
        aux[:,1] = position[0]
        CIRCULO.append(aux)

    CIRCULO = np.vstack(CIRCULO)    
    
    # print(len(CIRCULO))
    
    
    CIRCULO.reshape((len(CIRCULO),3))
    
    if voxel_downsampling_size < 1000:
        # Vamos a hacer voxel_downsampling:
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(CIRCULO)
        pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
        CIRCULO = np.array(pcd2.points)    

    
    # Ahora le añadimos algo de ruido gaussiano a cada point para hacerlo más real:
    noise_x_axis = np.random.normal(position[0],0.01,len(CIRCULO.take(0,1)))
    noise_y_axis = np.random.normal(position[1],0.008,len(CIRCULO.take(0,1)))
    noise_z_axis = np.random.normal(position[2],0.01,len(CIRCULO.take(0,1)[1:-2]))
    
    CIRCULO[:, 0] = np.reshape(CIRCULO.take(0,1) + noise_x_axis, -1)
    CIRCULO[:, 1] = np.reshape(CIRCULO.take(1,1) + noise_y_axis, -1)
    CIRCULO[1:-2, 2] = np.reshape(CIRCULO.take(2,1)[1:-2] + noise_z_axis, -1)
    
    circulo = o3d.geometry.PointCloud()
    circulo.points = o3d.utility.Vector3dVector(CIRCULO)
    circulo.paint_uniform_color(np.array([255/255.,0/255.,0/255.]))
    
    y_individual_signal = position[0]
    
    # pcd_pole_2 = copy.deepcopy(pcd_pole_1).translate((crossbar_width, center_first_pole[1], center_first_pole[2]))
    circulo = copy.deepcopy(circulo).translate((position[0],y_individual_signal+pole_radius*1.5,(altura_maxima_poste)),relative=False)
    circulo_orig = copy.deepcopy(circulo)
    N_copias = 10
    for k in range(N_copias):
        aux = copy.deepcopy(circulo_orig).translate((position[0],(y_individual_signal+pole_radius*1.5)+0.002*k,(altura_maxima_poste)),relative=False)
        circulo += aux
    
    # IGUAL habría que redondear un poco las esquinas, pero bueno...
    SIGNALS[len(SIGNALS)] = circulo
    #------------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # visor.custom_draw_geometry_with_key_callback(triangulo)
    #------------------------------------------------------------------------------

    SIGNAL = circulo + poste

    #------------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # visor.custom_draw_geometry_with_key_callback(SIGNAL)
    #------------------------------------------------------------------------------

    # import pdb
    # pdb.set_trace()

    # # Quito de forma random points para que no me quede sumamente densa la nube:
    points_SIGNAL = np.array(SIGNAL.points)
    colors_SIGNAL = np.array(SIGNAL.colors)
    # lista_auxiliar = np.arange(0,len(points_SIGNAL),1)
    # indices = np.random.choice(lista_auxiliar,size=50000,replace=False)
    
    # # print(indices)
    
    # points_SIGNAL = points_SIGNAL[indices]
    # colors_SIGNAL = colors_SIGNAL[indices]
    SIGNAL = o3d.geometry.PointCloud()
    SIGNAL.points = o3d.utility.Vector3dVector(points_SIGNAL)
    SIGNAL.colors = o3d.utility.Vector3dVector(colors_SIGNAL)


    SIGNAL.translate(([final_position[0],final_position[1],final_position[2]]))

    return SIGNAL




def create_square_signal(final_position,road_type,voxel_downsampling_size=1000000):
    
    position = [0,0,0]
    
    if road_type in ['autopista','autovia']:
        
        lado = 1.20 # Cada lado del cuadrado mide 1.20 por ley
        altura_maxima_poste = 3
        pole_radius = 0.03
        
    elif road_type in ['nacional','comarcal']:
        
        lado = 0.90 # Cada lado del cuadrado mide 0.90 por ley
        altura_maxima_poste = 3
        pole_radius = 0.03
        
    elif road_type == 'local':
        
        lado = 0.60 # Cada lado del cuadrado mide 0.60 por ley
        altura_maxima_poste = 3
        pole_radius = 0.03
    

    heightS_poste = np.linspace(position[2],altura_maxima_poste,50)       
    angleS = np.linspace(0,2*np.pi,100)
    
    POSTE = []
    
    for i in range(len(heightS_poste)):
        for j in range(len(angleS)):
            x = pole_radius*np.cos(angleS[j])
            y = pole_radius*np.sin(angleS[j])
            POSTE.append([x,y,heightS_poste[i]])
        
    POSTE = np.array(POSTE)    
    
    if voxel_downsampling_size < 1000:
        # Vamos a hacer voxel_downsampling:
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(POSTE)
        pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
        POSTE = np.array(pcd2.points)    

    
    
    # Ahora le añadimos algo de ruido gaussiano a cada point para hacerlo más real:
    noise_x_axis = np.random.normal(position[0],0.005,len(POSTE.take(0,1)))
    noise_y_axis = np.random.normal(position[1],0.005,len(POSTE.take(0,1)))
    noise_z_axis = np.random.normal(position[2],0.08,len(POSTE.take(0,1)[1:-2]))
    
    POSTE[:, 0] = np.reshape(POSTE.take(0,1) + noise_x_axis, -1)
    POSTE[:, 1] = np.reshape(POSTE.take(1,1) + noise_y_axis, -1)
    POSTE[1:-2, 2] = np.reshape(POSTE.take(2,1)[1:-2] + noise_z_axis, -1)
    
    poste = o3d.geometry.PointCloud()
    poste.points = o3d.utility.Vector3dVector(POSTE)
    poste.paint_uniform_color(np.array([160/255.,160/255.,160/255.]))
    
    SIGNALS[len(SIGNALS)] = poste

    #------------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # visor.custom_draw_geometry_with_key_callback(poste)
    #------------------------------------------------------------------------------


    # Vamos a crear un cuadrado.
    
    N_points = 100
    x_axis = np.linspace(0,lado,N_points)
    z_axis = np.linspace(0,lado,len(x_axis))

    heightS_cuadrado = np.linspace(0,np.max(z_axis),len(x_axis))

    CUADRADO = []

    for i in range(len(z_axis)):
        for j in range(len(x_axis)):
            x = x_axis[j]
            y = position[1]
            z = heightS_cuadrado[i]

            CUADRADO.append([x,y,z])

    CUADRADO = np.array(CUADRADO)
    
    if voxel_downsampling_size < 1000:
        # Vamos a hacer voxel_downsampling:
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(CUADRADO)
        pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
        CUADRADO = np.array(pcd2.points)    


    # Ahora le añadimos algo de ruido gaussiano a cada point para hacerlo más real:
    noise_x_axis = np.random.normal(position[0],0.008,len(CUADRADO.take(0,1)))
    noise_y_axis = np.random.normal(position[1],0.008,len(CUADRADO.take(0,1)))
    noise_z_axis = np.random.normal(position[2],0.008,len(CUADRADO.take(0,1)[1:-2]))
    
    CUADRADO[:, 0] = np.reshape(CUADRADO.take(0,1) + noise_x_axis, -1)
    CUADRADO[:, 1] = np.reshape(CUADRADO.take(1,1) + noise_y_axis, -1)
    CUADRADO[1:-2, 2] = np.reshape(CUADRADO.take(2,1)[1:-2] + noise_z_axis, -1)
    
    cuadrado = o3d.geometry.PointCloud()
    cuadrado.points = o3d.utility.Vector3dVector(CUADRADO)
    cuadrado.paint_uniform_color(np.array([10/255.,0/255.,255./255.]))
    
    y_individual_signal = position[0]
    
    # pcd_pole_2 = copy.deepcopy(pcd_pole_1).translate((crossbar_width, center_first_pole[1], center_first_pole[2]))
    cuadrado = copy.deepcopy(cuadrado).translate((position[0],y_individual_signal+pole_radius*1.5,(altura_maxima_poste-0.4)+(np.max(z_axis)/2.3)),relative=False)
    cuadrado_orig = copy.deepcopy(cuadrado)
    N_copias = 10
    for k in range(N_copias):
        aux = copy.deepcopy(cuadrado_orig).translate((position[0],(y_individual_signal+pole_radius*1.5)+0.002*k,(altura_maxima_poste-0.4)+(np.max(z_axis)/2.3)),relative=False)
        cuadrado += aux
    
    # IGUAL habría que redondear un poco las esquinas, pero bueno...
    SIGNALS[len(SIGNALS)] = cuadrado
    #------------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # visor.custom_draw_geometry_with_key_callback(cuadrado)
    #------------------------------------------------------------------------------

    SIGNAL = cuadrado + poste

    #------------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # visor.custom_draw_geometry_with_key_callback(SIGNAL)
    #------------------------------------------------------------------------------



    # Quito de forma random points para que no me quede sumamente densa la nube:
    points_SIGNAL = np.array(SIGNAL.points)
    colors_SIGNAL = np.array(SIGNAL.colors)
    # lista_auxiliar = np.arange(0,len(points_SIGNAL),1)
    # indices = np.random.choice(lista_auxiliar,size=50000,replace=False)
    
    # # print(indices)
    
    # points_SIGNAL = points_SIGNAL[indices]
    # colors_SIGNAL = colors_SIGNAL[indices]
    SIGNAL = o3d.geometry.PointCloud()
    SIGNAL.points = o3d.utility.Vector3dVector(points_SIGNAL)
    SIGNAL.colors = o3d.utility.Vector3dVector(colors_SIGNAL)

    SIGNAL.translate(([final_position[0],final_position[1],final_position[2]]))

    return SIGNAL
        


def create_rectangular_signal(final_position,road_type,voxel_downsampling_size=1000000):
    
    position = [0,0,0]
    
    
    if road_type in ['autopista','autovia']:
        
        lado_horizontal = 1.20
        lado_vertical = 1.8
        altura_maxima_poste = 3
        pole_radius = 0.03
        
    elif road_type in ['nacional','comarcal']:
        
        lado_horizontal = 0.9
        lado_vertical = 1.35
        altura_maxima_poste = 3
        pole_radius = 0.03
        
    elif road_type == 'local':
        
        lado_horizontal = 0.6
        lado_vertical = 0.9
        altura_maxima_poste = 3
        pole_radius = 0.03
    

    heightS_poste = np.linspace(position[2],altura_maxima_poste,50)       
    angleS = np.linspace(0,2*np.pi,100)
    
    POSTE = []
    
    for i in range(len(heightS_poste)):
        for j in range(len(angleS)):
            x = pole_radius*np.cos(angleS[j])
            y = pole_radius*np.sin(angleS[j])
            POSTE.append([x,y,heightS_poste[i]])
        
    POSTE = np.array(POSTE)    
    
    if voxel_downsampling_size < 1000:
        # Vamos a hacer voxel_downsampling:
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(POSTE)
        pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
        POSTE = np.array(pcd2.points)    
        
    # Ahora le añadimos algo de ruido gaussiano a cada point para hacerlo más real:
    noise_x_axis = np.random.normal(position[0],0.005,len(POSTE.take(0,1)))
    noise_y_axis = np.random.normal(position[1],0.005,len(POSTE.take(0,1)))
    noise_z_axis = np.random.normal(position[2],0.08,len(POSTE.take(0,1)[1:-2]))
    
    POSTE[:, 0] = np.reshape(POSTE.take(0,1) + noise_x_axis, -1)
    POSTE[:, 1] = np.reshape(POSTE.take(1,1) + noise_y_axis, -1)
    POSTE[1:-2, 2] = np.reshape(POSTE.take(2,1)[1:-2] + noise_z_axis, -1)
    
    poste = o3d.geometry.PointCloud()
    poste.points = o3d.utility.Vector3dVector(POSTE)
    poste.paint_uniform_color(np.array([160/255.,160/255.,160/255.]))
    
    SIGNALS[len(SIGNALS)] = poste

    #------------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # visor.custom_draw_geometry_with_key_callback(poste)
    #------------------------------------------------------------------------------


    # Vamos a crear un rectángulo.
    
    N_points = 100
    x_axis = np.linspace(0,lado_horizontal,N_points)
    z_axis = np.linspace(0,lado_vertical,len(x_axis))

    heightS_rectangulo = np.linspace(0,np.max(z_axis),len(x_axis))

    RECTANGULO = []

    for i in range(len(z_axis)):
        for j in range(len(x_axis)):
            x = x_axis[j]
            y = position[1]
            z = heightS_rectangulo[i]

            RECTANGULO.append([x,y,z])

    RECTANGULO = np.array(RECTANGULO)


    if voxel_downsampling_size < 1000:
        # Vamos a hacer voxel_downsampling:
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(RECTANGULO)
        pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
        RECTANGULO = np.array(pcd2.points)    
        
        
    # Ahora le añadimos algo de ruido gaussiano a cada point para hacerlo más real:
    noise_x_axis = np.random.normal(position[0],0.008,len(RECTANGULO.take(0,1)))
    noise_y_axis = np.random.normal(position[1],0.008,len(RECTANGULO.take(0,1)))
    noise_z_axis = np.random.normal(position[2],0.008,len(RECTANGULO.take(0,1)[1:-2]))
    
    RECTANGULO[:, 0] = np.reshape(RECTANGULO.take(0,1) + noise_x_axis, -1)
    RECTANGULO[:, 1] = np.reshape(RECTANGULO.take(1,1) + noise_y_axis, -1)
    RECTANGULO[1:-2, 2] = np.reshape(RECTANGULO.take(2,1)[1:-2] + noise_z_axis, -1)
    
    rectangulo = o3d.geometry.PointCloud()
    rectangulo.points = o3d.utility.Vector3dVector(RECTANGULO)
    rectangulo.paint_uniform_color(np.array([10/255.,0/255.,255./255.]))
    
    y_individual_signal = position[0]
    
    # pcd_pole_2 = copy.deepcopy(pcd_pole_1).translate((crossbar_width, center_first_pole[1], center_first_pole[2]))
    rectangulo = copy.deepcopy(rectangulo).translate((position[0],y_individual_signal+pole_radius*1.5,(altura_maxima_poste-0.4)+(np.max(z_axis)/2.3)),relative=False)
    rectangulo_orig = copy.deepcopy(rectangulo)
    N_copias = 10
    for k in range(N_copias):
        aux = copy.deepcopy(rectangulo_orig).translate((position[0],(y_individual_signal+pole_radius*1.5)+0.002*k,(altura_maxima_poste-0.4)+(np.max(z_axis)/2.3)),relative=False)
        rectangulo += aux
    
    # IGUAL habría que redondear un poco las esquinas, pero bueno...
    SIGNALS[len(SIGNALS)] = rectangulo
    #------------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # visor.custom_draw_geometry_with_key_callback(cuadrado)
    #------------------------------------------------------------------------------

    SIGNAL = rectangulo + poste

    #------------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # visor.custom_draw_geometry_with_key_callback(SIGNAL)
    #------------------------------------------------------------------------------


    # Quito de forma random points para que no me quede sumamente densa la nube:
    points_SIGNAL = np.array(SIGNAL.points)
    colors_SIGNAL = np.array(SIGNAL.colors)
    # lista_auxiliar = np.arange(0,len(points_SIGNAL),1)
    # indices = np.random.choice(lista_auxiliar,size=50000,replace=False)
    
    # print(indices)
    
    # points_SIGNAL = points_SIGNAL[indices]
    # colors_SIGNAL = colors_SIGNAL[indices]
    SIGNAL = o3d.geometry.PointCloud()
    SIGNAL.points = o3d.utility.Vector3dVector(points_SIGNAL)
    SIGNAL.colors = o3d.utility.Vector3dVector(colors_SIGNAL)

    SIGNAL.translate(([final_position[0],final_position[1],final_position[2]]))

    return SIGNAL


def quitamiedos(pcd_arcenes_exteriores):
    
    # Del script CREADOR_CARRETERAS_ARTIFICIALES sacamos una salida que era
    # pcd_arcen_2, una nube de points con los points de los arcenes más exter-
    # nos (los más lejanos a la mediana). Por lo que si se trata de una carre-
    # tera multiplataforma con mediana (tipo autopista o autovía) o de una ca-
    # rretera convencional (tipo nacional o local), es necesario diferenciar
    # primero ambos arcenes.
    
    # Para diferenciar ambos arcenes de pcd_arcenes_exteriores lo primero que
    # haremos será un fit a los points de pcd_arcenes_exteriores. Luego, los
    # points de pcd_arcenes_exteriores que estén por encima del fit en y se co-
    # rresponderán a un arcén y los que estén por debajo se corresponderán al
    # otro.
    
    
    points_arcenes_combinados = np.array(pcd_arcenes_exteriores.points)
    
    import matplotlib.pyplot as plt
    
    plt.plot(points_arcenes_combinados.take(0,1),points_arcenes_combinados.take(1,1),'.',color='navy')
    
    z = np.polyfit(points_arcenes_combinados.take(0,1),points_arcenes_combinados.take(1,1),2)
    p = np.poly1d(z)
    
    
    x_axis = np.linspace(points_arcenes_combinados.take(0,1).min(),points_arcenes_combinados.take(0,1).max(),100)
    y_axis = p(x_axis)
    
    plt.plot(x_axis,y_axis,'-',color='red')
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    return










