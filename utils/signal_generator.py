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
    # o3d.visualization.draw(SIGNAL)
    #------------------------------------------------------------------------------
    
    # crossbar_width = 10
    crossbar_width = 13.5
    
    pcd_pole_2 = copy.deepcopy(pcd_pole_1).translate((crossbar_width, center_first_pole[1], center_first_pole[2]))
    
    SIGNALS[len(SIGNALS)] = pcd_pole_2
    
    center_second_pole = [crossbar_width,center_first_pole[1],center_first_pole[2]]
    
    #------------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # o3d.visualization.draw(SIGNAL,segmentado=True,pcd2=SIGNAL[0])
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
    # o3d.visualization.draw(SIGNAL,segmentado=True,pcd2=SIGNAL[0])
    #------------------------------------------------------------------------------
    
    pcd_pole_4 = copy.deepcopy(pcd_pole_3).translate((center_first_pole[0], center_first_pole[1], crossbar_height/2.))
    
    SIGNALS[len(SIGNALS)] = pcd_pole_4
    
    #------------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # o3d.visualization.draw(SIGNAL,segmentado=True,pcd2=SIGNAL[0])
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
        # o3d.visualization.draw(SIGNALS,segmentado=True,pcd2=SIGNAL)
    #------------------------------------------------------------------------------

    return SIGNAL,signal_center,center_first_pole,center_second_pole






def create_triangular_signal(final_position,road_type,voxel_downsampling_size=1000000):
    """
    Function to generate vertical-triangular signals

    :param final_position: List with [X, Y, Z] coordinates of the main pole position
    :param road_type: String with the type of road
    :param voxel_downsampling_size: [Experimental] Size of the cell within the downsampling
    :return: Point cloud of the full signal as a o3d.geometry.PointCloud() object
    """ 
    position = [0,0,0]
    
    
    if road_type in ['highway','mixed']:
        
        edge = 1.750 # Each side of the triangle is 1.75 m long (Spanish laws)
        pole_maximum_height = 3
        pole_radius = 0.03
        
    elif road_type == 'national':
        
        edge = 1.350 # Each side of the triangle is 1.35 m long (Spanish laws)
        pole_maximum_height = 3
        pole_radius = 0.03
        
    elif road_type == 'local':
        
        edge = 0.900 # Each side of the triangle is 0.9 m long (Spanish laws)
        pole_maximum_height = 3
        pole_radius = 0.03
    

    pole_heightS = np.linspace(position[2],pole_maximum_height,50)       
    angleS = np.linspace(0,2*np.pi,100)
    
    POLE = []
    
    for i in range(len(pole_heightS)):
        for j in range(len(angleS)):
            x = pole_radius*np.cos(angleS[j])
            y = pole_radius*np.sin(angleS[j])
            POLE.append([x,y,pole_heightS[i]])
        
    POLE = np.array(POLE)    
    
    if voxel_downsampling_size < 1000:
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(POLE)
        pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
        POLE = np.array(pcd2.points)    
    
    # Now we add some gaussian noise:
    noise_x_axis = np.random.normal(position[0],0.005,len(POLE.take(0,1)))
    noise_y_axis = np.random.normal(position[1],0.005,len(POLE.take(0,1)))
    noise_z_axis = np.random.normal(position[2],0.08,len(POLE.take(0,1)[1:-2]))
    
    POLE[:, 0] = np.reshape(POLE.take(0,1) + noise_x_axis, -1)
    POLE[:, 1] = np.reshape(POLE.take(1,1) + noise_y_axis, -1)
    POLE[1:-2, 2] = np.reshape(POLE.take(2,1)[1:-2] + noise_z_axis, -1)
    
    poste = o3d.geometry.PointCloud()
    poste.points = o3d.utility.Vector3dVector(POLE)
    poste.paint_uniform_color(np.array([160/255.,160/255.,160/255.]))
    
    SIGNALS[len(SIGNALS)] = poste

    #------------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # o3d.visualization.draw(poste)
    #------------------------------------------------------------------------------


    # Let's create a triangle. The first point will be the upper vertex:
    
    N_points = int(300/2)
    x_axis_1 = np.linspace(0,edge/2.,N_points)
    x_axis_2 = np.linspace(edge/2.,edge,len(x_axis_1))

    maximum_height_triangle = np.sqrt(edge**2 - (edge/2.)**2)
    triangle_heightS = np.linspace(0,maximum_height_triangle,len(x_axis_1))

    TRIANGLE = []

    # We compute the equation of the rect that goes through the middle of the
    # triangle (z = mx + n):
    m1 = maximum_height_triangle/(edge/2.)
    n1 = 0
    
    m2 = -2*maximum_height_triangle/edge
    n2 = 2*maximum_height_triangle

    for i in range(len(triangle_heightS)):
        for j in range(len(x_axis_1)):
            x1 = x_axis_1[j]
            x2 = x_axis_2[j]
            y = position[1]
            z = triangle_heightS[i]

            if 0 < z <= (m1*x1) + n1 and position[0] < x1 <= edge/2.:
                TRIANGLE.append([x1,y,z])
            if 0 < z <= (m2*x2) + n2 and x2 >= edge/2.:
                TRIANGLE.append([x2,y,z])

    TRIANGLE = np.array(TRIANGLE)
    

    if voxel_downsampling_size < 1000:
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(TRIANGLE)
        pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
        TRIANGLE = np.array(pcd2.points)    

    # Now we add some gaussian noise:
    noise_x_axis = np.random.normal(position[0],0.008,len(TRIANGLE.take(0,1)))
    noise_y_axis = np.random.normal(position[1],0.008,len(TRIANGLE.take(0,1)))
    noise_z_axis = np.random.normal(position[2],0.008,len(TRIANGLE.take(0,1)[1:-2]))
    
    TRIANGLE[:, 0] = np.reshape(TRIANGLE.take(0,1) + noise_x_axis, -1)
    TRIANGLE[:, 1] = np.reshape(TRIANGLE.take(1,1) + noise_y_axis, -1)
    TRIANGLE[1:-2, 2] = np.reshape(TRIANGLE.take(2,1)[1:-2] + noise_z_axis, -1)
    
    triangle = o3d.geometry.PointCloud()
    triangle.points = o3d.utility.Vector3dVector(TRIANGLE)
    triangle.paint_uniform_color(np.array([255/255.,0/255.,0/255.]))
    
    y_individual_signal = position[0]
    
    triangle = copy.deepcopy(triangle).translate((position[0],y_individual_signal+pole_radius*1.5,(pole_maximum_height-0.4)+(maximum_height_triangle/2.3)),relative=False)
    triangle_orig = copy.deepcopy(triangle)
    N_copias = 10
    for k in range(N_copias):
        aux = copy.deepcopy(triangle_orig).translate((position[0],(y_individual_signal+pole_radius*1.5)+0.002*k,(pole_maximum_height-0.4)+(maximum_height_triangle/2.3)),relative=False)
        triangle += aux
    
    SIGNALS[len(SIGNALS)] = triangle
    #------------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # o3d.visualization.draw(triangle)
    #------------------------------------------------------------------------------

    SIGNAL = triangle + poste

    #------------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # o3d.visualization.draw(SIGNAL)
    #------------------------------------------------------------------------------



    SIGNAL.translate(([final_position[0],final_position[1],final_position[2]]))


    return SIGNAL
        


def create_circle_signal(final_position,road_type,voxel_downsampling_size=1000000):
    """
    Function to generate vertical-circle signals

    :param final_position: List with [X, Y, Z] coordinates of the main pole position
    :param road_type: String with the type of road
    :param voxel_downsampling_size: [Experimental] Size of the cell within the downsampling
    :return: Point cloud of the full signal as a o3d.geometry.PointCloud() object
    """ 
    position=[0,0,0]
    
    if road_type in ['highway','mixed']:
        
        diameter = 1.20 # The diameter of the circle is 1.2 m long (Spanish laws)
        pole_maximum_height = 3
        pole_radius = 0.03
        
    elif road_type == 'national':
        
        diameter = 0.90 # The diameter of the circle is 0.9 m long (Spanish laws)
        pole_maximum_height = 3
        pole_radius = 0.03
        
    elif road_type == 'local':
        
        diameter = 0.60 # The diameter of the circle is 0.6 m long (Spanish laws)
        pole_maximum_height = 3
        pole_radius = 0.03
    

    pole_heightS = np.linspace(position[2],pole_maximum_height,50)       
    angleS = np.linspace(0,2*np.pi,100)
    
    POLE = []
    
    for i in range(len(pole_heightS)):
        for j in range(len(angleS)):
            x = pole_radius*np.cos(angleS[j])
            y = pole_radius*np.sin(angleS[j])
            POLE.append([x,y,pole_heightS[i]])
        
    POLE = np.array(POLE)    
    
    if voxel_downsampling_size < 1000:
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(POLE)
        pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
        POLE = np.array(pcd2.points)    

    
    # Now we add some gaussian noise:
    noise_x_axis = np.random.normal(position[0],0.005,len(POLE.take(0,1)))
    noise_y_axis = np.random.normal(position[1],0.005,len(POLE.take(0,1)))
    noise_z_axis = np.random.normal(position[2],0.08,len(POLE.take(0,1)[1:-2]))
    
    POLE[:, 0] = np.reshape(POLE.take(0,1) + noise_x_axis, -1)
    POLE[:, 1] = np.reshape(POLE.take(1,1) + noise_y_axis, -1)
    POLE[1:-2, 2] = np.reshape(POLE.take(2,1)[1:-2] + noise_z_axis, -1)
    
    poste = o3d.geometry.PointCloud()
    poste.points = o3d.utility.Vector3dVector(POLE)
    poste.paint_uniform_color(np.array([160/255.,160/255.,160/255.]))
    
    SIGNALS[len(SIGNALS)] = poste

    #------------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # o3d.visualization.draw(poste)
    #------------------------------------------------------------------------------


    # Let's create a circle:
    N_radius = 500
    
    angleS = np.linspace(0, 2*np.pi, 50)
    radius = np.linspace(0,(diameter/2.),N_radius)
    
    CIRCLE = []
    
    for i in range(len(radius)):
        
        x, z = radius[i] * np.cos(angleS), radius[i] * np.sin(angleS) 
        aux = np.stack((x,x,z),axis=1)
        
        aux[:,1] = position[0]
        CIRCLE.append(aux)

    CIRCLE = np.vstack(CIRCLE)    
        
    CIRCLE.reshape((len(CIRCLE),3))
    
    if voxel_downsampling_size < 1000:
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(CIRCLE)
        pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
        CIRCLE = np.array(pcd2.points)    

    
    # Now we add some gaussian noise:
    noise_x_axis = np.random.normal(position[0],0.01,len(CIRCLE.take(0,1)))
    noise_y_axis = np.random.normal(position[1],0.008,len(CIRCLE.take(0,1)))
    noise_z_axis = np.random.normal(position[2],0.01,len(CIRCLE.take(0,1)[1:-2]))
    
    CIRCLE[:, 0] = np.reshape(CIRCLE.take(0,1) + noise_x_axis, -1)
    CIRCLE[:, 1] = np.reshape(CIRCLE.take(1,1) + noise_y_axis, -1)
    CIRCLE[1:-2, 2] = np.reshape(CIRCLE.take(2,1)[1:-2] + noise_z_axis, -1)
    
    circle = o3d.geometry.PointCloud()
    circle.points = o3d.utility.Vector3dVector(CIRCLE)
    circle.paint_uniform_color(np.array([255/255.,0/255.,0/255.]))
    
    y_individual_signal = position[0]
    
    circle = copy.deepcopy(circle).translate((position[0],y_individual_signal+pole_radius*1.5,(pole_maximum_height)),relative=False)
    circle_orig = copy.deepcopy(circle)
    N_copias = 10
    for k in range(N_copias):
        aux = copy.deepcopy(circle_orig).translate((position[0],(y_individual_signal+pole_radius*1.5)+0.002*k,(pole_maximum_height)),relative=False)
        circle += aux
    
    SIGNALS[len(SIGNALS)] = circle
    #------------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # o3d.visualization.draw(triangle)
    #------------------------------------------------------------------------------

    SIGNAL = circle + poste

    #------------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # o3d.visualization.draw(SIGNAL)
    #------------------------------------------------------------------------------


    SIGNAL.translate(([final_position[0],final_position[1],final_position[2]]))

    return SIGNAL




def create_square_signal(final_position,road_type,voxel_downsampling_size=1000000):
    """
    Function to generate vertical-square signals

    :param final_position: List with [X, Y, Z] coordinates of the main pole position
    :param road_type: String with the type of road
    :param voxel_downsampling_size: [Experimental] Size of the cell within the downsampling
    :return: Point cloud of the full signal as a o3d.geometry.PointCloud() object
    """ 
    position = [0,0,0]
    
    if road_type in ['highway','mixed']:
        
        edge = 1.20 # Each side of the square is 1.2 m long (Spanish laws)
        pole_maximum_height = 3
        pole_radius = 0.03
        
    elif road_type == 'national':
        
        edge = 0.90 # Each side of the square is 1.2 m long (Spanish laws)
        pole_maximum_height = 3
        pole_radius = 0.03
        
    elif road_type == 'local':
        
        edge = 0.60 # Each side of the square is 1.2 m long (Spanish laws)
        pole_maximum_height = 3
        pole_radius = 0.03
    

    pole_heightS = np.linspace(position[2],pole_maximum_height,50)       
    angleS = np.linspace(0,2*np.pi,100)
    
    POLE = []
    
    for i in range(len(pole_heightS)):
        for j in range(len(angleS)):
            x = pole_radius*np.cos(angleS[j])
            y = pole_radius*np.sin(angleS[j])
            POLE.append([x,y,pole_heightS[i]])
        
    POLE = np.array(POLE)    
    
    if voxel_downsampling_size < 1000:
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(POLE)
        pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
        POLE = np.array(pcd2.points)    

    
    
    # Now we add some gaussian noise:
    noise_x_axis = np.random.normal(position[0],0.005,len(POLE.take(0,1)))
    noise_y_axis = np.random.normal(position[1],0.005,len(POLE.take(0,1)))
    noise_z_axis = np.random.normal(position[2],0.08,len(POLE.take(0,1)[1:-2]))
    
    POLE[:, 0] = np.reshape(POLE.take(0,1) + noise_x_axis, -1)
    POLE[:, 1] = np.reshape(POLE.take(1,1) + noise_y_axis, -1)
    POLE[1:-2, 2] = np.reshape(POLE.take(2,1)[1:-2] + noise_z_axis, -1)
    
    poste = o3d.geometry.PointCloud()
    poste.points = o3d.utility.Vector3dVector(POLE)
    poste.paint_uniform_color(np.array([160/255.,160/255.,160/255.]))
    
    SIGNALS[len(SIGNALS)] = poste

    #------------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # o3d.visualization.draw(poste)
    #------------------------------------------------------------------------------


    # Let's create a square:
    
    N_points = 100
    x_axis = np.linspace(0,edge,N_points)
    z_axis = np.linspace(0,edge,len(x_axis))

    square_heightS = np.linspace(0,np.max(z_axis),len(x_axis))

    SQUARE = []

    for i in range(len(z_axis)):
        for j in range(len(x_axis)):
            x = x_axis[j]
            y = position[1]
            z = square_heightS[i]

            SQUARE.append([x,y,z])

    SQUARE = np.array(SQUARE)
    
    if voxel_downsampling_size < 1000:
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(SQUARE)
        pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
        SQUARE = np.array(pcd2.points)    


    # Now we add some gaussian noise:
    noise_x_axis = np.random.normal(position[0],0.008,len(SQUARE.take(0,1)))
    noise_y_axis = np.random.normal(position[1],0.008,len(SQUARE.take(0,1)))
    noise_z_axis = np.random.normal(position[2],0.008,len(SQUARE.take(0,1)[1:-2]))
    
    SQUARE[:, 0] = np.reshape(SQUARE.take(0,1) + noise_x_axis, -1)
    SQUARE[:, 1] = np.reshape(SQUARE.take(1,1) + noise_y_axis, -1)
    SQUARE[1:-2, 2] = np.reshape(SQUARE.take(2,1)[1:-2] + noise_z_axis, -1)
    
    square = o3d.geometry.PointCloud()
    square.points = o3d.utility.Vector3dVector(SQUARE)
    square.paint_uniform_color(np.array([10/255.,0/255.,255./255.]))
    
    y_individual_signal = position[0]
    
    square = copy.deepcopy(square).translate((position[0],y_individual_signal+pole_radius*1.5,(pole_maximum_height-0.4)+(np.max(z_axis)/2.3)),relative=False)
    square_orig = copy.deepcopy(square)
    N_copias = 10
    for k in range(N_copias):
        aux = copy.deepcopy(square_orig).translate((position[0],(y_individual_signal+pole_radius*1.5)+0.002*k,(pole_maximum_height-0.4)+(np.max(z_axis)/2.3)),relative=False)
        square += aux
    
    SIGNALS[len(SIGNALS)] = square
    #------------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # o3d.visualization.draw(square)
    #------------------------------------------------------------------------------

    SIGNAL = square + poste

    #------------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # o3d.visualization.draw(SIGNAL)
    #------------------------------------------------------------------------------


    SIGNAL.translate(([final_position[0],final_position[1],final_position[2]]))

    return SIGNAL
        


def create_rectangular_signal(final_position,road_type,voxel_downsampling_size=1000000):
    """
    Function to generate vertical-rectangular signals

    :param final_position: List with [X, Y, Z] coordinates of the main pole position
    :param road_type: String with the type of road
    :param voxel_downsampling_size: [Experimental] Size of the cell within the downsampling
    :return: Point cloud of the full signal as a o3d.geometry.PointCloud() object
    """ 
    position = [0,0,0]
    
    
    if road_type in ['highway','mixed']:
        
        edge_horizontal = 1.20
        edge_vertical = 1.8
        pole_maximum_height = 3
        pole_radius = 0.03
        
    elif road_type == 'national':
        
        edge_horizontal = 0.9
        edge_vertical = 1.35
        pole_maximum_height = 3
        pole_radius = 0.03
        
    elif road_type == 'local':
        
        edge_horizontal = 0.6
        edge_vertical = 0.9
        pole_maximum_height = 3
        pole_radius = 0.03
    

    pole_heightS = np.linspace(position[2],pole_maximum_height,50)       
    angleS = np.linspace(0,2*np.pi,100)
    
    POLE = []
    
    for i in range(len(pole_heightS)):
        for j in range(len(angleS)):
            x = pole_radius*np.cos(angleS[j])
            y = pole_radius*np.sin(angleS[j])
            POLE.append([x,y,pole_heightS[i]])
        
    POLE = np.array(POLE)    
    
    if voxel_downsampling_size < 1000:
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(POLE)
        pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
        POLE = np.array(pcd2.points)    
        
    # Now we add some gaussian noise:
    noise_x_axis = np.random.normal(position[0],0.005,len(POLE.take(0,1)))
    noise_y_axis = np.random.normal(position[1],0.005,len(POLE.take(0,1)))
    noise_z_axis = np.random.normal(position[2],0.08,len(POLE.take(0,1)[1:-2]))
    
    POLE[:, 0] = np.reshape(POLE.take(0,1) + noise_x_axis, -1)
    POLE[:, 1] = np.reshape(POLE.take(1,1) + noise_y_axis, -1)
    POLE[1:-2, 2] = np.reshape(POLE.take(2,1)[1:-2] + noise_z_axis, -1)
    
    poste = o3d.geometry.PointCloud()
    poste.points = o3d.utility.Vector3dVector(POLE)
    poste.paint_uniform_color(np.array([160/255.,160/255.,160/255.]))
    
    SIGNALS[len(SIGNALS)] = poste

    #------------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # o3d.visualization.draw(poste)
    #------------------------------------------------------------------------------


    # Let's create a rectangle:
    
    N_points = 100
    x_axis = np.linspace(0,edge_horizontal,N_points)
    z_axis = np.linspace(0,edge_vertical,len(x_axis))

    rectangle_heightS = np.linspace(0,np.max(z_axis),len(x_axis))

    RECTANGLE = []

    for i in range(len(z_axis)):
        for j in range(len(x_axis)):
            x = x_axis[j]
            y = position[1]
            z = rectangle_heightS[i]

            RECTANGLE.append([x,y,z])

    RECTANGLE = np.array(RECTANGLE)


    if voxel_downsampling_size < 1000:
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(RECTANGLE)
        pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
        RECTANGLE = np.array(pcd2.points)    
        
        
    # Now we add some gaussian noise:
    noise_x_axis = np.random.normal(position[0],0.008,len(RECTANGLE.take(0,1)))
    noise_y_axis = np.random.normal(position[1],0.008,len(RECTANGLE.take(0,1)))
    noise_z_axis = np.random.normal(position[2],0.008,len(RECTANGLE.take(0,1)[1:-2]))
    
    RECTANGLE[:, 0] = np.reshape(RECTANGLE.take(0,1) + noise_x_axis, -1)
    RECTANGLE[:, 1] = np.reshape(RECTANGLE.take(1,1) + noise_y_axis, -1)
    RECTANGLE[1:-2, 2] = np.reshape(RECTANGLE.take(2,1)[1:-2] + noise_z_axis, -1)
    
    rectangle = o3d.geometry.PointCloud()
    rectangle.points = o3d.utility.Vector3dVector(RECTANGLE)
    rectangle.paint_uniform_color(np.array([10/255.,0/255.,255./255.]))
    
    y_individual_signal = position[0]
    
    rectangle = copy.deepcopy(rectangle).translate((position[0],y_individual_signal+pole_radius*1.5,(pole_maximum_height-0.4)+(np.max(z_axis)/2.3)),relative=False)
    rectangle_orig = copy.deepcopy(rectangle)
    N_copias = 10
    for k in range(N_copias):
        aux = copy.deepcopy(rectangle_orig).translate((position[0],(y_individual_signal+pole_radius*1.5)+0.002*k,(pole_maximum_height-0.4)+(np.max(z_axis)/2.3)),relative=False)
        rectangle += aux
    
    SIGNALS[len(SIGNALS)] = rectangle
    #------------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # o3d.visualization.draw(square)
    #------------------------------------------------------------------------------

    SIGNAL = rectangle + poste

    #------------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # o3d.visualization.draw(SIGNAL)
    #------------------------------------------------------------------------------


    SIGNAL.translate(([final_position[0],final_position[1],final_position[2]]))

    return SIGNAL




