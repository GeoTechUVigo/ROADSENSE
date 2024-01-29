#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Oct 22 09:58:33 2021

@author: lino
"""

import os
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt # Not required for simulation, just debug visualizations
import math


def rotate(x,y,xo,yo,theta): #rotate (antihorario) x,y around xo,yo by theta (rad)
    """
    Function to rotate (counterclockwise) x,y around xo,yo by theta (rad)

    :param x: x coordinate of the point to be rotated
    :param y: y coordinate of the point to be rotated
    :param xo: x coordinate of the anchor reference point
    :param yo: y coordinate of the anchor reference point
    :param theta: Rotation angle in radians
    :return: list with the (xr,yr) coordinates of the rotated point
    """ 
    xr=math.cos(theta)*(x-xo)-math.sin(theta)*(y-yo)   + xo
    yr=math.sin(theta)*(x-xo)+math.cos(theta)*(y-yo)  + yo
    return [xr,yr]

dice = ['rect','curve']



def Road_Generator(height,road_type,reconstructed_surface,road_buffer):
    """
    Function to generate all ground components of the road (except traffic signals and barriers)

    :param height: Average road height. TODO: Change this to consider several heights
    :param road_type: Road type ('local', 'national' or 'highway')
    :param reconstructed_surface: numpy.ndarray() with all points of the simulated DTM
    :param road_buffer: Width of the road in meters
    :return: Each road element as open3d.geometry.PointCloud() objects and a numpy.ndarray() with the points of the road axis
    """ 
    
    # Spanish normatives of road alignment: 
    # http://www.carreteros.org/normativa/trazado/31ic_2016/apartados/4.htm
    # Table 4.5 is useful
    
    

    if road_type == 'highway':
        
        'Table 4.5 - 2, first 3'
        Radius = np.linspace(2.5,75,1000)
        Radius = np.random.choice(Radius)
        
        # DEBUG:
        Radius = 5
        
        if 2.5 <= Radius < 7:
            camber = 0.08
        elif 7 <= Radius < 50:
            camber = (8 - 7.3*((1-(700/Radius))**1.3))/100.
        elif 50 <= Radius <= 75:
            camber = 0.02

        width_refugee_island = 2.
        width_shoulder = 2.
        width_road = 10
        width_berm = 1.

    if road_type == 'national':
       
        'Tabla 4.5 - 3 first 2'
        Radius = np.linspace(50,1,10)
        Radius = np.random.choice(Radius)
        
        if 0.5 <= Radius < 3.5:
            camber = 0.07
        elif 3.5 <= Radius <= 25:
            camber = (7 - 6.65*((1-(350/Radius))**1.9))/100.
        
        width_refugee_island = 2.
        
    if road_type == 'local':
        
        'Tabla 4.5 - 3, last 3'
        Radius = np.linspace(3.5,35,1000)
        Radius = np.random.choice(Radius)

        if 3.5 <= Radius < 25:
            camber = (7 - 6.65*((1-(350/Radius))**1.9))/100.
        elif 25 <= Radius <= 35:
            camber = 0.02

        width_refugee_island = 2.


    lenght = 0.5
    curve_lenght = 400.
    
    road_axis = []

    
    x_axis_aux = np.linspace(reconstructed_surface.take(0,1).max()/4.,reconstructed_surface.take(0,1).max()/2.,100)
    y_axis_aux = np.linspace(reconstructed_surface.take(1,1).min(),reconstructed_surface.take(1,1).max(),100)

    x0 = np.random.choice(x_axis_aux)
    
    
    
    rect_1_y = np.linspace(y_axis_aux.min(),y_axis_aux.min()+lenght,100)
    rect_1_x = np.copy(rect_1_y)
    rect_1_x.fill(x0)
    rect_1 = np.stack((rect_1_x,rect_1_y),axis=-1)
    THETA_abs = 0
    final_point = rect_1[-1]

    # -------------------------------------------------------------------------
    # Uncomment to see a conceptual plot:
    # plt.plot(final_point[0],final_point[1],'.',color='black')
    # plt.plot(recta_1.take(0,1),recta_1.take(1,1),'-',color='navy')
    # -------------------------------------------------------------------------


    def adding_rect(final_point,THETA_abs):

        rect_i_y = np.linspace(final_point[1],final_point[1]+lenght,100)
        rect_i_x = np.copy(rect_i_y)
        rect_i_x.fill(final_point[0])
        rect_i = np.stack((rect_i_x,rect_i_y),axis=-1)

        # -------------------------------------------------------------------------
        # Uncomment to see a conceptual plot:
        # plt.plot(rect_i.take(0,1),rect_i.take(1,1),'-',color='green')
        # -------------------------------------------------------------------------

        rect_i_x,rect_i_y = rotate(rect_i_x,rect_i_y,final_point[0],final_point[1],-THETA_abs)

        # In Open3D we need an additional 3rd dimension:
        zeros_3rd_dimension = np.zeros(len(rect_i_x))
        rect_i = np.stack((rect_i_x,rect_i_y,zeros_3rd_dimension),axis=-1)
        final_point = [rect_i[-1][0],rect_i[-1][1]]

        # -------------------------------------------------------------------------
        # Uncomment to see a conceptual plot:
        # plt.plot(final_point[0],final_point[1],'.',color='black')
        # plt.plot(rect_i.take(0,1),rect_i.take(1,1),'-',color='red')
        # -------------------------------------------------------------------------

        return rect_i

    


    def adding_curve(final_point,THETA_abs):
        
        xc = final_point[0] + Radius 
        yc = final_point[1] 
        
        # -------------------------------------------------------------------------
        # Uncomment to see a conceptual plot:
        # plt.plot(xc,yc,'*',color='black')
        # -------------------------------------------------------------------------

        curve_center = rotate(xc,yc,final_point[0],final_point[1],-THETA_abs)
        
        # -------------------------------------------------------------------------
        # Uncomment to see a conceptual plot:
        # plt.plot(curve_center[0],curve_center[1],'*',color='red')
        # -------------------------------------------------------------------------

        x_axis_i_LEFT_1 = np.linspace(xc-Radius,xc-Radius+1,1000)
        x_axis_i_LEFT_2 = np.linspace(xc-Radius+1,xc-Radius+2,1000)
        x_axis_i_LEFT_3 = np.linspace(xc-Radius+2,xc-Radius+3,1000)
        x_axis_i_LEFT_4 = np.linspace(xc-Radius+3,xc-Radius+4,1000)
        x_axis_i_LEFT_5 = np.linspace(xc-Radius+4,xc-Radius+5,1000)
        
        x_axis_i = np.concatenate((x_axis_i_LEFT_1,x_axis_i_LEFT_2,x_axis_i_LEFT_3,x_axis_i_LEFT_4,x_axis_i_LEFT_5))
        
        
        y_axis_i = []
        for o in range(len(x_axis_i)):
            y_axis_i.append(np.sqrt(Radius**2-(x_axis_i[o]-curve_center[0])**2) + curve_center[1])
        y_axis_i = np.array(y_axis_i)
        
        # -------------------------------------------------------------------------
        # Uncomment to see a conceptual plot:
        # plt.plot(x_axis_i,y_axis_i,'-',color='lime')
        # -------------------------------------------------------------------------
        
        curve_i_points = np.stack((x_axis_i,y_axis_i),axis=-1)
        curve_i_points = curve_i_points.tolist() # It begings at the end of the semicircle
        # curve_i_points.reverse()
        curve_i_points = np.array(curve_i_points)
        
        
        # DO NOT WRITE: final_point = curve_i_points[-1]

        # -------------------------------------------------------------------------
        # Uncomment to see a conceptual plot:
        # plt.plot(final_point[0],final_point[1],'.',color='black')
        # plt.plot(eje_x_i,eje_y_i,'.',color='orange')
        # -------------------------------------------------------------------------

        def polar_distance(point_1,point_2):
            r = np.sqrt((point_1[0]-point_2[0])**2+(point_1[1]-point_2[1])**2)
            return r


        final_index_caught = False
        lenght_now = 0
        for o in range(len(curve_i_points)):
            l_i = polar_distance(final_point, curve_i_points[o])
            lenght_now += l_i
            if  o == 3:
                final_index = 3
                theta = lenght_now/Radius
            if lenght_now >= curve_lenght*(1+np.random.random()):
                final_index = o
                theta = lenght_now/Radius
                final_index_caught = True
                break

        if not final_index_caught:
            final_index = 3
            theta = lenght_now/Radius
            final_index_caught = True

        curve_i_points = curve_i_points[0:final_index]
        
        final_point = curve_i_points[-1]
        THETA_abs += theta

        # -------------------------------------------------------------------------
        # Uncomment to see a conceptual plot:
        # plt.plot(final_point[0],final_point[1],'.',color='black')
        # plt.plot(curve_i_points.take(0,1),curve_i_points.take(1,1),'-',color='lime')
        # -------------------------------------------------------------------------
        
        curve_i_points_x = curve_i_points.take(0,1)
        curve_i_points_y = curve_i_points.take(1,1)

        # In Open3D we need an additional 3rd dimension:
        zeros_3rd_dimension = np.zeros(len(curve_i_points))
        curve_i_points = np.stack((curve_i_points_x,curve_i_points_y,zeros_3rd_dimension),axis=-1)
        curve_i_points[:,2] = 0


        return curve_i_points

    # We perform the Rect/Curve addition operation 2000 times:
    for m in range(2000): 

        # Worst way to do this randomly:                    It's actually funny
        election = np.random.choice(dice)
        
        if election == 'rect':
            rect_i = adding_rect(final_point,THETA_abs)
            final_point = rect_i[-1]
            road_axis.append(rect_i)
        else:
            curve_i = adding_curve(final_point,THETA_abs)
            final_point = curve_i[-1]
            road_axis.append(curve_i)
                
    curve_points = np.concatenate(road_axis)
    
    # -------------------------------------------------------------------------
    # Uncomment to see a conceptual plot:
    # plt.plot(curve_points.take(0,1),curve_points.take(1,1),'.')
    # -------------------------------------------------------------------------


    # We have all straight (rect) and curve path points generated. Now we need to group
    # them properly in a 'axis_road' array.


    if road_type != 'highway':

        pcd2_1 = o3d.geometry.PointCloud()
        pcd2_1.points = o3d.utility.Vector3dVector(reconstructed_surface)
        
        # Lets paint the ground with some "brownish" colors:
        DTM_color_list = [np.array([51/255,51/255,0/255]),
                         np.array([102/255,102/255,0/255]),
                         np.array([153/255,153/255,0/255]),
                         np.array([204/255,204/255,0/255]),
                         np.array([255/255,255/255,0/255]),
                         np.array([102/255,51/255,0/255]),
                         np.array([153/255,76/255,0/255]),
                         np.array([204/255,102/255,0/255])]
        
        DTM_colors = np.zeros((len(reconstructed_surface),3))
        for e in range(len(DTM_colors)):
            DTM_colors[e] = DTM_color_list[np.random.choice(len(DTM_color_list))]
        
        pcd2_1.colors = o3d.utility.Vector3dVector(DTM_colors)
        
        pcd2_2 = o3d.geometry.PointCloud()
        pcd2_2.points = o3d.utility.Vector3dVector(curve_points)
        
        # Vamos a pintar el suelo con algunos colores:
        road_axis_pcd_colors = [np.array([0/255,0/255,255/255]),
                         np.array([0/255,0/255,255/255]),
                         np.array([0/255,0/255,255/255])]
        
        road_axis_colors = np.zeros((len(curve_points),3))
        for e in range(len(road_axis_colors)):
            road_axis_colors[e] = road_axis_pcd_colors[np.random.choice(len(road_axis_pcd_colors))]
        
        pcd2_2.colors = o3d.utility.Vector3dVector(road_axis_colors)
        
        pcd2_3 = o3d.geometry.PointCloud()
        plane_surface = np.copy(reconstructed_surface)
        plane_surface[:,2] = 0
        pcd2_3.points = o3d.utility.Vector3dVector(plane_surface)
        
        pcd2_4 = o3d.geometry.PointCloud()
        curve_on_plane = np.copy(curve_points)
        curve_on_plane[:,2] = 0
        pcd2_4.points = o3d.utility.Vector3dVector(curve_on_plane)
    
        
        
        
        
        
        
        
        distances = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
        road_axis_indexes = np.where(distances <= road_buffer)[0]
        rest_indexes = np.where(distances > road_buffer)[0]
        
        
        
        
        
        
        road_axis_points = np.array(pcd2_1.points)[road_axis_indexes]
        
        road_axis_points[:,[2]] = height
        
        
        
        
        
        
        
        rest_points = np.array(pcd2_1.points)[rest_indexes]
        
        pcd_road = o3d.geometry.PointCloud()
        pcd_road.points = o3d.utility.Vector3dVector(road_axis_points)
        pcd_road.paint_uniform_color([0,0,0])
        
        pcd_DTM = o3d.geometry.PointCloud()
        pcd_DTM.points = o3d.utility.Vector3dVector(rest_points)
    




        # We create an hipothetical refugee island to manage the barriers:
        
        pcd2_1 = o3d.geometry.PointCloud()
        pcd2_1.points = o3d.utility.Vector3dVector(reconstructed_surface)
        
        pcd2_2 = o3d.geometry.PointCloud()
        
        pcd2_2.points = o3d.utility.Vector3dVector(curve_points)
        
        
        
        
        pcd2_3 = o3d.geometry.PointCloud()
        plane_surface = np.copy(reconstructed_surface)
        plane_surface[:,2] = 0
        pcd2_3.points = o3d.utility.Vector3dVector(plane_surface)
        
        pcd2_4 = o3d.geometry.PointCloud()
        curve_on_plane = np.copy(curve_points)
        curve_on_plane[:,2] = 0
        pcd2_4.points = o3d.utility.Vector3dVector(curve_on_plane)
    
        
        
        
        distances = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
        refugee_island_indexes = np.where(distances <= width_refugee_island/2.)[0]
        rest_indexes = np.where(distances > width_refugee_island/2.)[0]
        
        
        
        
        
        
        refugee_island_points = np.array(pcd2_1.points)[refugee_island_indexes]
        
        
        
        
        
        refugee_island_points[:,[2]] = height
        
        rest_points = np.array(pcd2_1.points)[rest_indexes]
        
        
        
        pcd_refugee_island = o3d.geometry.PointCloud()
        pcd_refugee_island.points = o3d.utility.Vector3dVector(refugee_island_points)
        pcd_refugee_island.paint_uniform_color([0,0,1])


        return pcd_road,pcd_DTM,curve_points,pcd_refugee_island







    # If the road is a 'highway' type:
    else:
        
        pcd2_1 = o3d.geometry.PointCloud()
        pcd2_1.points = o3d.utility.Vector3dVector(reconstructed_surface)
        pcd2_2 = o3d.geometry.PointCloud()
        
        pcd2_2.points = o3d.utility.Vector3dVector(curve_points)
        
        
        
        
        pcd2_3 = o3d.geometry.PointCloud()
        plane_surface = np.copy(reconstructed_surface)
        plane_surface[:,2] = 0
        pcd2_3.points = o3d.utility.Vector3dVector(plane_surface)
        
        pcd2_4 = o3d.geometry.PointCloud()
        curve_on_plane = np.copy(curve_points)
        curve_on_plane[:,2] = 0
        pcd2_4.points = o3d.utility.Vector3dVector(curve_on_plane)
            
        
        distances = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
        refugee_island_indexes = np.where(distances <= width_refugee_island/2.)[0]
        rest_indexes = np.where(distances > width_refugee_island/2.)[0]
        
        
        refugee_island_points = np.array(pcd2_1.points)[refugee_island_indexes]

        refugee_island_points[:,[2]] = height
        
        rest_points = np.array(pcd2_1.points)[rest_indexes]
        
        
        
        pcd_refugee_island = o3d.geometry.PointCloud()
        pcd_refugee_island.points = o3d.utility.Vector3dVector(refugee_island_points)
        pcd_refugee_island.paint_uniform_color([0,0,1])
        
        pcd_DTM = o3d.geometry.PointCloud()
        pcd_DTM.points = o3d.utility.Vector3dVector(rest_points)
        surfacee = np.array(pcd_DTM.points)


        # -------------------------------------------------------------------------
        # Uncomment to debug visualization:
        # o3d.visualization.draw([pcd_refugee_island,pcd_DTM])
        # -------------------------------------------------------------------------
        
        # Adjusting the first shoulder:
            
        pcd2_3 = o3d.geometry.PointCloud()
        plane_surface = np.copy(surfacee)
        plane_surface[:,2] = 0
        pcd2_3.points = o3d.utility.Vector3dVector(plane_surface)
        
        pcd2_4 = o3d.geometry.PointCloud()
        curve_on_plane = np.copy(curve_points)
        curve_on_plane[:,2] = 0
        pcd2_4.points = o3d.utility.Vector3dVector(curve_on_plane)
    
        
        width_here = width_refugee_island + (2*width_shoulder)
        
        distances = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
        shoulder_indexes = np.where(distances <= width_here/2.)[0]
        rest_indexes = np.where(distances > width_here/2.)[0]
        
        
        
        points_shoulder_1 = np.array(pcd_DTM.points)[shoulder_indexes]
        
        
        points_shoulder_1[:,[2]] = height
        
        rest_points = np.array(pcd_DTM.points)[rest_indexes]
        
        
        
        pcd_shoulder_1 = o3d.geometry.PointCloud()
        pcd_shoulder_1.points = o3d.utility.Vector3dVector(points_shoulder_1)
        pcd_shoulder_1.paint_uniform_color(np.array([255/255.,233/255.,0.]))
        
        pcd_DTM = o3d.geometry.PointCloud()
        pcd_DTM.points = o3d.utility.Vector3dVector(rest_points)
        surfacee = np.array(pcd_DTM.points)


        #-------------------
        
        # Adjusting the road:
            
        pcd2_3 = o3d.geometry.PointCloud()
        plane_surface = np.copy(surfacee)
        plane_surface[:,2] = 0
        pcd2_3.points = o3d.utility.Vector3dVector(plane_surface)
        
        pcd2_4 = o3d.geometry.PointCloud()
        curve_on_plane = np.copy(curve_points)
        curve_on_plane[:,2] = 0
        pcd2_4.points = o3d.utility.Vector3dVector(curve_on_plane)
    
        
        
        
        
        
        width_here = width_here + (2*width_road)
        
        distances = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
        road_indexes = np.where(distances <= width_here/2.)[0]
        rest_indexes = np.where(distances > width_here/2.)[0]
        
        
        
        
        
        
        road_points = np.array(pcd_DTM.points)[road_indexes]
        
        
        
        road_points[:,[2]] = height
        
        rest_points = np.array(pcd_DTM.points)[rest_indexes]
        
        
        
        pcd_road = o3d.geometry.PointCloud()
        pcd_road.points = o3d.utility.Vector3dVector(road_points)
        pcd_road.paint_uniform_color(np.array([0.,0.,0.]))
        
        pcd_DTM = o3d.geometry.PointCloud()
        pcd_DTM.points = o3d.utility.Vector3dVector(rest_points)
        surfacee = np.array(pcd_DTM.points)

        #-------------------
        
        # Adjusting the second shoulder:
            
        pcd2_3 = o3d.geometry.PointCloud()
        plane_surface = np.copy(surfacee)
        plane_surface[:,2] = 0
        pcd2_3.points = o3d.utility.Vector3dVector(plane_surface)
        
        pcd2_4 = o3d.geometry.PointCloud()
        curve_on_plane = np.copy(curve_points)
        curve_on_plane[:,2] = 0
        pcd2_4.points = o3d.utility.Vector3dVector(curve_on_plane)
    
        
        width_here = width_here + (2*width_shoulder)
        
        distances = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
        shoulder_indexes = np.where(distances <= width_here/2.)[0]
        rest_indexes = np.where(distances > width_here/2.)[0]
        
        
        points_shoulder_2 = np.array(pcd_DTM.points)[shoulder_indexes]
        
        
        
        points_shoulder_2[:,[2]] = height
        
        rest_points = np.array(pcd_DTM.points)[rest_indexes]
        
        
        
        pcd_shoulder_2 = o3d.geometry.PointCloud()
        pcd_shoulder_2.points = o3d.utility.Vector3dVector(points_shoulder_2)
        pcd_shoulder_2.paint_uniform_color(np.array([255/255.,233/255.,0.]))
        
        pcd_DTM = o3d.geometry.PointCloud()
        pcd_DTM.points = o3d.utility.Vector3dVector(rest_points)
        surfacee = np.array(pcd_DTM.points)

        #-------------------

        # Adjusting both berms:



        pcd2_3 = o3d.geometry.PointCloud()
        plane_surface = np.copy(surfacee)
        plane_surface[:,2] = 0
        pcd2_3.points = o3d.utility.Vector3dVector(plane_surface)
        
        pcd2_4 = o3d.geometry.PointCloud()
        curve_on_plane = np.copy(curve_points)
        curve_on_plane[:,2] = 0
        pcd2_4.points = o3d.utility.Vector3dVector(curve_on_plane)
    
        
        
        width_here = width_here + (2*width_berm)
        
        distances = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
        berms_indexes = np.where(distances <= width_here/2.)[0]
        rest_indexes = np.where(distances > width_here/2.)[0]
        
        
        
        
        points_berms = np.array(pcd_DTM.points)[berms_indexes]
        
        
        
        points_berms[:,[2]] = height
        
        rest_points = np.array(pcd_DTM.points)[rest_indexes]
        
        pcd_berms = o3d.geometry.PointCloud()
        pcd_berms.points = o3d.utility.Vector3dVector(points_berms)
        pcd_berms.paint_uniform_color(np.array([82/255.,206/255.,231/255.]))
        
        pcd_DTM = o3d.geometry.PointCloud()
        pcd_DTM.points = o3d.utility.Vector3dVector(rest_points)




        # -------------------------------------------------------------------------
        # Uncomment any of these to debug visualizations:
        # o3d.visualization.draw(pcd_refugee_island)
        # o3d.visualization.draw(pcd_refugee_island+pcd_shoulder_1)
        # o3d.visualization.draw(pcd_refugee_island+pcd_shoulder_1+pcd_road_axis)
        # o3d.visualization.draw(pcd_refugee_island+pcd_shoulder_1+pcd_road_axis+pcd_shoulder_2)
        # o3d.visualization.draw(pcd_refugee_island+pcd_shoulder_1+pcd_road_axis+pcd_shoulder_2+pcd_berms)
        #------------------------------------------------------------------------------


        surfacee = np.array(pcd_DTM.points)

        # Lets paint the ground with some "brownish" colors:
        DTM_color_list = [np.array([51/255,51/255,0/255]),
                         np.array([102/255,102/255,0/255]),
                         np.array([153/255,153/255,0/255]),
                         np.array([204/255,204/255,0/255]),
                         np.array([255/255,255/255,0/255]),
                         np.array([102/255,51/255,0/255]),
                         np.array([153/255,76/255,0/255]),
                         np.array([204/255,102/255,0/255])]
        
        DTM_colors = np.zeros((len(surfacee),3))
        for e in range(len(DTM_colors)):
            DTM_colors[e] = DTM_color_list[np.random.choice(len(DTM_color_list))]
        
        pcd_DTM.colors = o3d.utility.Vector3dVector(DTM_colors)


        
        #------------------------------------------------------------------------------
        # Uncomment any of these to debug visualizations:
        # o3d.visualization.draw(pcd_refugee_island+pcd_shoulder_1)
        # o3d.visualization.draw(pcd_refugee_island+pcd_shoulder_1+pcd_road_axis)
        # o3d.visualization.draw(pcd_refugee_island+pcd_shoulder_2)
        # o3d.visualization.draw(pcd_refugee_island+pcd_shoulder_2+pcd_shoulder_1+pcd_road_axis+pcd_DTM)
        #------------------------------------------------------------------------------
        

        
        #------------------
        # Now we merge the shoulders into one single o3d.geometry.PointCloud() object:
        pcd_shoulders = pcd_shoulder_1 + pcd_shoulder_2


        # However, in the return part of the function we will pass also the
        # variable 'pcd_shoulder_2' because we need it for the barriers generation.
        

        # -------------------------------------------------------------------------
        # Uncomment any of these to debug visualizations:
        # o3d.visualization.draw(pcd_refugee_island+pcd_shoulders)
        # o3d.visualization.draw(pcd_berms+pcd_refugee_island+pcd_shoulders+pcd_shoulder_2+pcd_road_axis+pcd_DTM)
        #------------------------------------------------------------------------------
        return pcd_berms,pcd_refugee_island,pcd_shoulders,pcd_shoulder_2,pcd_road,pcd_DTM,curve_points









