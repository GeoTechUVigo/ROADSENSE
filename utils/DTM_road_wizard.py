#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 13 12:09:43 2021

@author: lino
"""


import open3d as o3d
import numpy as np
import itertools
import copy

from signal_generator import create_elevated_signal, create_triangular_signal, create_circle_signal, create_square_signal, create_rectangular_signal
from road_generator import Road_Generator


def DTM_road_generator(road_type,ORIGINAL_SEGMENTS,scale,number_points_DTM,road_buffer,slope_buffer,shoulder_buffer,berm_buffer,noise_DTM,noise_road,noise_slope,noise_shoulder,noise_berm,noise_refugee_island):
    """
    Function to generate from scratch all road-related geometries

    :param road_type: Type of road. Must be a string
    :param ORIGINAL_SEGMENTS: Dictionary with all stored tree segments
    :param scale: Scale of the point cloud
    :param number_points_DTM: Number of points per edge in the DTM grid
    :param road_buffer: Width of the road in meters
    :param slope_buffer: Width of the slope in meters
    :param shoulder_buffer: Width of the shoulder in meters
    :param noise_DTM: Noise threshold in each XYZ direction per point of the DTM. Must be a list with the following form: [f_x,f_y,f_z]
    :param noise_road: Noise threshold in each XYZ direction per point of the road. Must be a list with the following form: [f_x,f_y,f_z]
    :param noise_slope: Noise threshold in each XYZ direction per point of the slope. Must be a list with the following form: [f_x,f_y,f_z]
    :param noise_shoulder: Noise threshold in each XYZ direction per point of the shoulder. Must be a list with the following form: [f_x,f_y,f_z]
    :param noise_berm: Noise threshold in each XYZ direction per point of the berm. Must be a list with the following form: [f_x,f_y,f_z]
    :param noise_refugee_island: Noise threshold in each XYZ direction per point of the refugee island. Must be a list with the following form: [f_x,f_y,f_z]
    :return: All the road-related elements in different (but neccessary) formats
    """ 



#------------------------------------------------------------------------------
#============================ SURFACE SIMULATION ==============================

# First we will define a plane square grid. Then we will give rugosity to it. 

# To create the plane we will consider the real segments that we have already
# read. First, we will compute the average size in the XY plane of the segment
# collection and, then, we will consider that value as a reference to create a
# surface in which some trees can fit.


    average_x_global = 0
    average_y_global = 0
    
    average_point_density_global = 0
    
    for i in range(len(ORIGINAL_SEGMENTS)):
        
        segment = ORIGINAL_SEGMENTS[i]
        segment_points = np.array(segment.points)
                
        max_distance_x = np.abs(np.max(segment_points[:,0])-np.min(segment_points[:,0]))
        max_distance_y = np.abs(np.max(segment_points[:,1])-np.min(segment_points[:,1]))
    
        average_x_global += max_distance_x
        average_y_global += max_distance_y
        
        # Point density per square meter:
        average_point_density_global += len(segment_points)/(max_distance_x*max_distance_y)
    
    average_x_global = average_x_global/len(ORIGINAL_SEGMENTS)
    average_y_global = average_y_global/len(ORIGINAL_SEGMENTS)
    
    average_point_density_global = average_point_density_global/len(ORIGINAL_SEGMENTS)
    
    
    
    # Now we have a small square of sizes (average_x_global, average_y_global).
    # We are going to duplicate this square few times and define a surface model.
        
    x_axis = np.linspace(0,average_x_global*scale,int(average_point_density_global/5))
    y_axis = np.linspace(0,average_y_global*scale,int(average_point_density_global/5))
    z_axis = np.zeros(len(x_axis))
    
    # Now we choose random points of the XY plane and apply on them a translation
    # along the z axis (we change their height). Once done, we find the best
    # surface model that adapts to all points.
    
    SURFACE = {}
    SLOPE = {}
    SHOULDER = {}
    ROAD = {}
    SIGNALS = {}
    REFUGEE_ISLAND = {}
    BERM = {}
    
    # Number of points whose height will change:
    N_floating_points = 400
    
    mesh_x, mesh_y = np.meshgrid(x_axis,y_axis)
    
    xyz = np.zeros((np.size(mesh_x), 3))
    xyz[:, 0] = np.reshape(mesh_x, -1)
    xyz[:, 1] = np.reshape(mesh_y, -1)
    
    
    # -------------------------------------------------------------------------
    
    # Uncomment to debug visualizations:
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(xyz)
    # o3d.visualization.draw(pcd)
    
    # -------------------------------------------------------------------------
    
    # Changing randomly the heights:

    h_min = 0
    h_max = 10
        
    heights = np.linspace(h_min,h_max,20)
    max_height = np.random.choice(heights,size=1)[0]
        
    for i in range(N_floating_points):
        index = np.random.choice(range(len(xyz)))
        xyz[index][2] = np.random.randint(max_height)
    
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    
    # -------------------------------------------------------------------------
    
    # Uncomment to debug visualizations:
    # o3d.visualization.draw(pcd)
    
    # -------------------------------------------------------------------------
    
    
    # Let's make a quadric fit to those points, and the reconstructed surface
    # will be our DTM.
    
    def exponential_cov(x, y, params):
        return params[0] * np.exp( -0.5 * params[1] * np.subtract.outer(x, y)**2)
    
    def conditional(x_new, x, y, params):
        B = exponential_cov(x_new, x, params)
        C = exponential_cov(x, x, params)
        A = exponential_cov(x_new, x_new, params)
        mu = np.linalg.inv(C).dot(B.T).T.dot(y)
        sigma = A - B.dot(np.linalg.inv(C).dot(B.T))
        return(mu.squeeze(), sigma.squeeze())
       
    ordr = 4  # Order of the polinomial fit
    
    def least_squares_matrix(x, y, order=ordr):
        """ generate Matrix use with lstsq """
        ncolumns = (order + 1)**2
        G = np.zeros((x.size, ncolumns))
        ij = itertools.product(range(order+1), range(order+1))
        for k, (i, j) in enumerate(ij):
            G[:, k] = x**i * y**j
        return G
            
    # Just notation:
    points = xyz
    
    x, y, z = points.T # Hago la traspuesta
    x, y = x - x[0], y - y[0]  # Para mejorar la eficacia
    
    # We create the matrix that contains each regression per point:
    G = least_squares_matrix(x, y, ordr)
    # Solve for np.dot(G, m) = z:
    m = np.linalg.lstsq(G, z)[0]
    
    # We evaluae in a grid the fir that we made:
    nx, ny = number_points_DTM, number_points_DTM
    xx, yy = np.meshgrid(np.linspace(x.min(), x.max(), nx),
                          np.linspace(y.min(), y.max(), ny))
    
    
    GG = least_squares_matrix(xx.ravel(), yy.ravel(), ordr)
    zz = np.reshape(np.dot(GG, m), xx.shape)
    
    # -------------------------------------------------------------------------
    # Uncomment to see a reliability plot:
    # import matplotlib.pyplot as plt
    # fg, ax = plt.subplots(subplot_kw=dict(projection='3d'))
    # ls = LightSource(270, 45)
    # rgb = ls.shade(zz, cmap=cm.gist_earth, vert_exag=0.1, blend_mode='soft')
    # superficie = ax.plot_surface(xx, yy, zz, rstride=1, cstride=1, facecolors=rgb,
    #                         linewidth=0, antialiased=False, shade=False)
    # ax.plot3D(x, y, z, "o",color='red')
    # fg.canvas.draw()
    # plt.show()
    # -------------------------------------------------------------------------
    
    # We convert the surface obtained into a point cloud:
    reconstructed_surface = np.zeros((np.size(xx), 3))
    reconstructed_surface[:, 0] = np.reshape(xx, -1)
    reconstructed_surface[:, 1] = np.reshape(yy, -1)
    reconstructed_surface[:, 2] = np.reshape(zz, -1)
    
    maximum_height_gap = np.max(reconstructed_surface.take(2,1))-np.min(reconstructed_surface.take(2,1))
    print('Maximum height gap: ',maximum_height_gap,' (m)')
    
    
    #--------------------------------------------------------------------------
    #============================= ROAD SIMULATION ============================
    

    
    
    # if road_type == 'national':
    # Worst way to obtain random numbers:
    dice = np.random.randint(0,3)
    if dice == 0:
        road_height = np.max(reconstructed_surface.take(2,1))+5
    if dice == 1:
        road_height = np.min(reconstructed_surface.take(2,1))+5
    if dice == 2:
        road_height = np.mean(reconstructed_surface.take(2,1))+5
    # TODO: Change road_height parameter in order to create non flat roads.


    if road_type != 'highway':
        pcd_road,pcd_DTM,curve_points,pcd_refugee_island = Road_Generator(
            height=road_height,
            road_type=road_type,
            reconstructed_surface=reconstructed_surface,
            road_buffer=road_buffer)
    else:
        pcd_berm,pcd_refugee_island,pcd_shoulder,pcd_shoulder_2,pcd_road,pcd_DTM,curve_points = Road_Generator(
            height=road_height,
            road_type=road_type,
            reconstructed_surface=reconstructed_surface,
            road_buffer=road_buffer)



    road_points = np.array(pcd_road.points)


    pcd2_3 = o3d.geometry.PointCloud()
    plane_surface = np.copy(pcd_DTM.points)
    plane_surface[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(plane_surface)
    
    pcd2_4 = o3d.geometry.PointCloud()
    road_plane = np.copy(road_points)
    road_plane[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(road_plane)

    
    # -------------------------------------------------------------------------
    # Uncomment any of these to debug visualization:
    # o3d.visualization.draw(pcd2_3)
    # o3d.visualization.draw(pcd2_4)
    #----------------------------------------------------------------------
    
    
    distances = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    shoulder_indexes = np.where(distances <= shoulder_buffer)[0]
    rest_indexes = np.where(distances > shoulder_buffer)[0]
    
    
    shoulder_points = np.array(pcd_DTM.points)[shoulder_indexes]
    shoulder_points[:,[2]] = road_height
    
    
    rest_points = np.array(pcd_DTM.points)[rest_indexes]
    
    pcd_shoulder = o3d.geometry.PointCloud()
    pcd_shoulder.points = o3d.utility.Vector3dVector(shoulder_points)
    pcd_shoulder.paint_uniform_color(np.array([255/255.,233/255.,0.]))
    
    pcd_DTM = o3d.geometry.PointCloud()
    pcd_DTM.points = o3d.utility.Vector3dVector(rest_points)


    # -------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # o3d.visualization.draw(pcd_DTM+pcd_shoulder)
    # -------------------------------------------------------------------------


    if road_type == 'national':
        
        # Adding berms to national roads:
            
        pcd2_3 = o3d.geometry.PointCloud()
        plane_surface = np.copy(pcd_DTM.points)
        plane_surface[:,2] = 0
        pcd2_3.points = o3d.utility.Vector3dVector(plane_surface)
        
        pcd2_4 = o3d.geometry.PointCloud()
        plane_shoulder = np.copy(shoulder_points)
        plane_shoulder[:,2] = 0
        pcd2_4.points = o3d.utility.Vector3dVector(plane_shoulder)
        
        distances = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
        berm_indexes = np.where(distances <= 2*berm_buffer)[0]
        rest_indexes = np.where(distances > 2*berm_buffer)[0]
                        
        berm_points = np.array(pcd_DTM.points)[berm_indexes]
        berm_points[:,[2]] = road_height
        
        rest_points = np.array(pcd_DTM.points)[rest_indexes]
        
        pcd_berm = o3d.geometry.PointCloud()
        pcd_berm.points = o3d.utility.Vector3dVector(berm_points)
        pcd_berm.paint_uniform_color(np.array([0.,0.,0.]))
        
        pcd_DTM = o3d.geometry.PointCloud()
        pcd_DTM.points = o3d.utility.Vector3dVector(rest_points)


        # -------------------------------------------------------------------------
        # Uncomment to debug visualization:
        # o3d.visualization.draw(pcd_DTM+pcd_shoulder+pcd_berm)
        # -------------------------------------------------------------------------



    elif road_type == 'local':
        
            # Adding berms to local roads:
                
            pcd2_3 = o3d.geometry.PointCloud()
            plane_surface = np.copy(pcd_DTM.points)
            plane_surface[:,2] = 0
            pcd2_3.points = o3d.utility.Vector3dVector(plane_surface)
            
            pcd2_4 = o3d.geometry.PointCloud()
            plane_shoulder = np.copy(shoulder_points)
            plane_shoulder[:,2] = 0
            pcd2_4.points = o3d.utility.Vector3dVector(plane_shoulder)
            
            distances = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
            berm_indexes = np.where(distances <= 2*berm_buffer)[0]
            rest_indexes = np.where(distances > 2*berm_buffer)[0]
                            
            berm_points = np.array(pcd_DTM.points)[berm_indexes]
            berm_points[:,[2]] = road_height
            
            rest_points = np.array(pcd_DTM.points)[rest_indexes]
            
            pcd_berm = o3d.geometry.PointCloud()
            pcd_berm.points = o3d.utility.Vector3dVector(berm_points)
            pcd_berm.paint_uniform_color(np.array([0.,0.,0.]))
            
            pcd_DTM = o3d.geometry.PointCloud()
            pcd_DTM.points = o3d.utility.Vector3dVector(rest_points)
    
            # -------------------------------------------------------------------------
            # Uncomment to debug visualization:
            # o3d.visualization.draw(pcd_DTM+pcd_shoulder+pcd_berm)
            # -------------------------------------------------------------------------
    
    # Now we will create N_copies of the shoulder, road, refugee island and berm
    # pcd but with the points modified and some noise.
    
    N_copies = 40    
    
    shoulder_points = np.array(pcd_shoulder.points)
    shoulder_points_copy = np.copy(shoulder_points)
    pcd_shoulder = o3d.geometry.PointCloud()
    
    road_points = np.array(pcd_road.points)
    road_points_copy = np.copy(road_points)
    pcd_road = o3d.geometry.PointCloud()
    
    berm_points = np.array(pcd_berm.points)
    berm_points_copy = np.copy(berm_points)
    pcd_berm = o3d.geometry.PointCloud()
    
    if road_type in ['highway','mixed', 'national']:
        
        # We need to consider the national type also for the barrier generation
                    
        refugee_island_points = np.array(pcd_refugee_island.points)
        refugee_island_points_copy = np.copy(refugee_island_points)
        pcd_refugee_island = o3d.geometry.PointCloud()

    
    for h in range(N_copies):
            
        shoulder_points = np.copy(shoulder_points_copy)
    
        # if voxel_downsampling_size < 1000:
        #     # Vamos a hacer voxel_downsampling:
        #     pcd2 = o3d.geometry.PointCloud()
        #     pcd2.points = o3d.utility.Vector3dVector(shoulder_points)
        #     pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
        #     shoulder_points = np.array(pcd2.points)

    
        noise_x_axis = np.random.normal(0,(x_axis[2]-x_axis[1])*0.095,len(shoulder_points_copy))
        noise_y_axis = np.random.normal(0,(y_axis[2]-y_axis[1])*0.095,len(shoulder_points_copy))
        noise_z_axis = np.random.normal(0,0.005,len(shoulder_points_copy))
        
        shoulder_points[:, 0] = np.reshape(shoulder_points_copy.take(0,1) + noise_x_axis, -1)
        shoulder_points[:, 1] = np.reshape(shoulder_points_copy.take(1,1) + noise_y_axis, -1)
        shoulder_points[:, 2] = np.reshape(shoulder_points_copy.take(2,1) + noise_z_axis, -1)
        
        pcd_auxiliar = o3d.geometry.PointCloud()
        pcd_auxiliar.points = o3d.utility.Vector3dVector(shoulder_points)
    
        pcd_shoulder = pcd_shoulder + pcd_auxiliar
    
    
        road_points = np.copy(road_points_copy)
    
        # if voxel_downsampling_size < 1000:
        #     # Vamos a hacer voxel_downsampling:
        #     pcd2 = o3d.geometry.PointCloud()
        #     pcd2.points = o3d.utility.Vector3dVector(road_points)
        #     pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
        #     road_points = np.array(pcd2.points)
    
        noise_x_axis = np.random.normal(0,(x_axis[2]-x_axis[1])*0.095,len(road_points_copy))
        noise_y_axis = np.random.normal(0,(y_axis[2]-y_axis[1])*0.095,len(road_points_copy))
        noise_z_axis = np.random.normal(0,0.005,len(road_points_copy))
        
        road_points[:, 0] = np.reshape(road_points_copy.take(0,1) + noise_x_axis, -1)
        road_points[:, 1] = np.reshape(road_points_copy.take(1,1) + noise_y_axis, -1)
        road_points[:, 2] = np.reshape(road_points_copy.take(2,1) + noise_z_axis, -1)
        
        pcd_auxiliar = o3d.geometry.PointCloud()
        pcd_auxiliar.points = o3d.utility.Vector3dVector(road_points)
    
        pcd_road = pcd_road + pcd_auxiliar
    
    
        if road_type in ['highway','mixed', 'national']:
            # We need to consider the national type also for the barrier generation
            
            refugee_island_points = np.copy(refugee_island_points_copy)
        
            # if voxel_downsampling_size < 1000:
            #     # Vamos a hacer voxel_downsampling:
            #     pcd2 = o3d.geometry.PointCloud()
            #     pcd2.points = o3d.utility.Vector3dVector(refugee_island_points)
            #     pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
            #     refugee_island_points = np.array(pcd2.points)
        
        
            noise_x_axis = np.random.normal(0,(x_axis[2]-x_axis[1])*0.095,len(refugee_island_points_copy))
            noise_y_axis = np.random.normal(0,(y_axis[2]-y_axis[1])*0.095,len(refugee_island_points_copy))
            noise_z_axis = np.random.normal(0,0.005,len(refugee_island_points_copy))
            
            refugee_island_points[:, 0] = np.reshape(refugee_island_points_copy.take(0,1) + noise_x_axis, -1)
            refugee_island_points[:, 1] = np.reshape(refugee_island_points_copy.take(1,1) + noise_y_axis, -1)
            refugee_island_points[:, 2] = np.reshape(refugee_island_points_copy.take(2,1) + noise_z_axis, -1)
            
            pcd_auxiliar = o3d.geometry.PointCloud()
            pcd_auxiliar.points = o3d.utility.Vector3dVector(refugee_island_points)
        
            pcd_refugee_island = pcd_refugee_island + pcd_auxiliar

        
        berm_points = np.copy(berm_points_copy)
        
        # if voxel_downsampling_size < 1000:
        #     # Vamos a hacer voxel_downsampling:
        #     pcd2 = o3d.geometry.PointCloud()
        #     pcd2.points = o3d.utility.Vector3dVector(berm_points)
        #     pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
        #     berm_points = np.array(pcd2.points)
        
        noise_x_axis = np.random.normal(0,(x_axis[2]-x_axis[1])*0.095,len(berm_points_copy))
        noise_y_axis = np.random.normal(0,(y_axis[2]-y_axis[1])*0.095,len(berm_points_copy))
        noise_z_axis = np.random.normal(0,0.005,len(berm_points_copy))
        
        berm_points[:, 0] = np.reshape(berm_points_copy.take(0,1) + noise_x_axis, -1)
        berm_points[:, 1] = np.reshape(berm_points_copy.take(1,1) + noise_y_axis, -1)
        berm_points[:, 2] = np.reshape(berm_points_copy.take(2,1) + noise_z_axis, -1)
        
        pcd_auxiliar = o3d.geometry.PointCloud()
        pcd_auxiliar.points = o3d.utility.Vector3dVector(berm_points)
    
        pcd_berm = pcd_berm + pcd_auxiliar
    
    
    rest_points = np.array(pcd_DTM.points)

    shoulder_points = np.array(pcd_shoulder.points)
    road_points = np.array(pcd_road.points)
    if road_type in ['highway','mixed', 'national']:
        # We need to consider the national type also for the barrier generation
        refugee_island_points = np.array(pcd_refugee_island.points)

#----------        
    
    # -------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # o3d.visualization.draw(pcd_refugee_island+pcd_shoulder+pcd_road)
    #------------------------------------------------------------------------------
    
    
    
    pcd2_3 = o3d.geometry.PointCloud()
    plane_surface = np.array(pcd_DTM.points)
    plane_surface[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(plane_surface)
    
    pcd2_4 = o3d.geometry.PointCloud()
    plane_berm = np.copy(berm_points)
    plane_berm[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(plane_berm)
    
    
    distances = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    slope_indexes = np.where(distances <= slope_buffer)[0]
    rest_indexes = np.where(distances > slope_buffer)[0]
    
    
    slope_points = np.array(pcd_DTM.points)[slope_indexes]
    
    
    rest_points = np.array(pcd_DTM.points)[rest_indexes]
    
    
    pcd_auxiliar = o3d.geometry.PointCloud()
    pcd_auxiliar.points = o3d.utility.Vector3dVector(slope_points)
    pcd_auxiliar.paint_uniform_color([1,0,0])
    
    
    pcd_DTM = o3d.geometry.PointCloud()
    pcd_DTM.points = o3d.utility.Vector3dVector(rest_points)
    
    
    # -------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # o3d.visualization.draw(pcd_auxiliar)
    #----------------------------------------------------------------------
    
    
    # if voxel_downsampling_size < 1000:
    #     # Vamos a hacer voxel_downsampling:
    #     pcd2 = o3d.geometry.PointCloud()
    #     pcd2.points = o3d.utility.Vector3dVector(slope_points)
    #     pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
    #     slope_points = np.array(pcd2.points)

    


    copy_points = np.copy(slope_points)
    
    # Now we are going to create N_copies of the slope but with the points 
    # slightly disturbed with noise.
        
    N_copies = 50
    
    for h in range(N_copies):
        
    
        noise_x_axis = np.random.normal(0,(x_axis[2]-x_axis[1])*0.095,len(copy_points))
        noise_y_axis = np.random.normal(0,(y_axis[2]-y_axis[1])*0.095,len(copy_points))
        noise_z_axis = np.random.normal(0,0.005,len(copy_points))
        
        slope_points[:, 0] = np.reshape(copy_points.take(0,1) + noise_x_axis, -1)
        slope_points[:, 1] = np.reshape(copy_points.take(1,1) + noise_y_axis, -1)
        slope_points[:, 2] = np.reshape(copy_points.take(2,1) + noise_z_axis, -1)
        
        pcd_auxiliar = o3d.geometry.PointCloud()
        pcd_auxiliar.points = o3d.utility.Vector3dVector(slope_points)
    
        pcd_DTM = pcd_DTM + pcd_auxiliar
        # pcd_DTM = np.array(pcd_slope_original.points)
        slope_points = np.copy(copy_points)

    
    slope_bufferS = np.linspace(0.65,slope_buffer,500)
    percentages_road_heights = np.linspace(1,0,len(slope_bufferS))   
    
    
    pcd_slope = o3d.geometry.PointCloud()
    
    
    for p in range(len(slope_bufferS)):  
        slope_buffer_p = slope_bufferS[p]
    
        
    
        pcd2_3 = o3d.geometry.PointCloud()
        plane_surface = np.array(pcd_DTM.points)
        plane_surface[:,2] = 0
        pcd2_3.points = o3d.utility.Vector3dVector(plane_surface)
        
        pcd2_4 = o3d.geometry.PointCloud()
        plane_berm = np.copy(berm_points)
        plane_berm[:,2] = 0
        pcd2_4.points = o3d.utility.Vector3dVector(plane_berm)
        
        
        
        
        distances = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
        slope_indexes = np.where(distances <= slope_buffer_p)[0]
        rest_indexes = np.where(distances > slope_buffer_p)[0]
        
        slope_points = np.array(pcd_DTM.points)[slope_indexes]
        rest_points = np.array(pcd_DTM.points)[rest_indexes]
        
        # We rise and lower all shoulder points by road_height
        lower_indexes = np.where(slope_points[:,2] >= road_height)[0]
        rise_indexes = np.where(slope_points[:,2] < road_height)[0]
        
        slope_points[lower_indexes,2] = slope_points[lower_indexes,2] - percentages_road_heights[p]*np.abs(slope_points[lower_indexes,2]-road_height)
        slope_points[rise_indexes,2] = slope_points[rise_indexes,2] + percentages_road_heights[p]*np.abs(slope_points[rise_indexes,2]-road_height)
        
        
        pcd_auxiliar = o3d.geometry.PointCloud()
        pcd_auxiliar.points = o3d.utility.Vector3dVector(slope_points)
        pcd_auxiliar.paint_uniform_color([1,0,0])
        
        pcd_DTM = o3d.geometry.PointCloud()
        pcd_DTM.points = o3d.utility.Vector3dVector(rest_points)
    
        pcd_slope = pcd_slope + pcd_auxiliar
    
    
    slope_points = np.array(pcd_slope.points)
    
    pcd_slope_original = pcd_slope
    pcd_slope_original_puntos = np.array(pcd_slope_original.points)
    
    

    # Right now we have the following 4 point clouds:
    # · pcd_road
    # · pcd_slope
    # · pcd_DTM
    # · pcd_shoulder
    # · pcd_berm
    # And, in case of a highway or mixed road we will also have:
    # · pcd_refugee_island
    
    
    # -------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # o3d.visualization.draw(pcd_refugee_island)
    #------------------------------------------------------------------------------
    
    #------------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # o3d.visualization.draw(pcd_refugee_island+pcd_shoulder)
    #------------------------------------------------------------------------------
    
    #------------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # o3d.visualization.draw(pcd_refugee_island+pcd_shoulder+pcd_road)
    #------------------------------------------------------------------------------
    
    #------------------------------------------------------------------------------
    # Uncomment to debug visualization:
    # if road_type in ['highway','mixed']:
    #     pcd_DTM.paint_uniform_color(np.array([169/255,56/255,33/255]))
    #     pcd_slope.paint_uniform_color(np.array([255/255,190/255,51/255]))
    #     pcd_shoulder.paint_uniform_color(np.array([131/255,126/255,135/255]))
    #     pcd_road.paint_uniform_color(np.array([0/255,0/255,0/255]))
    #     pcd_refugee_island.paint_uniform_color(np.array([33/255,169/255,48/255]))        
    #     o3d.visualization.draw(pcd_DTM+pcd_refugee_island+pcd_shoulder+pcd_road+pcd_slope)
    # elif road_type == 'national':
    #     pcd_DTM.paint_uniform_color(np.array([169/255,56/255,33/255]))
    #     pcd_slope.paint_uniform_color(np.array([255/255,190/255,51/255]))
    #     pcd_shoulder.paint_uniform_color(np.array([0/255,0/255,0/255]))
    #     pcd_road.paint_uniform_color(np.array([0/255,0/255,0/255]))
    #     o3d.visualization.draw(pcd_DTM+pcd_shoulder+pcd_road+pcd_slope)
    # elif road_type == 'local':
    #     pcd_DTM.paint_uniform_color(np.array([169/255,56/255,33/255]))
    #     pcd_slope.paint_uniform_color(np.array([255/255,190/255,51/255]))
    #     pcd_road.paint_uniform_color(np.array([0/255,0/255,0/255]))
    #     o3d.visualization.draw(pcd_DTM+pcd_road+pcd_slope)
    #------------------------------------------------------------------------------
    
    # We are going to replicate each cloud few times at different depths:  
        
    reconstructed_surface = np.array(pcd_DTM.points)    
    reconstructed_road = np.array(pcd_road.points)
    reconstructed_slope = np.array(pcd_slope.points)
    reconstructed_shoulder = np.array(pcd_shoulder.points)
    reconstructed_berm = np.array(pcd_berm.points)

    # Set number_of_layers = 0 in case of lower densities:
    number_of_layers = 3
    # number_of_layers = 0
    for i in range(number_of_layers):
        aux = np.copy(reconstructed_surface)
        depth = np.zeros(3)
        depth[2] = 0.05
        aux = aux + depth
        reconstructed_surface = np.concatenate((reconstructed_surface,aux))

    # Set number_of_layers = 0 in case of lower densities:
    number_of_layers = 3
    # number_of_layers = 0
    for i in range(number_of_layers):
        aux = np.copy(reconstructed_road)
        depth = np.zeros(3)
        depth[2] = 0.05
        aux = aux + depth
        reconstructed_road = np.concatenate((reconstructed_road,aux))

    # Set number_of_layers = 0 in case of lower densities:
    number_of_layers = 3
    # number_of_layers = 0
    for i in range(number_of_layers):
        aux = np.copy(reconstructed_shoulder)
        depth = np.zeros(3)
        depth[2] = 0.05
        aux = aux + depth
        reconstructed_shoulder = np.concatenate((reconstructed_shoulder,aux))

    # Set number_of_layers = 0 in case of lower densities:
    number_of_layers = 3
    # number_of_layers = 0
    for i in range(number_of_layers):
        aux = np.copy(reconstructed_berm)
        depth = np.zeros(3)
        depth[2] = 0.05
        aux = aux + depth
        reconstructed_berm = np.concatenate((reconstructed_berm,aux))

    # Set number_of_layers = 0 in case of lower densities:
    number_of_layers = 1
    # number_of_layers = 0
    for i in range(number_of_layers):
        aux = np.copy(reconstructed_slope)
        depth = np.zeros(3)
        depth[2] = 0.2
        aux = aux + depth
        reconstructed_slope = np.concatenate((reconstructed_slope,aux))



    if road_type in ['highway','mixed','national']:
        # We need to consider the national type also for the barrier generation
        reconstructed_refugee_island = np.array(pcd_refugee_island.points)
        # Set number_of_layers = 0 in case of lower densities:
        number_of_layers = 3
        # number_of_layers = 0
        for i in range(number_of_layers):
            aux = np.copy(reconstructed_refugee_island)
            depth = np.zeros(3)
            depth[2] = 0.05
            aux = aux + depth
            reconstructed_refugee_island = np.concatenate((reconstructed_refugee_island,aux))

    
    # if voxel_downsampling_size < 1000:
    #     # Vamos a hacer voxel_downsampling:
    #     pcd2 = o3d.geometry.PointCloud()
    #     pcd2.points = o3d.utility.Vector3dVector(reconstructed_surface)
    #     pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
    #     reconstructed_surface = np.array(pcd2.points)

    
    # Now we are going to add some gaussian noise (reality approximation)
    noise_x_axis = np.random.normal(0,(x_axis[2]-x_axis[1])*noise_DTM[0],len(reconstructed_surface))
    noise_y_axis = np.random.normal(0,(y_axis[2]-y_axis[1])*noise_DTM[1],len(reconstructed_surface))
    noise_z_axis = np.random.normal(0,noise_DTM[2],len(reconstructed_surface))
    
    reconstructed_surface[:, 0] = np.reshape(reconstructed_surface.take(0,1) + noise_x_axis, -1)
    reconstructed_surface[:, 1] = np.reshape(reconstructed_surface.take(1,1) + noise_y_axis, -1)
    reconstructed_surface[:, 2] = np.reshape(reconstructed_surface.take(2,1) + noise_z_axis, -1)
    
    #-------------------------
    
    # if voxel_downsampling_size < 1000:
    #     # Vamos a hacer voxel_downsampling:
    #     pcd2 = o3d.geometry.PointCloud()
    #     pcd2.points = o3d.utility.Vector3dVector(reconstructed_road)
    #     pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
    #     reconstructed_road = np.array(pcd2.points)
    
    noise_x_axis = np.random.normal(0,(x_axis[2]-x_axis[1])*noise_road[0],len(reconstructed_road))
    noise_y_axis = np.random.normal(0,(y_axis[2]-y_axis[1])*noise_road[1],len(reconstructed_road))
    noise_z_axis = np.random.normal(0,noise_road[2],len(reconstructed_road))
    
    reconstructed_road[:, 0] = np.reshape(reconstructed_road.take(0,1) + noise_x_axis, -1)
    reconstructed_road[:, 1] = np.reshape(reconstructed_road.take(1,1) + noise_y_axis, -1)
    reconstructed_road[:, 2] = np.reshape(reconstructed_road.take(2,1) + noise_z_axis, -1)

    #-------------------------
    
    # if voxel_downsampling_size < 1000:
    #     # Vamos a hacer voxel_downsampling:
    #     pcd2 = o3d.geometry.PointCloud()
    #     pcd2.points = o3d.utility.Vector3dVector(reconstructed_shoulder)
    #     pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
    #     reconstructed_shoulder = np.array(pcd2.points)
    
    noise_x_axis = np.random.normal(0,(x_axis[2]-x_axis[1])*noise_shoulder[0],len(reconstructed_shoulder))
    noise_y_axis = np.random.normal(0,(y_axis[2]-y_axis[1])*noise_shoulder[1],len(reconstructed_shoulder))
    noise_z_axis = np.random.normal(0,noise_shoulder[2],len(reconstructed_shoulder))
    
    reconstructed_shoulder[:, 0] = np.reshape(reconstructed_shoulder.take(0,1) + noise_x_axis, -1)
    reconstructed_shoulder[:, 1] = np.reshape(reconstructed_shoulder.take(1,1) + noise_y_axis, -1)
    reconstructed_shoulder[:, 2] = np.reshape(reconstructed_shoulder.take(2,1) + noise_z_axis, -1)

    #-------------------------
    
    # if voxel_downsampling_size < 1000:
    #     # Vamos a hacer voxel_downsampling:
    #     pcd2 = o3d.geometry.PointCloud()
    #     pcd2.points = o3d.utility.Vector3dVector(reconstructed_slope)
    #     pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
    #     reconstructed_slope = np.array(pcd2.points)

    noise_x_axis = np.random.normal(0,(x_axis[2]-x_axis[1])*noise_slope[0],len(reconstructed_slope))
    noise_y_axis = np.random.normal(0,(y_axis[2]-y_axis[1])*noise_slope[1],len(reconstructed_slope))
    noise_z_axis = np.random.normal(0,noise_slope[2],len(reconstructed_slope))
    
    reconstructed_slope[:, 0] = np.reshape(reconstructed_slope.take(0,1) + noise_x_axis, -1)
    reconstructed_slope[:, 1] = np.reshape(reconstructed_slope.take(1,1) + noise_y_axis, -1)
    reconstructed_slope[:, 2] = np.reshape(reconstructed_slope.take(2,1) + noise_z_axis, -1)
    
    #-------------------------
    
    # if voxel_downsampling_size < 1000:
    #     # Vamos a hacer voxel_downsampling:
    #     pcd2 = o3d.geometry.PointCloud()
    #     pcd2.points = o3d.utility.Vector3dVector(reconstructed_berm)
    #     pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
    #     reconstructed_berm = np.array(pcd2.points)
    
    noise_x_axis = np.random.normal(0,(x_axis[2]-x_axis[1])*noise_berm[0],len(reconstructed_berm))
    noise_y_axis = np.random.normal(0,(y_axis[2]-y_axis[1])*noise_berm[1],len(reconstructed_berm))
    noise_z_axis = np.random.normal(0,noise_berm[2],len(reconstructed_berm))
    
    reconstructed_berm[:, 0] = np.reshape(reconstructed_berm.take(0,1) + noise_x_axis, -1)
    reconstructed_berm[:, 1] = np.reshape(reconstructed_berm.take(1,1) + noise_y_axis, -1)
    reconstructed_berm[:, 2] = np.reshape(reconstructed_berm.take(2,1) + noise_z_axis, -1)

    #-------------------------

    if road_type in ['highway','mixed','national']:

        # if voxel_downsampling_size < 1000:
        #     # Vamos a hacer voxel_downsampling:
        #     pcd2 = o3d.geometry.PointCloud()
        #     pcd2.points = o3d.utility.Vector3dVector(reconstructed_refugee_island)
        #     pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_downsampling_size)
        #     reconstructed_refugee_island = np.array(pcd2.points)
    

        noise_x_axis = np.random.normal(0,(x_axis[2]-x_axis[1])*noise_refugee_island[0],len(reconstructed_refugee_island))
        noise_y_axis = np.random.normal(0,(y_axis[2]-y_axis[1])*noise_refugee_island[1],len(reconstructed_refugee_island))
        noise_z_axis = np.random.normal(0,noise_refugee_island[2],len(reconstructed_refugee_island))
        
        reconstructed_refugee_island[:, 0] = np.reshape(reconstructed_refugee_island.take(0,1) + noise_x_axis, -1)
        reconstructed_refugee_island[:, 1] = np.reshape(reconstructed_refugee_island.take(1,1) + noise_y_axis, -1)
        reconstructed_refugee_island[:, 2] = np.reshape(reconstructed_refugee_island.take(2,1) + noise_z_axis, -1)
    
    
    pcd2_3 = o3d.geometry.PointCloud()
    plane_surface = np.copy(reconstructed_surface)
    plane_surface[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(reconstructed_surface)
    
    pcd2_4 = o3d.geometry.PointCloud()
    plane_slope = np.copy(reconstructed_slope)
    plane_slope[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(plane_slope)
    
    
    
    
    distances = np.array(pcd2_4.compute_point_cloud_distance(pcd2_3))
    slope_indexes = np.where(distances > .5)[0]
    rest_indexes = np.where(distances < .5)[0]
    
    slope_points = reconstructed_slope[slope_indexes]
    # rest_points = np.array(pcd_DTM.points)[rest_indexes]
    
    slope = o3d.geometry.PointCloud()
    slope.points = o3d.utility.Vector3dVector(slope_points)
    slope.paint_uniform_color([210/255,105/255,30/255])
    
    # pcd_DTM = o3d.geometry.PointCloud()
    # pcd_DTM.points = o3d.utility.Vector3dVector(rest_points)
    
    
    
    # pcd2_3 = o3d.geometry.PointCloud()
    # plane_surface = np.copy(reconstructed_surface)
    # plane_surface[:,2] = 0
    # pcd2_3.points = o3d.utility.Vector3dVector(reconstructed_surface)
    
    # pcd2_4 = o3d.geometry.PointCloud()
    # plane_slope = np.copy(slope_2_reconstruido)
    # plane_slope[:,2] = 0
    # pcd2_4.points = o3d.utility.Vector3dVector(plane_slope)
    
    
    
    
    # distances = np.array(pcd2_4.compute_point_cloud_distance(pcd2_3))
    # slope_indexes = np.where(distances > .5)[0]
    # rest_indexes = np.where(distances < .5)[0]
    
    # slope_points = slope_2_reconstruido[slope_indexes]
    # # rest_points = np.array(pcd_DTM.points)[rest_indexes]
    
    # slope_2 = o3d.geometry.PointCloud()
    # slope_2.points = o3d.utility.Vector3dVector(slope_points)
    # slope_2.paint_uniform_color([210/255,105/255,30/255])
    
    
    pcd_DTM = o3d.geometry.PointCloud()
    pcd_DTM.points = o3d.utility.Vector3dVector(reconstructed_surface)
    
    # We are going to paint the DTM with a gradient of browns (just to make easier the visualization):
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
    
    pcd_DTM.colors = o3d.utility.Vector3dVector(DTM_colors)
    
    
    # Also, we are going to paint every point cloud segment:
    
    #-------------------------
    
    pcd_road = o3d.geometry.PointCloud()
    pcd_road.points = o3d.utility.Vector3dVector(reconstructed_road)
    pcd_road.paint_uniform_color([0,0,0])
    
    #-------------------------
    
    pcd_shoulder = o3d.geometry.PointCloud()
    pcd_shoulder.points = o3d.utility.Vector3dVector(reconstructed_shoulder)
    pcd_shoulder.paint_uniform_color(np.array([255.,233/255.,0.]))
    
    #-------------------------
    
    pcd_berm = o3d.geometry.PointCloud()
    pcd_berm.points = o3d.utility.Vector3dVector(reconstructed_berm)
    pcd_berm.paint_uniform_color(np.array([82/255.,206/255.,231/255.]))
    
    #-------------------------
    
    slope = o3d.geometry.PointCloud()
    slope.points = o3d.utility.Vector3dVector(reconstructed_slope)
    slope.paint_uniform_color([210/255,105/255,30/255])
    
    #-------------------------
    
    if road_type in ['highway','mixed','national']:
        
        
        # We include also national roads even if the refugee island does not
        # exist in them (necessary for the barriers definition)
    
        pcd_refugee_island = o3d.geometry.PointCloud()
        pcd_refugee_island.points = o3d.utility.Vector3dVector(reconstructed_refugee_island)
        pcd_refugee_island.paint_uniform_color([0,0,1])
        
        
        # There is a variable in road_generator.py called curve_points, which
        # contains the skeleton points that follows the refugee island. It is
        # imposrtant because it is also the skeleton of other parallel elements
        # like barriers, shoulders, etc.
        
        # We are going to generate a jersey barrier over the refugee island, so
        # we need the curve_points:
        
        
        pcd_curve = o3d.geometry.PointCloud()
        pcd_curve.points = o3d.utility.Vector3dVector(curve_points)
        
        pcd_refugee_island_cero = o3d.geometry.PointCloud()
        refugee_island_points_cero = np.array(pcd_refugee_island.points) 
        
        
        minimum_height_barrier = np.max(refugee_island_points_cero.take(2,1))
        
        
        refugee_island_points_cero[:,2] = 0
        pcd_refugee_island_cero.points = o3d.utility.Vector3dVector(refugee_island_points_cero)
        
        distances = np.array(pcd_refugee_island_cero.compute_point_cloud_distance(pcd_curve),dtype=np.float64)
        barrier_indexes = np.where(distances <= 0.3)[0]
        
        barrier_points = reconstructed_refugee_island[barrier_indexes]
        
        ####minimum_height_barrier = np.min(barrier_points.take(2,1))
        
        
        road_heightS_barrier = np.linspace(minimum_height_barrier,minimum_height_barrier+0.84,50)
        
        
        
        pcd_barriers_zero = o3d.geometry.PointCloud()
        barrier_points_zero = np.copy(barrier_points)
        barrier_points_zero[:,2] = 0
        pcd_barriers_zero.points = o3d.utility.Vector3dVector(barrier_points_zero)

        distances = np.array(pcd_barriers_zero.compute_point_cloud_distance(pcd_curve),dtype=np.float64)
        
        barrier_indexes = np.where(distances <= 0.1/2)[0]
        
        ###barrier_points[barrier_indexes,2] += (10+25+49)/100.
        
        
        # We give some elevation to the barrier:
        for i in range(10):
            
            barrier_points[barrier_indexes,2] += 0.1
            elevated_points = barrier_points[barrier_indexes]
            barrier_points = np.vstack((barrier_points,elevated_points))
        
        
        pcd_barriers = o3d.geometry.PointCloud()
        pcd_barriers.points = o3d.utility.Vector3dVector(barrier_points)
        pcd_barriers.paint_uniform_color([160/255.,160/255.,160/255.])
        
        
        
    #-------------------------
    
    
    #======================================================================        
    #======================================================================        
    #======================================================================
    #                           TRAFFIC SIGNALS
    #======================================================================
    #======================================================================
    #======================================================================        
    
    Signals_list = []
    
    indexes_curve = np.arange(0,len(berm_points))
    location_signal = np.random.choice(indexes_curve)
        
    print('Generating traffic signals...')
    if road_type in ['highway','mixed']:
        
        pcd_barrier_1 = copy.deepcopy(pcd_barriers)
        pcd_barrier_1.translate(pcd_barriers.get_center()+np.array([2*road_buffer+0.5*shoulder_buffer,0,0]),relative=False)
        pcd_barrier_2 = copy.deepcopy(pcd_barriers)
        pcd_barrier_2.translate(pcd_barriers.get_center()+np.array([-2*road_buffer-0.5*shoulder_buffer,0,0]),relative=False)
        
        
        berm_indexes = np.arange(0,len(berm_points))
        
        location_signal = np.random.choice(berm_indexes)
        location_signal_1 = berm_points[location_signal]
        
        location_signal = np.random.choice(berm_indexes)
        location_signal_2 = berm_points[location_signal]
        
        location_signal = np.random.choice(berm_indexes)
        location_signal_3 = berm_points[location_signal]
        
        location_signal = np.random.choice(berm_indexes)
        location_signal_4 = berm_points[location_signal]
                            
        # Signals and barriers facing one direction:
            
        SIGNAL,signal_center,center_first_pole,center_second_pole = create_elevated_signal(road_height=0,middle=True,visualization=False)
        
        SIGNAL.translate(pcd_barriers.get_center())
        SIGNAL.translate(SIGNAL.get_center()+np.array([-5,0,0]),relative=False)
        rotation_matrix = SIGNAL.get_rotation_matrix_from_xyz((0,0,np.pi))
        SIGNAL.rotate(rotation_matrix,center=SIGNAL.get_center())
        Signals_list.append(SIGNAL)
        SIGNAL_1 = create_rectangular_signal(final_position=location_signal_1,road_type=road_type)
        
        # We compute wether is at the left or the right lane:
        distance_to_pcd_barrier_1 = np.mean(SIGNAL_1.compute_point_cloud_distance(pcd_barrier_1))
        distance_to_pcd_barrier_2 = np.mean(SIGNAL_1.compute_point_cloud_distance(pcd_barrier_2))
        if distance_to_pcd_barrier_1 < distance_to_pcd_barrier_2:
            # We rotate the signal to face the correct direction:
            rotation_matrix = SIGNAL_1.get_rotation_matrix_from_xyz((0,0,np.pi))
            SIGNAL_1.rotate(rotation_matrix,center=SIGNAL_1.get_center())
        Signals_list.append(SIGNAL_1)
    
        
        SIGNAL_2 = create_triangular_signal(final_position=location_signal_4,road_type=road_type)
        # We compute wether is at the left or the right lane:
        distance_to_pcd_barrier_1 = np.mean(SIGNAL_2.compute_point_cloud_distance(pcd_barrier_1))
        distance_to_pcd_barrier_2 = np.mean(SIGNAL_2.compute_point_cloud_distance(pcd_barrier_2))
        if distance_to_pcd_barrier_1 < distance_to_pcd_barrier_2:
            # We rotate the signal to face the correct direction:
            rotation_matrix = SIGNAL_2.get_rotation_matrix_from_xyz((0,0,np.pi))
            SIGNAL_2.rotate(rotation_matrix,center=SIGNAL_2.get_center())
        Signals_list.append(SIGNAL_2)
    
    
        # Signals and barriers in the other direction:
    
        SIGNAL_3 = create_circle_signal(final_position=location_signal_4,road_type=road_type)
    
        # We compute wether is at the left or the right lane:
        distance_to_pcd_barrier_1 = np.mean(SIGNAL_3.compute_point_cloud_distance(pcd_barrier_1))
        distance_to_pcd_barrier_2 = np.mean(SIGNAL_3.compute_point_cloud_distance(pcd_barrier_2))
        if distance_to_pcd_barrier_1 < distance_to_pcd_barrier_2:
            # We rotate the signal to face the correct direction:
            rotation_matrix = SIGNAL_3.get_rotation_matrix_from_xyz((0,0,np.pi))
            SIGNAL_3.rotate(rotation_matrix,center=SIGNAL_3.get_center())
        Signals_list.append(SIGNAL_3)
    
    
    
    
        SIGNAL_4 = create_square_signal(final_position=location_signal_4,road_type=road_type)
        
        # We compute wether is at the left or the right lane:
        distance_to_pcd_barrier_1 = np.mean(SIGNAL_4.compute_point_cloud_distance(pcd_barrier_1))
        distance_to_pcd_barrier_2 = np.mean(SIGNAL_4.compute_point_cloud_distance(pcd_barrier_2))
        if distance_to_pcd_barrier_1 < distance_to_pcd_barrier_2:
            # We rotate the signal to face the correct direction:
            rotation_matrix = SIGNAL_4.get_rotation_matrix_from_xyz((0,0,np.pi))
            SIGNAL_4.rotate(rotation_matrix,center=SIGNAL_4.get_center())
        Signals_list.append(SIGNAL_4)
        
        


    if road_type == 'national':
       
        
        pcd_barrier_1 = copy.deepcopy(pcd_barriers)
        pcd_barrier_1.translate(pcd_barriers.get_center()+np.array([road_buffer+shoulder_buffer,0,0]),relative=False)
        pcd_barrier_2 = copy.deepcopy(pcd_barriers)
        pcd_barrier_2.translate(pcd_barriers.get_center()+np.array([-road_buffer-shoulder_buffer,0,0]),relative=False)
       
        distance_to_pcd_barrier_1 = np.array(pcd_berm.compute_point_cloud_distance(pcd_barrier_1))
        distance_to_pcd_barrier_2 = np.array(pcd_berm.compute_point_cloud_distance(pcd_barrier_2))
       
        berm_points = np.array(pcd_berm.points)
       
        berm_indexes_1 = np.where(distance_to_pcd_barrier_1 <= 0.5)
        berm_indexes_2 = np.where(distance_to_pcd_barrier_2 > 0.5)
        
        berm_points_1 = berm_points[berm_indexes_1]
        berm_points_2 = berm_points[berm_indexes_2]
        
        # We guarantee 2 signals per direction (this can be changed by the user)
        
        location_signal = np.random.choice(len(berm_points_1))
        location_signal_1 = berm_points[location_signal]
        
        location_signal = np.random.choice(len(berm_points_1))
        location_signal_2 = berm_points[location_signal]
        
        
        location_signal = np.random.choice(len(berm_points_2))
        location_signal_3 = berm_points[location_signal]
        
        location_signal = np.random.choice(len(berm_points_2))
        location_signal_4 = berm_points[location_signal]

        # Signals and barriers facing one direction:
        SIGNAL_1 = create_rectangular_signal(final_position=location_signal_1,road_type=road_type)
        
        # We compute wether is at the left or the right lane:
        distance_to_pcd_barrier_1 = np.mean(SIGNAL_1.compute_point_cloud_distance(pcd_barrier_1))
        distance_to_pcd_barrier_2 = np.mean(SIGNAL_1.compute_point_cloud_distance(pcd_barrier_2))
        if distance_to_pcd_barrier_1 < distance_to_pcd_barrier_2:
            # We rotate the signal to face the correct direction:
            rotation_matrix = SIGNAL_1.get_rotation_matrix_from_xyz((0,0,np.pi))
            SIGNAL_1.rotate(rotation_matrix,center=SIGNAL_1.get_center())
        Signals_list.append(SIGNAL_1)
    
        
        SIGNAL_2 = create_triangular_signal(final_position=location_signal_2,road_type=road_type)
        # We compute wether is at the left or the right lane:
        distance_to_pcd_barrier_1 = np.mean(SIGNAL_2.compute_point_cloud_distance(pcd_barrier_1))
        distance_to_pcd_barrier_2 = np.mean(SIGNAL_2.compute_point_cloud_distance(pcd_barrier_2))
        if distance_to_pcd_barrier_1 < distance_to_pcd_barrier_2:
            # We rotate the signal to face the correct direction:
            rotation_matrix = SIGNAL_2.get_rotation_matrix_from_xyz((0,0,np.pi))
            SIGNAL_2.rotate(rotation_matrix,center=SIGNAL_2.get_center())
        Signals_list.append(SIGNAL_2)
    
    
        # Signals and barriers in the other direction:
    
        SIGNAL_3 = create_circle_signal(final_position=location_signal_3,road_type=road_type)
    
        # We compute wether is at the left or the right lane:
        distance_to_pcd_barrier_1 = np.mean(SIGNAL_3.compute_point_cloud_distance(pcd_barrier_1))
        distance_to_pcd_barrier_2 = np.mean(SIGNAL_3.compute_point_cloud_distance(pcd_barrier_2))
        if distance_to_pcd_barrier_1 < distance_to_pcd_barrier_2:
            # We rotate the signal to face the correct direction:
            rotation_matrix = SIGNAL_3.get_rotation_matrix_from_xyz((0,0,np.pi))
            SIGNAL_3.rotate(rotation_matrix,center=SIGNAL_3.get_center())
        Signals_list.append(SIGNAL_3)
    
    
    
    
        SIGNAL_4 = create_square_signal(final_position=location_signal_4,road_type=road_type)
        
        # We compute wether is at the left or the right lane:
        distance_to_pcd_barrier_1 = np.mean(SIGNAL_4.compute_point_cloud_distance(pcd_barrier_1))
        distance_to_pcd_barrier_2 = np.mean(SIGNAL_4.compute_point_cloud_distance(pcd_barrier_2))
        if distance_to_pcd_barrier_1 < distance_to_pcd_barrier_2:
            # We rotate the signal to face the correct direction:
            rotation_matrix = SIGNAL_4.get_rotation_matrix_from_xyz((0,0,np.pi))
            SIGNAL_4.rotate(rotation_matrix,center=SIGNAL_4.get_center())
        Signals_list.append(SIGNAL_4)
        

    if road_type == 'local':
        
        # No signals (this can be changed by the user)
        Signals_list = []
        pcd_barrier_1 = o3d.geometry.PointCloud()
        pcd_barrier_2 = o3d.geometry.PointCloud()


    if road_type == 'local':
        # We merge the shoulder and the berm with the road:
        pcd_road += pcd_shoulder
        pcd_road += pcd_berm
        pcd_road.paint_uniform_color([0,0,0])


    print('Signals simulation completed')

    # Just notation:
    SURFACE = pcd_DTM
    ROAD = pcd_road
    SLOPE = slope
    
    for LL in range(len(Signals_list)):
        SIGNALS = Signals_list[LL]
    
    if road_type in ['highway','mixed','national']:
        SHOULDER = pcd_shoulder
        BERMS = pcd_berm
        

    if road_type in ['highway','mixed']:
        REFUGEE_ISLAND = pcd_refugee_island
        return SURFACE,ROAD,SLOPE,SHOULDER,SIGNALS,BERMS,REFUGEE_ISLAND,pcd_barriers,pcd_barrier_1,pcd_barrier_2
    
    elif road_type == 'local':
        return SURFACE,ROAD,SLOPE,SHOULDER,SIGNALS,pcd_barrier_1,pcd_barrier_2
    elif road_type == 'nacional':
        return SURFACE,ROAD,SLOPE,SHOULDER,SIGNALS,BERMS,pcd_barriers,pcd_barrier_1,pcd_barrier_2




