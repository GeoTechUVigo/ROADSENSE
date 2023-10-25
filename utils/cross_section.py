#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun  6 09:43:01 2022

@author: lino
"""

import numpy as np
import open3d as o3d
import copy



def highway_vertical_pumping(SURFACE,ROAD,SLOPE,SHOULDER,SIGNALS,BERMS,REFUGEE_ISLAND,pcd_barriers,pcd_barrier_1,pcd_barrier_2,nubes_modificadas=''):
    """
    Function to generate some sort of ground elevation in the road-related parts of highway and mixed roads, i.e., the points closer to the axis road will have a different height than the ones that are further

    :param SURFACE: Point cloud of the DTM excluding road-related elements as a o3d.geometry.PointCloud() object
    :param ROAD: Point cloud of the road as a o3d.geometry.PointCloud() object
    :param SLOPE: Point cloud of the slope as a o3d.geometry.PointCloud() object
    :param SHOULDER: Point cloud of the shoulder as a o3d.geometry.PointCloud() object
    :param SIGNALS: Single point cloud cointaining all traffic signals as a o3d.geometry.PointCloud() object
    :param BERMS: Point cloud of the berm as a o3d.geometry.PointCloud() object
    :param REFUGEE_ISLAND: Point cloud of the road as a o3d.geometry.PointCloud() object
    :param pcd_barriers: Single point cloud containing all barriers as a o3d.geometry.PointCloud() object
    :param pcd_barrier_1: Point cloud of the left barrier as a o3d.geometry.PointCloud() object
    :param pcd_barrier_2: Point cloud of the right barrier as a o3d.geometry.PointCloud() object
    :return: All input elements but with the new heights distribution
    """ 
    
    # Caution! The following buffers are not the same that the ones specified
    # in the config file. They represent different realities.
    shoulder_buffer = 2
    shoulder_buffer_orig = shoulder_buffer*0.6
    berm_buffer = 1
    road_buffer = 15
    shoulder_bufferS = np.linspace(shoulder_buffer,0,30)
    road_bufferS = np.linspace(road_buffer,0,60)
    berm_bufferS = np.linspace(berm_buffer,0,30)
    
    percentages_shoulder_heights = np.linspace(0.5,0,len(shoulder_bufferS))   
    percentages_lanes_heights = np.linspace(1,0,len(road_bufferS)) 
    percentages_berm_heights = np.linspace(0,1,len(berm_bufferS))   

    
    
    # First we identify each element by individual:
    
    road = ROAD # Just notation
    road_points = np.array(road.points)
    
    # We identify each set of lanes by comparing their distances to the barriers:    
    
    distances_barrier_1 = road.compute_point_cloud_distance(pcd_barrier_1)
    average_distance_barrier_1 = np.mean(distances_barrier_1)
    
    distances_barrier_2 = road.compute_point_cloud_distance(pcd_barrier_2)
    average_distance_barrier_2 = np.mean(distances_barrier_2)
    
    indexes_lane_1 = np.where(average_distance_barrier_1 <= distances_barrier_1)[0]
    indexes_lane_2 = np.where(average_distance_barrier_2 < distances_barrier_2)[0]
    
    points_lane_1 = road_points[indexes_lane_1]
    points_lane_2 = road_points[indexes_lane_2]
    
    lane_1 = o3d.geometry.PointCloud()
    lane_1.points = o3d.utility.Vector3dVector(points_lane_1)
    lane_1.paint_uniform_color([1,0,0])
    
    lane_2 = o3d.geometry.PointCloud()
    lane_2.points = o3d.utility.Vector3dVector(points_lane_2)
    lane_2.paint_uniform_color([0,0,1])
    
    #--------------------------------------------------------------------------
    # Uncomment to debug visualizations:
    # o3d.visualization.draw(lane_1+lane_2)
    #--------------------------------------------------------------------------
    
    
    # Now we are going to differentiate 4 sets of shoulders:
        
    shoulder = SHOULDER # Just notation
    shoulder_points = np.array(shoulder.points)
    
    # We compare the distances to the barriers:
        
    distances_barrier_1 = shoulder.compute_point_cloud_distance(pcd_barrier_1)
    average_distance_barrier_1 = np.mean(distances_barrier_1)
    
    distances_barrier_2 = shoulder.compute_point_cloud_distance(pcd_barrier_2)
    average_distance_barrier_2 = np.mean(distances_barrier_2)
    
    indexes_shoulder_1 = np.where(average_distance_barrier_1 <= distances_barrier_1)[0]
    indexes_shoulder_2 = np.where(average_distance_barrier_2 < distances_barrier_2)[0]
    
    shoulder_points_1 = shoulder_points[indexes_shoulder_1]
    shoulder_points_2 = shoulder_points[indexes_shoulder_2]
    
    shoulder_1 = o3d.geometry.PointCloud()
    shoulder_1.points = o3d.utility.Vector3dVector(shoulder_points_1)
    shoulder_1.paint_uniform_color([0.1,0.1,0.8])
    
    shoulder_2 = o3d.geometry.PointCloud()
    shoulder_2.points = o3d.utility.Vector3dVector(shoulder_points_2)
    shoulder_2.paint_uniform_color([0,0,1])
    
    #--------------------------------------------------------------------------
    # Uncomment to debug visualizations:
    # o3d.visualization.draw(shoulder_1+shoulder_2)
    #--------------------------------------------------------------------------
    
    
    # If we repeat the same operation with each shoulder then we can obtain each
    # "subshoulder", i.e., the shoulders of each side of the road.    
    
    # We compare the distances to the barriers (1/2):
        
    distances_barrier_1 = shoulder_1.compute_point_cloud_distance(pcd_barrier_1)
    average_distance_barrier_1 = np.mean(distances_barrier_1)
    
    distances_barrier_2 = shoulder_1.compute_point_cloud_distance(pcd_barrier_2)
    average_distance_barrier_2 = np.mean(distances_barrier_2)
    
    indexes_shoulder_1_1 = np.where(average_distance_barrier_1 <= distances_barrier_1)[0]
    indexes_shoulder_1_2 = np.where(average_distance_barrier_2 < distances_barrier_2)[0]
    
    shoulder_points_1_1 = shoulder_points_1[indexes_shoulder_1_1]
    shoulder_points_1_2 = shoulder_points_1[indexes_shoulder_1_2]
    
    shoulder_1_1 = o3d.geometry.PointCloud()
    shoulder_1_1.points = o3d.utility.Vector3dVector(shoulder_points_1_1)
    shoulder_1_1.paint_uniform_color([0.7,0.3,0])
    
    shoulder_1_2 = o3d.geometry.PointCloud()
    shoulder_1_2.points = o3d.utility.Vector3dVector(shoulder_points_1_2)
    shoulder_1_2.paint_uniform_color([0,0.3,0.7])
    
    
    
    # We compare the distances to the barriers (2/2):
        
    distances_barrier_1 = shoulder_2.compute_point_cloud_distance(pcd_barrier_1)
    average_distance_barrier_1 = np.mean(distances_barrier_1)
    
    distances_barrier_2 = shoulder_2.compute_point_cloud_distance(pcd_barrier_2)
    average_distance_barrier_2 = np.mean(distances_barrier_2)
    
    indexes_shoulder_2_1 = np.where(average_distance_barrier_1 <= distances_barrier_1)[0]
    indexes_shoulder_2_2 = np.where(average_distance_barrier_2 < distances_barrier_2)[0]
    
    shoulder_points_2_1 = shoulder_points_2[indexes_shoulder_2_1]
    shoulder_points_2_2 = shoulder_points_2[indexes_shoulder_2_2]
    
    shoulder_2_1 = o3d.geometry.PointCloud()
    shoulder_2_1.points = o3d.utility.Vector3dVector(shoulder_points_2_1)
    shoulder_2_1.paint_uniform_color([1,1,0])
    
    shoulder_2_2 = o3d.geometry.PointCloud()
    shoulder_2_2.points = o3d.utility.Vector3dVector(shoulder_points_2_2)
    shoulder_2_2.paint_uniform_color([0,1,1])
    
    
    #--------------------------------------------------------------------------
    # Uncomment to debug visualizations:
    # o3d.visualization.draw(shoulder_1_1+shoulder_1_2+shoulder_2_1+shoulder_2_2)
    #--------------------------------------------------------------------------
    
    
    berms = BERMS # Just notation
    berms_points = np.array(berms.points)
    
    # Now we are going to differentiate 4 sets of berms by comparing their
    # distances to the barrier:
        
    distances_barrier_1 = berms.compute_point_cloud_distance(pcd_barrier_1)
    average_distance_barrier_1 = np.mean(distances_barrier_1)
    
    distances_barrier_2 = berms.compute_point_cloud_distance(pcd_barrier_2)
    average_distance_barrier_2 = np.mean(distances_barrier_2)
    
    indexes_berm_1 = np.where(average_distance_barrier_1 <= distances_barrier_1)[0]
    indexes_berm_2 = np.where(average_distance_barrier_2 < distances_barrier_2)[0]
    
    points_berm_1 = berms_points[indexes_berm_1]
    points_berm_2 = berms_points[indexes_berm_2]
    
    berm_1 = o3d.geometry.PointCloud()
    berm_1.points = o3d.utility.Vector3dVector(points_berm_1)
    berm_1.paint_uniform_color([1,0,1])
    
    berm_2 = o3d.geometry.PointCloud()
    berm_2.points = o3d.utility.Vector3dVector(points_berm_2)
    berm_2.paint_uniform_color([0.2,0.2,0.2])
    
    #--------------------------------------------------------------------------
    # Uncomment to debug visualizations:
    # o3d.visualization.draw(berm_1+berm_2)
    #--------------------------------------------------------------------------
    
    # We choose one of the berms, and we do not care which one of them because at
    # this stage both have the same height:
    z_berm_2 = points_berm_2.take(2,1)
    initial_height_berms = np.mean(z_berm_2.mean())
    
    
    # Now we have each element identified:
        
    # · berm_1
    # · shoulder_1_1
    # · lane_1
    # · shoulder_1_2
    # · refugee_island (unique)
    # · shoulder_2_1
    # · lane_2
    # · shoulder_2_2
    # · berm_2
    
    
    # Now we are going to rise the height of the refugee island and, from it,
    # modify the heights of the remaining elements in the road.
    
    refugee_island = REFUGEE_ISLAND # Just notation
    refugee_island_points = np.array(refugee_island.points)
    altura_refugee_island = np.min(refugee_island_points.take(2,1))
    
    INITIAL_HEIGHT = np.min(refugee_island_points.take(2,1))
    
    new_height = altura_refugee_island + 0.2
    # Tricky but required (trust me):
    FINAL_HEIGHT = new_height
    
    refugee_island.translate(([refugee_island.get_center()[0],
                        refugee_island.get_center()[1],
                        new_height]),relative=False)
    
    refugee_island.paint_uniform_color([0,1,0])
    
    
    #--------------------------------------------------------------------------
    # Uncomment to debug visualizations:
    # o3d.visualization.draw(refugee_island+shoulder_1_2+lane_2+lane_1+shoulder_2_1+shoulder_2_2)
    #--------------------------------------------------------------------------
    
    # In order to clarify the notation that we are following:
        
    '''  
    \   \   \\     \     \    \     \\       \\     \     \     \     \\   \   \
    \   \   \\     \     \    \     \\       \\     \     \     \     \\   \   \
    \   \   \\     \     \    \     \\       \\     \     \     \     \\   \   \
    \   \   \\     \  S  \    \  S  \\   R   \\  S  \     \  S  \     \\   \   \
    \   \   \\     \  H  \    \  H  \\   E   \\  H  \     \  H  \     \\   \   \
    \   \ S \\  B  \  O  \  L \  O  \\   F   \\  O  \  L  \  O  \  B  \\ S \   \
    \ D \ L \\  E  \  U  \  A \  U  \\   .   \\  U  \  A  \  U  \  E  \\ L \ D \
    \ T \ O \\  R  \  L  \  N \  L  \\   I   \\  L  \  N  \  L  \  R  \\ O \ T \
    \ M \ P \\  M  \  D  \  E \  D  \\   S   \\  D  \  E  \  D  \  M  \\ P \ M \
    \   \ E \\  1  \  E  \  1 \  E  \\   L   \\  E  \  2  \  E  \  2  \\ E \   \
    \   \   \\     \  R  \    \  R  \\   A   \\  R  \     \  R  \     \\   \   \
    \   \   \\     \  1  \    \  1  \\   N   \\  2  \     \  2  \     \\   \   \
    \   \   \\     \  1  \    \  2  \\   D   \\  1  \     \  2  \     \\   \   \
    \   \   \\     \     \    \     \\       \\     \     \     \     \\   \   \
    \   \   \\     \     \    \     \\       \\     \     \     \     \\   \   \
    \   \   \\     \     \    \     \\       \\     \     \     \     \\   \   \
    '''

    print('Modifying the cross section of the shoulder 1.2')


    #--------------------------------------------------------------------------
    # For bigger inclinations:
    # percentages_heights = np.linspace(1,0,len(shoulder_bufferS))   
    percentages_heights = percentages_shoulder_heights  
    #--------------------------------------------------------------------------


    pcd_shoulder_1_2 = o3d.geometry.PointCloud()
    shoulder_points_1_2 = np.array(shoulder_1_2.points)
    pcd_shoulder_1_2.points = o3d.utility.Vector3dVector(shoulder_points_1_2)
    pcd_shoulder_1_2.paint_uniform_color([0.11,0.22,0.33])
    
    pcd_auxiliar = o3d.geometry.PointCloud()
    
    
    pcd2_3 = o3d.geometry.PointCloud()
    shoulder_1_2_plane = np.array(pcd_shoulder_1_2.points)
    shoulder_1_2_plane[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(shoulder_1_2_plane)
    
    pcd2_4 = o3d.geometry.PointCloud()
    refugee_island_plane = np.array(refugee_island.points)
    refugee_island_plane[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(refugee_island_plane)
    
    
    distances = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    maximum_height_lane_1 = 0
    maximum_height_lane_1_TAKEN = False
    
    
    for p in range(len(shoulder_bufferS)-1):  
        shoulder_buffer = shoulder_bufferS[p]
        next_shoulder_buffer = shoulder_bufferS[p+1]
    
        indexes_shoulder_1_2_in_buffer_a = np.where(distances >= next_shoulder_buffer)[0]
        indexes_shoulder_1_2_in_buffer_b = np.where(distances <= shoulder_buffer)[0]
        
        common_indexes = np.intersect1d(indexes_shoulder_1_2_in_buffer_a,
                                         indexes_shoulder_1_2_in_buffer_b)
        
        shoulder_points_1_2_BUFFER = np.array(pcd_shoulder_1_2.points)[common_indexes]

        if len(shoulder_points_1_2_BUFFER) == 0:
            continue
        else:
        
            
            array_auxiliar_z = shoulder_points_1_2_BUFFER.take(2,1)
            array_auxiliar_z = (new_height) - percentages_heights[p]*(new_height-array_auxiliar_z)
            
            
            array_auxiliar_x = shoulder_points_1_2_BUFFER.take(0,1)
            array_auxiliar_y = shoulder_points_1_2_BUFFER.take(1,1)
            
            shoulder_points_1_2 = np.stack((array_auxiliar_x,
                                          array_auxiliar_y,
                                          array_auxiliar_z),axis=-1)
            
            
            pcd_auxiliar_2 = o3d.geometry.PointCloud()
            pcd_auxiliar_2.points = o3d.utility.Vector3dVector(shoulder_points_1_2)
            pcd_auxiliar_2.paint_uniform_color([0.4,0.2,1])
            pcd_auxiliar += pcd_auxiliar_2
            
            
            
            if np.min(array_auxiliar_z) > maximum_height_lane_1 == 0 and maximum_height_lane_1_TAKEN == False:
                # We define the height from which the next element will be lowered
                # (road in this specific case):
                maximum_height_lane_1 = np.min(array_auxiliar_z)
            
    
    shoulder_points_1_2 = np.array(pcd_auxiliar.points)
    pcd_shoulder_1_2 = o3d.geometry.PointCloud()
    pcd_shoulder_1_2.points = o3d.utility.Vector3dVector(shoulder_points_1_2)
    pcd_shoulder_1_2.paint_uniform_color([0.4,0.2,1])
    
    
    #--------------------------------------------------------------------------
    # Uncomment to debug visualizations:
    # o3d.visualization.draw(refugee_island+pcd_shoulder_1_2+shoulder_1_1+lane_2+lane_1+shoulder_2_1+shoulder_2_2)
    #--------------------------------------------------------------------------
    
    
    print('Modifying the cross section of the lane 1')
    
        
    # new_height = maximum_height_siguiente_elemento
      
    #--------------------------------------------------------------------------
    # For bigger inclinations:
    # percentages_heights = np.linspace(7.5,0,len(road_bufferS))   
    
    percentages_heights = percentages_lanes_heights  
    #--------------------------------------------------------------------------
    
    pcd_lane_1 = o3d.geometry.PointCloud()
    points_lane_1 = np.array(lane_1.points)
    pcd_lane_1.points = o3d.utility.Vector3dVector(points_lane_1)
    pcd_lane_1.paint_uniform_color([0.,0.,0.])
    
    pcd_auxiliar = o3d.geometry.PointCloud()
    
    
    
    pcd2_3 = o3d.geometry.PointCloud()
    lane_1_plane = np.array(pcd_lane_1.points)
    lane_1_plane[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(lane_1_plane)
    
    pcd2_4 = o3d.geometry.PointCloud()
    shoulder_1_2_plane = np.array(pcd_shoulder_1_2.points)
    shoulder_1_2_plane[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(shoulder_1_2_plane)
    
    
    
    
    distances = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    maximum_height_shoulder_1_1 = 0
    maximum_height_shoulder_1_1_TAKEN = False
    
    for p in range(len(road_bufferS)-1):  
        road_buffer = road_bufferS[p]
        siguiente_road_buffer = road_bufferS[p+1]
    
        indexes_lane_1_en_buffer_a = np.where(distances >= siguiente_road_buffer)[0]
        indexes_lane_1_en_buffer_b = np.where(distances <= road_buffer)[0]
        
        common_indexes = np.intersect1d(indexes_lane_1_en_buffer_a,
                                         indexes_lane_1_en_buffer_b)
        
        points_lane_1_BUFFER = np.array(pcd_lane_1.points)[common_indexes]

        if len(points_lane_1_BUFFER) == 0:
            continue
        else:
            
            array_auxiliar_z = points_lane_1_BUFFER.take(2,1)
            array_auxiliar_z = (new_height) - percentages_heights[p]*(new_height-array_auxiliar_z)
            
            array_auxiliar_x = points_lane_1_BUFFER.take(0,1)
            array_auxiliar_y = points_lane_1_BUFFER.take(1,1)
            
            points_lane_1 = np.stack((array_auxiliar_x,
                                          array_auxiliar_y,
                                          array_auxiliar_z),axis=-1)
            
            
            pcd_auxiliar_2 = o3d.geometry.PointCloud()
            pcd_auxiliar_2.points = o3d.utility.Vector3dVector(points_lane_1)
            pcd_auxiliar_2.paint_uniform_color([0.,0.,0.])
            pcd_auxiliar += pcd_auxiliar_2
            
            
            
            if np.min(array_auxiliar_z)>maximum_height_shoulder_1_1 and maximum_height_shoulder_1_1_TAKEN == False:
                # We define the height from which the next element will be lowered
                # (shoulder in this specific case):
                maximum_height_shoulder_1_1 = np.min(array_auxiliar_z)
                maximum_height_shoulder_1_1_TAKEN = True
            if p == len(road_bufferS)-2:
                maximum_height_current_element = np.mean(array_auxiliar_z)
                
    points_lane_1 = np.array(pcd_auxiliar.points)
    points_lane_1_z = points_lane_1.take(2,1)
    points_lane_1_z = points_lane_1_z - np.abs(maximum_height_current_element-maximum_height_lane_1)
    points_lane_1_x = points_lane_1.take(0,1)
    points_lane_1_y = points_lane_1.take(1,1)
    points_lane_1 = np.stack((points_lane_1_x,
                                points_lane_1_y,
                                points_lane_1_z),axis=-1)
    lane_1 = o3d.geometry.PointCloud()
    lane_1.points = o3d.utility.Vector3dVector(points_lane_1)
    lane_1.paint_uniform_color([0.,0.,0.])
    
    minimum_height_road = points_lane_1_z.min()
    
    #--------------------------------------------------------------------------
    # Uncomment to debug visualizations:
    # o3d.visualization.draw(refugee_island+pcd_shoulder_1_2+shoulder_1_1+lane_2+lane_1+shoulder_2_1+shoulder_2_2)
    #--------------------------------------------------------------------------
    
        
        
    print('Modifying the cross section of the shoulder 1.1')
    
        
    pcd_shoulder_1_1 = o3d.geometry.PointCloud()
    shoulder_points_1_1 = np.array(shoulder_1_1.points)
    pcd_shoulder_1_1.points = o3d.utility.Vector3dVector(shoulder_points_1_1)
    pcd_shoulder_1_1.paint_uniform_color([0.3,0.3,0.3])
    
    pcd_auxiliar = o3d.geometry.PointCloud()
    
    
    pcd2_3 = o3d.geometry.PointCloud()
    shoulder_1_1_plane = np.array(pcd_shoulder_1_1.points)
    shoulder_1_1_plane[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(shoulder_1_1_plane)
    
    pcd2_4 = o3d.geometry.PointCloud()
    lane_plane = np.array(lane_1.points)
    lane_plane[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(lane_plane)
    
    
    
    
    distances = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    minimum_height_berm_1 = 0
    minimum_height_berm_1_TAKEN = False
    
    
    #--------------------------------------------------------------------------
    # For bigger inclinations:
    percentages_heights = np.linspace(0,1.5,len(shoulder_bufferS))   
    # percentages_heights = np.flip(percentages_shoulder_heights)
    #--------------------------------------------------------------------------
    
    for p in range(len(shoulder_bufferS)-1):  
        shoulder_buffer = shoulder_bufferS[p]
        next_shoulder_buffer = shoulder_bufferS[p+1]
    
    
    
        indexes_shoulder_1_1_en_buffer_a = np.where(distances >= next_shoulder_buffer)[0]
        indexes_shoulder_1_1_en_buffer_b = np.where(distances <= shoulder_buffer)[0]
        
        common_indexes = np.intersect1d(indexes_shoulder_1_1_en_buffer_a,
                                         indexes_shoulder_1_1_en_buffer_b)
        
        shoulder_points_1_1_BUFFER = np.array(pcd_shoulder_1_1.points)[common_indexes]

        if len(shoulder_points_1_1_BUFFER) == 0:
            continue
        else:
                    
            array_auxiliar_z = shoulder_points_1_1_BUFFER.take(2,1)
            array_auxiliar_z = (minimum_height_road) - percentages_heights[p]*(minimum_height_road-array_auxiliar_z)
            
            
            array_auxiliar_x = shoulder_points_1_1_BUFFER.take(0,1)
            array_auxiliar_y = shoulder_points_1_1_BUFFER.take(1,1)
            
            shoulder_points_1_1 = np.stack((array_auxiliar_x,
                                          array_auxiliar_y,
                                          array_auxiliar_z),axis=-1)
            
            
            pcd_auxiliar_2 = o3d.geometry.PointCloud()
            pcd_auxiliar_2.points = o3d.utility.Vector3dVector(shoulder_points_1_1)
            pcd_auxiliar_2.paint_uniform_color([0.4,0.2,1])
            pcd_auxiliar += pcd_auxiliar_2
            
            
            
            if np.min(array_auxiliar_z) > minimum_height_berm_1 == 0 and minimum_height_berm_1_TAKEN == False:
                # We define the height from which the next element will be lowered
                # (road in this specific case):
                minimum_height_berm_1 = np.min(array_auxiliar_z)
            
        
            
    shoulder_points_1_1 = np.array(pcd_auxiliar.points)
    shoulder_points_1_1_z = shoulder_points_1_1.take(2,1)
    shoulder_points_1_1_x = shoulder_points_1_1.take(0,1)
    shoulder_points_1_1_y = shoulder_points_1_1.take(1,1)
    shoulder_points_1_1 = np.stack((shoulder_points_1_1_x,
                                shoulder_points_1_1_y,
                                shoulder_points_1_1_z),axis=-1)
    
    
    pcd_shoulder_1_1 = o3d.geometry.PointCloud()
    pcd_shoulder_1_1.points = o3d.utility.Vector3dVector(shoulder_points_1_1)
    pcd_shoulder_1_1.paint_uniform_color([0.3,0.3,0.3])
        
    
    # Now lets lower the shoulder:
    
    maximum_height_shoulder_1_1 = np.max(shoulder_points_1_1_z)
    distance_lower_shoulder_1_1 = np.abs(minimum_height_road-maximum_height_shoulder_1_1)
    distance_lower_shoulder_1_1 = distance_lower_shoulder_1_1 - 0.2*distance_lower_shoulder_1_1
    new_location = np.array([pcd_shoulder_1_1.get_center()[0],
                                pcd_shoulder_1_1.get_center()[1],
                                pcd_shoulder_1_1.get_center()[2]-distance_lower_shoulder_1_1])
    pcd_shoulder_1_1.translate(new_location,relative=False)
    
    
    #--------------------------------------------------------------------------
    # Uncomment to debug visualizations:
    # o3d.visualization.draw(refugee_island+pcd_shoulder_1_2+pcd_shoulder_1_1+lane_2+lane_1+shoulder_2_1+shoulder_2_2)
    #--------------------------------------------------------------------------
        
        
    print('Modifying the cross section of the berm 1')
    
        
    percentages_heights = percentages_berm_heights
    
    pcd_berm_1 = o3d.geometry.PointCloud()
    points_berm_1 = np.array(berm_1.points)
    pcd_berm_1.points = o3d.utility.Vector3dVector(points_berm_1)
    pcd_berm_1.paint_uniform_color([0.1,0.2,0.3])
    
    pcd_auxiliar = o3d.geometry.PointCloud()
    
    
    pcd2_3 = o3d.geometry.PointCloud()
    berm_1_plane = np.array(pcd_berm_1.points)
    berm_1_plane[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(berm_1_plane)
    
    pcd2_4 = o3d.geometry.PointCloud()
    shoulder_1_1_plane = np.array(pcd_shoulder_1_1.points)
    shoulder_1_1_plane[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(shoulder_1_1_plane)
    
    
    distances = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    
    minimum_height_berm_1 = 0
    minimum_height_berm_1_TAKEN = False
    
    
    for p in range(len(shoulder_bufferS)-1):  
        shoulder_buffer = shoulder_bufferS[p]
        next_shoulder_buffer = shoulder_bufferS[p+1]
    
    
    
        indexes_berm_1_en_buffer_a = np.where(distances >= next_shoulder_buffer)[0]
        indexes_berm_1_en_buffer_b = np.where(distances <= shoulder_buffer)[0]
        
        common_indexes = np.intersect1d(indexes_berm_1_en_buffer_a,
                                         indexes_berm_1_en_buffer_b)
        
        points_berm_1_BUFFER = np.array(pcd_berm_1.points)[common_indexes]

        if len(points_berm_1_BUFFER) == 0:
            continue
        else:
            
            array_auxiliar_z = points_berm_1_BUFFER.take(2,1)
            array_auxiliar_z = (new_height) - percentages_heights[p]*(new_height-array_auxiliar_z)
            
            
            array_auxiliar_x = points_berm_1_BUFFER.take(0,1)
            array_auxiliar_y = points_berm_1_BUFFER.take(1,1)
            
            points_berm_1 = np.stack((array_auxiliar_x,
                                          array_auxiliar_y,
                                          array_auxiliar_z),axis=-1)
            
            
            pcd_auxiliar_2 = o3d.geometry.PointCloud()
            pcd_auxiliar_2.points = o3d.utility.Vector3dVector(points_berm_1)
            pcd_auxiliar_2.paint_uniform_color([0.1,0.2,0.3])
            pcd_auxiliar += pcd_auxiliar_2
            
            
            
            if np.mean(array_auxiliar_z)>minimum_height_berm_1 and minimum_height_berm_1_TAKEN == False:
                # We define the height from which the next element will be lowered
                # (berm in this specific case):
                minimum_height_berm_1 = np.mean(array_auxiliar_z)
                minimum_height_berm_1_TAKEN = True
            if p == len(shoulder_bufferS)-2:
                minimum_height_elemento_actual = np.mean(array_auxiliar_z)
            
    
    points_berm_1 = np.array(pcd_auxiliar.points)
    
    
    pcd_berm_1 = o3d.geometry.PointCloud()
    pcd_berm_1.points = o3d.utility.Vector3dVector(points_berm_1)
    pcd_berm_1.paint_uniform_color([1,0,0])
    
    points_berm_1_z = points_berm_1.take(2,1)
    
    minimum_height_shoulder = np.min(shoulder_points_1_1_z)
    minimum_height_berm_auxiliar = np.min(points_berm_1_z)
    
    
    new_location = np.array([pcd_berm_1.get_center()[0],
                                pcd_berm_1.get_center()[1],
                                pcd_berm_1.get_center()[2]-(distance_lower_shoulder_1_1*(1+0.5))])
    pcd_berm_1.translate(new_location,relative=False)
    
    
    #--------------------------------------------------------------------------
    # Uncomment to debug visualizations:
    # o3d.visualization.draw(refugee_island+pcd_shoulder_1_2+pcd_shoulder_1_1+lane_2+lane_1+shoulder_2_1+shoulder_2_2+pcd_berm_1)
    #--------------------------------------------------------------------------
        
    # We have already finished the first half of all elements.
        
    print('Modifying the cross section of the shoulder 2.1')

    
    #--------------------------------------------------------------------------
    # For bigger inclinations:
    # percentages_heights = np.linspace(1,0,len(shoulder_bufferS))   
    percentages_heights = percentages_shoulder_heights  
    #--------------------------------------------------------------------------
    
    pcd_shoulder_2_1 = o3d.geometry.PointCloud()
    shoulder_points_2_1 = np.array(shoulder_2_1.points)
    pcd_shoulder_2_1.points = o3d.utility.Vector3dVector(shoulder_points_2_1)
    pcd_shoulder_2_1.paint_uniform_color([0.11,0.22,0.33])
    
    pcd_auxiliar = o3d.geometry.PointCloud()
    
    
    pcd2_3 = o3d.geometry.PointCloud()
    shoulder_2_1_plane = np.array(pcd_shoulder_2_1.points)
    shoulder_2_1_plane[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(shoulder_2_1_plane)
    
    pcd2_4 = o3d.geometry.PointCloud()
    refugee_island_plane = np.array(refugee_island.points)
    refugee_island_plane[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(refugee_island_plane)
    
    
    distances = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    maximum_height_lane_2 = 0
    maximum_height_lane_2_TAKEN = False
    
    
    for p in range(len(shoulder_bufferS)-1):  
        shoulder_buffer = shoulder_bufferS[p]
        next_shoulder_buffer = shoulder_bufferS[p+1]
    
        indexes_shoulder_2_1_en_buffer_a = np.where(distances >= next_shoulder_buffer)[0]
        indexes_shoulder_2_1_en_buffer_b = np.where(distances <= shoulder_buffer)[0]
        
        common_indexes = np.intersect1d(indexes_shoulder_2_1_en_buffer_a,
                                         indexes_shoulder_2_1_en_buffer_b)
        
        shoulder_points_2_1_BUFFER = np.array(pcd_shoulder_2_1.points)[common_indexes]

        if len(shoulder_points_2_1_BUFFER) == 0:
            continue
        else:
                    
            array_auxiliar_z = shoulder_points_2_1_BUFFER.take(2,1)
            array_auxiliar_z = (new_height) - percentages_heights[p]*(new_height-array_auxiliar_z)
            
            
            array_auxiliar_x = shoulder_points_2_1_BUFFER.take(0,1)
            array_auxiliar_y = shoulder_points_2_1_BUFFER.take(1,1)
            
            shoulder_points_2_1 = np.stack((array_auxiliar_x,
                                          array_auxiliar_y,
                                          array_auxiliar_z),axis=-1)
            
                     
            pcd_auxiliar_2 = o3d.geometry.PointCloud()
            pcd_auxiliar_2.points = o3d.utility.Vector3dVector(shoulder_points_2_1)
            pcd_auxiliar_2.paint_uniform_color([0.4,0.2,1])
            pcd_auxiliar += pcd_auxiliar_2
            
            
            
            if np.min(array_auxiliar_z) > maximum_height_lane_2 == 0 and maximum_height_lane_2_TAKEN == False:
                # We define the height from which the next element will be lowered
                # (road in this specific case):
                maximum_height_lane_2 = np.min(array_auxiliar_z)
            
    
    shoulder_points_2_1 = np.array(pcd_auxiliar.points)
    pcd_shoulder_2_1 = o3d.geometry.PointCloud()
    pcd_shoulder_2_1.points = o3d.utility.Vector3dVector(shoulder_points_2_1)
    pcd_shoulder_2_1.paint_uniform_color([1,1,0])

    
    #--------------------------------------------------------------------------
    # Uncomment to debug visualizations:
    # o3d.visualization.draw(refugee_island+pcd_shoulder_1_2+pcd_shoulder_1_1+lane_2+pcd_berm_1+lane_1+pcd_shoulder_2_1+shoulder_2_2)
    #--------------------------------------------------------------------------
        
        
      
    print('Modifying the cross section of the lane 2')
    
        
    #--------------------------------------------------------------------------
    # For bigger inclinations:
    # percentages_heights = np.linspace(7.5,0,len(road_bufferS))   
    
    percentages_heights = percentages_lanes_heights   
    #--------------------------------------------------------------------------
    
    pcd_lane_2 = o3d.geometry.PointCloud()
    points_lane_2 = np.array(lane_2.points)
    pcd_lane_2.points = o3d.utility.Vector3dVector(points_lane_2)
    pcd_lane_2.paint_uniform_color([0.,0.,0.])
    
    pcd_auxiliar = o3d.geometry.PointCloud()
    
    
    
    pcd2_3 = o3d.geometry.PointCloud()
    lane_2_plane = np.array(pcd_lane_2.points)
    lane_2_plane[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(lane_2_plane)
    
    pcd2_4 = o3d.geometry.PointCloud()
    shoulder_2_1_plane = np.array(pcd_shoulder_2_1.points)
    shoulder_2_1_plane[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(shoulder_2_1_plane)
    
    
    
    
    distances = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    maximum_height_shoulder_2_2 = 0
    maximum_height_shoulder_2_2_TAKEN = False
    
    for p in range(len(road_bufferS)-1):  
        road_buffer = road_bufferS[p]
        siguiente_road_buffer = road_bufferS[p+1]
    
    
    
        indexes_lane_2_en_buffer_a = np.where(distances >= siguiente_road_buffer)[0]
        indexes_lane_2_en_buffer_b = np.where(distances <= road_buffer)[0]
        
        common_indexes = np.intersect1d(indexes_lane_2_en_buffer_a,
                                         indexes_lane_2_en_buffer_b)
        
        points_lane_2_BUFFER = np.array(pcd_lane_2.points)[common_indexes]

        if len(points_lane_2_BUFFER) == 0:
            continue
        else:
            
            array_auxiliar_z = points_lane_2_BUFFER.take(2,1)
            array_auxiliar_z = (new_height) - percentages_heights[p]*(new_height-array_auxiliar_z)
            
            array_auxiliar_x = points_lane_2_BUFFER.take(0,1)
            array_auxiliar_y = points_lane_2_BUFFER.take(1,1)
            
            points_lane_2 = np.stack((array_auxiliar_x,
                                          array_auxiliar_y,
                                          array_auxiliar_z),axis=-1)
            
            
            pcd_auxiliar_2 = o3d.geometry.PointCloud()
            pcd_auxiliar_2.points = o3d.utility.Vector3dVector(points_lane_2)
            pcd_auxiliar_2.paint_uniform_color([0.,0.,0.])
            pcd_auxiliar += pcd_auxiliar_2
            
            
            
            if np.min(array_auxiliar_z)>maximum_height_shoulder_2_2 and maximum_height_shoulder_2_2_TAKEN == False:
                # We define the height from which the next element will be lowered
                # (shoulder in this specific case):
                maximum_height_shoulder_2_2 = np.min(array_auxiliar_z)
                maximum_height_shoulder_2_2_TAKEN = True
            if p == len(road_bufferS)-2:
                maximum_height_current_element = np.mean(array_auxiliar_z)
    
    points_lane_2 = np.array(pcd_auxiliar.points)
    points_lane_2_z = points_lane_2.take(2,1)
    points_lane_2_z = points_lane_2_z - np.abs(maximum_height_current_element-maximum_height_lane_2)
    points_lane_2_x = points_lane_2.take(0,1)
    points_lane_2_y = points_lane_2.take(1,1)
    points_lane_2 = np.stack((points_lane_2_x,
                                points_lane_2_y,
                                points_lane_2_z),axis=-1)
    lane_2 = o3d.geometry.PointCloud()
    lane_2.points = o3d.utility.Vector3dVector(points_lane_2)
    lane_2.paint_uniform_color([0.,0.,0.])
    
    minimum_height_road = points_lane_2_z.min()
    
    #--------------------------------------------------------------------------
    # Uncomment to debug visualizations:
    # o3d.visualization.draw(refugee_island+pcd_shoulder_1_2+shoulder_1_1+lane_2+lane_1+shoulder_2_1+shoulder_2_2)
    #--------------------------------------------------------------------------
    
        
        
    print('Modifying the cross section of the shoulder 2.2')
    
                
    pcd_shoulder_2_2 = o3d.geometry.PointCloud()
    shoulder_points_2_2 = np.array(shoulder_2_2.points)
    pcd_shoulder_2_2.points = o3d.utility.Vector3dVector(shoulder_points_2_2)
    pcd_shoulder_2_2.paint_uniform_color([0.3,0.3,0.3])
    
    pcd_auxiliar = o3d.geometry.PointCloud()
    
    
    pcd2_3 = o3d.geometry.PointCloud()
    shoulder_2_2_plane = np.array(pcd_shoulder_2_2.points)
    shoulder_2_2_plane[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(shoulder_2_2_plane)
    
    pcd2_4 = o3d.geometry.PointCloud()
    lane_plane = np.array(lane_2.points)
    lane_plane[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(lane_plane)
    
    
    distances = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    minimum_height_berm_1 = 0
    minimum_height_berm_1_TAKEN = False
    
    
    #--------------------------------------------------------------------------
    # For bigger inclinations:
    percentages_heights = np.linspace(0,1.5,len(shoulder_bufferS))       
    # percentages_heights = np.flip(percentages_shoulder_heights)
    #--------------------------------------------------------------------------
    
    for p in range(len(shoulder_bufferS)-1):  
        shoulder_buffer = shoulder_bufferS[p]
        next_shoulder_buffer = shoulder_bufferS[p+1]
    
        indexes_shoulder_2_2_en_buffer_a = np.where(distances >= next_shoulder_buffer)[0]
        indexes_shoulder_2_2_en_buffer_b = np.where(distances <= shoulder_buffer)[0]
        
        common_indexes = np.intersect1d(indexes_shoulder_2_2_en_buffer_a,
                                         indexes_shoulder_2_2_en_buffer_b)
        
        shoulder_points_2_2_BUFFER = np.array(pcd_shoulder_2_2.points)[common_indexes]

        if len(shoulder_points_2_2_BUFFER) == 0:
            continue
        else:
                    
            array_auxiliar_z = shoulder_points_2_2_BUFFER.take(2,1)
            array_auxiliar_z = (minimum_height_road) - percentages_heights[p]*(minimum_height_road-array_auxiliar_z)
            
            
            array_auxiliar_x = shoulder_points_2_2_BUFFER.take(0,1)
            array_auxiliar_y = shoulder_points_2_2_BUFFER.take(1,1)
            
            shoulder_points_2_2 = np.stack((array_auxiliar_x,
                                          array_auxiliar_y,
                                          array_auxiliar_z),axis=-1)
            
            
            pcd_auxiliar_2 = o3d.geometry.PointCloud()
            pcd_auxiliar_2.points = o3d.utility.Vector3dVector(shoulder_points_2_2)
            pcd_auxiliar_2.paint_uniform_color([0.4,0.2,1])
            pcd_auxiliar += pcd_auxiliar_2
            
            
            if np.min(array_auxiliar_z) > minimum_height_berm_1 == 0 and minimum_height_berm_1_TAKEN == False:
                # We define the height from which the next element will be lowered
                # (road in this specific case):
                minimum_height_berm_1 = np.min(array_auxiliar_z)
            
    shoulder_points_2_2 = np.array(pcd_auxiliar.points)
    shoulder_points_2_2_z = shoulder_points_2_2.take(2,1)
    shoulder_points_2_2_x = shoulder_points_2_2.take(0,1)
    shoulder_points_2_2_y = shoulder_points_2_2.take(1,1)
    shoulder_points_2_2 = np.stack((shoulder_points_2_2_x,
                                shoulder_points_2_2_y,
                                shoulder_points_2_2_z),axis=-1)
    
    
    pcd_shoulder_2_2 = o3d.geometry.PointCloud()
    pcd_shoulder_2_2.points = o3d.utility.Vector3dVector(shoulder_points_2_2)
    pcd_shoulder_2_2.paint_uniform_color([0.3,0.3,0.3])
        
        
    # Now we will lower the shoulder a bit:
    
    maximum_height_shoulder_2_2 = np.max(shoulder_points_2_2_z)
    distance_lower_shoulder_2_2 = np.abs(minimum_height_road-maximum_height_shoulder_2_2)
    distance_lower_shoulder_2_2 = distance_lower_shoulder_2_2 - 0.2*distance_lower_shoulder_2_2
    new_location = np.array([pcd_shoulder_2_2.get_center()[0],
                                pcd_shoulder_2_2.get_center()[1],
                                pcd_shoulder_2_2.get_center()[2]-distance_lower_shoulder_2_2])
    pcd_shoulder_2_2.translate(new_location,relative=False)
    
    #--------------------------------------------------------------------------
    # Uncomment to debug visualizations:
    # o3d.visualization.draw(refugee_island+pcd_shoulder_2_2+pcd_shoulder_2_1+lane_2+lane_1+shoulder_2_1+shoulder_2_2)
    #--------------------------------------------------------------------------
        
        
    print('Modifying the cross section of the shoulder 1')
    
        
    percentages_heights = percentages_berm_heights
    
    pcd_berm_2 = o3d.geometry.PointCloud()
    points_berm_2 = np.array(berm_2.points)
    pcd_berm_2.points = o3d.utility.Vector3dVector(points_berm_2)
    pcd_berm_2.paint_uniform_color([0.1,0.2,0.3])
    
    pcd_auxiliar = o3d.geometry.PointCloud()
    
    
    pcd2_3 = o3d.geometry.PointCloud()
    berm_2_plane = np.array(pcd_berm_2.points)
    berm_2_plane[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(berm_2_plane)
    
    pcd2_4 = o3d.geometry.PointCloud()
    shoulder_2_2_plane = np.array(pcd_shoulder_2_2.points)
    shoulder_2_2_plane[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(shoulder_2_2_plane)
    
    distances = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    
    minimum_height_berm_2 = 0
    minimum_height_berm_2_TAKEN = False
    
    
    for p in range(len(shoulder_bufferS)-1):  
        shoulder_buffer = shoulder_bufferS[p]
        next_shoulder_buffer = shoulder_bufferS[p+1]
    
        indexes_berm_2_en_buffer_a = np.where(distances >= next_shoulder_buffer)[0]
        indexes_berm_2_en_buffer_b = np.where(distances <= shoulder_buffer)[0]
        
        common_indexes = np.intersect1d(indexes_berm_2_en_buffer_a,
                                         indexes_berm_2_en_buffer_b)
        
        points_berm_2_BUFFER = np.array(pcd_berm_2.points)[common_indexes]

        if len(points_berm_2_BUFFER) == 0:
            continue
        else:
            
            array_auxiliar_z = points_berm_2_BUFFER.take(2,1)
            array_auxiliar_z = (new_height) - percentages_heights[p]*(new_height-array_auxiliar_z)
            
            
            array_auxiliar_x = points_berm_2_BUFFER.take(0,1)
            array_auxiliar_y = points_berm_2_BUFFER.take(1,1)
            
            points_berm_2 = np.stack((array_auxiliar_x,
                                          array_auxiliar_y,
                                          array_auxiliar_z),axis=-1)
            
            
            pcd_auxiliar_2 = o3d.geometry.PointCloud()
            pcd_auxiliar_2.points = o3d.utility.Vector3dVector(points_berm_2)
            pcd_auxiliar_2.paint_uniform_color([0.1,0.2,0.3])
            pcd_auxiliar += pcd_auxiliar_2
            
            
            
            if np.mean(array_auxiliar_z)>minimum_height_berm_2 and minimum_height_berm_2_TAKEN == False:
                # We define the height from which the next element will be lowered
                # (berm in this specific case):
                minimum_height_berm_2 = np.mean(array_auxiliar_z)
                minimum_height_berm_2_TAKEN = True
            if p == len(shoulder_bufferS)-2:
                minimum_height_elemento_actual = np.mean(array_auxiliar_z)
            
    
    points_berm_2 = np.array(pcd_auxiliar.points)
    
    
    pcd_berm_2 = o3d.geometry.PointCloud()
    pcd_berm_2.points = o3d.utility.Vector3dVector(points_berm_2)
    pcd_berm_2.paint_uniform_color([1,0,0])
    
    points_berm_2_z = points_berm_2.take(2,1)
    
    minimum_height_shoulder = np.min(shoulder_points_2_2_z)
    minimum_height_berm_auxiliar = np.min(points_berm_2_z)
    
    
    new_location = np.array([pcd_berm_2.get_center()[0],
                                pcd_berm_2.get_center()[1],
                                pcd_berm_2.get_center()[2]-(distance_lower_shoulder_1_1*(1+0.5))])
    pcd_berm_2.translate(new_location,relative=False)
    

    
    #--------------------------------------------------------------------------
    # Uncomment to debug visualizations:
    # o3d.visualization.draw(refugee_island+pcd_shoulder_1_2+pcd_shoulder_1_1+lane_2+lane_1+shoulder_2_1+shoulder_2_2+pcd_berm_2)
    #--------------------------------------------------------------------------
        
    
    #--------------------------------------------------------------------------
    # Uncomment to debug visualizations:
    # o3d.visualization.draw(refugee_island+pcd_shoulder_1_2+pcd_shoulder_1_1+lane_2+lane_1+pcd_shoulder_2_1+pcd_shoulder_2_2+pcd_berm_1+pcd_berm_2)
    #--------------------------------------------------------------------------
      
    # We update each input of the function with the new modified objects:
    ROAD = lane_1 + lane_2
    SHOULDER = pcd_shoulder_1_1 + pcd_shoulder_1_2 + pcd_shoulder_2_1 + pcd_shoulder_2_2
    BERMS = pcd_berm_1 + pcd_berm_2
    
    
    # We choose one of the berms, and we do not care which one of them because at
    # this stage both have the same height:
    z_berm_2 = points_berm_2.take(2,1)
    final_height_berms = np.mean(z_berm_2.mean())
      
    # This is the vertical distance that must be subtracted to the traffic signals
    # (only the lateral ones) and the barriers:
    vertical_distance_berms = np.abs(final_height_berms-initial_height_berms)
     
    # This is the vertical distance that must be added to the central traffic 
    # signal and the jersey barrier:
    vertical_distance_refugee_island = np.abs(FINAL_HEIGHT-INITIAL_HEIGHT)    
     
    # Now we are going to move vertically EVERYTHING to make sense:
            
    # First barrier:
    new_location = np.array([pcd_barrier_1.get_center()[0],
                                pcd_barrier_1.get_center()[1],
                                pcd_barrier_1.get_center()[2]-vertical_distance_berms])
    pcd_barrier_1.translate(new_location,relative=False)
      
    # Second barrier:
    new_location = np.array([pcd_barrier_2.get_center()[0],
                                pcd_barrier_2.get_center()[1],
                                pcd_barrier_2.get_center()[2]-vertical_distance_berms])
    pcd_barrier_2.translate(new_location,relative=False)
        
    # Central jersey barrier:
    new_location = np.array([pcd_barriers.get_center()[0],
                                pcd_barriers.get_center()[1],
                                pcd_barriers.get_center()[2]+vertical_distance_refugee_island])
    pcd_barriers.translate(new_location,relative=False)
        
    # Big central signal:
    new_location = np.array([SIGNALS.get_center()[0],
                                SIGNALS.get_center()[1],
                                SIGNALS.get_center()[2]+vertical_distance_refugee_island])
    SIGNALS.translate(new_location,relative=False)
    
    # Lateral signals:
    for i in range(1,len(SIGNALS)):
        new_location = np.array([SIGNALS[i].get_center()[0],
                                    SIGNALS[i].get_center()[1],
                                    SIGNALS[i].get_center()[2]-vertical_distance_berms])
        SIGNALS[i].translate(new_location,relative=False)
        
    # DTM:
    new_location = np.array([SURFACE.get_center()[0],
                                SURFACE.get_center()[1],
                                SURFACE.get_center()[2]-vertical_distance_berms])
    SURFACE.translate(new_location,relative=False)

    # Slopes:
    new_location = np.array([SLOPE.get_center()[0],
                                SLOPE.get_center()[1],
                                SLOPE.get_center()[2]-vertical_distance_berms])
    SLOPE.translate(new_location,relative=False)
        
        
    '''
    Ignore:
    
    
    
    
    
    # voy a añadir ahora una parte que es para diferenciar las autopistas arti-
    # ficiales que creo con respecto a las autopistas reales de algunos casos
    # que tengo:
        
    if nubes_modificadas == 'Santarem':
        
        print('Nubes Santarem')
                
        # Copiamos primero las barreras barrier en los laterales de la me-
        # diana:
            
        # 'altura_refugee_island' ya cogida al principio del script.
        
        pcd_barrier_1_refugee_island = copy.deepcopy(pcd_barriers)
        
        new_location = [refugee_island.get_center()[0]-shoulder_buffer_orig,
                           refugee_island.get_center()[1], 
                           pcd_barriers.get_center()[2]]
        pcd_barrier_1_refugee_island.translate(new_location,relative=False)
        
        
        pcd_barrier_2_refugee_island = copy.deepcopy(pcd_barriers)
        
        new_location = [refugee_island.get_center()[0]+shoulder_buffer_orig,
                           refugee_island.get_center()[1],
                           pcd_barriers.get_center()[2]]
        pcd_barrier_2_refugee_island.translate(new_location,relative=False)
        
        
        
        # Ahora seleccionamos uno de los lanees y le hacemos un DOWNSAMPLING
        # emulando del paso del escáner por el lane contrario:
        
        
        
        
        indexes_seleccionados = np.random.choice(range(len(points_lane_1)), 5000 , replace=False)
        points_lane_1 = points_lane_1[indexes_seleccionados]
        lane_1 = o3d.geometry.PointCloud()
        lane_1.points = o3d.utility.Vector3dVector(points_lane_1)
        lane_1.paint_uniform_color([0,0,0])
       
        ROAD[0] = lane_1 + lane_2

        # Hago lo mismo con los shoulderes y la berma correspondiente:
        indexes_seleccionados = np.random.choice(range(len(shoulder_points_1_1)), 5000 , replace=False)
        shoulder_points_1_1 = shoulder_points_1_1[indexes_seleccionados]
        pcd_shoulder_1_1 = o3d.geometry.PointCloud()
        pcd_shoulder_1_1.points = o3d.utility.Vector3dVector(shoulder_points_1_1)
        pcd_shoulder_1_1.paint_uniform_color([0.3,0.3,0.3])
        
        indexes_seleccionados = np.random.choice(range(len(shoulder_points_1_2)), 2000 , replace=False)
        shoulder_points_1_2 = shoulder_points_1_2[indexes_seleccionados]
        pcd_shoulder_1_2 = o3d.geometry.PointCloud()
        pcd_shoulder_1_2.points = o3d.utility.Vector3dVector(shoulder_points_1_2)
        pcd_shoulder_1_2.paint_uniform_color([0.11,0.22,0.33])

        SHOULDER[0] = pcd_shoulder_1_1 + pcd_shoulder_1_2 + pcd_shoulder_2_1 + pcd_shoulder_2_2
        
        
        indexes_seleccionados = np.random.choice(range(len(points_berm_1)), 1000 , replace=False)
        points_berm_1 = points_berm_1[indexes_seleccionados]
        pcd_berm_1 = o3d.geometry.PointCloud()
        pcd_berm_1.points = o3d.utility.Vector3dVector(points_berm_1)
        pcd_berm_1.paint_uniform_color([1,0,0])
        
        BERMS[0] = pcd_berm_1 + pcd_berm_2


        # import pdb
        # pdb.set_trace()
        
        
        #--------------------------------------------------------------------------
        # Uncomment to debug visualizations:
        # o3d.visualization.draw(refugee_island+pcd_shoulder_1_2+pcd_shoulder_1_1+lane_2+lane_1+pcd_shoulder_2_1+pcd_shoulder_2_2+pcd_berm_1+pcd_berm_2+pcd_barrier_1_refugee_island+pcd_barrier_2_refugee_island+pcd_barrier_1+pcd_barrier_2)
        #--------------------------------------------------------------------------
        
        
        
        
        
        
        
        # Ahora seleccionamos la parte del DTM más próxima a la trayectoria del
        # MLS y le hacemos un DOWNSAMPING especial, quitamos sólo unos cuantos
        # metros de talud y DTM:
        
        puntos_superficie = np.array(SURFACE[0].points)    
        points_lane_2 = np.array(lane_2.points)
        
        buffer_downsampling_DTM = 20
        distances = np.array(SURFACE[0].compute_point_cloud_distance(lane_2))
        indexes_seleccionados = np.where(distances >= buffer_downsampling_DTM)[0]
        puntos_superficie = puntos_superficie[indexes_seleccionados]
        
        pcd_superficie = o3d.geometry.PointCloud()
        pcd_superficie.points = o3d.utility.Vector3dVector(puntos_superficie)
        pcd_superficie.paint_uniform_color([102/255.,0/255.,51/255.])
        SURFACE[0] = pcd_superficie
        
        # Ídem para el talud contiguo:
            
        puntos_taludes = np.array(SLOPE[0].points)    
        points_lane_2 = np.array(lane_2.points)
        
        buffer_downsampling_DTM = 10
        distances = np.array(SLOPE[0].compute_point_cloud_distance(lane_2))
        indexes_seleccionados = np.where(distances >= buffer_downsampling_DTM)[0]
        puntos_taludes = puntos_taludes[indexes_seleccionados]
        
        pcd_taludes = o3d.geometry.PointCloud()
        pcd_taludes.points = o3d.utility.Vector3dVector(puntos_taludes)
        pcd_taludes.paint_uniform_color([102/255.,255/255.,255/255.])
        SLOPE[0] = pcd_taludes
        #------------------------------------------------------------------------------
        # Uncomment to debug visualizations:
        # visualizamos TODO en la nube excepto árboles y señales:
        # visor.custom_draw_geometry_with_key_callback(refugee_island+pcd_shoulder_1_2+pcd_shoulder_1_1+lane_2+lane_1+pcd_shoulder_2_1+pcd_shoulder_2_2+pcd_berm_1+pcd_berm_2+pcd_barrier_1_refugee_island+pcd_barrier_2_refugee_island+pcd_barrier_1+pcd_barrier_2+SURFACE[0]+SLOPE[0])
        #------------------------------------------------------------------------------
        return SURFACE,ROAD,SLOPE,SHOULDER,SIGNALS,BERMS,REFUGEE_ISLAND,pcd_barrier_1,pcd_barrier_2,pcd_barrier_1_refugee_island,pcd_barrier_2_refugee_island
        
    else:
    
        # Nubes artificiales por defecto
        print('Nubes artificiales por defecto')
        return SURFACE,ROAD,SLOPE,SHOULDER,SIGNALS,BERMS,REFUGEE_ISLAND,pcd_barriers,pcd_barrier_1,pcd_barrier_2,Lista_de_Senhales
        
    '''
        
    
    #------------------------------------------------------------------------------
    # Uncomment to debug visualizations:
    # We visualize everything in the scene except trees and signals:
    # visor.custom_draw_geometry_with_key_callback(refugee_island+pcd_shoulder_1_2+pcd_shoulder_1_1+lane_2+lane_1+pcd_shoulder_2_1+pcd_shoulder_2_2+pcd_berm_1+pcd_berm_2+pcd_barrier_1_refugee_island+pcd_barrier_2_refugee_island+pcd_barrier_1+pcd_barrier_2+SURFACE[0]+SLOPE[0])
    #------------------------------------------------------------------------------


    return SURFACE,ROAD,SLOPE,SHOULDER,SIGNALS,BERMS,REFUGEE_ISLAND,pcd_barrier_1,pcd_barrier_2











def national_vertical_pumping(SURFACE,ROAD,SLOPE,SHOULDER,SIGNALS,BERMS,pcd_barrier_1,pcd_barrier_2):
    """
    Function to generate some sort of ground elevation in the road-related parts of national roads, i.e., the points closer to the axis road will have a different height than the ones that are further

    :param SURFACE: Point cloud of the DTM excluding road-related elements as a o3d.geometry.PointCloud() object
    :param ROAD: Point cloud of the road as a o3d.geometry.PointCloud() object
    :param SLOPE: Point cloud of the slope as a o3d.geometry.PointCloud() object
    :param SHOULDER: Point cloud of the shoulder as a o3d.geometry.PointCloud() object
    :param SIGNALS: Single point cloud cointaining all traffic signals as a o3d.geometry.PointCloud() object
    :param BERMS: Point cloud of the berm as a o3d.geometry.PointCloud() object
    :param REFUGEE_ISLAND: Point cloud of the road as a o3d.geometry.PointCloud() object
    :param pcd_barriers: Single point cloud containing all barriers as a o3d.geometry.PointCloud() object
    :param pcd_barrier_1: Point cloud of the left barrier as a o3d.geometry.PointCloud() object
    :param pcd_barrier_2: Point cloud of the right barrier as a o3d.geometry.PointCloud() object
    :return: All input elements but with the new heights distribution
    """ 
    
    # TODO: Adapt highway and mixed roads methodology to exclusively national roads.
    
    return SURFACE,ROAD,SLOPE,SHOULDER,SIGNALS,BERMS,pcd_barrier_1,pcd_barrier_2





