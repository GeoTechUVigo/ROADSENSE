#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 16 15:02:04 2021

@author: lino
"""

import os
import open3d as o3d
import numpy as np
import copy

# Script for reading tree segments stored as point cloud objects from open3d.

def read_segments(path):
    """
    Function to read and store external tree segments

    :param path: Path to the directory where all tree segments are stored in .pcd format
    :return: Dictionary composed by all segments. Each element is an open3d.geometry.PointCloud() object
    """ 
    
    os.chdir(path)
    
    SEGMENTS_aux = []
    
    
    for i in range(len(os.listdir(os.getcwd()))):
        
        if os.listdir(os.getcwd())[i].endswith('.pcd'):
            SEGMENTS_aux.append(o3d.io.read_point_cloud(os.listdir(os.getcwd())[i]))
    
    if len(SEGMENTS_aux) == 0:
        return 0
    else:
        SEGMENTS = copy.deepcopy(SEGMENTS_aux)
        del SEGMENTS_aux
        
       
        #-----#-----#-----#-----#-----#-----#-----#-----#-----#-----#-----#-----#-----#
        # Uncomment for debug visualizations:
        # index = 0
        # o3d.visualization.draw(SEGMENTS[index])
        #-----#-----#-----#-----#-----#-----#-----#-----#-----#-----#-----#-----#-----#
    
    return SEGMENTS






