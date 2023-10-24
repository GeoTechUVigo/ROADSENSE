#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov  9 13:43:25 2021

@author: lino
"""

# INTERSECCIÓN MESH - TRAYECTORIA


import open3d as o3d
import numpy as np
import os
import copy

import sys
sys.path.insert(1, '/home/lino/Documentos/programas_pruebas_varias/modulo_visualizacion') # Ruta a mi visor customizado y módulo de lecturas
import visualizaciones_customizadas_open3d as visor
import lecturas_nubes_de_puntos as lectura


from LECTURAS_segmentacion_cilindrica import lectura_segmentaciones
from CREADOR_SUPERFICIES_ARTIFICIALES import creador_superficies
from CREADOR_ARBOLES_ARTIFICIALES import creador_arboles_artificiales
from CREADOR_SUPERFICIES_ARTIFICIALES_con_CARRETERA import creador_superficies_carretera

ruta_ejecutable = os.path.dirname(os.path.abspath(__file__))



ruta_segmentaciones_cilindricas = '/home/lino/Documentos/TESTEO_ZEBGO_Nubes/NDP_zebRevo_xures_julio_21/prueba/arboles_manuales_troncos_eliminados/automatizacion/PLANO_TRONCOS_1.000000_0.250000_DBSCAN_TRONCOS_0.100000_1_DBSCAN_CILINDRO_0.900000_10_UMBRAL_0.026263'
os.chdir(ruta_segmentaciones_cilindricas)

SEGMENTOS_ORIG = lectura_segmentaciones(ruta_segmentaciones_cilindricas)


'''
#==============================================================================
# METODOLOGÍA CON 2 ÁRBOLES:
# El segmento 10 tiene buena pinta:
arbol = SEGMENTOS_ORIG[10]


# tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(arbol)
# for alpha in np.logspace(np.log10(0.5), np.log10(0.01), num=4):
#     print(f"alpha={alpha:.3f}")
#     mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
#         arbol, alpha, tetra_mesh, pt_map)
#     mesh.compute_vertex_normals()
#     o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

# Creamos el mesh a partir de la nube de puntos:
tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(arbol)

alpha = 0.5

mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
    arbol, alpha, tetra_mesh, pt_map)
mesh.compute_vertex_normals()
# o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)



arbol_2 = copy.deepcopy(arbol)
arbol_2.translate((10,-10,2.5),relative=False)
tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(arbol_2)

alpha = 0.5

mesh_2 = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
    arbol_2, alpha, tetra_mesh, pt_map)
mesh_2.compute_vertex_normals()







puntos_arbol = np.array(arbol.points)

punto_trayectoria = np.array([np.min(puntos_arbol[:,0])-5,
                             np.min(puntos_arbol[:,1])-5,
                             0])


punto_trayectoria_scaner = o3d.geometry.PointCloud()
punto_trayectoria_scaner.points = o3d.utility.Vector3dVector(np.stack((punto_trayectoria,punto_trayectoria)))
punto_trayectoria_scaner.paint_uniform_color([1,0,0])

mesh_total = mesh + mesh_2


o3d.visualization.draw_geometries([mesh]+[mesh_2]+[punto_trayectoria_scaner], mesh_show_back_face=True)

#==============================================================================

'''




# OJO! CAMBIAMOS SEGMENTOS_ORIG POR LOS ÁRBOLES ARTIFICIALES:
LISTA_DE_ARBOLES_ARTIFICIALES = SEGMENTOS_ORIG







mesh_total = None
mesh_arboles = []

for i in range(len(LISTA_DE_ARBOLES_ARTIFICIALES)):
    
    arbol =  SEGMENTOS_ORIG[i]
    
    tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(arbol)
    
    alpha = 0.45
    
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
        arbol, alpha, tetra_mesh, pt_map)
    mesh.compute_vertex_normals()

    mesh_arboles.append(mesh)
    if i == 0:
        mesh_total = mesh
    else:
        mesh_total += mesh

o3d.visualization.draw_geometries([mesh_total], mesh_show_back_face=True)


# Ahora calculamos la trayectoria de un punto donde pasa el escáner a cada pun-
# to de la nube pero poniendo como restricción que no toque al mesh_total en
# ninguna parte.



mesh_transformado = o3d.t.geometry.TriangleMesh.from_legacy(mesh_total())

# Creamos una escena y añadimos el mesh_total:
scene = o3d.t.geometry.RaycastingScene()
cube_id = scene.add_triangles(mesh_transformado)



# We create two rays:
# The first ray starts at (0.5,0.5,10) and has direction (0,0,-1).
# The second ray start at (-1,-1,-1) and has direction (0,0,-1).
rays = o3d.core.Tensor([[0.5, 0.5, 10, 0, 0, -1], [-1, -1, -1, 0, 0, -1]],
                       dtype=o3d.core.Dtype.Float32)

ans = scene.cast_rays(rays)

















