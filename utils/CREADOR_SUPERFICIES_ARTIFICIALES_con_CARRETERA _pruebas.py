#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 13 12:09:43 2021

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





# #==========================LECTURA SEGMENTACIONES==============================

# Primero leemos las segmentaciones hechas a través del algoritmo de voxeliza-
# ción cilíndrica:
    
ruta_segmentaciones_cilindricas = '/home/lino/Documentos/TESTEO_ZEBGO_Nubes/NDP_zebRevo_xures_julio_21/prueba/arboles_manuales_troncos_eliminados/automatizacion/PLANO_TRONCOS_1.000000_0.250000_DBSCAN_TRONCOS_0.100000_1_DBSCAN_CILINDRO_0.900000_10_UMBRAL_0.026263'

SEGMENTOS = lectura_segmentaciones(ruta_segmentaciones_cilindricas)
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



ruta_segmentaciones_cilindricas = '/home/lino/Documentos/TESTEO_ZEBGO_Nubes/NDP_zebRevo_xures_julio_21/prueba/arboles_manuales_troncos_eliminados/automatizacion/PLANO_TRONCOS_1.000000_0.250000_DBSCAN_TRONCOS_0.100000_1_DBSCAN_CILINDRO_0.900000_10_UMBRAL_0.026263'
numero_de_nubes_a_crear = 1
factor_repeticion = 10
Numero_puntos_superficie = 100 
Numero_transformaciones_por_arbol = 3
Numero_arboles_por_nube = 180
montana = False
visualizacion = True


buffer_carretera = 2.5
buffer_arcen = 3.
buffer_terreno = 0.5





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

NUBES_CON_CARRETERA = {}

for s in range(numero_de_nubes_a_crear):

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
    
    
    
#------------------------------------------------------------------------------
#============================CREACIÓN CARRETERAS===============================
    
    punto_1 = np.max(superficie_reconstruida.take(0,1))
    indice_punto_1 = np.where(superficie_reconstruida==punto_1)[0]
    punto_1 = superficie_reconstruida[indice_punto_1][0]
    altura_punto_1 = punto_1[2]
    
    punto_2 = np.min(superficie_reconstruida.take(0,1))
    indice_punto_2 = np.where(superficie_reconstruida==punto_2)[0]
    punto_2 = superficie_reconstruida[indice_punto_2][0]
    altura_punto_2 = punto_2[2]
    
    mitad_x = (punto_1[0] + punto_2[0])/2.
    
    altura = min(altura_punto_1,altura_punto_2)
    
    sector_1 = superficie_reconstruida[np.where(superficie_reconstruida.take(0,1)>mitad_x)[0]]
    sector_2 = superficie_reconstruida[np.where(superficie_reconstruida.take(0,1)<mitad_x)[0]]
    
    maximo_sector_1 = np.max(sector_1.take(2,1))
    indice_punto_1 = np.where(sector_1==maximo_sector_1)[0]
    maximo_sector_1 = sector_1[indice_punto_1][0]
    
    maximo_sector_2 = np.max(sector_2.take(2,1))
    indice_punto_2 = np.where(sector_2==maximo_sector_2)[0]
    maximo_sector_2 = sector_2[indice_punto_2][0]
    
    # Hacemos ahora un fit a estos 4 puntos:
        
    coeficientes_fit = np.polyfit([punto_1[0],maximo_sector_1[0],maximo_sector_2[0],punto_2[0]],
                                  [punto_1[1],maximo_sector_1[1],maximo_sector_2[1],punto_2[1]], deg=3)
    p = np.poly1d(coeficientes_fit)

    
    # coeficientes_fit_lineal = np.polyfit([punto_1[0],punto_2[0]],
    #                                      [punto[1],punto_2[1]],deg=1)
    # q = np.poly1d(coeficientes_fit_lineal)
    
    puntos_curva = []
    
    for i in range(len(superficie_reconstruida.take(0,1))):
        x = superficie_reconstruida.take(0,1)[i]
        y = superficie_reconstruida.take(1,1)[i]
        z = superficie_reconstruida.take(2,1)[i]
        if np.abs(y-p(x)) <= 0.2:
            # print('hey')
            y = p(x)
            # z = altura
            puntos_curva.append(np.array([x,y,altura]))
            superficie_reconstruida[i] = np.array([x,y,z])
            # superficie_reconstruida[i] = np.array([x,y,altura])
        else:
            superficie_reconstruida[i] = np.array([x,y,z])
        
    
    
    
    
    pcd2_1 = o3d.geometry.PointCloud()
    pcd2_1.points = o3d.utility.Vector3dVector(superficie_reconstruida)
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
    
    pcd2_1.colors = o3d.utility.Vector3dVector(colores_suelo)

    
    
    
    
    
    pcd2_2 = o3d.geometry.PointCloud()
    pcd2_2.points = o3d.utility.Vector3dVector(puntos_curva)
    # o3d.io.write_point_cloud("../../test_data/sync.ply", pcd)
    
    # Vamos a pintar el suelo con algunos colores:
    lista_colores_suelo = [np.array([0/255,0/255,255/255]),
                     np.array([0/255,0/255,255/255]),
                     np.array([0/255,0/255,255/255])]
    
    colores_suelo = np.zeros((len(puntos_curva),3))
    for e in range(len(colores_suelo)):
        colores_suelo[e] = lista_colores_suelo[np.random.choice(len(lista_colores_suelo))]
    
    pcd2_2.colors = o3d.utility.Vector3dVector(colores_suelo)
    
    pcd2_3 = o3d.geometry.PointCloud()
    superficie_plano = np.copy(superficie_reconstruida)
    superficie_plano[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(superficie_plano)
    
    pcd2_4 = o3d.geometry.PointCloud()
    curva_plano = np.copy(puntos_curva)
    curva_plano[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(curva_plano)

    
    
    
    
    
    
    
    distancias = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    indices_carretera = np.where(distancias <= buffer_carretera)[0]
    indices_resto = np.where(distancias > buffer_carretera)[0]
    
    
    
    
    
    
    puntos_carretera = np.array(pcd2_1.points)[indices_carretera]
    puntos_carretera[:,[2]] = altura
    
    # Repito varias veces porque me cunde que se ajuste el fit del arcén después
    # puntos_carretera = np.concatenate((puntos_carretera,puntos_carretera))
    # puntos_carretera = np.concatenate((puntos_carretera,puntos_carretera))    
    # puntos_carretera = np.concatenate((puntos_carretera,puntos_carretera))
    
    puntos_resto = np.array(pcd2_1.points)[indices_resto]
    
    pcd_carretera = o3d.geometry.PointCloud()
    pcd_carretera.points = o3d.utility.Vector3dVector(puntos_carretera)
    pcd_carretera.paint_uniform_color([0,0,0])
    
    pcd_superficie = o3d.geometry.PointCloud()
    pcd_superficie.points = o3d.utility.Vector3dVector(puntos_resto)
    
    
    
    o3d.visualization.draw_geometries([pcd_carretera]+[pcd_superficie])
    
    
    
    # Ya tenemos aislada una nube que es la carretera y otra nube que es todo
    # lo demás (la superficie, vaya). Ahora aplicamos el buffer del arcén y un
    # smoothing a los puntos de la superficie que caigan en ese buffer:
    
    pcd2_3 = o3d.geometry.PointCloud()
    superficie_plano = np.array(pcd_superficie.points)
    superficie_plano[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(superficie_plano)
    
    pcd2_4 = o3d.geometry.PointCloud()
    carretera_plano = np.copy(puntos_carretera)
    carretera_plano[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(carretera_plano)
    
    
    
    
    distancias = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    indices_arcen = np.where(distancias <= buffer_arcen)[0]
    indices_resto = np.where(distancias > buffer_arcen)[0]
    
    puntos_arcen = np.array(pcd_superficie.points)[indices_arcen]
    puntos_resto = np.array(pcd_superficie.points)[indices_resto]
    
    pcd_arcen = o3d.geometry.PointCloud()
    pcd_arcen.points = o3d.utility.Vector3dVector(puntos_arcen)
    pcd_arcen.paint_uniform_color([1,0,0])
    
    pcd_superficie = o3d.geometry.PointCloud()
    pcd_superficie.points = o3d.utility.Vector3dVector(puntos_resto)
    
    
    
    # Pillo unos cuantos puntos en la frontera arcén/terreno para repetirlos y
    # así mejorar el fit de después:
        
        
    pcd2_3 = o3d.geometry.PointCloud()
    superficie_plano = np.array(pcd_superficie.points)
    superficie_plano[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(superficie_plano)
    
    pcd2_4 = o3d.geometry.PointCloud()
    arcen_plano = np.copy(puntos_arcen)
    arcen_plano[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(arcen_plano)
    
    
    
    
    distancias = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    indices_arcen = np.where(distancias <= buffer_terreno)[0]
    indices_resto = np.where(distancias > buffer_terreno)[0]
    
    puntos_frontera_arcen = np.array(pcd_superficie.points)[indices_arcen]
    puntos_resto = np.array(pcd_superficie.points)[indices_resto]
    
    
    # Repetimos los puntos seleccionados:
    puntos_arcen = np.concatenate((puntos_arcen,puntos_frontera_arcen,puntos_frontera_arcen,puntos_frontera_arcen))
    
    pcd_arcen = o3d.geometry.PointCloud()
    pcd_arcen.points = o3d.utility.Vector3dVector(puntos_arcen)
    pcd_arcen.paint_uniform_color([1,0,0])
    
    
    # Pillo unos cuantos puntos en la frontera arcén/carretera para repetirlos 
    # y así mejorar el fit de después:
        
        
    pcd2_3 = o3d.geometry.PointCloud()
    arcen_plano = np.array(pcd_arcen.points)
    arcen_plano[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(arcen_plano)
    
    pcd2_4 = o3d.geometry.PointCloud()
    carretera_plano = np.copy(puntos_carretera)
    carretera_plano[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(carretera_plano)
    
    
    
    
    distancias = np.array(pcd2_4.compute_point_cloud_distance(pcd2_3))
    indices_carretera = np.where(distancias <= buffer_terreno*0.1)[0]
    indices_resto = np.where(distancias > buffer_terreno)[0]
    
    puntos_frontera_arcen = np.array(pcd_carretera.points)[indices_carretera]
    puntos_resto = np.array(pcd_superficie.points)[indices_resto]
    
    
    # Repetimos los puntos seleccionados:
    puntos_arcen = np.concatenate((puntos_arcen,puntos_frontera_arcen,puntos_frontera_arcen,puntos_frontera_arcen,puntos_frontera_arcen,puntos_frontera_arcen,puntos_frontera_arcen))
    
    pcd_arcen = o3d.geometry.PointCloud()
    pcd_arcen.points = o3d.utility.Vector3dVector(puntos_arcen)
    pcd_arcen.paint_uniform_color([1,0,0])
    
    
    
    
    
    
    
    
    
    
    
    o3d.visualization.draw_geometries([pcd_carretera]+[pcd_superficie]+[pcd_arcen])
    
    
    # Ahora lo que hago es combinar todos los puntos de arcén+carretera y les
    # hago un fit:
        
    # puntos_1 = np.array(pcd_arcen.points)
    # puntos_2 = np.array(pcd_carretera.points)
    puntos_carretera_arcen = np.array((pcd_carretera + pcd_arcen).points)
    
    
    def matriz_minimos_cuadrados_orden_2(x, y, order=2):
        """ generate Matrix use with lstsq """
        # Genero una matriz con los valores obtenidos por mínimos cuadrados
        ncolumnas = (order + 1)**2
        G = np.zeros((x.size, ncolumnas))
        ij = itertools.product(range(order+1), range(order+1))
        for k, (i, j) in enumerate(ij):
            G[:, k] = x**i * y**j
        return G
    
    x, y, z = puntos_carretera_arcen.T # Hago la traspuesta
    # x, y = x - x[0], y - y[0]  # Para mejorar la eficacia
    
    # Creamos la matriz que contiene las regresiones por punto:
    G = matriz_minimos_cuadrados_orden_2(x, y, order=3)
    # Solve for np.dot(G, m) = z:
    # Quiero saber qué valores de m hacen que G·m = z (es decir, np.dot(G,m)=z)
    m = np.linalg.lstsq(G, z)[0]
    
    
    # Evaluamos en una grid el ajuste que acabamos de hacer...
    nx, ny = int(Numero_puntos_superficie*0.9), int(Numero_puntos_superficie*0.9)
    xx, yy = np.meshgrid(np.linspace(x.min(), x.max(), nx),
                          np.linspace(y.min(), y.max(), ny))
    
    
    GG = matriz_minimos_cuadrados_orden_2(xx.ravel(), yy.ravel(),order=3)
    zz = np.reshape(np.dot(GG, m), xx.shape)
    
    
    
    
    
    # # Ploteamos y cruzamos dedos
    # # fg, ax = plt.subplots(subplot_kw=dict(projection='3d'))
    # # ls = LightSource(270, 45)
    # # rgb = ls.shade(zz, cmap=cm.gist_earth, vert_exag=0.1, blend_mode='soft')
    # # superficie = ax.plot_surface(xx, yy, zz, rstride=1, cstride=1, facecolors=rgb,
    # #                         linewidth=0, antialiased=False, shade=False)
    # # ax.plot3D(x, y, z, "o",color='red')
    
    # # fg.canvas.draw()
    # # plt.show()
    
    
    # Convierto la superficie del fit a una nube de puntos:
    superficie_new = np.zeros((np.size(xx), 3))
    superficie_new[:, 0] = np.reshape(xx, -1)
    superficie_new[:, 1] = np.reshape(yy, -1)
    superficie_new[:, 2] = np.reshape(zz, -1)

    
    
    
    
    # Quitamos algunos puntos del fit (arcén) en la frontera con la carretera:
    
    
    pcd2_3 = o3d.geometry.PointCloud()
    superficie_plano = np.copy(superficie_new)
    superficie_plano[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(superficie_plano)
    
    pcd2_4 = o3d.geometry.PointCloud()
    carretera_plano = np.copy(puntos_carretera)
    carretera_plano[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(carretera_plano)
    
    
    
    
    distancias = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    indices_arcen = np.where(distancias > 0.35)[0]
    indices_resto = np.where(distancias < 0.35)[0]
    
    puntos_arcen = superficie_new[indices_arcen]
    # puntos_resto = np.array(pcd_superficie.points)[indices_resto]
    
    pcd_arcen = o3d.geometry.PointCloud()
    pcd_arcen.points = o3d.utility.Vector3dVector(puntos_arcen)
    pcd_arcen.paint_uniform_color([1,0,0])
    
    # pcd_superficie = o3d.geometry.PointCloud()
    # pcd_superficie.points = o3d.utility.Vector3dVector(puntos_resto)
    
    
    
    o3d.visualization.draw_geometries([pcd_carretera]+[pcd_superficie]+[pcd_arcen])
    
    
    
    
    
    
    # Quitamos algunos puntos del fit (arcén) en la frontera con el terreno:
    
    pcd2_3 = o3d.geometry.PointCloud()
    superficie_plano = np.copy(np.array(pcd_superficie.points))
    superficie_plano[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(superficie_plano)
    
    pcd2_4 = o3d.geometry.PointCloud()
    arcen_plano = np.copy(puntos_arcen)
    arcen_plano[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(arcen_plano)
    
    
    
    
    distancias = np.array(pcd2_4.compute_point_cloud_distance(pcd2_3))
    indices_arcen = np.where(distancias > .5)[0]
    indices_resto = np.where(distancias < .5)[0]
    
    puntos_arcen = puntos_arcen[indices_arcen]
    # puntos_resto = np.array(pcd_superficie.points)[indices_resto]
    
    pcd_arcen = o3d.geometry.PointCloud()
    pcd_arcen.points = o3d.utility.Vector3dVector(puntos_arcen)
    pcd_arcen.paint_uniform_color([210/255,105/255,30/255])
    
    # pcd_superficie = o3d.geometry.PointCloud()
    # pcd_superficie.points = o3d.utility.Vector3dVector(puntos_resto)
    
    
    
    o3d.visualization.draw_geometries([pcd_carretera]+[pcd_superficie]+[pcd_arcen])
    
    
    
    # Tenemos en este punto 3 nubes:
    # · pcd_carretera
    # · pcd_arcen
    # · pcd_superficie
    
    
    # La historia es que en este momento deberíamos conferirle una etiqueta a
    # cada punto de cada nube. Lo dejaremos para el paso final (cuando esté con
    # el GAN).
    
    # Vamos a replicar cada nube de puntos unas cuantas veces a distintas pro-
    # fundidades:
    
    superficie_reconstruida = np.array(pcd_superficie.points)    
    carretera_reconstruida = np.array(pcd_carretera.points)
    arcen_reconstruido = np.array(pcd_arcen.points)
    
    numero_capas = 3
    for i in range(numero_capas):
        aux = np.copy(superficie_reconstruida)
        profundidad = np.zeros(3)
        profundidad[2] = 0.05
        aux = aux + profundidad
        superficie_reconstruida = np.concatenate((superficie_reconstruida,aux))
    
    numero_capas = 3
    for i in range(numero_capas):
        aux = np.copy(carretera_reconstruida)
        profundidad = np.zeros(3)
        profundidad[2] = 0.05
        aux = aux + profundidad
        carretera_reconstruida = np.concatenate((carretera_reconstruida,aux))

    numero_capas = 2
    for i in range(numero_capas):
        aux = np.copy(arcen_reconstruido)
        profundidad = np.zeros(3)
        profundidad[2] = 0.2
        aux = aux + profundidad
        arcen_reconstruido = np.concatenate((arcen_reconstruido,aux))


    
    
    
    # Ahora le añadimos algo de ruido gaussiano a cada punto para hacerlo más real:
    ruido_x = np.random.normal(0,(eje_x[2]-eje_x[1])*0.5,len(superficie_reconstruida))
    ruido_y = np.random.normal(0,(eje_y[2]-eje_y[1])*0.5,len(superficie_reconstruida))
    ruido_z = np.random.normal(0,0.015,len(superficie_reconstruida))
    
    superficie_reconstruida[:, 0] = np.reshape(superficie_reconstruida.take(0,1) + ruido_x, -1)
    superficie_reconstruida[:, 1] = np.reshape(superficie_reconstruida.take(1,1) + ruido_y, -1)
    superficie_reconstruida[:, 2] = np.reshape(superficie_reconstruida.take(2,1) + ruido_z, -1)
    
    #-------------------------
    
    ruido_x = np.random.normal(0,(eje_x[2]-eje_x[1])*0.3,len(carretera_reconstruida))
    ruido_y = np.random.normal(0,(eje_y[2]-eje_y[1])*0.3,len(carretera_reconstruida))
    ruido_z = np.random.normal(0,0.0015,len(carretera_reconstruida))
    
    carretera_reconstruida[:, 0] = np.reshape(carretera_reconstruida.take(0,1) + ruido_x, -1)
    carretera_reconstruida[:, 1] = np.reshape(carretera_reconstruida.take(1,1) + ruido_y, -1)
    carretera_reconstruida[:, 2] = np.reshape(carretera_reconstruida.take(2,1) + ruido_z, -1)

    #-------------------------

    ruido_x = np.random.normal(0,(eje_x[2]-eje_x[1])*0.095,len(arcen_reconstruido))
    ruido_y = np.random.normal(0,(eje_y[2]-eje_y[1])*0.095,len(arcen_reconstruido))
    ruido_z = np.random.normal(0,0.0005,len(arcen_reconstruido))
    
    arcen_reconstruido[:, 0] = np.reshape(arcen_reconstruido.take(0,1) + ruido_x, -1)
    arcen_reconstruido[:, 1] = np.reshape(arcen_reconstruido.take(1,1) + ruido_y, -1)
    arcen_reconstruido[:, 2] = np.reshape(arcen_reconstruido.take(2,1) + ruido_z, -1)
    
    
    
    
    
    pcd2_3 = o3d.geometry.PointCloud()
    superficie_plano = np.copy(superficie_reconstruida)
    superficie_plano[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(superficie_reconstruida)
    
    pcd2_4 = o3d.geometry.PointCloud()
    arcen_plano = np.copy(arcen_reconstruido)
    arcen_plano[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(arcen_plano)
    
    
    
    
    distancias = np.array(pcd2_4.compute_point_cloud_distance(pcd2_3))
    indices_arcen = np.where(distancias > .5)[0]
    indices_resto = np.where(distancias < .5)[0]
    
    puntos_arcen = arcen_reconstruido[indices_arcen]
    # puntos_resto = np.array(pcd_superficie.points)[indices_resto]
    
    pcd_arcen = o3d.geometry.PointCloud()
    pcd_arcen.points = o3d.utility.Vector3dVector(puntos_arcen)
    pcd_arcen.paint_uniform_color([210/255,105/255,30/255])
    
    # pcd_superficie = o3d.geometry.PointCloud()
    # pcd_superficie.points = o3d.utility.Vector3dVector(puntos_resto)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    pcd_superficie = o3d.geometry.PointCloud()
    pcd_superficie.points = o3d.utility.Vector3dVector(superficie_reconstruida)
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
    
    pcd_superficie.colors = o3d.utility.Vector3dVector(colores_suelo)
    
    #-------------------------
    
    pcd_carretera = o3d.geometry.PointCloud()
    pcd_carretera.points = o3d.utility.Vector3dVector(carretera_reconstruida)
    # o3d.io.write_point_cloud("../../test_data/sync.ply", pcd)
    
    pcd_carretera.paint_uniform_color([0,0,0])
    
    #-------------------------
    
    pcd_arcen = o3d.geometry.PointCloud()
    pcd_arcen.points = o3d.utility.Vector3dVector(arcen_reconstruido)
    # o3d.io.write_point_cloud("../../test_data/sync.ply", pcd)
    
    pcd_arcen.paint_uniform_color([210/255,105/255,30/255])
    
    
    
    
    
    
    
    
    
    
    
    if visualizacion:
    
        # PARÓN PARA VISUALIZAR -------------------------------------------------------
        
        visor.custom_draw_geometry_with_key_callback(pcd_superficie+pcd_carretera+pcd_arcen)
        
        #------------------------------------------------------------------------------


    NUBES_CON_CARRETERA[s] = [pcd_superficie,pcd_carretera,pcd_arcen]







