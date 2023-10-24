#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun  6 09:43:01 2022

@author: lino
"""

import numpy as np
import open3d as o3d
import copy

import sys
sys.path.insert(1, '/home/lino/Documentos/programas_pruebas_varias/modulo_visualizacion') # Ruta a mi visor customizado y módulo de lecturas
import visualizaciones_customizadas_open3d as visor
import lecturas_nubes_de_puntos as lectura



# En este script voy a añadir los bombeos, es decir, modificar la sección
# transversal de la carretera para incluir peraltes y así.



def highway_vertical_pumping(SUPERFICIES,CARRETERAS,TALUDES,ARCENES,SENHALES,BERMAS,MEDIANAS,pcd_barrera,barrera_quitamiedos_1,barrera_quitamiedos_2,Lista_de_Senhales,nubes_modificadas):
    
    
    
    buffer_arcen = 2
    buffer_arcen_orig = buffer_arcen*0.6 # Para el tipo de nubes 'Santarem'
    
    
    buffer_berma = 1
    buffer_carretera = 15
    
    buffer_arceneS = np.linspace(buffer_arcen,0,30)
    buffer_carreteraS = np.linspace(buffer_carretera,0,60)
    buffer_bermaS = np.linspace(buffer_berma,0,30)
    
    porcentajes_alturas_arcenes = np.linspace(0.5,0,len(buffer_arceneS))   
    porcentajes_alturas_carriles = np.linspace(1,0,len(buffer_carreteraS)) 
    porcentajes_alturas_bermas = np.linspace(0,1,len(buffer_bermaS))   

    
    
    # PRIMERO VOY A IDENTIFICAR CADA ELEMENTO POR INDIVIDUAL:
    
    carretera = CARRETERAS[0]
    puntos_carretera = np.array(carretera.points)
    
    # Voy a pillar cada set de carriles comparando sus distancias a las barre-
    # ras quitamiedos:
        
    distancias_quitamiedos_1 = carretera.compute_point_cloud_distance(barrera_quitamiedos_1)
    distancia_media_quitamiedos_1 = np.mean(distancias_quitamiedos_1)
    
    distancias_quitamiedos_2 = carretera.compute_point_cloud_distance(barrera_quitamiedos_2)
    distancia_media_quitamiedos_2 = np.mean(distancias_quitamiedos_2)
    
    indices_carril_1 = np.where(distancia_media_quitamiedos_1 <= distancias_quitamiedos_1)[0]
    indices_carril_2 = np.where(distancia_media_quitamiedos_2 < distancias_quitamiedos_2)[0]
    
    puntos_carril_1 = puntos_carretera[indices_carril_1]
    puntos_carril_2 = puntos_carretera[indices_carril_2]
    
    carril_1 = o3d.geometry.PointCloud()
    carril_1.points = o3d.utility.Vector3dVector(puntos_carril_1)
    carril_1.paint_uniform_color([1,0,0])
    
    carril_2 = o3d.geometry.PointCloud()
    carril_2.points = o3d.utility.Vector3dVector(puntos_carril_2)
    carril_2.paint_uniform_color([0,0,1])
    
    # #--------------------------------------------------------------------------
    # # PARÓN PARA VISUALIZAR:
    # o3d.visualization.draw(carril_1+carril_2)
    # #--------------------------------------------------------------------------
    
    
    
    # AHORA VAMOS A DIFERENCIAL 4 TRAMOS DE ARCÉN:
        
    arcen = ARCENES[0]
    puntos_arcen = np.array(arcen.points)
    
    # Comparo distancias a los quitamiedos:
        
    distancias_quitamiedos_1 = arcen.compute_point_cloud_distance(barrera_quitamiedos_1)
    distancia_media_quitamiedos_1 = np.mean(distancias_quitamiedos_1)
    
    distancias_quitamiedos_2 = arcen.compute_point_cloud_distance(barrera_quitamiedos_2)
    distancia_media_quitamiedos_2 = np.mean(distancias_quitamiedos_2)
    
    indices_arcen_1 = np.where(distancia_media_quitamiedos_1 <= distancias_quitamiedos_1)[0]
    indices_arcen_2 = np.where(distancia_media_quitamiedos_2 < distancias_quitamiedos_2)[0]
    
    puntos_arcen_1 = puntos_arcen[indices_arcen_1]
    puntos_arcen_2 = puntos_arcen[indices_arcen_2]
    
    arcen_1 = o3d.geometry.PointCloud()
    arcen_1.points = o3d.utility.Vector3dVector(puntos_arcen_1)
    arcen_1.paint_uniform_color([0.1,0.1,0.8])
    
    arcen_2 = o3d.geometry.PointCloud()
    arcen_2.points = o3d.utility.Vector3dVector(puntos_arcen_2)
    arcen_2.paint_uniform_color([0,0,1])
    
    # #--------------------------------------------------------------------------
    # # PARÓN PARA VISUALIZAR:
    # o3d.visualization.draw(arcen_1+arcen_2)
    # #--------------------------------------------------------------------------
    
    
    # Si volvemos a hacer lo mismo con cada arcén entonces obtendremos cada
    # "subarcén", es decir, los arcenes de cada lado:
        
    # Comparo distancias a los quitamiedos (1/2):
        
    distancias_quitamiedos_1 = arcen_1.compute_point_cloud_distance(barrera_quitamiedos_1)
    distancia_media_quitamiedos_1 = np.mean(distancias_quitamiedos_1)
    
    distancias_quitamiedos_2 = arcen_1.compute_point_cloud_distance(barrera_quitamiedos_2)
    distancia_media_quitamiedos_2 = np.mean(distancias_quitamiedos_2)
    
    indices_arcen_1_1 = np.where(distancia_media_quitamiedos_1 <= distancias_quitamiedos_1)[0]
    indices_arcen_1_2 = np.where(distancia_media_quitamiedos_2 < distancias_quitamiedos_2)[0]
    
    puntos_arcen_1_1 = puntos_arcen_1[indices_arcen_1_1]
    puntos_arcen_1_2 = puntos_arcen_1[indices_arcen_1_2]
    
    arcen_1_1 = o3d.geometry.PointCloud()
    arcen_1_1.points = o3d.utility.Vector3dVector(puntos_arcen_1_1)
    arcen_1_1.paint_uniform_color([0.7,0.3,0])
    
    arcen_1_2 = o3d.geometry.PointCloud()
    arcen_1_2.points = o3d.utility.Vector3dVector(puntos_arcen_1_2)
    arcen_1_2.paint_uniform_color([0,0.3,0.7])
    
    
    
    # Comparo distancias a los quitamiedos (2/2):
        
    distancias_quitamiedos_1 = arcen_2.compute_point_cloud_distance(barrera_quitamiedos_1)
    distancia_media_quitamiedos_1 = np.mean(distancias_quitamiedos_1)
    
    distancias_quitamiedos_2 = arcen_2.compute_point_cloud_distance(barrera_quitamiedos_2)
    distancia_media_quitamiedos_2 = np.mean(distancias_quitamiedos_2)
    
    indices_arcen_2_1 = np.where(distancia_media_quitamiedos_1 <= distancias_quitamiedos_1)[0]
    indices_arcen_2_2 = np.where(distancia_media_quitamiedos_2 < distancias_quitamiedos_2)[0]
    
    puntos_arcen_2_1 = puntos_arcen_2[indices_arcen_2_1]
    puntos_arcen_2_2 = puntos_arcen_2[indices_arcen_2_2]
    
    arcen_2_1 = o3d.geometry.PointCloud()
    arcen_2_1.points = o3d.utility.Vector3dVector(puntos_arcen_2_1)
    arcen_2_1.paint_uniform_color([1,1,0])
    
    arcen_2_2 = o3d.geometry.PointCloud()
    arcen_2_2.points = o3d.utility.Vector3dVector(puntos_arcen_2_2)
    arcen_2_2.paint_uniform_color([0,1,1])
    
    
    
    # #--------------------------------------------------------------------------
    # # PARÓN PARA VISUALIZAR:
    # o3d.visualization.draw(arcen_1_1+arcen_1_2+arcen_2_1+arcen_2_2)
    # #--------------------------------------------------------------------------
    
    
    bermas = BERMAS[0]
    puntos_bermas = np.array(bermas.points)
    
    # Voy a pillar cada set de bermas comparando sus distancias a las barreras
    # quitamiedos:
        
    distancias_quitamiedos_1 = bermas.compute_point_cloud_distance(barrera_quitamiedos_1)
    distancia_media_quitamiedos_1 = np.mean(distancias_quitamiedos_1)
    
    distancias_quitamiedos_2 = bermas.compute_point_cloud_distance(barrera_quitamiedos_2)
    distancia_media_quitamiedos_2 = np.mean(distancias_quitamiedos_2)
    
    indices_berma_1 = np.where(distancia_media_quitamiedos_1 <= distancias_quitamiedos_1)[0]
    indices_berma_2 = np.where(distancia_media_quitamiedos_2 < distancias_quitamiedos_2)[0]
    
    puntos_berma_1 = puntos_bermas[indices_berma_1]
    puntos_berma_2 = puntos_bermas[indices_berma_2]
    
    berma_1 = o3d.geometry.PointCloud()
    berma_1.points = o3d.utility.Vector3dVector(puntos_berma_1)
    berma_1.paint_uniform_color([1,0,1])
    
    berma_2 = o3d.geometry.PointCloud()
    berma_2.points = o3d.utility.Vector3dVector(puntos_berma_2)
    berma_2.paint_uniform_color([0.2,0.2,0.2])
    
    # #--------------------------------------------------------------------------
    # # PARÓN PARA VISUALIZAR:
    # o3d.visualization.draw(berma_1+berma_2)
    # #--------------------------------------------------------------------------
    
    
    # Cojo una de las bermas (me da igual porque ambas tienen la misma altura aquí):
    z_berma_2 = puntos_berma_2.take(2,1)
    altura_inicial_bermas = np.mean(z_berma_2.mean())
    
    
    
    
    # YA TENEMOS CADA ELEMENTO IDENTIFICADO:
        
    # · berma_1
    # · arcen_1_1
    # · carril_1
    # · arcen_1_2
    # · mediana
    # · arcen_2_1
    # · carril_2
    # · arcen_2_2
    # · berma_2
    
    
    
    # LO QUE VOY A HACER AHORA ES SUBIR LA ALTURA DE LA MEDIANA Y A PARTIR DE 
    # AHÍ MODIFICAR LA ALTURA DE LOS DEMÁS ELEMENTOS.
    
    
    mediana = MEDIANAS[0]
    puntos_mediana = np.array(mediana.points)
    altura_mediana = np.min(puntos_mediana.take(2,1))
    
    ALTURA_INICIAL = np.min(puntos_mediana.take(2,1))
    
    nueva_altura = altura_mediana + 0.2

    ALTURA_FINAL = nueva_altura
    
    mediana.translate(([mediana.get_center()[0],
                        mediana.get_center()[1],
                        nueva_altura]),relative=False)
    
    mediana.paint_uniform_color([0,1,0])
    
    # angulo = np.arctan(nueva_altura/(buffer_arcen+buffer_berma+buffer_carretera))
    
    # matriz_rotacion = arcen_1_2.get_rotation_matrix_from_xyz((0,angulo, 0))
    # arcen_1_2.rotate(matriz_rotacion, center=arcen_1_2.get_center())
    
    # arcen_1_2.translate(([arcen_1_2.get_center()[0],
    #                     arcen_1_2.get_center()[1],
    #                     arcen_1_2.get_center()[2]+nueva_altura]),relative=False)
    
    # #--------------------------------------------------------------------------
    # # PARÓN PARA VISUALIZAR:
    # o3d.visualization.draw(mediana+arcen_1_2+carril_2+carril_1+arcen_2_1+arcen_2_2)
    # #--------------------------------------------------------------------------
    
    








    print('Modificando sección transversal del arcén 1.2')

    
    # import pdb
    # pdb.set_trace()


    #--------------------------------------------------------------------------
    # MUCHA INCLINACIÓN:
    # porcentajes_alturas = np.linspace(1,0,len(buffer_arceneS))   
    porcentajes_alturas = porcentajes_alturas_arcenes  
    #--------------------------------------------------------------------------


    pcd_arcen_1_2 = o3d.geometry.PointCloud()
    puntos_arcen_1_2 = np.array(arcen_1_2.points)
    pcd_arcen_1_2.points = o3d.utility.Vector3dVector(puntos_arcen_1_2)
    pcd_arcen_1_2.paint_uniform_color([0.11,0.22,0.33])
    
    pcd_auxiliar = o3d.geometry.PointCloud()
    
    
    pcd2_3 = o3d.geometry.PointCloud()
    arcen_1_2_plano = np.array(pcd_arcen_1_2.points)
    arcen_1_2_plano[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(arcen_1_2_plano)
    
    pcd2_4 = o3d.geometry.PointCloud()
    mediana_plano = np.array(mediana.points)
    mediana_plano[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(mediana_plano)
    
    
    
    
    distancias = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    maxima_altura_carril_1 = 0
    maxima_altura_carril_1_COGIDA = False
    
    
    for p in range(len(buffer_arceneS)-1):  
        buffer_arcen = buffer_arceneS[p]
        siguiente_buffer_arcen = buffer_arceneS[p+1]
    
    
    
        indices_arcen_1_2_en_buffer_a = np.where(distancias >= siguiente_buffer_arcen)[0]
        indices_arcen_1_2_en_buffer_b = np.where(distancias <= buffer_arcen)[0]
        
        indices_comunes = np.intersect1d(indices_arcen_1_2_en_buffer_a,
                                         indices_arcen_1_2_en_buffer_b)
        
        puntos_arcen_1_2_BUFFER = np.array(pcd_arcen_1_2.points)[indices_comunes]

        if len(puntos_arcen_1_2_BUFFER) == 0:
            continue
        else:
        
            # Bajamos y subimos en altura los puntos del arcén para que se 
            # ajuste un poco
            # indices_bajar = np.where(puntos_arcen_1_2_BUFFER[:,2] >= nueva_altura)[0]
            # indices_subir = np.where(puntos_arcen_1_2_BUFFER[:,2] < nueva_altura)[0]
            
            # puntos_arcen_1_2_BUFFER[indices_bajar,2] = puntos_arcen_1_2_BUFFER[indices_bajar,2] - porcentajes_alturas[p]*np.abs(puntos_arcen_1_2_BUFFER[indices_bajar,2]-nueva_altura)
            # puntos_arcen_1_2_BUFFER[indices_subir,2] = puntos_arcen_1_2_BUFFER[indices_subir,2] + porcentajes_alturas[p]*np.abs(puntos_arcen_1_2_BUFFER[indices_subir,2]-nueva_altura)
            
            
            # if p == len(buffer_arceneS)-10:
            #     import pdb
            #     pdb.set_trace()
            
            array_auxiliar_z = puntos_arcen_1_2_BUFFER.take(2,1)
            # array_auxiliar_z = array_auxiliar_z - porcentajes_alturas[p]*np.abs(array_auxiliar_z-nueva_altura)
            array_auxiliar_z = (nueva_altura) - porcentajes_alturas[p]*(nueva_altura-array_auxiliar_z)
            
            
            array_auxiliar_x = puntos_arcen_1_2_BUFFER.take(0,1)
            array_auxiliar_y = puntos_arcen_1_2_BUFFER.take(1,1)
            
            puntos_arcen_1_2 = np.stack((array_auxiliar_x,
                                          array_auxiliar_y,
                                          array_auxiliar_z),axis=-1)
            
         
            # puntos_arcen_1_2[indices_arcen_1_2_en_buffer] = puntos_arcen_1_2_BUFFER
            
            pcd_auxiliar_2 = o3d.geometry.PointCloud()
            pcd_auxiliar_2.points = o3d.utility.Vector3dVector(puntos_arcen_1_2)
            pcd_auxiliar_2.paint_uniform_color([0.4,0.2,1])
            pcd_auxiliar += pcd_auxiliar_2
            
            # pcd_arcen_1_2.points = o3d.utility.Vector3dVector(puntos_arcen_1_2)
            # pcd_arcen_1_2.paint_uniform_color([0.4,0.2,1])
            
            
            
            
            if np.min(array_auxiliar_z) > maxima_altura_carril_1 == 0 and maxima_altura_carril_1_COGIDA == False:
                # Defino la altura de la que se empezará a bajar el siguiente
                # elemento (carretera en este caso):
                maxima_altura_carril_1 = np.min(array_auxiliar_z)
            
        
            
            
    
    puntos_arcen_1_2 = np.array(pcd_auxiliar.points)
    pcd_arcen_1_2 = o3d.geometry.PointCloud()
    pcd_arcen_1_2.points = o3d.utility.Vector3dVector(puntos_arcen_1_2)
    pcd_arcen_1_2.paint_uniform_color([0.4,0.2,1])
    
    
    
    #--------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # o3d.visualization.draw(mediana+pcd_arcen_1_2+arcen_1_1+carril_2+carril_1+arcen_2_1+arcen_2_2)
    #--------------------------------------------------------------------------
    
    
    print('Modificando sección transversal del carril 1')
    
    # SEGUIMOS CON LA CARRETERA:
        
        
        
    # nueva_altura = maxima_altura_siguiente_elemento
      
    #--------------------------------------------------------------------------
    # MUCHA INCLINACIÓN:
    # porcentajes_alturas = np.linspace(7.5,0,len(buffer_carreteraS))   
    
    porcentajes_alturas = porcentajes_alturas_carriles  
    #--------------------------------------------------------------------------
    
    pcd_carril_1 = o3d.geometry.PointCloud()
    puntos_carril_1 = np.array(carril_1.points)
    pcd_carril_1.points = o3d.utility.Vector3dVector(puntos_carril_1)
    pcd_carril_1.paint_uniform_color([0.,0.,0.])
    
    pcd_auxiliar = o3d.geometry.PointCloud()
    
    
    
    pcd2_3 = o3d.geometry.PointCloud()
    carril_1_plano = np.array(pcd_carril_1.points)
    carril_1_plano[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(carril_1_plano)
    
    pcd2_4 = o3d.geometry.PointCloud()
    arcen_1_2_plano = np.array(pcd_arcen_1_2.points)
    arcen_1_2_plano[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(arcen_1_2_plano)
    
    
    
    
    distancias = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    maxima_altura_arcen_1_1 = 0
    maxima_altura_arcen_1_1_COGIDA = False
    
    for p in range(len(buffer_carreteraS)-1):  
        buffer_carretera = buffer_carreteraS[p]
        siguiente_buffer_carretera = buffer_carreteraS[p+1]
    
    
    
        indices_carril_1_en_buffer_a = np.where(distancias >= siguiente_buffer_carretera)[0]
        indices_carril_1_en_buffer_b = np.where(distancias <= buffer_carretera)[0]
        
        indices_comunes = np.intersect1d(indices_carril_1_en_buffer_a,
                                         indices_carril_1_en_buffer_b)
        
        puntos_carril_1_BUFFER = np.array(pcd_carril_1.points)[indices_comunes]

        if len(puntos_carril_1_BUFFER) == 0:
            continue
        else:
            
            array_auxiliar_z = puntos_carril_1_BUFFER.take(2,1)
            # array_auxiliar_z = array_auxiliar_z - porcentajes_alturas[p]*np.abs(array_auxiliar_z-nueva_altura)
            array_auxiliar_z = (nueva_altura) - porcentajes_alturas[p]*(nueva_altura-array_auxiliar_z)
            
            array_auxiliar_x = puntos_carril_1_BUFFER.take(0,1)
            array_auxiliar_y = puntos_carril_1_BUFFER.take(1,1)
            
            puntos_carril_1 = np.stack((array_auxiliar_x,
                                          array_auxiliar_y,
                                          array_auxiliar_z),axis=-1)
            
            
            pcd_auxiliar_2 = o3d.geometry.PointCloud()
            pcd_auxiliar_2.points = o3d.utility.Vector3dVector(puntos_carril_1)
            pcd_auxiliar_2.paint_uniform_color([0.,0.,0.])
            pcd_auxiliar += pcd_auxiliar_2
            
            
            
            if np.min(array_auxiliar_z)>maxima_altura_arcen_1_1 and maxima_altura_arcen_1_1_COGIDA == False:
                # Defino la altura de la que se empezará a bajar el siguiente
                # elemento (arcén en este caso):
                maxima_altura_arcen_1_1 = np.min(array_auxiliar_z)
                maxima_altura_arcen_1_1_COGIDA = True
            if p == len(buffer_carreteraS)-2:
                maxima_altura_elemento_actual = np.mean(array_auxiliar_z)
            
        # if p ==10:
        #     puntos_carril_1 = np.array(pcd_auxiliar.points)
        #     carril_1 = o3d.geometry.PointCloud()
        #     carril_1.points = o3d.utility.Vector3dVector(puntos_carril_1)
        #     carril_1.paint_uniform_color([0.,0.,0.])
        #     import pdb
        #     pdb.set_trace()
    
    puntos_carril_1 = np.array(pcd_auxiliar.points)
    puntos_carril_1_z = puntos_carril_1.take(2,1)
    puntos_carril_1_z = puntos_carril_1_z - np.abs(maxima_altura_elemento_actual-maxima_altura_carril_1)
    puntos_carril_1_x = puntos_carril_1.take(0,1)
    puntos_carril_1_y = puntos_carril_1.take(1,1)
    puntos_carril_1 = np.stack((puntos_carril_1_x,
                                puntos_carril_1_y,
                                puntos_carril_1_z),axis=-1)
    carril_1 = o3d.geometry.PointCloud()
    carril_1.points = o3d.utility.Vector3dVector(puntos_carril_1)
    carril_1.paint_uniform_color([0.,0.,0.])
    
    altura_minima_carretera = puntos_carril_1_z.min()
    
    #--------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # o3d.visualization.draw(mediana+pcd_arcen_1_2+arcen_1_1+carril_2+carril_1+arcen_2_1+arcen_2_2)
    #--------------------------------------------------------------------------
    
        
        
    print('Modificando sección transversal del arcén 1.1')
    
    # SEGUIMOS CON EL SEGUNDO ARCÉN:
        
    pcd_arcen_1_1 = o3d.geometry.PointCloud()
    puntos_arcen_1_1 = np.array(arcen_1_1.points)
    pcd_arcen_1_1.points = o3d.utility.Vector3dVector(puntos_arcen_1_1)
    pcd_arcen_1_1.paint_uniform_color([0.3,0.3,0.3])
    
    pcd_auxiliar = o3d.geometry.PointCloud()
    
    
    pcd2_3 = o3d.geometry.PointCloud()
    arcen_1_1_plano = np.array(pcd_arcen_1_1.points)
    arcen_1_1_plano[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(arcen_1_1_plano)
    
    pcd2_4 = o3d.geometry.PointCloud()
    carril_plano = np.array(carril_1.points)
    carril_plano[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(carril_plano)
    
    
    
    
    distancias = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    minima_altura_berma_1 = 0
    minima_altura_berma_1_COGIDA = False
    
    
    #--------------------------------------------------------------------------
    # MUCHA INCLINACIÓN:
    porcentajes_alturas = np.linspace(0,1.5,len(buffer_arceneS))   
    
    # porcentajes_alturas = np.flip(porcentajes_alturas_arcenes)
    #--------------------------------------------------------------------------
    
    for p in range(len(buffer_arceneS)-1):  
        buffer_arcen = buffer_arceneS[p]
        siguiente_buffer_arcen = buffer_arceneS[p+1]
    
    
    
        indices_arcen_1_1_en_buffer_a = np.where(distancias >= siguiente_buffer_arcen)[0]
        indices_arcen_1_1_en_buffer_b = np.where(distancias <= buffer_arcen)[0]
        
        indices_comunes = np.intersect1d(indices_arcen_1_1_en_buffer_a,
                                         indices_arcen_1_1_en_buffer_b)
        
        puntos_arcen_1_1_BUFFER = np.array(pcd_arcen_1_1.points)[indices_comunes]

        if len(puntos_arcen_1_1_BUFFER) == 0:
            continue
        else:
                    
            array_auxiliar_z = puntos_arcen_1_1_BUFFER.take(2,1)
            # array_auxiliar_z = array_auxiliar_z - porcentajes_alturas[p]*np.abs(array_auxiliar_z-nueva_altura)
            array_auxiliar_z = (altura_minima_carretera) - porcentajes_alturas[p]*(altura_minima_carretera-array_auxiliar_z)
            
            
            array_auxiliar_x = puntos_arcen_1_1_BUFFER.take(0,1)
            array_auxiliar_y = puntos_arcen_1_1_BUFFER.take(1,1)
            
            puntos_arcen_1_1 = np.stack((array_auxiliar_x,
                                          array_auxiliar_y,
                                          array_auxiliar_z),axis=-1)
            
            
            pcd_auxiliar_2 = o3d.geometry.PointCloud()
            pcd_auxiliar_2.points = o3d.utility.Vector3dVector(puntos_arcen_1_1)
            pcd_auxiliar_2.paint_uniform_color([0.4,0.2,1])
            pcd_auxiliar += pcd_auxiliar_2
            
            
            
            if np.min(array_auxiliar_z) > minima_altura_berma_1 == 0 and minima_altura_berma_1_COGIDA == False:
                # Defino la altura de la que se empezará a bajar el siguiente
                # elemento (carretera en este caso):
                minima_altura_berma_1 = np.min(array_auxiliar_z)
            
        
            
    puntos_arcen_1_1 = np.array(pcd_auxiliar.points)
    puntos_arcen_1_1_z = puntos_arcen_1_1.take(2,1)
    # puntos_arcen_1_1_z = puntos_arcen_1_1_z - np.abs(maxima_altura_elemento_actual-maxima_altura_arcen_1_1)
    puntos_arcen_1_1_x = puntos_arcen_1_1.take(0,1)
    puntos_arcen_1_1_y = puntos_arcen_1_1.take(1,1)
    puntos_arcen_1_1 = np.stack((puntos_arcen_1_1_x,
                                puntos_arcen_1_1_y,
                                puntos_arcen_1_1_z),axis=-1)
    
    
    pcd_arcen_1_1 = o3d.geometry.PointCloud()
    pcd_arcen_1_1.points = o3d.utility.Vector3dVector(puntos_arcen_1_1)
    pcd_arcen_1_1.paint_uniform_color([0.3,0.3,0.3])
        
        
    # Ahora cojo y bajo el arcén un poco:
    
    maxima_altura_arcen_1_1 = np.max(puntos_arcen_1_1_z)
    distancia_bajada_arcen_1_1 = np.abs(altura_minima_carretera-maxima_altura_arcen_1_1)
    distancia_bajada_arcen_1_1 = distancia_bajada_arcen_1_1 - 0.2*distancia_bajada_arcen_1_1
    nueva_ubicacion = np.array([pcd_arcen_1_1.get_center()[0],
                                pcd_arcen_1_1.get_center()[1],
                                pcd_arcen_1_1.get_center()[2]-distancia_bajada_arcen_1_1])
    pcd_arcen_1_1.translate(nueva_ubicacion,relative=False)
    
    # print(distancia_bajada_arcen_1_1)        
    
    # puntos_arcen_1_1 = np.array(pcd_auxiliar.points)
    # pcd_arcen_1_1 = o3d.geometry.PointCloud()
    # pcd_arcen_1_1.points = o3d.utility.Vector3dVector(puntos_arcen_1_1)
    # pcd_arcen_1_1.paint_uniform_color([0.3,0.3,0.3])
    
    '''
        
    # nueva_altura = maxima_altura_siguiente_elemento
        
    #--------------------------------------------------------------------------
    # MUCHA INCLINACIÓN:
    # porcentajes_alturas = np.linspace(1,0,len(buffer_arceneS))   
    porcentajes_alturas = porcentajes_alturas_arcenes  
    #--------------------------------------------------------------------------
    
    pcd_arcen_1_1 = o3d.geometry.PointCloud()
    
    # nueva_ubicacion = np.array([arcen_1_1.get_center()[0],
    #                             arcen_1_1.get_center()[1],
    #                             altura_minima_carretera])
    # arcen_1_1.translate(nueva_ubicacion,relative=False)
    
    puntos_arcen_1_1 = np.array(arcen_1_1.points)
    pcd_arcen_1_1.points = o3d.utility.Vector3dVector(puntos_arcen_1_1)
    pcd_arcen_1_1.paint_uniform_color([0.1,0.2,0.3])
    
    pcd_auxiliar = o3d.geometry.PointCloud()
    
    
    pcd2_3 = o3d.geometry.PointCloud()
    arcen_1_1_plano = np.array(pcd_arcen_1_1.points)
    arcen_1_1_plano[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(arcen_1_1_plano)
    
    pcd2_4 = o3d.geometry.PointCloud()
    carril_1_plano = np.array(carril_1.points)
    carril_1_plano[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(carril_1_plano)
    
    
    #--------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # o3d.visualization.draw(mediana+pcd_arcen_1_2+arcen_1_1+carril_2+carril_1+arcen_2_1+arcen_2_2)
    #--------------------------------------------------------------------------
    
    distancias = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    
    minima_altura_berma_1 = 0
    minima_altura_berma_1_COGIDA = False
    
    
    for p in range(len(buffer_arceneS)-1):  
        buffer_arcen = buffer_arceneS[p]
        siguiente_buffer_arcen = buffer_arceneS[p+1]
    
    
    
        indices_arcen_1_1_en_buffer_a = np.where(distancias >= siguiente_buffer_arcen)[0]
        indices_arcen_1_1_en_buffer_b = np.where(distancias <= buffer_arcen)[0]
        
        indices_comunes = np.intersect1d(indices_arcen_1_1_en_buffer_a,
                                         indices_arcen_1_1_en_buffer_b)
        
        puntos_arcen_1_1_BUFFER = np.array(pcd_arcen_1_1.points)[indices_comunes]

        if len(puntos_arcen_1_1_BUFFER) == 0:
            continue
        else:
            
            # print('heyyyyyyyy')
            
            array_auxiliar_z = puntos_arcen_1_1_BUFFER.take(2,1)
            # array_auxiliar_z = array_auxiliar_z - porcentajes_alturas[p]*np.abs(array_auxiliar_z-nueva_altura)
            array_auxiliar_z = (altura_minima_carretera) - porcentajes_alturas[p]*(altura_minima_carretera-array_auxiliar_z)
            
            
            array_auxiliar_x = puntos_arcen_1_1_BUFFER.take(0,1)
            array_auxiliar_y = puntos_arcen_1_1_BUFFER.take(1,1)
            
            puntos_arcen_1_1 = np.stack((array_auxiliar_x,
                                          array_auxiliar_y,
                                          array_auxiliar_z),axis=-1)
            
            
            pcd_auxiliar_2 = o3d.geometry.PointCloud()
            pcd_auxiliar_2.points = o3d.utility.Vector3dVector(puntos_arcen_1_1)
            pcd_auxiliar_2.paint_uniform_color([0.1,0.2,0.3])
            pcd_auxiliar += pcd_auxiliar_2
            
            
            
            if np.mean(array_auxiliar_z)>minima_altura_berma_1 and minima_altura_berma_1_COGIDA == False:
                # Defino la altura de la que se empezará a bajar el siguiente
                # elemento (berma en este caso):
                minima_altura_berma_1 = np.mean(array_auxiliar_z)
                minima_altura_berma_1_COGIDA = True
            if p == len(buffer_arceneS)-2:
                maxima_altura_elemento_actual = np.mean(array_auxiliar_z)
            
        
#------------------------------------------------------------------------------
            # # DEBUG PUNTUAL:
            # if p == int(len(buffer_arceneS)/4):
                
            #     puntos_arcen_1_1 = np.array(pcd_auxiliar.points)
            #     puntos_arcen_1_1_z = puntos_arcen_1_1.take(2,1)
            #     puntos_arcen_1_1_z = puntos_arcen_1_1_z - np.abs(maxima_altura_elemento_actual-maxima_altura_arcen_1_1)
            #     puntos_arcen_1_1_x = puntos_arcen_1_1.take(0,1)
            #     puntos_arcen_1_1_y = puntos_arcen_1_1.take(1,1)
            #     puntos_arcen_1_1 = np.stack((puntos_arcen_1_1_x,
            #                                 puntos_arcen_1_1_y,
            #                                 puntos_arcen_1_1_z),axis=-1)
                
                
            #     pcd_arcen_1_1 = o3d.geometry.PointCloud()
            #     pcd_arcen_1_1.points = o3d.utility.Vector3dVector(puntos_arcen_1_1)
            #     pcd_arcen_1_1.paint_uniform_color([0.3,0.3,0.3])
                
            #     #--------------------------------------------------------------------------
            #     # PARÓN PARA VISUALIZAR:
            #     o3d.visualization.draw(mediana+pcd_arcen_1_2+pcd_arcen_1_1+carril_2+carril_1+arcen_2_1+arcen_2_2)
            #     #--------------------------------------------------------------------------
#------------------------------------------------------------------------------

    puntos_arcen_1_1 = np.array(pcd_auxiliar.points)
    puntos_arcen_1_1_z = puntos_arcen_1_1.take(2,1)
    puntos_arcen_1_1_z = puntos_arcen_1_1_z - np.abs(maxima_altura_elemento_actual-maxima_altura_arcen_1_1)
    puntos_arcen_1_1_x = puntos_arcen_1_1.take(0,1)
    puntos_arcen_1_1_y = puntos_arcen_1_1.take(1,1)
    puntos_arcen_1_1 = np.stack((puntos_arcen_1_1_x,
                                puntos_arcen_1_1_y,
                                puntos_arcen_1_1_z),axis=-1)
    
    
    pcd_arcen_1_1 = o3d.geometry.PointCloud()
    pcd_arcen_1_1.points = o3d.utility.Vector3dVector(puntos_arcen_1_1)
    pcd_arcen_1_1.paint_uniform_color([0.3,0.3,0.3])
    
    
    '''
    #--------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # o3d.visualization.draw(mediana+pcd_arcen_1_2+pcd_arcen_1_1+carril_2+carril_1+arcen_2_1+arcen_2_2)
    #--------------------------------------------------------------------------
        
        
    # import pdb
    # pdb.set_trace()
        
        
    print('Modificando sección transversal de la berma 1')
    
    # SEGUIMOS CON LA BERMA:
        
        
        
    # nueva_altura = maxima_altura_siguiente_elemento
        
    porcentajes_alturas = porcentajes_alturas_bermas
    
    pcd_berma_1 = o3d.geometry.PointCloud()
    puntos_berma_1 = np.array(berma_1.points)
    pcd_berma_1.points = o3d.utility.Vector3dVector(puntos_berma_1)
    pcd_berma_1.paint_uniform_color([0.1,0.2,0.3])
    
    pcd_auxiliar = o3d.geometry.PointCloud()
    
    
    pcd2_3 = o3d.geometry.PointCloud()
    berma_1_plano = np.array(pcd_berma_1.points)
    berma_1_plano[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(berma_1_plano)
    
    pcd2_4 = o3d.geometry.PointCloud()
    arcen_1_1_plano = np.array(pcd_arcen_1_1.points)
    arcen_1_1_plano[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(arcen_1_1_plano)
    
    
    
    
    distancias = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    
    minima_altura_berma_1 = 0
    minima_altura_berma_1_COGIDA = False
    
    
    for p in range(len(buffer_arceneS)-1):  
        buffer_arcen = buffer_arceneS[p]
        siguiente_buffer_arcen = buffer_arceneS[p+1]
    
    
    
        indices_berma_1_en_buffer_a = np.where(distancias >= siguiente_buffer_arcen)[0]
        indices_berma_1_en_buffer_b = np.where(distancias <= buffer_arcen)[0]
        
        indices_comunes = np.intersect1d(indices_berma_1_en_buffer_a,
                                         indices_berma_1_en_buffer_b)
        
        puntos_berma_1_BUFFER = np.array(pcd_berma_1.points)[indices_comunes]

        if len(puntos_berma_1_BUFFER) == 0:
            continue
        else:
            
            # print('heyyyyyyyy')
            
            array_auxiliar_z = puntos_berma_1_BUFFER.take(2,1)
            # array_auxiliar_z = array_auxiliar_z - porcentajes_alturas[p]*np.abs(array_auxiliar_z-nueva_altura)
            array_auxiliar_z = (nueva_altura) - porcentajes_alturas[p]*(nueva_altura-array_auxiliar_z)
            
            
            array_auxiliar_x = puntos_berma_1_BUFFER.take(0,1)
            array_auxiliar_y = puntos_berma_1_BUFFER.take(1,1)
            
            puntos_berma_1 = np.stack((array_auxiliar_x,
                                          array_auxiliar_y,
                                          array_auxiliar_z),axis=-1)
            
            
            pcd_auxiliar_2 = o3d.geometry.PointCloud()
            pcd_auxiliar_2.points = o3d.utility.Vector3dVector(puntos_berma_1)
            pcd_auxiliar_2.paint_uniform_color([0.1,0.2,0.3])
            pcd_auxiliar += pcd_auxiliar_2
            
            
            
            if np.mean(array_auxiliar_z)>minima_altura_berma_1 and minima_altura_berma_1_COGIDA == False:
                # Defino la altura de la que se empezará a bajar el siguiente
                # elemento (berma en este caso):
                minima_altura_berma_1 = np.mean(array_auxiliar_z)
                # ALTURA_FINAL = minima_altura_berma_1
                minima_altura_berma_1_COGIDA = True
            if p == len(buffer_arceneS)-2:
                minima_altura_elemento_actual = np.mean(array_auxiliar_z)
            
        
    
    
    puntos_berma_1 = np.array(pcd_auxiliar.points)
    
    
    pcd_berma_1 = o3d.geometry.PointCloud()
    pcd_berma_1.points = o3d.utility.Vector3dVector(puntos_berma_1)
    pcd_berma_1.paint_uniform_color([1,0,0])
    
    puntos_berma_1_z = puntos_berma_1.take(2,1)
    
    minima_altura_arcen = np.min(puntos_arcen_1_1_z)
    minima_altura_berma_auxiliar = np.min(puntos_berma_1_z)
    
    # distancia_vertical_berma_arcen = np.abs(minima_altura_arcen-minima_altura_berma_auxiliar)
    # distancia_vertical_berma_arcen = distancia_vertical_berma_arcen + 
    
    # print(distancia_vertical_berma_arcen)
    
    nueva_ubicacion = np.array([pcd_berma_1.get_center()[0],
                                pcd_berma_1.get_center()[1],
                                pcd_berma_1.get_center()[2]-(distancia_bajada_arcen_1_1*(1+0.5))])
    pcd_berma_1.translate(nueva_ubicacion,relative=False)
    
    # puntos_berma_1_z = puntos_berma_1_z - distancia_vertical_berma_arcen
    
    # puntos_berma_1_x = puntos_berma_1.take(0,1)
    # puntos_berma_1_y = puntos_berma_1.take(1,1)
    # puntos_berma_1 = np.stack((puntos_berma_1_x,
    #                             puntos_berma_1_y,
    #                             puntos_berma_1_z),axis=-1)

    
    #--------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # o3d.visualization.draw(mediana+pcd_arcen_1_2+pcd_arcen_1_1+carril_2+carril_1+arcen_2_1+arcen_2_2+pcd_berma_1)
    #--------------------------------------------------------------------------
        
        
    # import pdb
    # pdb.set_trace()
        
        
    # Esta cantidad es lo que tiene que bajar en altura la SUPERFICIE, los TA-
    # LUDES, los ÁRBOLES y las BARRERAS QUITAMIEDOS:
    # desplazamiento_entero = np.abs(ALTURA_FINAL-ALTURA_INICIAL)
        
    
    # Hemos terminado con una mitad del tramo de carretera, vamos con la otra
    # mitad:
        
        
        
        
        
    print('Modificando sección transversal del arcén 2.1')

    
    # import pdb
    # pdb.set_trace()



    #--------------------------------------------------------------------------
    # MUCHA INCLINACIÓN:
    # porcentajes_alturas = np.linspace(1,0,len(buffer_arceneS))   
    porcentajes_alturas = porcentajes_alturas_arcenes  
    #--------------------------------------------------------------------------
    
    pcd_arcen_2_1 = o3d.geometry.PointCloud()
    puntos_arcen_2_1 = np.array(arcen_2_1.points)
    pcd_arcen_2_1.points = o3d.utility.Vector3dVector(puntos_arcen_2_1)
    pcd_arcen_2_1.paint_uniform_color([0.11,0.22,0.33])
    
    pcd_auxiliar = o3d.geometry.PointCloud()
    
    
    pcd2_3 = o3d.geometry.PointCloud()
    arcen_2_1_plano = np.array(pcd_arcen_2_1.points)
    arcen_2_1_plano[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(arcen_2_1_plano)
    
    pcd2_4 = o3d.geometry.PointCloud()
    mediana_plano = np.array(mediana.points)
    mediana_plano[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(mediana_plano)
    
    
    
    
    distancias = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    maxima_altura_carril_2 = 0
    maxima_altura_carril_2_COGIDA = False
    
    
    for p in range(len(buffer_arceneS)-1):  
        buffer_arcen = buffer_arceneS[p]
        siguiente_buffer_arcen = buffer_arceneS[p+1]
    
    
    
        indices_arcen_2_1_en_buffer_a = np.where(distancias >= siguiente_buffer_arcen)[0]
        indices_arcen_2_1_en_buffer_b = np.where(distancias <= buffer_arcen)[0]
        
        indices_comunes = np.intersect1d(indices_arcen_2_1_en_buffer_a,
                                         indices_arcen_2_1_en_buffer_b)
        
        puntos_arcen_2_1_BUFFER = np.array(pcd_arcen_2_1.points)[indices_comunes]

        if len(puntos_arcen_2_1_BUFFER) == 0:
            continue
        else:
        
            # Bajamos y subimos en altura los puntos del arcén para que se 
            # ajuste un poco
            # indices_bajar = np.where(puntos_arcen_2_1_BUFFER[:,2] >= nueva_altura)[0]
            # indices_subir = np.where(puntos_arcen_2_1_BUFFER[:,2] < nueva_altura)[0]
            
            # puntos_arcen_2_1_BUFFER[indices_bajar,2] = puntos_arcen_2_1_BUFFER[indices_bajar,2] - porcentajes_alturas[p]*np.abs(puntos_arcen_2_1_BUFFER[indices_bajar,2]-nueva_altura)
            # puntos_arcen_2_1_BUFFER[indices_subir,2] = puntos_arcen_2_1_BUFFER[indices_subir,2] + porcentajes_alturas[p]*np.abs(puntos_arcen_2_1_BUFFER[indices_subir,2]-nueva_altura)
            
            
            # if p == len(buffer_arceneS)-10:
            #     import pdb
            #     pdb.set_trace()
            
            array_auxiliar_z = puntos_arcen_2_1_BUFFER.take(2,1)
            # array_auxiliar_z = array_auxiliar_z - porcentajes_alturas[p]*np.abs(array_auxiliar_z-nueva_altura)
            array_auxiliar_z = (nueva_altura) - porcentajes_alturas[p]*(nueva_altura-array_auxiliar_z)
            
            
            array_auxiliar_x = puntos_arcen_2_1_BUFFER.take(0,1)
            array_auxiliar_y = puntos_arcen_2_1_BUFFER.take(1,1)
            
            puntos_arcen_2_1 = np.stack((array_auxiliar_x,
                                          array_auxiliar_y,
                                          array_auxiliar_z),axis=-1)
            
         
            # puntos_arcen_2_1[indices_arcen_2_1_en_buffer] = puntos_arcen_2_1_BUFFER
            
            pcd_auxiliar_2 = o3d.geometry.PointCloud()
            pcd_auxiliar_2.points = o3d.utility.Vector3dVector(puntos_arcen_2_1)
            pcd_auxiliar_2.paint_uniform_color([0.4,0.2,1])
            pcd_auxiliar += pcd_auxiliar_2
            
            # pcd_arcen_2_1.points = o3d.utility.Vector3dVector(puntos_arcen_2_1)
            # pcd_arcen_2_1.paint_uniform_color([0.4,0.2,1])
            
            
            
            
            if np.min(array_auxiliar_z) > maxima_altura_carril_2 == 0 and maxima_altura_carril_2_COGIDA == False:
                # Defino la altura de la que se empezará a bajar el siguiente
                # elemento (carretera en este caso):
                maxima_altura_carril_2 = np.min(array_auxiliar_z)
            
        
            
            
    
    puntos_arcen_2_1 = np.array(pcd_auxiliar.points)
    pcd_arcen_2_1 = o3d.geometry.PointCloud()
    pcd_arcen_2_1.points = o3d.utility.Vector3dVector(puntos_arcen_2_1)
    pcd_arcen_2_1.paint_uniform_color([1,1,0])

    
    
    #--------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # o3d.visualization.draw(mediana+pcd_arcen_1_2+pcd_arcen_1_1+carril_2+pcd_berma_1+carril_1+pcd_arcen_2_1+arcen_2_2)
    #--------------------------------------------------------------------------
        
      
        
      
        
      # ojo
        
      
    print('Modificando sección transversal del carril 2')
    
    # SEGUIMOS CON LA CARRETERA:
        
        
        
    # nueva_altura = maxima_altura_siguiente_elemento
        
    #--------------------------------------------------------------------------
    # MUCHA INCLINACIÓN:
    # porcentajes_alturas = np.linspace(7.5,0,len(buffer_carreteraS))   
    
    porcentajes_alturas = porcentajes_alturas_carriles   
    #--------------------------------------------------------------------------
    
    pcd_carril_2 = o3d.geometry.PointCloud()
    puntos_carril_2 = np.array(carril_2.points)
    pcd_carril_2.points = o3d.utility.Vector3dVector(puntos_carril_2)
    pcd_carril_2.paint_uniform_color([0.,0.,0.])
    
    pcd_auxiliar = o3d.geometry.PointCloud()
    
    
    
    pcd2_3 = o3d.geometry.PointCloud()
    carril_2_plano = np.array(pcd_carril_2.points)
    carril_2_plano[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(carril_2_plano)
    
    pcd2_4 = o3d.geometry.PointCloud()
    arcen_2_1_plano = np.array(pcd_arcen_2_1.points)
    arcen_2_1_plano[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(arcen_2_1_plano)
    
    
    
    
    distancias = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    maxima_altura_arcen_2_2 = 0
    maxima_altura_arcen_2_2_COGIDA = False
    
    for p in range(len(buffer_carreteraS)-1):  
        buffer_carretera = buffer_carreteraS[p]
        siguiente_buffer_carretera = buffer_carreteraS[p+1]
    
    
    
        indices_carril_2_en_buffer_a = np.where(distancias >= siguiente_buffer_carretera)[0]
        indices_carril_2_en_buffer_b = np.where(distancias <= buffer_carretera)[0]
        
        indices_comunes = np.intersect1d(indices_carril_2_en_buffer_a,
                                         indices_carril_2_en_buffer_b)
        
        puntos_carril_2_BUFFER = np.array(pcd_carril_2.points)[indices_comunes]

        if len(puntos_carril_2_BUFFER) == 0:
            continue
        else:
            
            array_auxiliar_z = puntos_carril_2_BUFFER.take(2,1)
            # array_auxiliar_z = array_auxiliar_z - porcentajes_alturas[p]*np.abs(array_auxiliar_z-nueva_altura)
            array_auxiliar_z = (nueva_altura) - porcentajes_alturas[p]*(nueva_altura-array_auxiliar_z)
            
            array_auxiliar_x = puntos_carril_2_BUFFER.take(0,1)
            array_auxiliar_y = puntos_carril_2_BUFFER.take(1,1)
            
            puntos_carril_2 = np.stack((array_auxiliar_x,
                                          array_auxiliar_y,
                                          array_auxiliar_z),axis=-1)
            
            
            pcd_auxiliar_2 = o3d.geometry.PointCloud()
            pcd_auxiliar_2.points = o3d.utility.Vector3dVector(puntos_carril_2)
            pcd_auxiliar_2.paint_uniform_color([0.,0.,0.])
            pcd_auxiliar += pcd_auxiliar_2
            
            
            
            if np.min(array_auxiliar_z)>maxima_altura_arcen_2_2 and maxima_altura_arcen_2_2_COGIDA == False:
                # Defino la altura de la que se empezará a bajar el siguiente
                # elemento (arcén en este caso):
                maxima_altura_arcen_2_2 = np.min(array_auxiliar_z)
                maxima_altura_arcen_2_2_COGIDA = True
            if p == len(buffer_carreteraS)-2:
                maxima_altura_elemento_actual = np.mean(array_auxiliar_z)
            
        # if p ==10:
        #     puntos_carril_2 = np.array(pcd_auxiliar.points)
        #     carril_2 = o3d.geometry.PointCloud()
        #     carril_2.points = o3d.utility.Vector3dVector(puntos_carril_2)
        #     carril_2.paint_uniform_color([0.,0.,0.])
        #     import pdb
        #     pdb.set_trace()
    
    puntos_carril_2 = np.array(pcd_auxiliar.points)
    puntos_carril_2_z = puntos_carril_2.take(2,1)
    puntos_carril_2_z = puntos_carril_2_z - np.abs(maxima_altura_elemento_actual-maxima_altura_carril_2)
    puntos_carril_2_x = puntos_carril_2.take(0,1)
    puntos_carril_2_y = puntos_carril_2.take(1,1)
    puntos_carril_2 = np.stack((puntos_carril_2_x,
                                puntos_carril_2_y,
                                puntos_carril_2_z),axis=-1)
    carril_2 = o3d.geometry.PointCloud()
    carril_2.points = o3d.utility.Vector3dVector(puntos_carril_2)
    carril_2.paint_uniform_color([0.,0.,0.])
    
    altura_minima_carretera = puntos_carril_2_z.min()
    
    #--------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # o3d.visualization.draw(mediana+pcd_arcen_1_2+arcen_1_1+carril_2+carril_1+arcen_2_1+arcen_2_2)
    #--------------------------------------------------------------------------
    
        
        
    print('Modificando sección transversal del arcén 2.2')
    
    # SEGUIMOS CON EL SEGUNDO ARCÉN:
                
    pcd_arcen_2_2 = o3d.geometry.PointCloud()
    puntos_arcen_2_2 = np.array(arcen_2_2.points)
    pcd_arcen_2_2.points = o3d.utility.Vector3dVector(puntos_arcen_2_2)
    pcd_arcen_2_2.paint_uniform_color([0.3,0.3,0.3])
    
    pcd_auxiliar = o3d.geometry.PointCloud()
    
    
    pcd2_3 = o3d.geometry.PointCloud()
    arcen_2_2_plano = np.array(pcd_arcen_2_2.points)
    arcen_2_2_plano[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(arcen_2_2_plano)
    
    pcd2_4 = o3d.geometry.PointCloud()
    carril_plano = np.array(carril_2.points)
    carril_plano[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(carril_plano)
    
    
    
    
    distancias = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    minima_altura_berma_1 = 0
    minima_altura_berma_1_COGIDA = False
    
    
    #--------------------------------------------------------------------------
    # MUCHA INCLINACIÓN:
    porcentajes_alturas = np.linspace(0,1.5,len(buffer_arceneS))   
    
    # porcentajes_alturas = np.flip(porcentajes_alturas_arcenes)
    #--------------------------------------------------------------------------
    
    for p in range(len(buffer_arceneS)-1):  
        buffer_arcen = buffer_arceneS[p]
        siguiente_buffer_arcen = buffer_arceneS[p+1]
    
    
    
        indices_arcen_2_2_en_buffer_a = np.where(distancias >= siguiente_buffer_arcen)[0]
        indices_arcen_2_2_en_buffer_b = np.where(distancias <= buffer_arcen)[0]
        
        indices_comunes = np.intersect1d(indices_arcen_2_2_en_buffer_a,
                                         indices_arcen_2_2_en_buffer_b)
        
        puntos_arcen_2_2_BUFFER = np.array(pcd_arcen_2_2.points)[indices_comunes]

        if len(puntos_arcen_2_2_BUFFER) == 0:
            continue
        else:
                    
            array_auxiliar_z = puntos_arcen_2_2_BUFFER.take(2,1)
            # array_auxiliar_z = array_auxiliar_z - porcentajes_alturas[p]*np.abs(array_auxiliar_z-nueva_altura)
            array_auxiliar_z = (altura_minima_carretera) - porcentajes_alturas[p]*(altura_minima_carretera-array_auxiliar_z)
            
            
            array_auxiliar_x = puntos_arcen_2_2_BUFFER.take(0,1)
            array_auxiliar_y = puntos_arcen_2_2_BUFFER.take(1,1)
            
            puntos_arcen_2_2 = np.stack((array_auxiliar_x,
                                          array_auxiliar_y,
                                          array_auxiliar_z),axis=-1)
            
            
            pcd_auxiliar_2 = o3d.geometry.PointCloud()
            pcd_auxiliar_2.points = o3d.utility.Vector3dVector(puntos_arcen_2_2)
            pcd_auxiliar_2.paint_uniform_color([0.4,0.2,1])
            pcd_auxiliar += pcd_auxiliar_2
            
            
            
            if np.min(array_auxiliar_z) > minima_altura_berma_1 == 0 and minima_altura_berma_1_COGIDA == False:
                # Defino la altura de la que se empezará a bajar el siguiente
                # elemento (carretera en este caso):
                minima_altura_berma_1 = np.min(array_auxiliar_z)
            
        
            
    puntos_arcen_2_2 = np.array(pcd_auxiliar.points)
    puntos_arcen_2_2_z = puntos_arcen_2_2.take(2,1)
    # puntos_arcen_2_2_z = puntos_arcen_2_2_z - np.abs(maxima_altura_elemento_actual-maxima_altura_arcen_2_2)
    puntos_arcen_2_2_x = puntos_arcen_2_2.take(0,1)
    puntos_arcen_2_2_y = puntos_arcen_2_2.take(1,1)
    puntos_arcen_2_2 = np.stack((puntos_arcen_2_2_x,
                                puntos_arcen_2_2_y,
                                puntos_arcen_2_2_z),axis=-1)
    
    
    pcd_arcen_2_2 = o3d.geometry.PointCloud()
    pcd_arcen_2_2.points = o3d.utility.Vector3dVector(puntos_arcen_2_2)
    pcd_arcen_2_2.paint_uniform_color([0.3,0.3,0.3])
        
        
    # Ahora cojo y bajo el arcén un poco:
    # print(puntos_arcen_2_2_z)
    
    # import pdb
    # pdb.set_trace()
    
    maxima_altura_arcen_2_2 = np.max(puntos_arcen_2_2_z)
    distancia_bajada_arcen_2_2 = np.abs(altura_minima_carretera-maxima_altura_arcen_2_2)
    distancia_bajada_arcen_2_2 = distancia_bajada_arcen_2_2 - 0.2*distancia_bajada_arcen_2_2
    nueva_ubicacion = np.array([pcd_arcen_2_2.get_center()[0],
                                pcd_arcen_2_2.get_center()[1],
                                pcd_arcen_2_2.get_center()[2]-distancia_bajada_arcen_2_2])
    pcd_arcen_2_2.translate(nueva_ubicacion,relative=False)
    
    # print(distancia_bajada_arcen_2_2)        
    
    # puntos_arcen_2_2 = np.array(pcd_auxiliar.points)
    # pcd_arcen_2_2 = o3d.geometry.PointCloud()
    # pcd_arcen_2_2.points = o3d.utility.Vector3dVector(puntos_arcen_2_2)
    # pcd_arcen_2_2.paint_uniform_color([0.3,0.3,0.3])
    
    '''
        
    # nueva_altura = maxima_altura_siguiente_elemento
        
    #--------------------------------------------------------------------------
    # MUCHA INCLINACIÓN:
    # porcentajes_alturas = np.linspace(1,0,len(buffer_arceneS))   
    porcentajes_alturas = porcentajes_alturas_arcenes  
    #--------------------------------------------------------------------------
    
    pcd_arcen_2_1 = o3d.geometry.PointCloud()
    
    # nueva_ubicacion = np.array([arcen_2_1.get_center()[0],
    #                             arcen_2_1.get_center()[1],
    #                             altura_minima_carretera])
    # arcen_2_1.translate(nueva_ubicacion,relative=False)
    
    puntos_arcen_2_1 = np.array(arcen_2_1.points)
    pcd_arcen_2_1.points = o3d.utility.Vector3dVector(puntos_arcen_2_1)
    pcd_arcen_2_1.paint_uniform_color([0.1,0.2,0.3])
    
    pcd_auxiliar = o3d.geometry.PointCloud()
    
    
    pcd2_3 = o3d.geometry.PointCloud()
    arcen_2_1_plano = np.array(pcd_arcen_2_1.points)
    arcen_2_1_plano[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(arcen_2_1_plano)
    
    pcd2_4 = o3d.geometry.PointCloud()
    carril_1_plano = np.array(carril_1.points)
    carril_1_plano[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(carril_1_plano)
    
    
    #--------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # o3d.visualization.draw(mediana+pcd_arcen_2_2+arcen_2_1+carril_2+carril_1+arcen_2_1+arcen_2_2)
    #--------------------------------------------------------------------------
    
    distancias = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    
    minima_altura_berma_1 = 0
    minima_altura_berma_1_COGIDA = False
    
    
    for p in range(len(buffer_arceneS)-1):  
        buffer_arcen = buffer_arceneS[p]
        siguiente_buffer_arcen = buffer_arceneS[p+1]
    
    
    
        indices_arcen_2_1_en_buffer_a = np.where(distancias >= siguiente_buffer_arcen)[0]
        indices_arcen_2_1_en_buffer_b = np.where(distancias <= buffer_arcen)[0]
        
        indices_comunes = np.intersect1d(indices_arcen_2_1_en_buffer_a,
                                         indices_arcen_2_1_en_buffer_b)
        
        puntos_arcen_2_1_BUFFER = np.array(pcd_arcen_2_1.points)[indices_comunes]

        if len(puntos_arcen_2_1_BUFFER) == 0:
            continue
        else:
            
            # print('heyyyyyyyy')
            
            array_auxiliar_z = puntos_arcen_2_1_BUFFER.take(2,1)
            # array_auxiliar_z = array_auxiliar_z - porcentajes_alturas[p]*np.abs(array_auxiliar_z-nueva_altura)
            array_auxiliar_z = (altura_minima_carretera) - porcentajes_alturas[p]*(altura_minima_carretera-array_auxiliar_z)
            
            
            array_auxiliar_x = puntos_arcen_2_1_BUFFER.take(0,1)
            array_auxiliar_y = puntos_arcen_2_1_BUFFER.take(1,1)
            
            puntos_arcen_2_1 = np.stack((array_auxiliar_x,
                                          array_auxiliar_y,
                                          array_auxiliar_z),axis=-1)
            
            
            pcd_auxiliar_2 = o3d.geometry.PointCloud()
            pcd_auxiliar_2.points = o3d.utility.Vector3dVector(puntos_arcen_2_1)
            pcd_auxiliar_2.paint_uniform_color([0.1,0.2,0.3])
            pcd_auxiliar += pcd_auxiliar_2
            
            
            
            if np.mean(array_auxiliar_z)>minima_altura_berma_1 and minima_altura_berma_1_COGIDA == False:
                # Defino la altura de la que se empezará a bajar el siguiente
                # elemento (berma en este caso):
                minima_altura_berma_1 = np.mean(array_auxiliar_z)
                minima_altura_berma_1_COGIDA = True
            if p == len(buffer_arceneS)-2:
                maxima_altura_elemento_actual = np.mean(array_auxiliar_z)
            
        
#------------------------------------------------------------------------------
            # # DEBUG PUNTUAL:
            # if p == int(len(buffer_arceneS)/4):
                
            #     puntos_arcen_2_1 = np.array(pcd_auxiliar.points)
            #     puntos_arcen_2_1_z = puntos_arcen_2_1.take(2,1)
            #     puntos_arcen_2_1_z = puntos_arcen_2_1_z - np.abs(maxima_altura_elemento_actual-maxima_altura_arcen_2_1)
            #     puntos_arcen_2_1_x = puntos_arcen_2_1.take(0,1)
            #     puntos_arcen_2_1_y = puntos_arcen_2_1.take(1,1)
            #     puntos_arcen_2_1 = np.stack((puntos_arcen_2_1_x,
            #                                 puntos_arcen_2_1_y,
            #                                 puntos_arcen_2_1_z),axis=-1)
                
                
            #     pcd_arcen_2_1 = o3d.geometry.PointCloud()
            #     pcd_arcen_2_1.points = o3d.utility.Vector3dVector(puntos_arcen_2_1)
            #     pcd_arcen_2_1.paint_uniform_color([0.3,0.3,0.3])
                
            #     #--------------------------------------------------------------------------
            #     # PARÓN PARA VISUALIZAR:
            #     o3d.visualization.draw(mediana+pcd_arcen_2_2+pcd_arcen_2_1+carril_2+carril_1+arcen_2_1+arcen_2_2)
            #     #--------------------------------------------------------------------------
#------------------------------------------------------------------------------

    puntos_arcen_2_1 = np.array(pcd_auxiliar.points)
    puntos_arcen_2_1_z = puntos_arcen_2_1.take(2,1)
    puntos_arcen_2_1_z = puntos_arcen_2_1_z - np.abs(maxima_altura_elemento_actual-maxima_altura_arcen_2_1)
    puntos_arcen_2_1_x = puntos_arcen_2_1.take(0,1)
    puntos_arcen_2_1_y = puntos_arcen_2_1.take(1,1)
    puntos_arcen_2_1 = np.stack((puntos_arcen_2_1_x,
                                puntos_arcen_2_1_y,
                                puntos_arcen_2_1_z),axis=-1)
    
    
    pcd_arcen_2_1 = o3d.geometry.PointCloud()
    pcd_arcen_2_1.points = o3d.utility.Vector3dVector(puntos_arcen_2_1)
    pcd_arcen_2_1.paint_uniform_color([0.3,0.3,0.3])
    
    
    '''
    #--------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # o3d.visualization.draw(mediana+pcd_arcen_2_2+pcd_arcen_2_1+carril_2+carril_1+arcen_2_1+arcen_2_2)
    #--------------------------------------------------------------------------
        
        
    # import pdb
    # pdb.set_trace()
        
        
    print('Modificando sección transversal de la berma 1')
    
    # SEGUIMOS CON LA BERMA:
        
        
        
    # nueva_altura = maxima_altura_siguiente_elemento
        
    porcentajes_alturas = porcentajes_alturas_bermas
    
    pcd_berma_2 = o3d.geometry.PointCloud()
    puntos_berma_2 = np.array(berma_2.points)
    pcd_berma_2.points = o3d.utility.Vector3dVector(puntos_berma_2)
    pcd_berma_2.paint_uniform_color([0.1,0.2,0.3])
    
    pcd_auxiliar = o3d.geometry.PointCloud()
    
    
    pcd2_3 = o3d.geometry.PointCloud()
    berma_2_plano = np.array(pcd_berma_2.points)
    berma_2_plano[:,2] = 0
    pcd2_3.points = o3d.utility.Vector3dVector(berma_2_plano)
    
    pcd2_4 = o3d.geometry.PointCloud()
    arcen_2_2_plano = np.array(pcd_arcen_2_2.points)
    arcen_2_2_plano[:,2] = 0
    pcd2_4.points = o3d.utility.Vector3dVector(arcen_2_2_plano)
    
    
    
    
    distancias = np.array(pcd2_3.compute_point_cloud_distance(pcd2_4))
    
    minima_altura_berma_2 = 0
    minima_altura_berma_2_COGIDA = False
    
    
    for p in range(len(buffer_arceneS)-1):  
        buffer_arcen = buffer_arceneS[p]
        siguiente_buffer_arcen = buffer_arceneS[p+1]
    
    
    
        indices_berma_2_en_buffer_a = np.where(distancias >= siguiente_buffer_arcen)[0]
        indices_berma_2_en_buffer_b = np.where(distancias <= buffer_arcen)[0]
        
        indices_comunes = np.intersect1d(indices_berma_2_en_buffer_a,
                                         indices_berma_2_en_buffer_b)
        
        puntos_berma_2_BUFFER = np.array(pcd_berma_2.points)[indices_comunes]

        if len(puntos_berma_2_BUFFER) == 0:
            continue
        else:
            
            # print('heyyyyyyyy')
            
            array_auxiliar_z = puntos_berma_2_BUFFER.take(2,1)
            # array_auxiliar_z = array_auxiliar_z - porcentajes_alturas[p]*np.abs(array_auxiliar_z-nueva_altura)
            array_auxiliar_z = (nueva_altura) - porcentajes_alturas[p]*(nueva_altura-array_auxiliar_z)
            
            
            array_auxiliar_x = puntos_berma_2_BUFFER.take(0,1)
            array_auxiliar_y = puntos_berma_2_BUFFER.take(1,1)
            
            puntos_berma_2 = np.stack((array_auxiliar_x,
                                          array_auxiliar_y,
                                          array_auxiliar_z),axis=-1)
            
            
            pcd_auxiliar_2 = o3d.geometry.PointCloud()
            pcd_auxiliar_2.points = o3d.utility.Vector3dVector(puntos_berma_2)
            pcd_auxiliar_2.paint_uniform_color([0.1,0.2,0.3])
            pcd_auxiliar += pcd_auxiliar_2
            
            
            
            if np.mean(array_auxiliar_z)>minima_altura_berma_2 and minima_altura_berma_2_COGIDA == False:
                # Defino la altura de la que se empezará a bajar el siguiente
                # elemento (berma en este caso):
                minima_altura_berma_2 = np.mean(array_auxiliar_z)
                # ALTURA_FINAL = minima_altura_berma_2
                minima_altura_berma_2_COGIDA = True
            if p == len(buffer_arceneS)-2:
                minima_altura_elemento_actual = np.mean(array_auxiliar_z)
            
        
    
    
    puntos_berma_2 = np.array(pcd_auxiliar.points)
    
    
    pcd_berma_2 = o3d.geometry.PointCloud()
    pcd_berma_2.points = o3d.utility.Vector3dVector(puntos_berma_2)
    pcd_berma_2.paint_uniform_color([1,0,0])
    
    puntos_berma_2_z = puntos_berma_2.take(2,1)
    
    minima_altura_arcen = np.min(puntos_arcen_2_2_z)
    minima_altura_berma_auxiliar = np.min(puntos_berma_2_z)
    
    # distancia_vertical_berma_arcen = np.abs(minima_altura_arcen-minima_altura_berma_auxiliar)
    # distancia_vertical_berma_arcen = distancia_vertical_berma_arcen + 
    
    # print(distancia_vertical_berma_arcen)
    
    nueva_ubicacion = np.array([pcd_berma_2.get_center()[0],
                                pcd_berma_2.get_center()[1],
                                pcd_berma_2.get_center()[2]-(distancia_bajada_arcen_1_1*(1+0.5))])
    pcd_berma_2.translate(nueva_ubicacion,relative=False)
    
    # puntos_berma_2_z = puntos_berma_2_z - distancia_vertical_berma_arcen
    
    # puntos_berma_2_x = puntos_berma_2.take(0,1)
    # puntos_berma_2_y = puntos_berma_2.take(1,1)
    # puntos_berma_2 = np.stack((puntos_berma_2_x,
    #                             puntos_berma_2_y,
    #                             puntos_berma_2_z),axis=-1)

    
    #--------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # o3d.visualization.draw(mediana+pcd_arcen_1_2+pcd_arcen_1_1+carril_2+carril_1+arcen_2_1+arcen_2_2+pcd_berma_2)
    #--------------------------------------------------------------------------
        
        
    # import pdb
    # pdb.set_trace()
        
        
    # Esta cantidad es lo que tiene que bajar en altura la SUPERFICIE, los TA-
    # LUDES, los ÁRBOLES y las BARRERAS QUITAMIEDOS:
    # desplazamiento_entero = np.abs(ALTURA_FINAL-ALTURA_INICIAL)
    
    
    
    #--------------------------------------------------------------------------
    # PARÓN PARA VISUALIZAR:
    # o3d.visualization.draw(mediana+pcd_arcen_1_2+pcd_arcen_1_1+carril_2+carril_1+pcd_arcen_2_1+pcd_arcen_2_2+pcd_berma_1+pcd_berma_2)
    #--------------------------------------------------------------------------
      
    
    CARRETERAS[0] = carril_1 + carril_2
    ARCENES[0] = pcd_arcen_1_1 + pcd_arcen_1_2 + pcd_arcen_2_1 + pcd_arcen_2_2
    BERMAS[0] = pcd_berma_1 + pcd_berma_2
    
    
    # Cojo una de las bermas (me da igual porque ambas tienen la misma altura aquí):
    z_berma_2 = puntos_berma_2.take(2,1)
    altura_final_bermas = np.mean(z_berma_2.mean())
      
        
    # Esta es la distancia vertical que habrá que restarle a las señales late-
    # rales y a las barreras quitamiedos:
    distancia_vertical_bermas = np.abs(altura_final_bermas-altura_inicial_bermas)
      
    # Esta es la distancia vertical que habrá que sumarle a la señal central y
    # la barrera tipo jersey:
    distancia_vertical_mediana = np.abs(ALTURA_FINAL-ALTURA_INICIAL)    
     
    
    # Ahora vamos a mover en alturas TODO para que cuadre:
        
    # Primer quitamiedos:
    nueva_ubicacion = np.array([barrera_quitamiedos_1.get_center()[0],
                                barrera_quitamiedos_1.get_center()[1],
                                barrera_quitamiedos_1.get_center()[2]-distancia_vertical_bermas])
    barrera_quitamiedos_1.translate(nueva_ubicacion,relative=False)
      
    # Segundo quitamiedos:
    nueva_ubicacion = np.array([barrera_quitamiedos_2.get_center()[0],
                                barrera_quitamiedos_2.get_center()[1],
                                barrera_quitamiedos_2.get_center()[2]-distancia_vertical_bermas])
    barrera_quitamiedos_2.translate(nueva_ubicacion,relative=False)
        
    # Barrera jersey central:
    nueva_ubicacion = np.array([pcd_barrera.get_center()[0],
                                pcd_barrera.get_center()[1],
                                pcd_barrera.get_center()[2]+distancia_vertical_mediana])
    pcd_barrera.translate(nueva_ubicacion,relative=False)
        
    # Señal grande central:
    nueva_ubicacion = np.array([SENHALES[0].get_center()[0],
                                SENHALES[0].get_center()[1],
                                SENHALES[0].get_center()[2]+distancia_vertical_mediana])
    SENHALES[0].translate(nueva_ubicacion,relative=False)
    
    # Señales laterales:
    for i in range(1,len(SENHALES)):
        nueva_ubicacion = np.array([SENHALES[i].get_center()[0],
                                    SENHALES[i].get_center()[1],
                                    SENHALES[i].get_center()[2]-distancia_vertical_bermas])
        SENHALES[i].translate(nueva_ubicacion,relative=False)
        
    # DTM:
    nueva_ubicacion = np.array([SUPERFICIES[0].get_center()[0],
                                SUPERFICIES[0].get_center()[1],
                                SUPERFICIES[0].get_center()[2]-distancia_vertical_bermas])
    SUPERFICIES[0].translate(nueva_ubicacion,relative=False)
        # se hace algo
    # Taludes:
    nueva_ubicacion = np.array([TALUDES[0].get_center()[0],
                                TALUDES[0].get_center()[1],
                                TALUDES[0].get_center()[2]-distancia_vertical_bermas])
    TALUDES[0].translate(nueva_ubicacion,relative=False)
        
        
    
    
    
    
    
    
    # voy a añadir ahora una parte que es para diferenciar las autopistas arti-
    # ficiales que creo con respecto a las autopistas reales de algunos casos
    # que tengo:
        
    if nubes_modificadas == 'Santarem':
        
        print('Nubes Santarem')
                
        # Copiamos primero las barreras quitamiedos en los laterales de la me-
        # diana:
            
        # 'altura_mediana' ya cogida al principio del script.
        
        barrera_quitamiedos_1_mediana = copy.deepcopy(pcd_barrera)
        
        nueva_ubicacion = [mediana.get_center()[0]-buffer_arcen_orig,
                           mediana.get_center()[1], 
                           pcd_barrera.get_center()[2]]
        barrera_quitamiedos_1_mediana.translate(nueva_ubicacion,relative=False)
        
        
        barrera_quitamiedos_2_mediana = copy.deepcopy(pcd_barrera)
        
        nueva_ubicacion = [mediana.get_center()[0]+buffer_arcen_orig,
                           mediana.get_center()[1],
                           pcd_barrera.get_center()[2]]
        barrera_quitamiedos_2_mediana.translate(nueva_ubicacion,relative=False)
        
        
        
        # Ahora seleccionamos uno de los carriles y le hacemos un DOWNSAMPLING
        # emulando del paso del escáner por el carril contrario:
        
        
        
        
        indices_seleccionados = np.random.choice(range(len(puntos_carril_1)), 5000 , replace=False)
        puntos_carril_1 = puntos_carril_1[indices_seleccionados]
        carril_1 = o3d.geometry.PointCloud()
        carril_1.points = o3d.utility.Vector3dVector(puntos_carril_1)
        carril_1.paint_uniform_color([0,0,0])
       
        CARRETERAS[0] = carril_1 + carril_2

        # Hago lo mismo con los arcenes y la berma correspondiente:
        indices_seleccionados = np.random.choice(range(len(puntos_arcen_1_1)), 5000 , replace=False)
        puntos_arcen_1_1 = puntos_arcen_1_1[indices_seleccionados]
        pcd_arcen_1_1 = o3d.geometry.PointCloud()
        pcd_arcen_1_1.points = o3d.utility.Vector3dVector(puntos_arcen_1_1)
        pcd_arcen_1_1.paint_uniform_color([0.3,0.3,0.3])
        
        indices_seleccionados = np.random.choice(range(len(puntos_arcen_1_2)), 2000 , replace=False)
        puntos_arcen_1_2 = puntos_arcen_1_2[indices_seleccionados]
        pcd_arcen_1_2 = o3d.geometry.PointCloud()
        pcd_arcen_1_2.points = o3d.utility.Vector3dVector(puntos_arcen_1_2)
        pcd_arcen_1_2.paint_uniform_color([0.11,0.22,0.33])

        ARCENES[0] = pcd_arcen_1_1 + pcd_arcen_1_2 + pcd_arcen_2_1 + pcd_arcen_2_2
        
        
        indices_seleccionados = np.random.choice(range(len(puntos_berma_1)), 1000 , replace=False)
        puntos_berma_1 = puntos_berma_1[indices_seleccionados]
        pcd_berma_1 = o3d.geometry.PointCloud()
        pcd_berma_1.points = o3d.utility.Vector3dVector(puntos_berma_1)
        pcd_berma_1.paint_uniform_color([1,0,0])
        
        BERMAS[0] = pcd_berma_1 + pcd_berma_2


        # import pdb
        # pdb.set_trace()
        
        
        #--------------------------------------------------------------------------
        # PARÓN PARA VISUALIZAR:
        # o3d.visualization.draw(mediana+pcd_arcen_1_2+pcd_arcen_1_1+carril_2+carril_1+pcd_arcen_2_1+pcd_arcen_2_2+pcd_berma_1+pcd_berma_2+barrera_quitamiedos_1_mediana+barrera_quitamiedos_2_mediana+barrera_quitamiedos_1+barrera_quitamiedos_2)
        #--------------------------------------------------------------------------
        
        
        
        
        
        
        
        # Ahora seleccionamos la parte del DTM más próxima a la trayectoria del
        # MLS y le hacemos un DOWNSAMPING especial, quitamos sólo unos cuantos
        # metros de talud y DTM:
        
        puntos_superficie = np.array(SUPERFICIES[0].points)    
        puntos_carril_2 = np.array(carril_2.points)
        
        buffer_downsampling_DTM = 20
        distancias = np.array(SUPERFICIES[0].compute_point_cloud_distance(carril_2))
        indices_seleccionados = np.where(distancias >= buffer_downsampling_DTM)[0]
        puntos_superficie = puntos_superficie[indices_seleccionados]
        
        pcd_superficie = o3d.geometry.PointCloud()
        pcd_superficie.points = o3d.utility.Vector3dVector(puntos_superficie)
        pcd_superficie.paint_uniform_color([102/255.,0/255.,51/255.])
        SUPERFICIES[0] = pcd_superficie
        
        # Ídem para el talud contiguo:
            
        puntos_taludes = np.array(TALUDES[0].points)    
        puntos_carril_2 = np.array(carril_2.points)
        
        buffer_downsampling_DTM = 10
        distancias = np.array(TALUDES[0].compute_point_cloud_distance(carril_2))
        indices_seleccionados = np.where(distancias >= buffer_downsampling_DTM)[0]
        puntos_taludes = puntos_taludes[indices_seleccionados]
        
        pcd_taludes = o3d.geometry.PointCloud()
        pcd_taludes.points = o3d.utility.Vector3dVector(puntos_taludes)
        pcd_taludes.paint_uniform_color([102/255.,255/255.,255/255.])
        TALUDES[0] = pcd_taludes
        
        
        
        
        
        #------------------------------------------------------------------------------
        # PARÓN PARA VISUALIZAR:
        # visualizamos TODO en la nube excepto árboles y señales:
        # visor.custom_draw_geometry_with_key_callback(mediana+pcd_arcen_1_2+pcd_arcen_1_1+carril_2+carril_1+pcd_arcen_2_1+pcd_arcen_2_2+pcd_berma_1+pcd_berma_2+barrera_quitamiedos_1_mediana+barrera_quitamiedos_2_mediana+barrera_quitamiedos_1+barrera_quitamiedos_2+SUPERFICIES[0]+TALUDES[0])
        #------------------------------------------------------------------------------


        return SUPERFICIES,CARRETERAS,TALUDES,ARCENES,SENHALES,BERMAS,MEDIANAS,barrera_quitamiedos_1,barrera_quitamiedos_2,barrera_quitamiedos_1_mediana,barrera_quitamiedos_2_mediana,Lista_de_Senhales





        
    else:
    
        # Nubes artificiales por defecto
        print('Nubes artificiales por defecto')
    
    
    
    
    
    
    
        return SUPERFICIES,CARRETERAS,TALUDES,ARCENES,SENHALES,BERMAS,MEDIANAS,pcd_barrera,barrera_quitamiedos_1,barrera_quitamiedos_2,Lista_de_Senhales


































def national_vertical_pumping(SUPERFICIES,CARRETERAS,TALUDES,ARCENES,SENHALES,BERMAS,barrera_quitamiedos_1,barrera_quitamiedos_2,Lista_de_Senhales):
    
    
    return SUPERFICIES,CARRETERAS,TALUDES,ARCENES,SENHALES,BERMAS,barrera_quitamiedos_1,barrera_quitamiedos_2,Lista_de_Senhales





