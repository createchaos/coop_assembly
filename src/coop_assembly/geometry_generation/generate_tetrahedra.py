
'''
                                                                                                 
    ****       *****       ******       ****      ******  ******          **           **       
   **  **      **  **      **          **  **       **    **              **           **       
   **          *****       ****        ******       **    ****            **   *****   *****    
   **  **      **  **      **          **  **       **    **              **  **  **   **  **   
    ****   **  **  **  **  ******  **  **  **  **   **    ******          **   ******  *****    
                           
                                           
created on 30.06.2019
author: stefanaparascho
'''

from __future__ import print_function

import random
import itertools
import math

from compas.geometry.basic import add_vectors, normalize_vector, vector_from_points, scale_vector, cross_vectors, subtract_vectors
from compas.geometry.distance import distance_point_point
from compas.geometry.transformations import rotate_points
from compas.geometry.angles import angle_vectors
from compas.geometry.average import centroid_points

from coop_assembly.help_functions.helpers_geometry import calculate_coord_sys, calculate_bar_z, dropped_perpendicular_points, update_bar_lengths, correct_point, adjust_gripping_plane, find_bar_ends
from coop_assembly.help_functions.tangents import tangent_from_point, check_length_sol_one, first_tangent, second_tangent, third_tangent




def generate_first_tri(o_struct, b_struct, r, points = None):

    if points == None:
        st_pt       = (0.,0.,0.)
        end_pts_0   = (add_vectors(st_pt,(0.0,0.0,0.0)),add_vectors(st_pt,(1000.0,0.0,2*r)))
        end_pts_1   = (add_vectors(st_pt,(1000.0,0.0,0.0)),add_vectors(st_pt,(500.0,820.0,2*r)))
        end_pts_2   = (add_vectors(st_pt,(500.0,820.0,0.0)),add_vectors(st_pt,(0.0,0.0,2*r)))
    else:
        pt_0    = points[0]
        pt_1    = points[1]
        pt_2    = points[2]
        
        vec_0   = normalize_vector(vector_from_points(pt_0, pt_1))
        vec_1   = normalize_vector(vector_from_points(pt_1, pt_2))
        vec_2   = normalize_vector(vector_from_points(pt_2, pt_0))
        c_0     = scale_vector(normalize_vector(cross_vectors(vec_0, vec_1)), 2*r)
        c_1     = scale_vector(normalize_vector(cross_vectors(vec_1, vec_2)), 2*r)
        c_2     = scale_vector(normalize_vector(cross_vectors(vec_2, vec_0)), 2*r)

        end_pts_0   = (pt_0, add_vectors(pt_1, c_0))
        end_pts_1   = (pt_1, add_vectors(pt_2, c_1))
        end_pts_2   = (pt_2, add_vectors(pt_0, c_2))

    pt_int = centroid_points((end_pts_0[0], end_pts_0[1], end_pts_1[0], end_pts_1[1], end_pts_2[0], end_pts_2[1]))

    vec_x_0, vec_y_0, vec_z_0 = calculate_coord_sys(end_pts_0, pt_int)
    vec_x_1, vec_y_1, vec_z_1 = calculate_coord_sys(end_pts_1, pt_int)
    vec_x_2, vec_y_2, vec_z_2 = calculate_coord_sys(end_pts_2, pt_int)

    vec_z_0     = calculate_bar_z(end_pts_0)
    vec_z_1     = calculate_bar_z(end_pts_1)
    vec_z_2     = calculate_bar_z(end_pts_2)


    b_v0    = b_struct.add_bar(0, end_pts_0, "tube", (25.0, 2.0), vec_z_0)
    b_v1    = b_struct.add_bar(0, end_pts_1, "tube", (25.0, 2.0), vec_z_1)
    b_v2    = b_struct.add_bar(0, end_pts_2, "tube", (25.0, 2.0), vec_z_2)

    pt_o_0  = centroid_points(end_pts_0)
    pt_o_1  = centroid_points(end_pts_1)
    pt_o_2  = centroid_points(end_pts_2)

    b_struct.vertex[b_v0].update({"gripping_plane_no_offset": (pt_o_0, vec_x_0, vec_y_0, vec_z_0)})
    b_struct.vertex[b_v1].update({"gripping_plane_no_offset": (pt_o_1, vec_x_1, vec_y_1, vec_z_1)})
    b_struct.vertex[b_v2].update({"gripping_plane_no_offset": (pt_o_2, vec_x_2, vec_y_2, vec_z_2)})

    b_struct.connect_bars(b_v0, b_v1)
    b_struct.connect_bars(b_v1, b_v2)
    b_struct.connect_bars(b_v2, b_v0)

    # update_edges(b_struct)

    if points == None:
        o_v0    = o_struct.add_node(add_vectors(st_pt,[0.0,0.0,0.0]), 0)
        o_v1    = o_struct.add_node(add_vectors(st_pt,[900.0,0.0,0.0]), 0)
        o_v2    = o_struct.add_node(add_vectors(st_pt,[450.0,750,0.0]), 0)
    else:
        o_v0    = o_struct.add_node(pt_0, 0)
        o_v1    = o_struct.add_node(pt_1, 0)
        o_v2    = o_struct.add_node(pt_2, 0)
    
    o_struct.add_bar(o_v0, o_v1, b_v0)
    o_struct.add_bar(o_v1, o_v2, b_v1)
    o_struct.add_bar(o_v0, o_v2, b_v2)

    o_struct.calculate_point(o_v0)
    o_struct.calculate_point(o_v1)
    o_struct.calculate_point(o_v2)

    # o_struct.vertex[o_v0].update({"fixed":True})
    # o_struct.vertex[o_v1].update({"fixed":True})
    # o_struct.vertex[o_v2].update({"fixed":True})

    #### dummy definition of alternating robot numbers - to be changed in path planning calculation
    # rob_num_0 = "21" if b_v0 % 2 == 0 else "22"
    # rob_num_1 = "21" if b_v1 % 2 == 0 else "22"
    # rob_num_2 = "21" if b_v2 % 2 == 0 else "22"

    # b_struct.vertex[b_v0].update({"rob_num":rob_num_0})
    # b_struct.vertex[b_v1].update({"rob_num":rob_num_1})
    # b_struct.vertex[b_v2].update({"rob_num":rob_num_2})

    return b_struct, o_struct


def generate_structure_from_points(o_struct, b_struct, r, points, dict_nodes, supports = None, loads = None, correct=True, load=None, check_col=False):

    generate_first_tri(o_struct, b_struct, r, points)
    # generate_structure_points(o_struct, b_struct, points, dict_nodes, r, correct=correct, check_col=check_col)
    for i,n in enumerate(points):
        if i > 2:
            nodes   = dict_nodes[str(i)]
            #b1_1    = b_struct.vertex[o_struct.edge[bars1[0][0]][bars1[0][1]]["vertex_bar"]]
            list_bars_1 = o_struct.vertex_connected_edges(nodes[0])
            com_bars_1 = itertools.combinations(list_bars_1, 2)
            comb_1 = [x for x in com_bars_1]
            comb_1.reverse()
            list_bars_2 = o_struct.vertex_connected_edges(nodes[1])
            com_bars_2 = itertools.combinations(list_bars_2, 2)
            comb_2 = [x for x in com_bars_2]
            comb_2.reverse()
            list_bars_3 = o_struct.vertex_connected_edges(nodes[2])
            com_bars_3 = itertools.combinations(list_bars_3, 2)
            comb_3 = [x for x in com_bars_3]
            comb_3.reverse()

            add_tetra(o_struct, b_struct, nodes, comb_1, comb_2, comb_3,
                      1, n, r, correct=correct, check_col=check_col)


def add_tetra(o_struct, b_struct, nodes, comb_bars_1, comb_bars_2, comb_bars_3, dir_factor, pt_new_input, r, b_v0=None, b_v1=None, b_v2=None, bool_add=True, o_v_key=None, correct=True, check_col=False):
    
    # adds a new point and tetrahedron to the structure
    # input: nodes, bars from o_struct as vertex_key_integer and edge_vertex_key_tuples 
    
#     len_vec_min     = 1500
#     len_vec_max     = 1500
    
    len_vec_min     = 500
    len_vec_max     = 1400
    len_vec         = (random.random()*(len_vec_max - len_vec_min))+len_vec_min
    max_len         = 1800
    
    jnd = 0

    bars1 = comb_bars_1[jnd]
    bars2 = comb_bars_2[jnd]
    bars3 = comb_bars_3[jnd]

    b1_1    = b_struct.vertex[o_struct.edge[bars1[0][0]][bars1[0][1]]["vertex_bar"]]
    b_v1_1  = o_struct.edge[bars1[0][0]][bars1[0][1]]["vertex_bar"]
    b1_2    = b_struct.vertex[o_struct.edge[bars1[1][0]][bars1[1][1]]["vertex_bar"]]
    b_v1_2  = o_struct.edge[bars1[1][0]][bars1[1][1]]["vertex_bar"]
    b2_1    = b_struct.vertex[o_struct.edge[bars2[0][0]][bars2[0][1]]["vertex_bar"]]
    b_v2_1  = o_struct.edge[bars2[0][0]][bars2[0][1]]["vertex_bar"]
    b2_2    = b_struct.vertex[o_struct.edge[bars2[1][0]][bars2[1][1]]["vertex_bar"]]
    b_v2_2  = o_struct.edge[bars2[1][0]][bars2[1][1]]["vertex_bar"]
    b3_1    = b_struct.vertex[o_struct.edge[bars3[0][0]][bars3[0][1]]["vertex_bar"]]
    b_v3_1  = o_struct.edge[bars3[0][0]][bars3[0][1]]["vertex_bar"]
    b3_2    = b_struct.vertex[o_struct.edge[bars3[1][0]][bars3[1][1]]["vertex_bar"]]
    b_v3_2  = o_struct.edge[bars3[1][0]][bars3[1][1]]["vertex_bar"]
    
    
    dpp1        = dropped_perpendicular_points(b1_1["axis_endpoints"][0], b1_1["axis_endpoints"][1], b1_2["axis_endpoints"][0], b1_2["axis_endpoints"][1])
    pt_mean_1   = centroid_points(dpp1)
    dpp2        = dropped_perpendicular_points(b2_1["axis_endpoints"][0], b2_1["axis_endpoints"][1], b2_2["axis_endpoints"][0], b2_2["axis_endpoints"][1])
    pt_mean_2   = centroid_points(dpp2)
    dpp3        = dropped_perpendicular_points(b3_1["axis_endpoints"][0], b3_1["axis_endpoints"][1], b3_2["axis_endpoints"][0], b3_2["axis_endpoints"][1])
    pt_mean_3   = centroid_points(dpp3)
    
    pt_mean     = centroid_points([pt_mean_1, pt_mean_2, pt_mean_3])
    
    if bool_add == True:
        vec1    = subtract_vectors(pt_mean_1, pt_mean_2)
        vec2    = subtract_vectors(pt_mean_2, pt_mean_3)
        vec_n   = normalize_vector(cross_vectors(vec1, vec2))
        vec_n   = scale_vector(vec_n, len_vec)
    
        dir_factor  = random.sample((1, -1), 1)[0]
        #dir_factor = 1
        vec_n       = scale_vector(vec_n, dir_factor)
    
        pt_new = add_vectors(pt_mean, vec_n)
        
    if pt_new_input:
        pt_new = pt_new_input
    # check if new point is inside of structure
    if not pt_new_input:
        for t in o_struct.tetrahedra:
            if len(o_struct.tetrahedra[t]) > 3:
                if not o_struct.isOutside(pt_new, t):
                    vec_n   = scale_vector(vec_n, -1)
                    pt_new  = add_vectors(pt_mean, vec_n)
    
    if correct==True:
        pt_new = correct_point(b_struct, o_struct, o_v_key, pt_new, (b_v1_1, b_v1_2), (b_v2_1, b_v2_2), (b_v3_1, b_v3_2))
    pt1     = pt_new
    
    for j, bar_jnd_1 in enumerate(comb_bars_1):
        bars1 = bar_jnd_1
        b1_1 = b_struct.vertex[o_struct.edge[bars1[0][0]]
                                [bars1[0][1]]["vertex_bar"]]
        b_v1_1 = o_struct.edge[bars1[0][0]][bars1[0][1]]["vertex_bar"]
        b1_2 = b_struct.vertex[o_struct.edge[bars1[1][0]]
                                [bars1[1][1]]["vertex_bar"]]
        b_v1_2 = o_struct.edge[bars1[1][0]][bars1[1][1]]["vertex_bar"]
        
        if correct == True:
            pt_new = correct_point(b_struct, o_struct, o_v_key, pt_new,
                               (b_v1_1, b_v1_2), (b_v2_1, b_v2_2), (b_v3_1, b_v3_2))
        if bool_add:
            ret_ft = first_tangent(pt1, b1_1, b1_2, pt_mean_1, max_len,
                                b_v1_1, b_v1_2, b_struct, pt_mean, r, check_col=check_col)
        else:
            ret_ft = first_tangent(pt1, b1_1, b1_2, pt_mean_1, max_len,
                                b_v1_1, b_v1_2, b_struct, pt_mean, r, b_v0, check_col=check_col)
        
        if ret_ft:
            b_v0, end_pts_0        = ret_ft
            break
        else:
            print("tangent 1 not found")
            if j == len(comb_bars_1)-1:
                print("no point found for first tangent calculation - 430, add_tetra")
                return None
            
    for j, bar_jnd_2 in enumerate(comb_bars_2):
        bars2 = bar_jnd_2
        b2_1 = b_struct.vertex[o_struct.edge[bars2[0][0]]
                               [bars2[0][1]]["vertex_bar"]]
        b_v2_1 = o_struct.edge[bars2[0][0]][bars2[0][1]]["vertex_bar"]
        b2_2 = b_struct.vertex[o_struct.edge[bars2[1][0]]
                            [bars2[1][1]]["vertex_bar"]]
        b_v2_2 = o_struct.edge[bars2[1][0]][bars2[1][1]]["vertex_bar"]

        if correct == True:
            pt_new = correct_point(b_struct, o_struct, o_v_key, pt_new,
                               (b_v1_1, b_v1_2), (b_v2_1, b_v2_2), (b_v3_1, b_v3_2))
        if bool_add:
            ret_st = second_tangent(b2_1, b2_2, pt_mean_2, b_v2_1, b_v2_2,
                                    b_struct, b_v0, pt1, r, max_len, pt_mean, check_col=check_col)
        else:
            ret_st = second_tangent(b2_1, b2_2, pt_mean_2, b_v2_1, b_v2_2,
                                    b_struct, b_v0, pt1, r, max_len, pt_mean, b_v1, check_col=check_col)
        if ret_st:
            b_v1, pt2, end_pts_1    = ret_st
            break
        else:
            print("tangent 2 not found")
            if j == len(comb_bars_2) - 1:
                print("no point found for second tangent calculation - 430, add_tetra")
                return None
    

    for j, bar_jnd_3 in enumerate(comb_bars_3):
        bars3 = bar_jnd_3
        b3_1 = b_struct.vertex[o_struct.edge[bars3[0][0]]
                               [bars3[0][1]]["vertex_bar"]]
        b_v3_1 = o_struct.edge[bars3[0][0]][bars3[0][1]]["vertex_bar"]
        b3_2 = b_struct.vertex[o_struct.edge[bars3[1][0]]
                            [bars3[1][1]]["vertex_bar"]]
        b_v3_2 = o_struct.edge[bars3[1][0]][bars3[1][1]]["vertex_bar"]


        if correct == True:
            pt_new = correct_point(b_struct, o_struct, o_v_key, pt_new,
                               (b_v1_1, b_v1_2), (b_v2_1, b_v2_2), (b_v3_1, b_v3_2))
        if bool_add:
            ret_tt = third_tangent(b_struct, b_v0, b_v1, b3_1, b3_2, pt_mean_3,
                                max_len, b_v3_1, b_v3_2, pt_mean, r, check_col=check_col)
        else:
            ret_tt = third_tangent(b_struct, b_v0, b_v1, b3_1, b3_2, pt_mean_3,
                                max_len, b_v3_1, b_v3_2, pt_mean, r, b_v2, check_col=check_col)
        if ret_tt:
            b_v2, pt3, end_pts_2    = ret_tt
            break
        else:
            print("tangent 3 not found")
            if j == len(comb_bars_3) - 1:
                print("no point found for third tangent calculation - 430, add_tetra")
                return None

    
    if bool_add:
        b_struct.connect_bars(b_v0, b_v1)
        b_struct.connect_bars(b_v1, b_v2)
        b_struct.connect_bars(b_v2, b_v0)
    
    dpp_1   = dropped_perpendicular_points(b_struct.vertex[b_v1]["axis_endpoints"][0], b_struct.vertex[b_v1]["axis_endpoints"][1], b_struct.vertex[b_v2]["axis_endpoints"][0], b_struct.vertex[b_v2]["axis_endpoints"][1])
    key     = b_struct.edge[b_v1][b_v2]["endpoints"].keys()[0]
    b_struct.edge[b_v1][b_v2]["endpoints"].update({key:(dpp_1[0], dpp_1[1])})
    dpp_2   = dropped_perpendicular_points(b_struct.vertex[b_v2]["axis_endpoints"][0], b_struct.vertex[b_v2]["axis_endpoints"][1], b_struct.vertex[b_v0]["axis_endpoints"][0], b_struct.vertex[b_v0]["axis_endpoints"][1])
    key     = b_struct.edge[b_v2][b_v0]["endpoints"].keys()[0]
    b_struct.edge[b_v2][b_v0]["endpoints"].update({key:(dpp_2[0], dpp_2[1])})
    dpp_3   = dropped_perpendicular_points(b_struct.vertex[b_v0]["axis_endpoints"][0], b_struct.vertex[b_v0]["axis_endpoints"][1], b_struct.vertex[b_v1]["axis_endpoints"][0], b_struct.vertex[b_v1]["axis_endpoints"][1])
    key     = b_struct.edge[b_v0][b_v1]["endpoints"].keys()[0]
    b_struct.edge[b_v0][b_v1]["endpoints"].update({key:(dpp_3[0], dpp_3[1])})

    
    if bool_add:
        o_n_new     = o_struct.add_node(pt_new)
    
    ### check length of bar and adjust gripper position ###
    pt_bar_1    = b_struct.vertex[b_v0]["axis_endpoints"]
    pt_bar_2    = b_struct.vertex[b_v1]["axis_endpoints"]
    pt_bar_3    = b_struct.vertex[b_v2]["axis_endpoints"]
    
    adjust_gripping_plane(pt_bar_1, pt_new, b_struct, b_v0)
    adjust_gripping_plane(pt_bar_2, pt_new, b_struct, b_v1)
    adjust_gripping_plane(pt_bar_3, pt_new, b_struct, b_v2)
    ### ###
    
    if bool_add:
        o_n1    = nodes[0]
        o_n2    = nodes[1]
        o_n3    = nodes[2]
        o_struct.add_bar(o_n_new, o_n1, b_v0)
        o_struct.add_bar(o_n_new, o_n2, b_v1)
        o_struct.add_bar(o_n_new, o_n3, b_v2)
    
    #draw(b_struct, o_struct, 0)
    
    #### dummy definition of alternationg robot numbers - to be changed in path planning calculation
        rob_num_0 = "21" if b_v0 % 2 == 0 else "22"
        rob_num_1 = "21" if b_v1 % 2 == 0 else "22"
        rob_num_2 = "21" if b_v2 % 2 == 0 else "22"
        
        b_struct.vertex[b_v0].update({"rob_num":rob_num_0})
        b_struct.vertex[b_v1].update({"rob_num":rob_num_1})
        b_struct.vertex[b_v2].update({"rob_num":rob_num_2})
    
    find_bar_ends(b_struct, b_struct.vertex[b_v0], b_v0)
    find_bar_ends(b_struct, b_struct.vertex[b_v1], b_v1)
    find_bar_ends(b_struct, b_struct.vertex[b_v2], b_v2)
    
    find_bar_ends(b_struct, b_struct.vertex[b_v1_1], b_v1_1)
    find_bar_ends(b_struct, b_struct.vertex[b_v1_2], b_v1_2)
    find_bar_ends(b_struct, b_struct.vertex[b_v2_1], b_v2_1)
    find_bar_ends(b_struct, b_struct.vertex[b_v2_2], b_v2_2)
    find_bar_ends(b_struct, b_struct.vertex[b_v3_1], b_v3_1)
    find_bar_ends(b_struct, b_struct.vertex[b_v3_2], b_v3_2)
    
    return o_struct, b_struct