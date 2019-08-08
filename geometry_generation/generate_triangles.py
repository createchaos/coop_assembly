
'''
                                                                                                 
    ****       *****       ******       ****      ******  ******          **           **       
   **  **      **  **      **          **  **       **    **              **           **       
   **          *****       ****        ******       **    ****            **   *****   *****    
   **  **      **  **      **          **  **       **    **              **  **  **   **  **   
    ****   **  **  **  **  ******  **  **  **  **   **    ******          **   ******  *****    
                           
                                           
created on 30.06.2019
author: stefanaparascho
'''


import random
import itertools
import math

from compas.geometry.basic import add_vectors, normalize_vector, vector_from_points, scale_vector, cross_vectors, subtract_vectors
from compas.geometry.distance import distance_point_point
from compas.geometry.transformations import rotate_points
from compas.geometry.angles import angle_vectors
from compas.geometry.average import centroid_points

from coop_assembly.help_functions.helpers_geometry import calculate_coord_sys, calculate_bar_z, dropped_perpendicular_points, update_bar_lengths
from coop_assembly.help_functions.tangents import tangent_from_point, check_length_sol_one

def generate_first_tetra(o_struct, b_struct, r, points = None):

    pt_0  = points[0]
    pt_1  = points[1]
    pt_2  = points[2]
    pt_3  = points[3]

    vec_0   = normalize_vector(vector_from_points(pt_0, pt_1))
    vec_1   = normalize_vector(vector_from_points(pt_1, pt_2))
    vec_2   = normalize_vector(vector_from_points(pt_2, pt_0))
    vec_3   = normalize_vector(vector_from_points(pt_0, pt_3))
    vec_4   = normalize_vector(vector_from_points(pt_1, pt_3))
    vec_5   = normalize_vector(vector_from_points(pt_2, pt_3))
    c_0     = scale_vector(normalize_vector(cross_vectors(vec_0, vec_1)), 2*r)
    c_1     = scale_vector(normalize_vector(cross_vectors(vec_1, vec_2)), 2*r)
    c_2     = scale_vector(normalize_vector(cross_vectors(vec_2, vec_0)), 2*r)
    c_3     = scale_vector(normalize_vector(cross_vectors(vec_0, vec_3)), 2*r)
    c_4     = scale_vector(normalize_vector(cross_vectors(vec_0, vec_5)), 2*r)
    c_5     = scale_vector(normalize_vector(cross_vectors(vec_1, vec_3)), 2*r)
    c_6     = scale_vector(normalize_vector(cross_vectors(vec_1, vec_4)), 2*r)
    c_7     = scale_vector(normalize_vector(cross_vectors(vec_2, vec_4)), 2*r)
    c_8     = scale_vector(normalize_vector(cross_vectors(vec_2, vec_5)), 2*r)

    end_pts_0   = (pt_0, add_vectors(pt_1, c_0))
    end_pts_1   = (pt_1, add_vectors(pt_2, c_1))
    end_pts_2   = (pt_2, add_vectors(pt_0, c_2))
    end_pts_3   = (pt_0, add_vectors(pt_3, c_3))
    end_pts_4   = (pt_1, add_vectors(pt_3, c_6))
    end_pts_5   = (pt_2, add_vectors(pt_3, c_8))

    pt_int = centroid_points((end_pts_0[0], end_pts_0[1], end_pts_1[0], end_pts_1[1], end_pts_2[0], end_pts_2[1], end_pts_3[0], end_pts_3[1], end_pts_4[0], end_pts_4[1], end_pts_5[0], end_pts_5[1]))

    vec_x_0, vec_y_0, vec_z_0 = calculate_coord_sys(end_pts_0, pt_int)
    vec_x_1, vec_y_1, vec_z_1 = calculate_coord_sys(end_pts_1, pt_int)
    vec_x_2, vec_y_2, vec_z_2 = calculate_coord_sys(end_pts_2, pt_int)
    vec_x_3, vec_y_3, vec_z_3 = calculate_coord_sys(end_pts_3, pt_int)
    vec_x_4, vec_y_4, vec_z_4 = calculate_coord_sys(end_pts_4, pt_int)
    vec_x_5, vec_y_5, vec_z_5 = calculate_coord_sys(end_pts_5, pt_int)

    vec_z_0     = calculate_bar_z(end_pts_0)
    vec_z_1     = calculate_bar_z(end_pts_1)
    vec_z_2     = calculate_bar_z(end_pts_2)
    vec_z_3     = calculate_bar_z(end_pts_3)
    vec_z_4     = calculate_bar_z(end_pts_4)
    vec_z_5     = calculate_bar_z(end_pts_5)

    b_v0    = b_struct.add_bar(0, end_pts_0, "tube", (25.0, 2.0), vec_z_0)
    b_v1    = b_struct.add_bar(0, end_pts_1, "tube", (25.0, 2.0), vec_z_1)
    b_v2    = b_struct.add_bar(0, end_pts_2, "tube", (25.0, 2.0), vec_z_2)
    b_v3    = b_struct.add_bar(0, end_pts_3, "tube", (25.0, 2.0), vec_z_3)
    b_v4    = b_struct.add_bar(0, end_pts_4, "tube", (25.0, 2.0), vec_z_4)
    b_v5    = b_struct.add_bar(0, end_pts_5, "tube", (25.0, 2.0), vec_z_5)

    b_struct.connect_bars(b_v0, b_v1)
    b_struct.connect_bars(b_v0, b_v3)
    b_struct.connect_bars(b_v0, b_v4)
    b_struct.connect_bars(b_v1, b_v2)
    b_struct.connect_bars(b_v1, b_v4)
    b_struct.connect_bars(b_v1, b_v5)
    b_struct.connect_bars(b_v2, b_v0)
    b_struct.connect_bars(b_v2, b_v5)
    b_struct.connect_bars(b_v2, b_v3)
    b_struct.connect_bars(b_v3, b_v4)
    b_struct.connect_bars(b_v4, b_v5)
    b_struct.connect_bars(b_v5, b_v3)



    # o_v0    = o_struct.add_node(pt_0, 0)
    # o_v1    = o_struct.add_node(pt_1, 0)
    # o_v2    = o_struct.add_node(pt_2, 0)

    # o_struct.add_bar(o_v0, o_v1, b_v0)
    # o_struct.add_bar(o_v1, o_v2, b_v1)
    # o_struct.add_bar(o_v0, o_v2, b_v2)

    # o_struct.calculate_point(o_v0)
    # o_struct.calculate_point(o_v1)
    # o_struct.calculate_point(o_v2)

    return b_struct, o_struct


def generate_first_tri(o_struct, b_struct, r, points = None):

    #st_pt       = (10000, 5000, 500)
    #st_pt       = (15362.49, 6987.53, 310.10)
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

    # b_v0    = b_struct.add_bar(0, end_pts_0, "tube", (2*r, 2.0), vec_z_0)
    # b_v1    = b_struct.add_bar(0, end_pts_1, "tube", (2*r, 2.0), vec_z_1)
    # b_v2    = b_struct.add_bar(0, end_pts_2, "tube", (2*r, 2.0), vec_z_2)

    b_v0    = b_struct.add_bar(0, end_pts_0, "tube", (25.0, 2.0), vec_z_0)
    b_v1    = b_struct.add_bar(0, end_pts_1, "tube", (25.0, 2.0), vec_z_1)
    b_v2    = b_struct.add_bar(0, end_pts_2, "tube", (25.0, 2.0), vec_z_2)

    # pt_o_0  = centroid_points(end_pts_0)
    # pt_o_1  = centroid_points(end_pts_1)
    # pt_o_2  = centroid_points(end_pts_2)

    # b_struct.vertex[b_v0].update({"gripping_plane_no_offset": (pt_o_0, vec_x_0, vec_y_0, vec_z_0)})
    # b_struct.vertex[b_v1].update({"gripping_plane_no_offset": (pt_o_1, vec_x_1, vec_y_1, vec_z_1)})
    # b_struct.vertex[b_v2].update({"gripping_plane_no_offset": (pt_o_2, vec_x_2, vec_y_2, vec_z_2)})

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



def generate_structure(o_struct, b_struct, bool_draw, r, points = None, supports=None, loads=None, correct=True):

    if points: 
        iterations = len(points)
    else:
        iterations = 1
    print("iterations", iterations)
        

    for i in range(iterations):

        list_bars_all = list(b_struct.vertices())
        # bars_rnd    = random.sample(list_bars_all,2)
        bars_rnd    = [i, i+1]

        for b in bars_rnd:
            list_bars_all.remove(b)

        bar1        = b_struct.vertex[bars_rnd[0]]["axis_endpoints"]
        bp1         = bar1[0]
        lv1         = subtract_vectors(bar1[1], bar1[0])
        bar2        = b_struct.vertex[bars_rnd[1]]["axis_endpoints"]
        bp2         = bar2[0]
        lv2         = subtract_vectors(bar2[1], bar2[0])
        dist1       = r
        dist2       = r

        if points:
            pt_test = points[i]
        else:
            if i == 0: 
                pt_test     = (300,300,1000)
            else:
                pt_test = (1200, 500, 700)

        rp          = pt_test

        bar_index   = add_tangent(b_struct, bp1, lv1, bp2, lv2, rp, dist1, dist2, bars_rnd)

        # bars_rnd    = random.sample(list_bars_all,1)
        bars_rnd    = [len(list_bars_all)-1]
        bars_rnd.append(bar_index)

        vec_move = normalize_vector(b_struct.vertex[bar_index]["zdir"])
        vec_axis = normalize_vector(subtract_vectors(b_struct.vertex[bar_index]["axis_endpoints"][1], b_struct.vertex[bar_index]["axis_endpoints"][0]))
        ang_rnd = math.radians(180)
        vec_new = scale_vector(rotate_points([vec_move], ang_rnd, vec_axis)[0], 30)

        rp          = (rp[0] + vec_new[0], rp[1] + vec_new[1], rp[2] + vec_new[2])

        bar1        = b_struct.vertex[bars_rnd[0]]["axis_endpoints"]
        bp1         = bar1[0]
        lv1         = subtract_vectors(bar1[1], bar1[0])

        bar2        = b_struct.vertex[bars_rnd[1]]["axis_endpoints"]
        bp2         = bar2[0]
        lv2         = subtract_vectors(bar2[1], bar2[0])
        dist1       = r
        dist2       = r

        bar_index   = add_tangent(b_struct, bp1, lv1, bp2, lv2, rp, dist1, dist2, bars_rnd)

        update_bar_lengths(b_struct)
        

    return b_struct


def add_tangent(b_struct, bp1, lv1, bp2, lv2, rp, dist1, dist2, bars_rnd):
    
    sol     = tangent_from_point(bp1, lv1, bp2, lv2, rp, dist1, dist2)
    
    b1_key  = bars_rnd[0]
    b2_key  = bars_rnd[1]
    b1      = b_struct.vertex[b1_key]
    b2      = b_struct.vertex[b2_key]

    ind_max_ang = find_sol_interlock(b_struct, b1_key, b2_key, sol, rp)

    # vec_sol = sol[0]
    vec_sol = sol[ind_max_ang]
    end_pts_0 = [rp, add_vectors(rp, vec_sol)]

    # pt_mean_1 = (200,200,500)
    # ret_cls = check_length_sol_one(sol[0], pt_mean_1, rp, b1, b2, b1_key, b2_key, b_struct)

    # vec_sol_1, l1, pts_b1_1, pts_b1_2 = ret_cls
    # # pt1_e = add_vectors(bp1, scale_vector(vec_sol_1, l1))
    # pt1_e = add_vectors(rp, scale_vector(vec_sol_1, l1))
    # # end_pts_0 = (bp1, pt1_e)
    # end_pts_0 = (rp, pt1_e)
    
    # # add extension
    # ext_len = 30
    # # end_pts_0 = (add_vectors(bp1, scale_vector(normalize_vector(vector_from_points(pt1_e, bp1)), ext_len)), add_vectors(
    # #     pt1_e, scale_vector(normalize_vector(vector_from_points(bp1, pt1_e)), ext_len)))


    # end_pts_0 = [map(float, p) for p in end_pts_0]
    # end_pts_0 = [list(p) for p in end_pts_0]
    
    vec_x, vec_y, vec_z = calculate_coord_sys(end_pts_0, (500,500,500))
    pt_o        = centroid_points(end_pts_0)
    
    b_v0 = b_struct.add_bar(0, end_pts_0, "tube", (25.0, 2.0), vec_z)


    # # b_struct.vertex[b_v0].update({"index_sol":[ind]})
    b_struct.vertex[b_v0].update({"gripping_plane_no_offset":(pt_o, vec_x, vec_y, vec_z)})

    # # b1_1.update({"axis_endpoints" :  pts_b1_1})
    # # b1_2.update({"axis_endpoints" :  pts_b1_2})
    b_struct.connect_bars(b_v0, b1_key)
    b_struct.connect_bars(b_v0, b2_key)

    dpp_1   = dropped_perpendicular_points(b_struct.vertex[b_v0]["axis_endpoints"][0], b_struct.vertex[b_v0]["axis_endpoints"][1], b_struct.vertex[b1_key]["axis_endpoints"][0], b_struct.vertex[b1_key]["axis_endpoints"][1])
    dpp_2   = dropped_perpendicular_points(b_struct.vertex[b_v0]["axis_endpoints"][0], b_struct.vertex[b_v0]["axis_endpoints"][1], b_struct.vertex[b2_key]["axis_endpoints"][0], b_struct.vertex[b2_key]["axis_endpoints"][1])

    k_1     = list(b_struct.edge[b_v0][b1_key]["endpoints"].keys())[0]
    k_2     = list(b_struct.edge[b_v0][b2_key]["endpoints"].keys())[0]
    b_struct.edge[b_v0][b1_key]["endpoints"].update({k_1:(dpp_1[0], dpp_1[1])})
    b_struct.edge[b_v0][b2_key]["endpoints"].update({k_2:(dpp_2[0], dpp_2[1])})

    return b_v0


def find_sol_interlock(b_struct, b1_key, b2_key, sol, rp):

    angles = []
    for vec in sol:
        pts_b1 = b_struct.vertex[b1_key]["axis_endpoints"] 
        dpp1 = dropped_perpendicular_points(
            pts_b1[0], pts_b1[1], rp, add_vectors(rp, vec))
        pts_b2 = b_struct.vertex[b2_key]["axis_endpoints"]
        dpp2 = dropped_perpendicular_points(
            pts_b2[0], pts_b2[1], rp, add_vectors(rp, vec))
        vec_1 = (subtract_vectors(dpp1[0], dpp1[1]))
        vec_2 = (subtract_vectors(dpp2[0], dpp2[1]))

        ang_vec = angle_vectors(vec_1, vec_2, deg=True)
        angles.append(ang_vec)
    print("all angles", angles)
    ang_max = max(angles)
    ind = angles.index(ang_max)
    print("index max angle", ind, ang_max)

    return ind
