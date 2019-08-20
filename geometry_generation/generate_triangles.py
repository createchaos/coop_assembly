
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
from compas.geometry import translate_points

from coop_assembly.help_functions.helpers_geometry import calculate_coord_sys, calculate_bar_z, dropped_perpendicular_points, update_bar_lengths
from coop_assembly.help_functions.tangents import tangent_from_point, check_length_sol_one, tangent_through_two_points
from coop_assembly.geometry_generation.generate_tetrahedra import add_tetra


def generate_first_tetra(o_struct, b_struct, r, points = None):

    pt_0  = points[0]
    pt_1  = points[1]
    pt_2  = points[2]
    pt_3  = points[3]

    vec_0   = normalize_vector(vector_from_points(pt_0, pt_1))
    vec_1   = normalize_vector(vector_from_points(pt_1, pt_2))
    vec_2   = normalize_vector(vector_from_points(pt_2, pt_0))

    c_0     = scale_vector(normalize_vector(cross_vectors(vec_0, vec_2)), 2*r)
    c_1     = scale_vector(normalize_vector(cross_vectors(vec_1, vec_0)), 2*r)
    c_2     = scale_vector(normalize_vector(cross_vectors(vec_2, vec_1)), 2*r)

    end_pts_0   = (pt_0, add_vectors(pt_1, c_1))
    end_pts_1   = (pt_1, add_vectors(pt_2, c_2))
    end_pts_2   = (pt_2, add_vectors(pt_0, c_0))

    vec_0   = normalize_vector(vector_from_points(end_pts_0[0], end_pts_0[1]))
    vec_1   = normalize_vector(vector_from_points(end_pts_1[0], end_pts_1[1]))
    vec_2   = normalize_vector(vector_from_points(end_pts_2[0], end_pts_2[1]))

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

    b_struct.connect_bars(b_v0, b_v1)
    b_struct.connect_bars(b_v1, b_v2)
    b_struct.connect_bars(b_v2, b_v0)


    add_tetra(o_struct, b_struct, (b_v0, b_v1), (b_v1, b_v2), (b_v2, b_v0), pt_3, r)



    return b_struct, o_struct



def generate_structure(o_struct, b_struct, bool_draw, r, points = None, supports=None, loads=None, correct=True):

    if points: 
        iterations = len(points)
    else:
        iterations = 1
    print("iterations", iterations)
        
    # repeat for each point
    for i in range(iterations):

        # pick two bars - ??
        bars_rnd  = [i*2, i*2+1]

        # identify input parameters for tangent generation function
        bar1        = b_struct.vertex[bars_rnd[0]]["axis_endpoints"]
        bp1         = bar1[0]
        lv1         = subtract_vectors(bar1[1], bar1[0])
        bar2        = b_struct.vertex[bars_rnd[1]]["axis_endpoints"]
        bp2         = bar2[0]
        lv2         = subtract_vectors(bar2[1], bar2[0])
        dist1       = r
        dist2       = r

        # if points:
        pt_test = points[i]
        # else:
        #     if i == 0: 
        #         pt_test     = (300,300,1000)
        #     else:
        #         pt_test = (1200, 500, 700)

        rp          = pt_test

        # calculate tangent
        bar_index   = add_tangent(b_struct, bp1, lv1, bp2, lv2, rp, dist1, dist2, bars_rnd)

        # pick 2 new bars - last one added and another one
        bars_rnd = [bar_index]
        bars_rnd.append(bar_index-1)

        # move input point outside of the last bar's volume
        vec_move = normalize_vector(b_struct.vertex[bar_index]["zdir"])
        vec_axis = normalize_vector(subtract_vectors(b_struct.vertex[bar_index]["axis_endpoints"][1], b_struct.vertex[bar_index]["axis_endpoints"][0]))
        ang_rnd = math.radians(180)
        vec_new = scale_vector(rotate_points([vec_move], ang_rnd, vec_axis)[0], 30)

        rp          = (rp[0] + vec_new[0], rp[1] + vec_new[1], rp[2] + vec_new[2])

        # identify input parameters for tangent generation
        bar1        = b_struct.vertex[bars_rnd[0]]["axis_endpoints"]
        bp1         = bar1[0]
        lv1         = subtract_vectors(bar1[1], bar1[0])

        bar2        = b_struct.vertex[bars_rnd[1]]["axis_endpoints"]
        bp2         = bar2[0]
        lv2         = subtract_vectors(bar2[1], bar2[0])
        dist1       = r
        dist2       = r

        # calculate second tangent
        bar_index   = add_tangent(b_struct, bp1, lv1, bp2, lv2, rp, dist1, dist2, bars_rnd)

        # update lengths of bars of entire structure
        update_bar_lengths(b_struct)
        

    return b_struct



def generate_structure_no_points(o_struct, b_struct, bool_draw, r, iterations, supports=None, loads=None, correct=True):

    for i in range(iterations):
        
        bars_ind = []
        bars_ind.append(i+5)
        bar1 = b_struct.vertex[bars_ind[0]]["axis_endpoints"]
        bp1  = bar1[0]
        lv1  = subtract_vectors(bar1[1], bar1[0])
        rp1  = point_on_bar(bar1, lv1, 0, 1200)

        second_bar_key = second_bar_rnd(bars_ind[0])
        bars_ind.append(second_bar_key)
        bar2 = b_struct.vertex[bars_ind[1]]["axis_endpoints"]
        bp2  = bar2[0]
        lv2  = subtract_vectors(bar2[1], bar2[0])
        rp2  = point_on_bar(bar2, lv2, 0, 1200)

        radius1 = r
        radius2 = r

        add_tangent_no_points(b_struct, bp1, lv1, bp2, lv2, rp1, rp2, radius1, radius2, bars_ind)

    return b_struct


def point_on_bar(bar_keys, line_vector, min, max):
    cp  = centroid_points(bar_keys)
    sv  = scale_vector(normalize_vector(line_vector), random.randrange(min, max))
    rp  = add_vectors(cp, sv)

    return rp

def second_bar_rnd(end):
    start = 0
    bar_2_key = random.randrange(start, end)

    return bar_2_key


def add_tangent_no_points(b_struct, bp1, lv1, bp2, lv2, rp1, rp2, radius1, radius2, bars_ind):
   
    sols_two_points     = tangent_through_two_points(bp1, lv1, rp1, bp2, lv2, rp2, radius1, radius2)
    # list of possible solutions in vectors
    sol = find_sol_interlock(b_struct, bars_ind[0], bars_ind[1], sols_two_points)
    
    vec_x, vec_y, vec_z = calculate_coord_sys(sol, (500,500,500))
    b_new_bar = b_struct.add_bar(0, sol, "tube", (25.0, 2.0), vec_z)
    b_struct.connect_bars(b_new_bar, bars_ind[0])
    b_struct.connect_bars(b_new_bar, bars_ind[1])

    update_bar_lengths(b_struct)



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


def find_sol_interlock(b_struct, b1_key, b2_key, sol):

    angles = []
    for pts in sol:
        pts_b1 = b_struct.vertex[b1_key]["axis_endpoints"] 
        dpp1 = dropped_perpendicular_points(
            pts_b1[0], pts_b1[1], sol[0][0], sol[0][1])
        pts_b2 = b_struct.vertex[b2_key]["axis_endpoints"]
        dpp2 = dropped_perpendicular_points(
            pts_b2[0], pts_b2[1], sol[0][0], sol[0][1])
        vec_1 = (subtract_vectors(dpp1[0], dpp1[1]))
        vec_2 = (subtract_vectors(dpp2[0], dpp2[1]))

        ang_vec = angle_vectors(vec_1, vec_2, deg=True)
        angles.append(ang_vec)
    print("all angles", angles)
    ang_max = max(angles)
    ind = angles.index(ang_max)
    print("index max angle", ind, ang_max)
    solution = sol[ind]
    return solution
