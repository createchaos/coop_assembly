
'''
                                                                                                 
    ****       *****       ******       ****      ******  ******          **           **       
   **  **      **  **      **          **  **       **    **              **           **       
   **          *****       ****        ******       **    ****            **   *****   *****    
   **  **      **  **      **          **  **       **    **              **  **  **   **  **   
    ****   **  **  **  **  ******  **  **  **  **   **    ******          **   ******  *****    
                           
                                           
created on 30.06.2019
author: stefanaparascho
'''

import math

from compas.geometry.basic import add_vectors, subtract_vectors, cross_vectors, normalize_vector, scale_vector, vector_from_points, dot_vectors, length_vector
from compas.geometry.distance import distance_point_point, distance_point_line
from compas.geometry.queries import is_point_on_segment
from compas.geometry.angles import angle_vectors
from compas.geometry.average import centroid_points

from coop_assembly.help_functions.helpers_geometry import dropped_perpendicular_points, find_points_extreme, check_dir


def tangent_from_point(base_point1, line_vect1, base_point2, line_vect2, ref_point, dist1, dist2):

    solutions = lines_tangent_to_two_cylinder(base_point1, line_vect1, base_point2, line_vect2, ref_point, dist1, dist2)
    if solutions == None:
        # print "dist_1", distance_point_line(ref_point, (base_point1, add_vectors(base_point1, line_vect1)))
        # print "dist_2", distance_point_line(ref_point, (base_point2, add_vectors(base_point1, line_vect2)))
        print("no solutions for tangent_from_point")
        return None
    sol_0   = solutions[0]
    sol_1   = solutions[1]
    sol_2   = solutions[2]
    sol_3   = solutions[3]

    return [sol_0, sol_1, sol_2, sol_3]


def lines_tangent_to_two_cylinder(base_point1, line_vect1, base_point2, line_vect2, ref_point, dist1, dist2):

    planes1     = p_planes_tangent_to_cylinder(base_point1, line_vect1, ref_point, dist1)
    planes2     = c_planes_tangent_to_cylinder(base_point2, line_vect2, ref_point, dist2)
    if planes1 == None or planes2 == None:
        print("planes1", planes1)
        print("planes2", planes2)
        return None

    s1  = intersect_plane_plane_u(planes1[0][1], planes1[0][2], planes2[0][0])
    s1  = normalize_vector(s1)
    s2  = intersect_plane_plane_u(planes1[0][1], planes1[0][2], planes2[1][0])
    s2  = normalize_vector(s2)
    s3  = intersect_plane_plane_u(planes1[1][1], planes1[1][2], planes2[0][0])
    s3  = normalize_vector(s3)
    s4  = intersect_plane_plane_u(planes1[1][1], planes1[1][2], planes2[1][0])
    s4  = normalize_vector(s4)

    return [s1, s2, s3, s4]


def lines_tangent_to_cylinder(base_point, line_vect, ref_point, dist):

    l_vect      = line_vect
    l_vect      = normalize_vector(l_vect)
    ppol        = add_vectors(base_point, scale_vector(l_vect, dot_vectors(subtract_vectors(ref_point, base_point), l_vect)))
    ppolr_vect  = subtract_vectors(ref_point, ppol)
    e1          = ppolr_vect
    e1          = normalize_vector(e1)
    e2          = cross_vectors(e1, l_vect)
    if length_vector(ppolr_vect)== 0: return None
    x           = dist / length_vector(ppolr_vect)
    d_e1        = scale_vector(e1, dist * x)
    # d(radius of bar section) has to be larger than l(distance from point to bar axis), otherwise the sqrt turns negative
    # if d < l: change ref_point
    if x*x < 1.0:
        d_e2 = scale_vector(e2, dist*math.sqrt(1.0-x*x))
    else:
        return None

    d_e_add     = add_vectors(d_e1, d_e2)
    d_e_sub     = subtract_vectors(d_e1, d_e2)

    return [ppol, d_e_add, d_e_sub]


def p_planes_tangent_to_cylinder(base_point, line_vect, ref_point, dist):

    l_vect  = line_vect
    l_vect  = normalize_vector(l_vect)
    lines   = lines_tangent_to_cylinder(base_point, line_vect, ref_point, dist)
    if lines == None:
        return None
    r1  = subtract_vectors(add_vectors(lines[0], lines[1]), ref_point)
    r1  = normalize_vector(r1)
    r2  = subtract_vectors(add_vectors(lines[0], lines[2]), ref_point)
    r2  = normalize_vector(r2)

    return [[ref_point, l_vect, r1], [ref_point, l_vect, r2]]


def c_planes_tangent_to_cylinder(base_point, line_vect, ref_point, dist):

    lines   = lines_tangent_to_cylinder(base_point, line_vect, ref_point, dist)
    if lines == None:
        return None
    pv      = ref_point
    dot_1   = dot_vectors(pv, lines[1])
    dot_2   = dot_vectors(pv, lines[2])

    return [[lines[1], dot_1], [lines[2], dot_2]]


def intersect_plane_plane_u(u_vect, v_vect, abc_vect):

    A           = dot_vectors(abc_vect, u_vect) / dot_vectors(abc_vect, v_vect)
    u_vect_sub  = subtract_vectors(u_vect, scale_vector(v_vect, A))

    return u_vect_sub


def check_length_sol_one(solution, pt_mean, pt, b1, b2, b1_key, b2_key, b_struct):

    vec_sol = solution
    # print("input", pt, vec_sol, b1["axis_endpoints"][0], b1["axis_endpoints"][1])
    dpp = dropped_perpendicular_points(pt, add_vectors(
        pt, vec_sol), b1["axis_endpoints"][0], b1["axis_endpoints"][1])
    pt_x1 = dpp[0]
    pt_1 = dpp[1]
    # print("point dpp", dpp)
    dpp = dropped_perpendicular_points(pt, add_vectors(
        pt, vec_sol), b2["axis_endpoints"][0], b2["axis_endpoints"][1])
    pt_x2 = dpp[0]
    pt_2 = dpp[1]

    if pt_x1 == None:
        return None
    if pt_x2 == None:
        return None

    pts_all_b1 = []
    b_vert_n = b_struct.vertex_neighbors(b1_key)
    pts_all_b1.append(b_struct.vertex[b1_key]["axis_endpoints"][0])
    pts_all_b1.append(b_struct.vertex[b1_key]["axis_endpoints"][1])
    for n in b_vert_n:
        pts_all_b1.append(dropped_perpendicular_points(
            b1["axis_endpoints"][0], b1["axis_endpoints"][1], b_struct.vertex[n]["axis_endpoints"][0], b_struct.vertex[n]["axis_endpoints"][1])[0])

    pts_all_b1.append(pt_1)
    pts_b1 = find_points_extreme(pts_all_b1, b1["axis_endpoints"])

    pts_all_b2 = []
    b_vert_n = b_struct.vertex_neighbors(b2_key)
    pts_all_b2.append(b_struct.vertex[b2_key]["axis_endpoints"][0])
    pts_all_b2.append(b_struct.vertex[b2_key]["axis_endpoints"][1])
    for n in b_vert_n:
        pts_all_b2.append(dropped_perpendicular_points(
            b2["axis_endpoints"][0], b2["axis_endpoints"][1], b_struct.vertex[n]["axis_endpoints"][0], b_struct.vertex[n]["axis_endpoints"][1])[0])

    pts_all_b2.append(pt_2)
    pts_b2 = find_points_extreme(pts_all_b2, b2["axis_endpoints"])

    vec_test_dir_1 = subtract_vectors(pt_mean, pt)
    if not check_dir(vec_sol, vec_test_dir_1):
        vec_sol = scale_vector(vec_sol, -1)

    lx1 = distance_point_point(pt, pt_x1)
    lx2 = distance_point_point(pt, pt_x2)
    l_max = lx1 if lx1 > lx2 else lx2

    sol = [vec_sol, l_max, pts_b1, pts_b2]

    return sol