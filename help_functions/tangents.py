
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
from compas.geometry.transformations import project_points_plane
from compas.utilities import XFunc

import coop_assembly
from coop_assembly.help_functions.helpers_geometry import dropped_perpendicular_points, find_points_extreme, check_dir, calculate_coord_sys


def tangent_from_point(base_point1, line_vect1, base_point2, line_vect2, ref_point, dist1, dist2):

    solutions = lines_tangent_to_two_cylinder(base_point1, line_vect1, base_point2, line_vect2, ref_point, dist1, dist2)
    if solutions == None:
        print("no solutions for tangent_from_point")
        return None
    sol_0   = solutions[0]
    sol_1   = solutions[1]
    sol_2   = solutions[2]
    sol_3   = solutions[3]

    return [sol_0, sol_1, sol_2, sol_3]

def tangent_from_point_one(base_point1, line_vect1, base_point2, line_vect2, ref_point, dist1, dist2, nb):

    solution = lines_tangent_to_two_cylinder_one(
        base_point1, line_vect1, base_point2, line_vect2, ref_point, dist1, dist2, nb)
    if solution == None:
        print("no solutions for tangent_from_point")
        return None

    return [solution]


def tangent_through_two_points(base_point1, line_vect1, ref_point1, base_point2, line_vect2, ref_point2, dist1, dist2):

    ind = [0,1]

    sols_vec = []
    sols_pts = []

    # print("tangent_through_two_points", base_point1, line_vect1, ref_point1, base_point2, line_vect2, ref_point2, dist1, dist2)

    for i in ind:
        ret_p1 = p_planes_tangent_to_cylinder(base_point1, line_vect1, ref_point2, dist1 + dist2 + dist1 + dist2)
        print(ret_p1)
        ret1 = ret_p1[i]
        z_vec = cross_vectors(line_vect1, ret1[2])
        plane1 = (ret1[0], z_vec)
        # print("plane1", plane1)
        pp1 = project_points_plane([ref_point1], plane1)[0]
        vec_move = scale_vector(subtract_vectors(ref_point1, pp1), 0.5)
        pt1 = add_vectors(pp1, vec_move)
        
        for j in ind:
            ret_p2 = p_planes_tangent_to_cylinder(base_point2, line_vect2, ref_point1, dist1 + dist2 + dist1 + dist2)
            ret2 = ret_p2[j]
            z_vec = cross_vectors(line_vect2, ret2[2])
            plane2 = (ret2[0], z_vec)
            pp2 = project_points_plane([ref_point2], plane2)[0]
            vec_move = scale_vector(subtract_vectors(ref_point2, pp2), 0.5)
            pt2 = add_vectors(pp2, vec_move)

            sols_pts.append([pt1, pt2])
            sol_vec = subtract_vectors(pt1, pt2)            
            sols_vec.append(sol_vec)
    print(sols_pts)
    return sols_pts

    

    # ret_p1 = p_planes_tangent_to_cylinder(base_point1, line_vect1, ref_point2, dist1 + dist2)
    # print("this is it", ret_p1)
    # ret1 = ret_p1[ind1]
    # z_vec = cross_vectors(line_vect1, ret1[2])
    # plane1 = (ret1[0], z_vec)
    # print("plane1", plane1)
    # pp1 = project_points_plane([ref_point1], plane1)[0]
    # vec_move = scale_vector(subtract_vectors(ref_point1, pp1), 0.5)
    # pt1 = add_vectors(pp1, vec_move)

    
    # ret_p2 = p_planes_tangent_to_cylinder(base_point2, line_vect2, ref_point1, dist1 + dist2)
    # ret2 = ret_p2[ind2]
    # z_vec = cross_vectors(line_vect2, ret2[2])
    # plane2 = (ret2[0], z_vec)
    # pp2 = project_points_plane([ref_point2], plane2)[0]
    # vec_move = scale_vector(subtract_vectors(ref_point2, pp2), 0.5)
    # pt2 = add_vectors(pp2, vec_move)

    # return pt1, pt2


def lines_tangent_to_two_cylinder(base_point1, line_vect1, base_point2, line_vect2, ref_point, dist1, dist2):

    planes1     = p_planes_tangent_to_cylinder(base_point1, line_vect1, ref_point, dist1)
    planes2     = c_planes_tangent_to_cylinder(base_point2, line_vect2, ref_point, dist2)
    if planes1 == None or planes2 == None:
        # print("planes1", planes1)
        # print("planes2", planes2)
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

def lines_tangent_to_two_cylinder_one(base_point1, line_vect1, base_point2, line_vect2, ref_point, dist1, dist2, nb):

    planes1 = p_planes_tangent_to_cylinder(
        base_point1, line_vect1, ref_point, dist1)
    planes2 = c_planes_tangent_to_cylinder(
        base_point2, line_vect2, ref_point, dist2)
    if planes1 == None or planes2 == None:
        # print("planes1", planes1)
        # print("planes2", planes2)
        return None

    if nb == 0:
        s = intersect_plane_plane_u(planes1[0][1], planes1[0][2], planes2[0][0])
        s = normalize_vector(s)
    if nb == 1:
        s = intersect_plane_plane_u(planes1[0][1], planes1[0][2], planes2[1][0])
        s = normalize_vector(s)
    if nb == 2:
        s = intersect_plane_plane_u(planes1[1][1], planes1[1][2], planes2[0][0])
        s = normalize_vector(s)
    if nb == 3:
        s = intersect_plane_plane_u(planes1[1][1], planes1[1][2], planes2[1][0])
        s = normalize_vector(s)

    return s

def lines_tangent_to_cylinder(base_point, line_vect, ref_point, dist):

    l_vect      = line_vect
    l_vect      = normalize_vector(l_vect)
    ppol        = add_vectors(base_point, scale_vector(l_vect, dot_vectors(subtract_vectors(ref_point, base_point), l_vect)))
    ppolr_vect  = subtract_vectors(ref_point, ppol)
    # print("reference points", ref_point)
    # print(length_vector(ppolr_vect))
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
    dpp = dropped_perpendicular_points(pt, add_vectors(
        pt, vec_sol), b1["axis_endpoints"][0], b1["axis_endpoints"][1])
    pt_x1 = dpp[0]
    pt_1 = dpp[1]
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


def first_tangent(pt1, b1_1, b1_2, pt_mean_1, max_len, b_v1_1, b_v1_2, b_struct, pt_mean, r, b_v0_n=None, check_col=False):


    if check_col==False:
        if b_v0_n:
            ind = b_struct.vertex[b_v0_n]["index_sol"][0]
        else:
            ind = 0
        solutions_1 = tangent_from_point_one(b1_1["axis_endpoints"][0], subtract_vectors(b1_1["axis_endpoints"][1], b1_1["axis_endpoints"][0]),
                                        b1_2["axis_endpoints"][0], subtract_vectors(b1_2["axis_endpoints"][1], b1_2["axis_endpoints"][0]), pt1, 2 * r, 2 * r, ind)
        if solutions_1 == None:
            return None

        ret_cls = check_length_sol_one(
            solutions_1[0], pt_mean_1, pt1, b1_1, b1_2, b_v1_1, b_v1_2, b_struct)

        vec_sol_1, l1, pts_b1_1, pts_b1_2 = ret_cls
        pt1_e = add_vectors(pt1, scale_vector(vec_sol_1, l1))
        end_pts_0 = (pt1, pt1_e)
    else:
        for ind in range (4):
            solutions_1 = tangent_from_point_one(b1_1["axis_endpoints"][0], subtract_vectors(b1_1["axis_endpoints"][1], b1_1["axis_endpoints"][0]),
                                             b1_2["axis_endpoints"][0], subtract_vectors(b1_2["axis_endpoints"][1], b1_2["axis_endpoints"][0]), pt1, 2 * r, 2 * r, ind)

            if ind == 3 and solutions_1 == None:
                print("jumping out")
                return None
            if solutions_1 == None:
                print("no solutions 1", ind)
                print("ind", ind)
                continue

            ret_cls = check_length_sol_one(
                solutions_1[0], pt_mean_1, pt1, b1_1, b1_2, b_v1_1, b_v1_2, b_struct)

            vec_sol_1, l1, pts_b1_1, pts_b1_2 = ret_cls
            pt1_e       = add_vectors(pt1, scale_vector(vec_sol_1, l1))
            end_pts_0   = (pt1, pt1_e)
            
            # add extension
            ext_len = 30
            end_pts_0 = (add_vectors(pt1, scale_vector(normalize_vector(vector_from_points(pt1_e, pt1)), ext_len)), add_vectors(
                pt1_e, scale_vector(normalize_vector(vector_from_points(pt1, pt1_e)), ext_len)))

            bool_col = check_colisions(b_struct, end_pts_0, r, bar_nb=b_v0_n)

            if bool_col == True:
                end_pts_check = b_struct.vertex[b_v1_1]["axis_endpoints"]
                bool_col = check_colisions(
                    b_struct, end_pts_check, r, bar_nb=b_v0_n, bar_checking=b_v1_1)
                if bool_col == True:
                    end_pts_check = b_struct.vertex[b_v1_2]["axis_endpoints"]
                    bool_col = check_colisions(
                        b_struct, end_pts_check, r, bar_nb=b_v0_n, bar_checking=b_v1_2)

            if bool_col == False:
                print("COLLIDE", len(b_struct.vertex))
            if bool_col == True:
                break
            if ind == 3 and bool_col == False:
                print("NO TANGENT 1 FOUND IN ONE BAR COMBINATION")
                return None
    ####################################################################
    # end_pts_0 = (pt1, add_vectors(pt1, solutions_1[0]))
    ##################################################################
    end_pts_0 = [map(float, p) for p in end_pts_0]
    vec_x, vec_y, vec_z = calculate_coord_sys(end_pts_0, pt_mean)
    pt_o        = centroid_points(end_pts_0)
    if not b_v0_n:
        # pts_e = [map(float, p) for p in end_pts_0]
        b_v0 = b_struct.add_bar(0, end_pts_0, "tube", (25.0, 2.0), vec_z)
    else:
        b_v0        = b_v0_n
        b_struct.vertex[b_v0].update(
            {"axis_endpoints": end_pts_0})

    b_struct.vertex[b_v0].update({"index_sol":[ind]})
    b_struct.vertex[b_v0].update({"gripping_plane_no_offset":(pt_o, vec_x, vec_y, vec_z)})

    # b1_1.update({"axis_endpoints" :  pts_b1_1})
    # b1_2.update({"axis_endpoints" :  pts_b1_2})
    if not b_v0_n:
        b_struct.connect_bars(b_v0, b_v1_1)
        b_struct.connect_bars(b_v0, b_v1_2)

    dpp_1   = dropped_perpendicular_points(b_struct.vertex[b_v0]["axis_endpoints"][0], b_struct.vertex[b_v0]["axis_endpoints"][1], b_struct.vertex[b_v1_1]["axis_endpoints"][0], b_struct.vertex[b_v1_1]["axis_endpoints"][1])
    dpp_2   = dropped_perpendicular_points(b_struct.vertex[b_v0]["axis_endpoints"][0], b_struct.vertex[b_v0]["axis_endpoints"][1], b_struct.vertex[b_v1_2]["axis_endpoints"][0], b_struct.vertex[b_v1_2]["axis_endpoints"][1])

    k_1     = b_struct.edge[b_v0][b_v1_1]["endpoints"].keys()[0]
    k_2     = b_struct.edge[b_v0][b_v1_2]["endpoints"].keys()[0]
    b_struct.edge[b_v0][b_v1_1]["endpoints"].update({k_1:(dpp_1[0], dpp_1[1])})
    b_struct.edge[b_v0][b_v1_2]["endpoints"].update({k_2:(dpp_2[0], dpp_2[1])})

    return b_v0, end_pts_0


def second_tangent(b2_1, b2_2, pt_mean_2, b_v2_1, b_v2_2, b_struct, b_v_old, pt1, r, max_len, pt_mean, b_v0_n=None, check_col=False):


    line        = b_struct.vertex[b_v_old]["axis_endpoints"]
    vec_l_0     = vector_from_points(line[0], line[1])
    ex          = normalize_vector(cross_vectors(normalize_vector(vec_l_0), (1,0,0)))
    ey          = normalize_vector(cross_vectors(normalize_vector(vec_l_0), ex))
    ptM         = pt1
    pt_b_1      = b2_1["axis_endpoints"][0]
    pt_b_1_2    = b2_1["axis_endpoints"][1]
    l_1         = vector_from_points(pt_b_1, pt_b_1_2)
    pt_b_2      = b2_2["axis_endpoints"][0]
    pt_b_2_2    = b2_2["axis_endpoints"][1]
    l_2         = vector_from_points(pt_b_2, pt_b_2_2)

    # sols_test = tangent_from_point(pt_b_1, l_1, pt_b_2, l_2, ptM, 2*r, 2*r)
    if check_col == False:
        if b_v0_n:
            ind = b_struct.vertex[b_v0_n]["index_sol"][0]
        else:
            ind = 0
        sols_test = tangent_from_point_one(pt_b_1, l_1, pt_b_2, l_2, ptM, 2 * r, 2 * r, ind)
        if not sols_test:
            return None

        args    = ptM, ex, ey, r, pt_b_1, l_1, pt_b_2, l_2, 2*r, 2*r, ind
        xfunc = XFunc('coop_assembly.help_functions.tangents.solve_second_tangent', r'C:\Users\ChaseGalis\Desktop\ECL\GITRepository')
        xfunc(ptM, ex, ey, r, pt_b_1, l_1, pt_b_2, l_2, 2*r, 2*r, ind)
        ret_sst = xfunc.data
        # ret_sst = solve_second_tangent(*args)
        if ret_sst:
            pt2, vec_l = ret_sst
        else:
            return None

        solution = vec_l

        ret_cls = check_length_sol_one(
            solution, pt_mean_2, pt2, b2_1, b2_2, b_v2_1, b_v2_2, b_struct)

        if not ret_cls:
            return None

        vec_sol_2, l2, pts_b2_1, pts_b2_2 = ret_cls
        pt2_e = add_vectors(pt2, scale_vector(vec_sol_2, l2))
        end_pts_0 = (pt2, pt2_e)

    else:
        for ind in range(4):
            sols_test = tangent_from_point_one(
                pt_b_1, l_1, pt_b_2, l_2, ptM, 2 * r, 2 * r, ind)
            if ind == 3 and sols_test == None:
                return None
            if sols_test == None:
                continue


            args    = ptM, ex, ey, r, pt_b_1, l_1, pt_b_2, l_2, 2*r, 2*r, ind
            ret_sst = solve_second_tangent(*args)
            if ret_sst:
                pt2, vec_l = ret_sst
            else:
                return None
            solution = vec_l
            ret_cls = check_length_sol_one(
            solution, pt_mean_2, pt2, b2_1, b2_2, b_v2_1, b_v2_2, b_struct)
            if not ret_cls:
                return None
            vec_sol_2, l2, pts_b2_1, pts_b2_2 = ret_cls

            pt2_e       = add_vectors(pt2, scale_vector(vec_sol_2, l2))
            end_pts_0   = (pt2, pt2_e)

            ext_len = 30
            end_pts_0 = (add_vectors(pt2, scale_vector(normalize_vector(vector_from_points(pt2_e, pt2)), ext_len)), add_vectors(
                pt2_e, scale_vector(normalize_vector(vector_from_points(pt2, pt2_e)), ext_len)))

            bool_col = check_colisions(b_struct, end_pts_0, r, bar_nb=b_v0_n)

            if bool_col == True:
                end_pts_check = b_struct.vertex[b_v2_1]["axis_endpoints"]
                bool_col = check_colisions(
                    b_struct, end_pts_check, r, bar_nb=b_v0_n, bar_checking=b_v2_1)
                if bool_col == True:
                    end_pts_check = b_struct.vertex[b_v2_2]["axis_endpoints"]
                    bool_col = check_colisions(
                        b_struct, end_pts_check, r, bar_nb=b_v0_n, bar_checking=b_v2_2)
            # bool_col = True
            if bool_col == False:
                print("COLLIDE", len(b_struct.vertex))
            if ind == 3 and bool_col == False:
                print("NO TANGENT 2 FOUND IN ONE BAR COMBINATION")
                return None
            if bool_col == True:
                break

    end_pts_0 = [map(float, p) for p in end_pts_0]

    vec_x, vec_y, vec_z = calculate_coord_sys(end_pts_0, pt_mean)
    pt_o        = centroid_points(end_pts_0)
    if not b_v0_n:
        # b_v0    = b_struct.add_bar(0, end_pts_0, "tube", (2*r, 2.0), vec_z)
        b_v0    = b_struct.add_bar(0, end_pts_0, "tube", (25.0, 2.0), vec_z)
    else:
        b_v0    = b_v0_n
        b_struct.vertex[b_v0].update(
            {"axis_endpoints": end_pts_0})

    b_struct.vertex[b_v0].update({"index_sol": [ind]})
    b_struct.vertex[b_v0].update({"gripping_plane_no_offset":(pt_o, vec_x, vec_y, vec_z)})

    b2_1.update({"axis_endpoints" :  pts_b2_1})
    b2_2.update({"axis_endpoints" :  pts_b2_2})
    if not b_v0_n:
        b_struct.connect_bars(b_v0, b_v2_1)
        b_struct.connect_bars(b_v0, b_v2_2)

    dpp_1   = dropped_perpendicular_points(b_struct.vertex[b_v0]["axis_endpoints"][0], b_struct.vertex[b_v0]["axis_endpoints"][1], b_struct.vertex[b_v2_1]["axis_endpoints"][0], b_struct.vertex[b_v2_1]["axis_endpoints"][1])
    dpp_2   = dropped_perpendicular_points(b_struct.vertex[b_v0]["axis_endpoints"][0], b_struct.vertex[b_v0]["axis_endpoints"][1], b_struct.vertex[b_v2_2]["axis_endpoints"][0], b_struct.vertex[b_v2_2]["axis_endpoints"][1])

#     b_struct.edge[b_v0][b_v2_1].update({"endpoints":[dpp_1[0], dpp_1[1]]})
#     b_struct.edge[b_v0][b_v2_2].update({"endpoints":[dpp_2[0], dpp_2[1]]})
    k_1     = b_struct.edge[b_v0][b_v2_1]["endpoints"].keys()[0]
    k_2     = b_struct.edge[b_v0][b_v2_2]["endpoints"].keys()[0]
    b_struct.edge[b_v0][b_v2_1]["endpoints"].update({k_1:(dpp_1[0], dpp_1[1])})
    b_struct.edge[b_v0][b_v2_2]["endpoints"].update({k_2:(dpp_2[0], dpp_2[1])})

    return b_v0, pt2, end_pts_0


def third_tangent(b_struct, b_v_old, b_v1, b3_1, b3_2, pt_mean_3, max_len, b_v3_1, b_v3_2, pt_mean, r, b_v0_n=None, check_col=False):

    line_1  =  b_struct.vertex[b_v_old]["axis_endpoints"]
    line_2  =  b_struct.vertex[b_v1]["axis_endpoints"]

    b1      = b_struct.vertex[b_v_old]
    b2      = b_struct.vertex[b_v1]

    pt_b_1  = line_1[0]
    pt_b_2  = line_2[0]
    pt_b_3  = b3_1["axis_endpoints"][0]
    pt_b_4  = b3_2["axis_endpoints"][0]
    l_1 = normalize_vector(vector_from_points(line_1[0], line_1[1]))
    l_2 = normalize_vector(vector_from_points(line_2[0], line_2[1]))
    l_3 = normalize_vector(vector_from_points(b3_1["axis_endpoints"][0], b3_1["axis_endpoints"][1]))
    l_4 = normalize_vector(vector_from_points(b3_2["axis_endpoints"][0], b3_2["axis_endpoints"][1]))

    pts_axis_1  = dropped_perpendicular_points(line_1[0], line_1[1], line_2[0], line_2[1])
    pt_axis_1   = centroid_points(pts_axis_1)
    pts_axis_2  = dropped_perpendicular_points(b3_1["axis_endpoints"][0], b3_1["axis_endpoints"][1], b3_2["axis_endpoints"][0], b3_2["axis_endpoints"][1])
    pt_axis_2   = centroid_points(pts_axis_2)
    pt_mid      = centroid_points((pt_axis_1, pt_axis_2))
    axis        = vector_from_points(pt_axis_1, pt_axis_2)
    ex          = normalize_vector(cross_vectors(normalize_vector(axis), (1,0,0)))
    ey          = normalize_vector(cross_vectors(normalize_vector(axis), ex))
    bounds      = (-100.0, 100.0)
    args        = pt_mid, ex, ey, r, pt_b_1, l_1, pt_b_2, l_2, pt_b_3, l_3, pt_b_4, l_4, bounds

    # solutions_1     = []
    # solutions_2     = []
    # pts_3           = []

    if check_col == False:
        if b_v0_n:
            ind_1 = b_struct.vertex[b_v0_n]["index_sol"][0]
            ind_2 = b_struct.vertex[b_v0_n]["index_sol"][1]
        else:
            ind_1 = 0
            ind_2 = 0


        args = pt_mid, ex, ey, r, pt_b_1, l_1, pt_b_2, l_2, pt_b_3, l_3, pt_b_4, l_4, bounds, ind_1, ind_2

        xfunc = XFunc('coop_assembly.help_functions.tangents.solve_third_tangent', r'C:\Users\ChaseGalis\Desktop\ECL\GITRepository')
        xfunc(pt_mid, ex, ey, r, pt_b_1, l_1, pt_b_2, l_2, pt_b_3, l_3, pt_b_4, l_4, bounds, ind_1, ind_2)
        ret_stt = xfunc.data
        # ret_stt = solve_third_tangent(*args)

        # if max(b_struct.vertex.keys()) == 67: print("args", args)
        
        if ret_stt:
            pt3, vec_l1, vec_l2, ang_check = ret_stt
        else:
            return None

        # pts_3.append(pt3)
        # solutions_1.append(vec_l1)
        # solutions_2.append(vec_l2)
        solution_1 = vec_l1
        solution_2 = vec_l2

        test_1 = check_length_sol_one(
            solution_2, pt_mean_3, pt3, b3_1, b3_2, b_v3_1, b_v3_2, b_struct)
        test_2 = check_length_sol_one(
            solution_1, pt_mean_3, pt3, b1, b2, b_v_old, b_v1, b_struct)

        if not test_1 or not test_2:
            return None

        # for n in test_1:
        #     for m in test_2:
        #         if n[4] == m[4]:
        #             vec_sol_31, l31, pts_b3_11, pts_b3_21, ind = n
        #             vec_sol_32, l32, pts_b3_12, pts_b3_22, ind_2 = m
        vec_sol_31, l31, pts_b3_11, pts_b3_21 = test_1
        vec_sol_32, l32, pts_b3_12, pts_b3_22 = test_2

        pt3_e1 = add_vectors(pt3, scale_vector(vec_sol_31, l31))
        pt3_e2 = add_vectors(pt3, scale_vector(vec_sol_32, -1 * l32))

        end_pts_0 = (pt3_e2, pt3_e1)

    else:
        bool_test = False
        for i in range(4):
            for j in range(4):
                ind_1 = i
                ind_2 = j

                args    = pt_mid, ex, ey, r, pt_b_1, l_1, pt_b_2, l_2, pt_b_3, l_3, pt_b_4, l_4, bounds, ind_1, ind_2
                ret_stt = solve_third_tangent(*args)
                if ret_stt:
                    pt3, vec_l1, vec_l2, ang_check  = ret_stt
                else:
                    return None

                # pts_3.append(pt3)
                # solutions_1.append(vec_l1)
                # solutions_2.append(vec_l2)
                solution_1 = vec_l1
                solution_2 = vec_l2

                #for j in range(4):
                test_1 = check_length_sol_one(solution_2, pt_mean_3, pt3, b3_1, b3_2, b_v3_1, b_v3_2, b_struct)
                test_2 = check_length_sol_one(solution_1, pt_mean_3, pt3, b1, b2, b_v_old, b_v1, b_struct)

                if not test_1 or not test_2:
                    return None

                vec_sol_31, l31, pts_b3_11, pts_b3_21  = test_1
                vec_sol_32, l32, pts_b3_12, pts_b3_22 = test_2

                pt3_e1 = add_vectors(pt3, scale_vector(vec_sol_31, l31))
                pt3_e2 = add_vectors(pt3, scale_vector(vec_sol_32, -1 * l32))

                end_pts_0 = (pt3_e2, pt3_e1)
                
                ext_len = 30
                end_pts_0 = (add_vectors(pt3_e2, scale_vector(normalize_vector(vector_from_points(pt3_e1, pt3_e2)), ext_len)), add_vectors(
                    pt3_e1, scale_vector(normalize_vector(vector_from_points(pt3_e2, pt3_e1)), ext_len)))
                
                bool_col = check_colisions(b_struct, end_pts_0, r, bar_nb=b_v0_n)

                if bool_col == True:
                    end_pts_check = b_struct.vertex[b_v3_1]["axis_endpoints"]
                    bool_col = check_colisions(
                        b_struct, end_pts_check, r, bar_nb=b_v0_n, bar_checking=b_v3_1)
                    if bool_col == True:
                        end_pts_check = b_struct.vertex[b_v3_2]["axis_endpoints"]
                        bool_col = check_colisions(
                            b_struct, end_pts_check, r, bar_nb=b_v0_n, bar_checking=b_v3_2)

                # bool_col = True
                if bool_col == False:
                    print("COLLIDE", len(b_struct.vertex))
                if i == 3 and j == 3 and bool_col == False:
                    print("NO TANGENT 3 FOUND IN ONE BAR COMBINATION")
                    return None
                if bool_col == True:
                    bool_test = True
                    break
            if bool_test == True: break

    end_pts_0 = [map(float, p) for p in end_pts_0]
    vec_x, vec_y, vec_z = calculate_coord_sys(end_pts_0, pt_mean)
    pt_o        = centroid_points(end_pts_0)
    if not b_v0_n:
        # b_v0    = b_struct.add_bar(0, end_pts_0, "tube", (2*r, 2.0), vec_z)
        b_v0    = b_struct.add_bar(0, end_pts_0, "tube", (25.0, 2.0), vec_z)
    else:
        b_v0    = b_v0_n
        b_struct.vertex[b_v0].update(
            {"axis_endpoints": end_pts_0})

    b_struct.vertex[b_v0].update({"index_sol": [ind_1, ind_2]})
    b_struct.vertex[b_v0].update({"gripping_plane_no_offset":(pt_o, vec_x, vec_y, vec_z)})

    b3_1.update({"axis_endpoints" :  pts_b3_11})
    b3_2.update({"axis_endpoints" :  pts_b3_21})
    if not b_v0_n:
        b_struct.connect_bars(b_v0, b_v3_1)
        b_struct.connect_bars(b_v0, b_v3_2)

    dpp_1   = dropped_perpendicular_points(b_struct.vertex[b_v0]["axis_endpoints"][0], b_struct.vertex[b_v0]["axis_endpoints"][1], b_struct.vertex[b_v3_1]["axis_endpoints"][0], b_struct.vertex[b_v3_1]["axis_endpoints"][1])
    dpp_2   = dropped_perpendicular_points(b_struct.vertex[b_v0]["axis_endpoints"][0], b_struct.vertex[b_v0]["axis_endpoints"][1], b_struct.vertex[b_v3_2]["axis_endpoints"][0], b_struct.vertex[b_v3_2]["axis_endpoints"][1])

#     b_struct.edge[b_v0][b_v3_1].update({"endpoints":[dpp_1[0], dpp_1[1]]})
#     b_struct.edge[b_v0][b_v3_2].update({"endpoints":[dpp_2[0], dpp_2[1]]})
    k_1     = b_struct.edge[b_v0][b_v3_1]["endpoints"].keys()[0]
    k_2     = b_struct.edge[b_v0][b_v3_2]["endpoints"].keys()[0]
    b_struct.edge[b_v0][b_v3_1]["endpoints"].update({k_1:(dpp_1[0], dpp_1[1])})
    b_struct.edge[b_v0][b_v3_2]["endpoints"].update({k_2:(dpp_2[0], dpp_2[1])})

    return b_v0, pt3, end_pts_0


def solve_second_tangent(*args):

    r       = args[3]
    import scipy.optimize
    for i in range(2):
        res_opt = scipy.optimize.fminbound(f_tangent_point_2, -2*r, 2*r, args, full_output=True, disp=0)
        if res_opt[1] > 0.1: return None
        x       = float(res_opt[0])
        ret_fp2 = find_point_2(x, *args)
        if not ret_fp2:
            return None
        else:
            pt_2, vec_l = ret_fp2
            return pt_2, vec_l


def solve_third_tangent(*args):

    import scipy.optimize


    res_opt = scipy.optimize.fmin(f_tangent_point_3, [0.0, 0.0], args, full_output=True, disp=0)
    if res_opt[1] > 0.1: return None
    ret_fp3 = find_point_3(list(map(float, res_opt[0])), *args)
    if not ret_fp3:
        return None
    else:
        ang, ref_point, vec_l1, vec_l2 = ret_fp3
        return ref_point, vec_l1, vec_l2, ang


def f_tangent_point_2(x, *args):

    ptM, ex, ey, r, pt_b_1, l_1, pt_b_2, l_2, d1, d2, ind = args

    r_c         = 2*r
    ref_point_tmp = add_vectors(scale_vector(ex, x), scale_vector(ey, math.sqrt(r_c*r_c - x*x)))
    ref_point = add_vectors(ref_point_tmp, ptM)

    vecs_l_all = tangent_from_point_one(
        pt_b_1, l_1, pt_b_2, l_2, ref_point, d1, d2, ind)

    if vecs_l_all:
        vec_l = vecs_l_all[0]
    else:
        print("error in f")
        f = 1
        return f

    f = abs(dot_vectors(normalize_vector(vec_l), normalize_vector(vector_from_points(ptM, ref_point))))

    return f


def f_tangent_point_3(x, *args):

    ptM, ex, ey, r, pt_b_1, l_1, pt_b_2, l_2, pt_b_3, l_3, pt_b_4, l_4, b, ind_1, ind_2 = args

    x1 = x[0]
    x2 = x[1]

    ref_point_tmp = add_vectors(scale_vector(ex, x1), scale_vector(ey, x2))
    ref_point = add_vectors(ref_point_tmp, ptM)

    tfp_1 = tangent_from_point_one(
        pt_b_1, l_1, pt_b_2, l_2, ref_point, 2 * r, 2 * r, ind_1)

    if tfp_1:
        vec_l1 = tfp_1[0]
    else:
        print("problem in opt 3 - 1")
        f = 180
        return f

    tfp_2         = tangent_from_point_one(pt_b_3, l_3, pt_b_4, l_4, ref_point, 2*r, 2*r, ind_2)

    if tfp_2:
    #     vec_l2 = tfp_2[ind_2]
        vec_l2 = tfp_2[0]
    else:
        print("problem in opt 3 - 1")
        f = 180
        return f
        #return None
    ang_v = angle_vectors(vec_l1, vec_l2, deg=True)
    if 180 - ang_v < 90:
        f = 180 - ang_v
    else:
        f = ang_v


    return f


def find_point_2(x, *args):

    ptM, ex, ey, r, pt_b_1, l_1, pt_b_2, l_2, d1, d2, ind = args

    r_c         = 2*r
    ref_point_tmp = add_vectors(scale_vector(ex, x), scale_vector(ey, math.sqrt(r_c*r_c - x*x)))
    ref_point = add_vectors(ref_point_tmp, ptM)

    vec_l = tangent_from_point_one(
        pt_b_1, l_1, pt_b_2, l_2, ref_point, d1, d2, ind)[0]

    return ref_point, vec_l


def find_point_3(x, *args):

    ptM, ex, ey, r, pt_b_1, l_1, pt_b_2, l_2, pt_b_3, l_3, pt_b_4, l_4, b, ind_1, ind_2 = args

    x1  = x[0]
    x2  = x[1]

    pt_1_tmp = add_vectors(scale_vector(ex, x1), scale_vector(ey, x2))
    pt_1 = add_vectors(pt_1_tmp, ptM)
    vec_l1      = tangent_from_point_one(pt_b_1, l_1, pt_b_2, l_2, pt_1, 2*r, 2*r, ind_1)[0]
    vec_l2      = tangent_from_point_one(pt_b_3, l_3, pt_b_4, l_4, pt_1, 2*r, 2*r, ind_2)[0]
    ref_point   = pt_1
    if not vec_l1 or not vec_l2:
        return None
    ang         = angle_vectors(vec_l1, vec_l2)

    return ang, ref_point, vec_l1, vec_l2


def check_colisions(b_struct, pts, r, bar_nb=None, bar_checking=None):

    # print "bar_checking", bar_checking
    for b in b_struct.vertex:
        if not bar_nb: bar_nb = 100000000000000
        if bar_checking != None and b < 3: continue
        if b < bar_nb and b != bar_checking:
            pts_b = b_struct.vertex[b]["axis_endpoints"]
            dpp = dropped_perpendicular_points(pts[0], pts[1], pts_b[0], pts_b[1])
            try:
                dist = distance_point_point(dpp[0], dpp[1])
            except:
                return False
                # dist = 100
            #print "distance check", dist
            if round(dist, 1) < round(2*r, 1):
                # print "b", b
                # print "possible collision"
                test = is_point_on_segment(dpp[0], pts, tol=50)
                if test:
                    test2 = is_point_on_segment(dpp[1], pts_b, tol=50)
                    if test2:
                        print("COLLISION", len(b_struct.vertex))

                        return False
                # return False

    return True



