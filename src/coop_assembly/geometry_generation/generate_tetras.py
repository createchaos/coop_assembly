'''
Created on 25.10.2017

@author: stefanap
'''


import random
from lws_geometry.help_functions.help_functions_compas import *
from lws_geometry.help_functions.tangents import *
from lws_geometry.path_planning.path_planning import adjust_gripping_plane


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
    pt_mean_1   = point_mean(dpp1)
    dpp2        = dropped_perpendicular_points(b2_1["axis_endpoints"][0], b2_1["axis_endpoints"][1], b2_2["axis_endpoints"][0], b2_2["axis_endpoints"][1])
    pt_mean_2   = point_mean(dpp2)
    dpp3        = dropped_perpendicular_points(b3_1["axis_endpoints"][0], b3_1["axis_endpoints"][1], b3_2["axis_endpoints"][0], b3_2["axis_endpoints"][1])
    pt_mean_3   = point_mean(dpp3)
    
    pt_mean     = point_mean([pt_mean_1, pt_mean_2, pt_mean_3])
    
    if bool_add == True:
        vec1    = subtract_vectors(pt_mean_1, pt_mean_2)
        vec2    = subtract_vectors(pt_mean_2, pt_mean_3)
        print "vecs12", pt_mean_1, pt_mean_2, pt_mean_3
        vec_n   = normalize_vector(cross_vectors(vec1, vec2))
        vec_n   = scale_vector(vec_n, len_vec)
    
        dir_factor  = random.sample((1, -1), 1)[0]
        #dir_factor = 1
        vec_n       = scale_vector(vec_n, dir_factor)
        print "vec_n", vec_n
    
        pt_new = add_vectors(pt_mean, vec_n)
        
    if pt_new_input:
        pt_new = pt_new_input
    #print "point_input", pt_new_input
    #print o_struct.tetrahedra
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
    #print "point corrected", pt_new
    
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
            # print "tangent 1 found"
            break
        else:
            print "tangent 1 not found"
            if j == len(comb_bars_1)-1:
                print "no point found for first tangent calculation - 430, add_tetra"
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
            # print "tangent 2 found"
            break
        else:
            print "tangent 2 not found"
            if j == len(comb_bars_2) - 1:
                print "no point found for second tangent calculation - 430, add_tetra"
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
            # print "tangent 3 found"
            break
        else:
            print "tangent 3 not found"
            if j == len(comb_bars_3) - 1:
                print "no point found for third tangent calculation - 430, add_tetra"
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


def add_tetra_cen(o_struct, b_struct, nodes, comb_bars_1, comb_bars_2, comb_bars_3, dir_factor, pt_new_input, r, b_v0=None, b_v1=None, b_v2=None, bool_add=True, o_v_key=None, correct=True, check_col=False):

    len_vec_min = 950
    len_vec_max = 950
    len_vec = (random.random() * (len_vec_max - len_vec_min)) + len_vec_min
    max_len = 1500

    jnd = 0

    bars1 = comb_bars_1[jnd]
    bars2 = comb_bars_2[jnd]
    bars3 = comb_bars_3[jnd]

    b1_1 = b_struct.vertex[o_struct.edge[bars1[0][0]]
                           [bars1[0][1]]["vertex_bar"]]
    b_v1_1 = o_struct.edge[bars1[0][0]][bars1[0][1]]["vertex_bar"]
    b1_2 = b_struct.vertex[o_struct.edge[bars1[1][0]]
                           [bars1[1][1]]["vertex_bar"]]
    b_v1_2 = o_struct.edge[bars1[1][0]][bars1[1][1]]["vertex_bar"]
    b2_1 = b_struct.vertex[o_struct.edge[bars2[0][0]]
                           [bars2[0][1]]["vertex_bar"]]
    b_v2_1 = o_struct.edge[bars2[0][0]][bars2[0][1]]["vertex_bar"]
    b2_2 = b_struct.vertex[o_struct.edge[bars2[1][0]]
                           [bars2[1][1]]["vertex_bar"]]
    b_v2_2 = o_struct.edge[bars2[1][0]][bars2[1][1]]["vertex_bar"]
    b3_1 = b_struct.vertex[o_struct.edge[bars3[0][0]]
                           [bars3[0][1]]["vertex_bar"]]
    b_v3_1 = o_struct.edge[bars3[0][0]][bars3[0][1]]["vertex_bar"]
    b3_2 = b_struct.vertex[o_struct.edge[bars3[1][0]]
                           [bars3[1][1]]["vertex_bar"]]
    b_v3_2 = o_struct.edge[bars3[1][0]][bars3[1][1]]["vertex_bar"]

    print "bars all", b_v1_1, b_v1_2, b_v2_1, b_v2_2, b_v3_1, b_v3_2

    pt_mean = point_mean(
        [o_struct.vertex[nodes[0]]["point_xyz"], o_struct.vertex[nodes[1]]["point_xyz"], o_struct.vertex[nodes[2]]["point_xyz"]])

    if pt_new_input:
        pt_new = pt_new_input
    
    pts_con_0_1 = [(),()]
    pts_con_1_2 = [(),()]

    pts_all_b1_1 = b1_1["axis_endpoints"]
    pts_all_b1_2 = b1_2["axis_endpoints"]
    print "pts b 1 1", pts_all_b1_1
    print "pts b 1 2", pts_all_b1_2
    pts_comm = []
    dists_all = []
    inds_all = []
    for i, p1 in enumerate(pts_all_b1_1):
        for j, p2 in enumerate(pts_all_b1_2):
            dists_all.append(distance_point_point(p1, p2))
            inds_all.append([i,j])
    min_d = min(dists_all)
    ind_min = dists_all.index(min_d)
    pts_comm = inds_all[ind_min]
    
    if pts_comm == []: print "no common points found! 1"
    v1 = vector_from_points(pts_all_b1_1[0], pts_all_b1_1[1]) if pts_comm[0] == 0 else vector_from_points(
        pts_all_b1_1[1], pts_all_b1_1[0])
    pb1 = pts_all_b1_1[pts_comm[0]] 

    v2 = vector_from_points(o_struct.vertex[nodes[0]]["point_xyz"], pt_new)
    pts_con_0_1[0] = add_vectors(pt_new, scale_vector(normalize_vector(v2), -1*r))
    end_p_1 = add_vectors(o_struct.vertex[nodes[0]]["point_xyz"], scale_vector(normalize_vector(v2), r))
    pts_con_1 = [pb1, end_p_1]
    end_pts_0 = [pts_con_0_1[0], end_p_1]

    if bool_add:
        vec_x, vec_y, vec_z = calculate_coord_sys(end_pts_0, pt_mean)
        b_v0 = b_struct.add_bar(0, end_pts_0, "tube", (25.0, 2.0), vec_z)

    pts_all_b2_1 = b2_1["axis_endpoints"]
    pts_all_b2_2 = b2_2["axis_endpoints"]

    pts_comm = []
    dists_all = []
    inds_all = []
    for i, p1 in enumerate(pts_all_b2_1):
        for j, p2 in enumerate(pts_all_b2_2):
            dists_all.append(distance_point_point(p1, p2))
            inds_all.append([i, j])
    min_d = min(dists_all)
    ind_min = dists_all.index(min_d)
    pts_comm = inds_all[ind_min]

    if pts_comm == []:
        print "no common points found! 2"
    v1 = vector_from_points(pts_all_b2_1[0], pts_all_b2_1[1]) if pts_comm[0] == 0 else vector_from_points(
        pts_all_b2_1[1], pts_all_b2_1[0])
    pb2 = pts_all_b2_1[pts_comm[0]]

    v2 = vector_from_points(o_struct.vertex[nodes[1]]["point_xyz"], pt_new)
    pts_con_0_1[1] = add_vectors(
        pt_new, scale_vector(normalize_vector(v2), -1 * r))
    pts_con_1_2[0] = add_vectors(
        pt_new, scale_vector(normalize_vector(v2), -1 * r))
    end_p_2 = add_vectors(
        o_struct.vertex[nodes[1]]["point_xyz"], scale_vector(normalize_vector(v2), r))
    pts_con_2 = [pb2, end_p_2]
    end_pts_1 = [pts_con_0_1[1], end_p_2]

    if bool_add:
        vec_x, vec_y, vec_z = calculate_coord_sys(end_pts_1, pt_mean)
        b_v1 = b_struct.add_bar(0, end_pts_1, "tube", (25.0, 2.0), vec_z)

    pts_all_b3_1 = b3_1["axis_endpoints"]
    pts_all_b3_2 = b3_2["axis_endpoints"]
    print "pts 3 1", pts_all_b3_1

    pts_comm = []
    dists_all = []
    inds_all = []
    for i, p1 in enumerate(pts_all_b3_1):
        for j, p2 in enumerate(pts_all_b3_2):
            dists_all.append(distance_point_point(p1, p2))
            inds_all.append([i, j])
    min_d = min(dists_all)
    ind_min = dists_all.index(min_d)
    pts_comm = inds_all[ind_min]

    if pts_comm == []:
        print "no common points found! 3"
    v1 = vector_from_points(pts_all_b3_1[0], pts_all_b3_1[1]) if pts_comm[0] == 0 else vector_from_points(
        pts_all_b3_1[1], pts_all_b3_1[0])
    pb3 = pts_all_b3_1[pts_comm[0]] 
    v2 = vector_from_points(o_struct.vertex[nodes[2]]["point_xyz"], pt_new)

    end_p_3 = add_vectors(
        o_struct.vertex[nodes[2]]["point_xyz"], scale_vector(normalize_vector(v2), r))
    pts_con_3 = [pb3, end_p_3]

    pts_con_1_2[1] = add_vectors(
        pt_new, scale_vector(normalize_vector(v2), -1 * r))

    # end_pts_2 = [pts_con_1_2[1], pts_con_3[1]]
    end_pts_2 = [pts_con_1_2[1], end_p_3]

    if bool_add:
        vec_x, vec_y, vec_z = calculate_coord_sys(end_pts_2, pt_mean)
        b_v2 = b_struct.add_bar(0, end_pts_2, "tube", (25.0, 2.0), vec_z)

    if bool_add:
        b_struct.connect_bars(b_v1_1, b_v0, pts_con_1)
        b_struct.connect_bars(b_v2_1, b_v1, pts_con_2)
        b_struct.connect_bars(b_v3_1, b_v2, pts_con_3)

        b_struct.connect_bars(b_v0, b_v1, pts_con_0_1)
        b_struct.connect_bars(b_v1, b_v2, pts_con_1_2)

        o_n_new = o_struct.add_node(pt_new)

    ### check length of bar and adjust gripper position ###
    # pt_bar_1 = b_struct.vertex[b_v0]["axis_endpoints"]
    # pt_bar_2 = b_struct.vertex[b_v1]["axis_endpoints"]
    # pt_bar_3 = b_struct.vertex[b_v2]["axis_endpoints"]

    # adjust_gripping_plane(pt_bar_1, pt_new, b_struct, b_v0)
    # adjust_gripping_plane(pt_bar_2, pt_new, b_struct, b_v1)
    # adjust_gripping_plane(pt_bar_3, pt_new, b_struct, b_v2)
    ### ###

    if bool_add:
        o_n1 = nodes[0]
        o_n2 = nodes[1]
        o_n3 = nodes[2]
        o_struct.add_bar(o_n_new, o_n1, b_v0)
        o_struct.add_bar(o_n_new, o_n2, b_v1)
        o_struct.add_bar(o_n_new, o_n3, b_v2)

    #draw(b_struct, o_struct, 0)

    #### dummy definition of alternationg robot numbers - to be changed in path planning calculation
        rob_num_0 = "21" if b_v0 % 2 == 0 else "22"
        rob_num_1 = "21" if b_v1 % 2 == 0 else "22"
        rob_num_2 = "21" if b_v2 % 2 == 0 else "22"

        b_struct.vertex[b_v0].update({"rob_num": rob_num_0})
        b_struct.vertex[b_v1].update({"rob_num": rob_num_1})
        b_struct.vertex[b_v2].update({"rob_num": rob_num_2})

    return o_struct, b_struct


def add_tetra_one(o_struct, b_struct, nodes, comb_bars_1, comb_bars_2, comb_bars_3, dir_factor, pt_new_input, r, b_v0=None, b_v1=None, b_v2=None, bool_add=True, o_v_key=None, correct=True, check_col=False):

    # adds a new point and tetrahedron to the structure
    # input: nodes, bars from o_struct as vertex_key_integer and edge_vertex_key_tuples

    len_vec_min = 950
    len_vec_max = 950
    len_vec = (random.random() * (len_vec_max - len_vec_min)) + len_vec_min
    max_len = 1500

    jnd = 0

    bars1 = comb_bars_1[jnd]
    bars2 = comb_bars_2[jnd]
    bars3 = comb_bars_3[jnd]

    b1_1 = b_struct.vertex[o_struct.edge[bars1[0][0]]
                           [bars1[0][1]]["vertex_bar"]]
    b_v1_1 = o_struct.edge[bars1[0][0]][bars1[0][1]]["vertex_bar"]
    b1_2 = b_struct.vertex[o_struct.edge[bars1[1][0]]
                           [bars1[1][1]]["vertex_bar"]]
    b_v1_2 = o_struct.edge[bars1[1][0]][bars1[1][1]]["vertex_bar"]
    b2_1 = b_struct.vertex[o_struct.edge[bars2[0][0]]
                           [bars2[0][1]]["vertex_bar"]]
    b_v2_1 = o_struct.edge[bars2[0][0]][bars2[0][1]]["vertex_bar"]
    b2_2 = b_struct.vertex[o_struct.edge[bars2[1][0]]
                           [bars2[1][1]]["vertex_bar"]]
    b_v2_2 = o_struct.edge[bars2[1][0]][bars2[1][1]]["vertex_bar"]
    b3_1 = b_struct.vertex[o_struct.edge[bars3[0][0]]
                           [bars3[0][1]]["vertex_bar"]]
    b_v3_1 = o_struct.edge[bars3[0][0]][bars3[0][1]]["vertex_bar"]
    b3_2 = b_struct.vertex[o_struct.edge[bars3[1][0]]
                           [bars3[1][1]]["vertex_bar"]]
    b_v3_2 = o_struct.edge[bars3[1][0]][bars3[1][1]]["vertex_bar"]

    dpp1 = dropped_perpendicular_points(
        b1_1["axis_endpoints"][0], b1_1["axis_endpoints"][1], b1_2["axis_endpoints"][0], b1_2["axis_endpoints"][1])
    pt_mean_1 = point_mean(dpp1)
    dpp2 = dropped_perpendicular_points(
        b2_1["axis_endpoints"][0], b2_1["axis_endpoints"][1], b2_2["axis_endpoints"][0], b2_2["axis_endpoints"][1])
    pt_mean_2 = point_mean(dpp2)
    dpp3 = dropped_perpendicular_points(
        b3_1["axis_endpoints"][0], b3_1["axis_endpoints"][1], b3_2["axis_endpoints"][0], b3_2["axis_endpoints"][1])
    pt_mean_3 = point_mean(dpp3)

    pt_mean = point_mean([pt_mean_1, pt_mean_2, pt_mean_3])

    if bool_add == True:
        vec1 = subtract_vectors(pt_mean_1, pt_mean_2)
        vec2 = subtract_vectors(pt_mean_2, pt_mean_3)
        vec_n = normalize_vector(cross_vectors(vec1, vec2))
        vec_n = scale_vector(vec_n, len_vec)

        #dir_factor  = random.sample((1, -1), 1)[0]
        #dir_factor = 1
        vec_n = scale_vector(vec_n, dir_factor)

        pt_new = add_vectors(pt_mean, vec_n)

    if pt_new_input:
        pt_new = pt_new_input

    # check if new point is inside of structure
    if not pt_new_input:
        for t in o_struct.tetrahedra:
            if len(o_struct.tetrahedra[t]) > 3:
                if not o_struct.isOutside(pt_new, t):
                    vec_n = scale_vector(vec_n, -1)
                    pt_new = add_vectors(pt_mean, vec_n)

    if correct == True:
        pt_new = correct_point(b_struct, o_struct, o_v_key, pt_new,
                               (b_v1_1, b_v1_2), (b_v2_1, b_v2_2), (b_v3_1, b_v3_2))
    pt1 = pt_new
    #print "point corrected", pt_new

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
            ret_ft = first_tangent_one(pt1, b1_1, b1_2, pt_mean_1, max_len,
                                   b_v1_1, b_v1_2, b_struct, pt_mean, r, check_col=check_col)
        else:
            ret_ft = first_tangent_one(pt1, b1_1, b1_2, pt_mean_1, max_len,
                                   b_v1_1, b_v1_2, b_struct, pt_mean, r, b_v0, check_col=check_col)

        if ret_ft:
            b_v0, end_pts_0 = ret_ft
            # print "tangent 1 found"
            break
        else:
            print "tangent 1 not found"
            if j == len(comb_bars_1) - 1:
                print "no point found for first tangent calculation - 430, add_tetra"
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
            ret_st = second_tangent_one(b2_1, b2_2, pt_mean_2, b_v2_1, b_v2_2,
                                    b_struct, b_v0, pt1, r, max_len, pt_mean, check_col=check_col)
        else:
            ret_st = second_tangent_one(b2_1, b2_2, pt_mean_2, b_v2_1, b_v2_2,
                                    b_struct, b_v0, pt1, r, max_len, pt_mean, b_v1, check_col=check_col)
        if ret_st:
            b_v1, pt2, end_pts_1 = ret_st
            break
        else:
            print "tangent 2 not found"
            if j == len(comb_bars_2) - 1:
                print "no point found for second tangent calculation - 430, add_tetra"
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
            ret_tt = third_tangent_one(b_struct, b_v0, b_v1, b3_1, b3_2, pt_mean_3,
                                   max_len, b_v3_1, b_v3_2, pt_mean, r, check_col=check_col)
        else:
            ret_tt = third_tangent_one(b_struct, b_v0, b_v1, b3_1, b3_2, pt_mean_3,
                                   max_len, b_v3_1, b_v3_2, pt_mean, r, b_v2, check_col=check_col)
        if ret_tt:
            b_v2, pt3, end_pts_2 = ret_tt
            # print "tangent 3 found"
            break
        else:
            print "tangent 3 not found"
            if j == len(comb_bars_3) - 1:
                print "no point found for third tangent calculation - 430, add_tetra"
                return None

    if bool_add:
        b_struct.connect_bars(b_v0, b_v1)
        b_struct.connect_bars(b_v1, b_v2)
        # b_struct.connect_bars(b_v2, b_v0)

    dpp_1 = dropped_perpendicular_points(b_struct.vertex[b_v1]["axis_endpoints"][0], b_struct.vertex[b_v1]
                                         ["axis_endpoints"][1], b_struct.vertex[b_v2]["axis_endpoints"][0], b_struct.vertex[b_v2]["axis_endpoints"][1])
    key = b_struct.edge[b_v1][b_v2]["endpoints"].keys()[0]
    b_struct.edge[b_v1][b_v2]["endpoints"].update({key: (dpp_1[0], dpp_1[1])})
    # dpp_2 = dropped_perpendicular_points(b_struct.vertex[b_v2]["axis_endpoints"][0], b_struct.vertex[b_v2]
    #                                      ["axis_endpoints"][1], b_struct.vertex[b_v0]["axis_endpoints"][0], b_struct.vertex[b_v0]["axis_endpoints"][1])
    # key = b_struct.edge[b_v2][b_v0]["endpoints"].keys()[0]
    # b_struct.edge[b_v2][b_v0]["endpoints"].update({key: (dpp_2[0], dpp_2[1])})
    dpp_3 = dropped_perpendicular_points(b_struct.vertex[b_v0]["axis_endpoints"][0], b_struct.vertex[b_v0]
                                         ["axis_endpoints"][1], b_struct.vertex[b_v1]["axis_endpoints"][0], b_struct.vertex[b_v1]["axis_endpoints"][1])
    key = b_struct.edge[b_v0][b_v1]["endpoints"].keys()[0]
    b_struct.edge[b_v0][b_v1]["endpoints"].update({key: (dpp_3[0], dpp_3[1])})

    if bool_add:
        o_n_new = o_struct.add_node(pt_new)

    ### check length of bar and adjust gripper position ###
    pt_bar_1 = b_struct.vertex[b_v0]["axis_endpoints"]
    pt_bar_2 = b_struct.vertex[b_v1]["axis_endpoints"]
    pt_bar_3 = b_struct.vertex[b_v2]["axis_endpoints"]

    adjust_gripping_plane(pt_bar_1, pt_new, b_struct, b_v0)
    adjust_gripping_plane(pt_bar_2, pt_new, b_struct, b_v1)
    adjust_gripping_plane(pt_bar_3, pt_new, b_struct, b_v2)
    ### ###

    if bool_add:
        o_n1 = nodes[0]
        o_n2 = nodes[1]
        o_n3 = nodes[2]
        o_struct.add_bar(o_n_new, o_n1, b_v0)
        o_struct.add_bar(o_n_new, o_n2, b_v1)
        o_struct.add_bar(o_n_new, o_n3, b_v2)

    #draw(b_struct, o_struct, 0)

    #### dummy definition of alternationg robot numbers - to be changed in path planning calculation
        rob_num_0 = "21" if b_v0 % 2 == 0 else "22"
        rob_num_1 = "21" if b_v1 % 2 == 0 else "22"
        rob_num_2 = "21" if b_v2 % 2 == 0 else "22"

        b_struct.vertex[b_v0].update({"rob_num": rob_num_0})
        b_struct.vertex[b_v1].update({"rob_num": rob_num_1})
        b_struct.vertex[b_v2].update({"rob_num": rob_num_2})

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

def add_tetra_two(o_struct, b_struct, nodes, bars1, bars2, bars3, dir_factor, pt_new_input, r, b_v0 = None, b_v1 = None, b_v2 = None, bool_add = True, o_v_key=None, correct=True, check_col=False):
    
    """ adds a new point and tetrahedron to the structure
    input: nodes, bars from o_struct as vertex_key_integer and edge_vertex_key_tuples """
    
#     len_vec_min     = 1500
#     len_vec_max     = 1500
    
    len_vec_min     = 950
    len_vec_max     = 950
    len_vec         = (random.random()*(len_vec_max - len_vec_min))+len_vec_min
    max_len         = 1500
    

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
    pt_mean_1   = point_mean(dpp1)
    dpp2        = dropped_perpendicular_points(b2_1["axis_endpoints"][0], b2_1["axis_endpoints"][1], b2_2["axis_endpoints"][0], b2_2["axis_endpoints"][1])
    pt_mean_2   = point_mean(dpp2)
    dpp3        = dropped_perpendicular_points(b3_1["axis_endpoints"][0], b3_1["axis_endpoints"][1], b3_2["axis_endpoints"][0], b3_2["axis_endpoints"][1])
    pt_mean_3   = point_mean(dpp3)
    
    pt_mean     = point_mean([pt_mean_1, pt_mean_2, pt_mean_3])
    
    if bool_add == True:
        vec1    = subtract_vectors(pt_mean_1, pt_mean_2)
        vec2    = subtract_vectors(pt_mean_2, pt_mean_3)
        vec_n   = normalize_vector(cross_vectors(vec1, vec2))
        vec_n   = scale_vector(vec_n, len_vec)
    
        #dir_factor  = random.sample((1, -1), 1)[0]
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
    
    # find_bar_ends(b_struct, b_struct.vertex[b_v1_1], b_v1_1)
    # find_bar_ends(b_struct, b_struct.vertex[b_v1_2], b_v1_2)
    # find_bar_ends(b_struct, b_struct.vertex[b_v2_1], b_v2_1)
    # find_bar_ends(b_struct, b_struct.vertex[b_v2_2], b_v2_2)
    # find_bar_ends(b_struct, b_struct.vertex[b_v3_1], b_v3_1)
    # find_bar_ends(b_struct, b_struct.vertex[b_v3_2], b_v3_2)
    
    if bool_add:
        ret_ft = first_tangent_two(pt1, b1_1, b1_2, pt_mean_1, max_len,
                               b_v1_1, b_v1_2, b_struct, pt_mean, r, check_col=check_col)
    else:
        ret_ft = first_tangent_two(pt1, b1_1, b1_2, pt_mean_1, max_len,
                               b_v1_1, b_v1_2, b_struct, pt_mean, r, b_v0, check_col=check_col)
    if ret_ft:
        b_v0, end_pts_0        = ret_ft

    else:
        pt_1 = correct_point(b_struct, o_struct, o_v_key, pt_new,
                    (b_v1_1, b_v1_2), (b_v2_1, b_v2_2), (b_v3_1, b_v3_2))
        print "no point found for first tangent calculation - 430, add_tetra"
        return None

    # find_bar_ends(b_struct, b_struct.vertex[b_v1_1], b_v1_1)
    # find_bar_ends(b_struct, b_struct.vertex[b_v1_2], b_v1_2)
    # find_bar_ends(b_struct, b_struct.vertex[b_v2_1], b_v2_1)
    # find_bar_ends(b_struct, b_struct.vertex[b_v2_2], b_v2_2)
    # find_bar_ends(b_struct, b_struct.vertex[b_v3_1], b_v3_1)
    # find_bar_ends(b_struct, b_struct.vertex[b_v3_2], b_v3_2)
    if bool_add:
        ret_st = second_tangent_two(b2_1, b2_2, pt_mean_2, b_v2_1, b_v2_2,
                                b_struct, b_v0, pt1, r, max_len, pt_mean, check_col=check_col)
    else:
        ret_st = second_tangent_two(b2_1, b2_2, pt_mean_2, b_v2_1, b_v2_2,
                                b_struct, b_v0, pt1, r, max_len, pt_mean, b_v1, check_col=check_col)
    if ret_st:
        b_v1, pt2, end_pts_1    = ret_st
    else:
        print "no point found for second tangent calculation - 430, add_tetra"
        return None
    
    # find_bar_ends(b_struct, b_struct.vertex[b_v1_1], b_v1_1)
    # find_bar_ends(b_struct, b_struct.vertex[b_v1_2], b_v1_2)
    # find_bar_ends(b_struct, b_struct.vertex[b_v2_1], b_v2_1)
    # find_bar_ends(b_struct, b_struct.vertex[b_v2_2], b_v2_2)
    # find_bar_ends(b_struct, b_struct.vertex[b_v3_1], b_v3_1)
    # find_bar_ends(b_struct, b_struct.vertex[b_v3_2], b_v3_2)
    if bool_add:
        ret_tt = third_tangent_two(b_struct, b_v0, b_v1, b3_1, b3_2, pt_mean_3,
                               max_len, b_v3_1, b_v3_2, pt_mean, r, check_col=check_col)
    else:
        ret_tt = third_tangent_two(b_struct, b_v0, b_v1, b3_1, b3_2, pt_mean_3,
                               max_len, b_v3_1, b_v3_2, pt_mean, r, b_v2, check_col=check_col)
    if ret_tt:
        b_v2, pt3, end_pts_2    = ret_tt
    else:
        print "no point found for third tangent calculation - 430, add_tetra"
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
