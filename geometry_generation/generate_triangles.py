
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

from compas.geometry.basic import add_vectors, normalize_vector, vector_from_points, scale_vector, cross_vectors, subtract_vectors
from compas.geometry.distance import distance_point_point
from compas.geometry.transformations import rotate_points
from compas.geometry.angles import angle_vectors
from compas.geometry.average import centroid_points

from coop_assembly.help_functions.helpers_geometry import calculate_coord_sys, calculate_bar_z, dropped_perpendicular_points
from coop_assembly.help_functions.tangents import tangent_from_point, check_length_sol_one

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

    pt_o_0  = centroid_points(end_pts_0)
    pt_o_1  = centroid_points(end_pts_1)
    pt_o_2  = centroid_points(end_pts_2)

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

def generate_structure(o_struct, b_struct, bool_draw, r, supports=None, loads=None, correct=True):

    for i in range(20):
        # choose two random nodes to connect to
        nodes_rnd   = random.sample(o_struct.vertex.keys(), 2)

        # choose nodes close to target
        # target      = [(10000,0,0)]
        # nodes_all   = choose_nodes(o_struct, target)
        # nodes_rnd   = random.sample(nodes_all, 3)

        # choose random two edges per node to connect new bar to
        o_e1_rnd    = random.sample(o_struct.vertex_connected_edges(nodes_rnd[0]), 2)
        o_e2_rnd    = random.sample(o_struct.vertex_connected_edges(nodes_rnd[1]), 2)
        o_e3_rnd    = random.sample(o_struct.vertex_connected_edges(nodes_rnd[2]), 2)

        # choose not random two edges per node to connect new bar to
        # o_e1_rnd = [o_struct.vertex_connected_edges(nodes_rnd[0])[-1], o_struct.vertex_connected_edges(nodes_rnd[0])[-2]]
        # o_e2_rnd = [o_struct.vertex_connected_edges(nodes_rnd[1])[-1], o_struct.vertex_connected_edges(nodes_rnd[1])[-2]]
        # o_e3_rnd = [o_struct.vertex_connected_edges(nodes_rnd[2])[-1], o_struct.vertex_connected_edges(nodes_rnd[2])[-2]]

        list_bars_1 = o_struct.vertex_connected_edges(nodes_rnd[0])
        com_bars_1 = itertools.combinations(list_bars_1, 2)
        comb_1 = [x for x in com_bars_1]
        comb_1.reverse()
        list_bars_2 = o_struct.vertex_connected_edges(nodes_rnd[1])
        com_bars_2 = itertools.combinations(list_bars_2, 2)
        comb_2 = [x for x in com_bars_2]
        comb_2.reverse()
        list_bars_3 = o_struct.vertex_connected_edges(nodes_rnd[2])
        com_bars_3 = itertools.combinations(list_bars_3, 2)
        comb_3 = [x for x in com_bars_3]
        comb_3.reverse()

        # add new tetrahedron connected at chosen bars
        # return_at   = add_tetra(o_struct, b_struct, nodes_rnd, o_e1_rnd, o_e2_rnd, o_e3_rnd, 1, None, r, correct=correct, check_col=True)
        # return_at   = add_tetra(o_struct, b_struct, nodes_rnd, comb_1, comb_2, comb_3, 1, None, r, correct=correct, check_col=True)
        # if return_at != None:
        #     o_struct, b_struct = return_at

        # update vertex points of overall structure according to the newly added connection points from b_struct
        # o_struct.update_points()

    # for sups in supports:
    #     add_supports(b_struct, sups[0], sups[1])
    # if loads:
    #     for load in loads:
    #         add_loads(b_struct, load[0], load[1], (0,0,-3000))


def generate_structure_rnd(o_struct, b_struct, bool_draw, r, supports=None, loads=None, correct=True):

    for i in range(1):
        # choose two random nodes to connect to
        # nodes_rnd   = random.sample(o_struct.vertex.keys(), 2)

        # choose nodes close to target
        # target      = [(10000,0,0)]
        # nodes_all   = choose_nodes(o_struct, target)
        # nodes_rnd   = random.sample(nodes_all, 3)

        # choose random two edges per node to connect new bar to
        # o_e1_rnd    = random.sample(o_struct.vertex_connected_edges(nodes_rnd[0]), 2)
        # o_e2_rnd    = random.sample(o_struct.vertex_connected_edges(nodes_rnd[1]), 2)
        # o_e3_rnd    = random.sample(o_struct.vertex_connected_edges(nodes_rnd[2]), 2)

        # o_e1_rnd    = random.sample(o_struct.edges, 2)
        # print("vertices", type(b_struct.vertices()))
        list_bars_all = list(b_struct.vertices())
        bars_rnd    = random.sample(list_bars_all,2)

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

        pt_test     = (300,300,1000)
        rp          = pt_test

        bar_index   = add_tangent(b_struct, bp1, lv1, bp2, lv2, rp, dist1, dist2, bars_rnd)

        bars_rnd    = random.sample(list_bars_all,1)
        bars_rnd.append(bar_index)
        # print("bars random", bars_rnd)

        bar1        = b_struct.vertex[bars_rnd[0]]["axis_endpoints"]
        bp1         = bar1[0]
        lv1         = subtract_vectors(bar1[1], bar1[0])
        bar2        = b_struct.vertex[bars_rnd[1]]["axis_endpoints"]
        bp2         = bar2[0]
        lv2         = subtract_vectors(bar2[1], bar2[0])
        dist1       = r
        dist2       = r

        bar_index   = add_tangent(b_struct, bp1, lv1, bp2, lv2, rp, dist1, dist2, bars_rnd)


def add_tangent(b_struct, bp1, lv1, bp2, lv2, rp, dist1, dist2, bars_rnd):
    print("input tangent", bars_rnd, rp)
    sol     = tangent_from_point(bp1, lv1, bp2, lv2, rp, dist1, dist2)
    print("tangent solution", sol)
    b1_key  = bars_rnd[0]
    b2_key  = bars_rnd[1]
    b1      = b_struct.vertex[b1_key]
    b2      = b_struct.vertex[b2_key]


    pt_mean_1 = (150,150,500)
    ret_cls = check_length_sol_one(sol[0], pt_mean_1, rp, b1, b2, b1_key, b2_key, b_struct)

    vec_sol_1, l1, pts_b1_1, pts_b1_2 = ret_cls
    pt1_e = add_vectors(bp1, scale_vector(vec_sol_1, l1))
    end_pts_0 = (bp1, pt1_e)
    
    # add extension
    ext_len = 30
    end_pts_0 = (add_vectors(bp1, scale_vector(normalize_vector(vector_from_points(pt1_e, bp1)), ext_len)), add_vectors(
        pt1_e, scale_vector(normalize_vector(vector_from_points(bp1, pt1_e)), ext_len)))


    end_pts_0 = [map(float, p) for p in end_pts_0]
    end_pts_0 = [list(p) for p in end_pts_0]
    
    vec_x, vec_y, vec_z = calculate_coord_sys(end_pts_0, (500,500,500))
    pt_o        = centroid_points(end_pts_0)
    
    b_v0 = b_struct.add_bar(0, end_pts_0, "tube", (25.0, 2.0), vec_z)


    # b_struct.vertex[b_v0].update({"index_sol":[ind]})
    b_struct.vertex[b_v0].update({"gripping_plane_no_offset":(pt_o, vec_x, vec_y, vec_z)})

    # b1_1.update({"axis_endpoints" :  pts_b1_1})
    # b1_2.update({"axis_endpoints" :  pts_b1_2})
    b_struct.connect_bars(b_v0, b1_key)
    b_struct.connect_bars(b_v0, b2_key)

    dpp_1   = dropped_perpendicular_points(b_struct.vertex[b_v0]["axis_endpoints"][0], b_struct.vertex[b_v0]["axis_endpoints"][1], b_struct.vertex[b1_key]["axis_endpoints"][0], b_struct.vertex[b1_key]["axis_endpoints"][1])
    dpp_2   = dropped_perpendicular_points(b_struct.vertex[b_v0]["axis_endpoints"][0], b_struct.vertex[b_v0]["axis_endpoints"][1], b_struct.vertex[b2_key]["axis_endpoints"][0], b_struct.vertex[b2_key]["axis_endpoints"][1])

    k_1     = list(b_struct.edge[b_v0][b1_key]["endpoints"].keys())[0]
    k_2     = list(b_struct.edge[b_v0][b2_key]["endpoints"].keys())[0]
    b_struct.edge[b_v0][b1_key]["endpoints"].update({k_1:(dpp_1[0], dpp_1[1])})
    b_struct.edge[b_v0][b2_key]["endpoints"].update({k_2:(dpp_2[0], dpp_2[1])})

    return b_v0
