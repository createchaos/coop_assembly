
'''
                                                                                                 
    ****       *****       ******       ****      ******  ******          **           **       
   **  **      **  **      **          **  **       **    **              **           **       
   **          *****       ****        ******       **    ****            **   *****   *****    
   **  **      **  **      **          **  **       **    **              **  **  **   **  **   
    ****   **  **  **  **  ******  **  **  **  **   **    ******          **   ******  *****    
                           
                                           
created on 28.06.2019
author: stefanaparascho
'''



import pickle
import time 

from spatial_structures.bar_structure import Bar_Structure
from coop_assembly.data_structure.data_structure_compas import Overall_Structure
from coop_assembly.help_functions.helpers_geometry import calculate_coord_sys, calculate_bar_z

from compas.geometry.basic import add_vectors, normalize_vector, vector_from_points, scale_vector, cross_vectors, subtract_vectors
from compas.geometry.distance import distance_point_point
from compas.geometry.transformations import rotate_points
from compas.geometry.angles import angle_vectors
from compas.geometry.average import centroid_points


def execute():

    a = "hello world"
    print("execute")
    bool_draw = True
    try:
        import compas_rhino
    except: bool_draw = False

    b_struct    = Bar_Structure()
    o_struct    = Overall_Structure(b_struct)

    r           = 12.5
    # r = 0.1
    extension   = 50
    support_nodes = (0, 1, 2)
    support_bars = [(0,1), (1,2), (2,0)]
    load_bars = [(3,4)]
    # load_bars = None

    t1      = time.time()

    generate_first_tri(o_struct, b_struct, r)
    steps   = 3
    # generate_structure(o_struct, b_struct, bool_draw, r, support_bars, load_bars, correct=False)
    # print ("output",b_struct, o_struct)
    return (b_struct.data, o_struct.data)


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