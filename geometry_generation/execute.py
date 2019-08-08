
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
from coop_assembly.geometry_generation.generate_triangles import generate_first_tri, generate_structure
from coop_assembly.help_functions.helpers_geometry import update_bar_lengths


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

    update_bar_lengths(b_struct)
    generate_structure(o_struct, b_struct, bool_draw, r)
    steps   = 3
    # generate_structure(o_struct, b_struct, bool_draw, r, support_bars, load_bars, correct=False)
    # print ("output",b_struct, o_struct)
    return (b_struct.data, o_struct.data)


def execute_from_points(points, dict_nodes, support_nodes=None, support_bars=None, load_bars=None, load=None, check_col=False):
    print("hi")
    save_struct_info = False
    r = 12.5
    # r = 2.0
    # r = 30.0
    # check_col = True
    b_struct    = Bar_Structure()
    o_struct    = Overall_Structure(b_struct)
    generate_from_points(o_struct, b_struct, r, points, dict_nodes, support_bars,
                         load_bars, correct=True, load=load, check_col=check_col)
    analyse_structure(b_struct)

    path_plan = False
    extend_bars(b_struct)   #extend the "feet"
    sequence = sorted(b_struct.vertex.keys())
    b_struct.attributes.update({"sequence": sequence})
    update_gripping_plane(b_struct, o_struct, sequence)

    # hardcoded changes in gripping plane
    # for v in b_struct.vertex:
    #     if v == 11:
    #         gp = b_struct.vertex[v]["gripping_plane_no_offset"]
    #         print "gp", gp
    #         axis = gp[1]
    #         vec_rot = rotate_points([gp[2], gp[3]], axis, math.radians(30))
    #         b_struct.vertex[v].update({"gripping_plane_no_offset":(gp[0], gp[1], vec_rot[0], vec_rot[1])})
    #     if v == 9:
    #         gp = b_struct.vertex[v]["gripping_plane_no_offset"]
    #         axis = gp[1]
    #         vec_rot = rotate_points([gp[2], gp[3]], axis, math.radians(-30))
    #         b_struct.vertex[v].update({"gripping_plane_no_offset":(gp[0], gp[1], vec_rot[0], vec_rot[1])})

    if path_plan:
        #sequence = sorted(b_struct.vertex.keys())

        # sequence[0] = 3
        # sequence[1] = 4
        # sequence[2] = 5
        # sequence[3] = 0
        # sequence[4] = 1
        # sequence[5] = 2

        #update_gripping_plane(b_struct, o_struct, sequence)
        extension = 30
        # extend_bars(b_struct)

        b_struct.attributes.update({"sequence":sequence})

        paths_all            = None
        building_members_all = None
        structures_all = None
        rob_nums_all = None
        configs_b_all = None
        pick_up_configs_all = None

        #meshes, element_mesh, paths_all, building_members_all, structures_all, rob_nums_all, configs_b_all, pick_up_configs_all = generate_robot_paths(b_struct, o_struct, r, steps, sequence, 0)
        meshes = generate_mesh(b_struct, r, sorted(b_struct.vertex))

        s_start = 30     #in bars number
        steps = 1       #in bars number

        #update if not starting from step 0
        # are being overwritten in generate_robot_paths
        # j_21    = [0, 0, 0, 0, 0, 0]
        # a_21    = [37201, -2000, -4500]
        # j_22    = [0, 0, 0, 0, 0, 0]
        # a_22    = [37201, -10000, -4500]

        meshes, element_mesh, paths_all, building_members_all, structures_all, rob_nums_all, configs_b_all, pick_up_configs_all = generate_robot_paths(
            b_struct, o_struct, r, steps, sequence, s_start)

        b_meshes    = [m.data for m in meshes]

    u_stresses = utils_total_stresses(b_struct)

    if save_struct_info:
        if check_lengths(b_struct):
            p = str(path.dirname(lws_geometry.__file__)) + r"\log_data\final_barstruct\two_con\barstructure_info_" + \
                datetime.now().strftime("%Y%m%d-%H%M%S")
            pickle_objects(b_struct, p)
            success = True
        else:
            success = False
    else:
        success=False

    path_pickle = path.dirname(lws_geometry.__file__) + r"\log_data\barstruct_all\bst_" + \
        datetime.now().strftime("%Y%m%d-%H%M%S")
    pickle_objects(b_struct, path_pickle)

    path_pickle = path.dirname(lws_geometry.__file__) + r"\log_data\ovstruct_all\ost_" + \
        datetime.now().strftime("%Y%m%d-%H%M%S")
    pickle_objects(o_struct, path_pickle)

    return pickle.dumps((b_struct, o_struct, success))