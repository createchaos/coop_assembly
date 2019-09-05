
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
from coop_assembly.geometry_generation.generate_triangles import generate_structure
from coop_assembly.geometry_generation.generate_tetrahedra import generate_first_tri, generate_structure_from_points
from coop_assembly.help_functions.helpers_geometry import update_bar_lengths


def execute():
    print("execute")
    bool_draw = True
    try:
        import compas_rhino
    except: 
        bool_draw = False

    b_struct    = Bar_Structure()
    o_struct    = Overall_Structure(b_struct)

    r           = 12.5
    # r = 0.1
    extension   = 50
    support_nodes = (0, 1, 2)
    support_bars = [(0,1), (1,2), (2,0)]
    load_bars = [(3,4)]
    # load_bars = None

    t1 = time.time()

    generate_first_tri(o_struct, b_struct, r)

    update_bar_lengths(b_struct)
    generate_structure(o_struct, b_struct, bool_draw, r)
    steps   = 3
    # generate_structure(o_struct, b_struct, bool_draw, r, support_bars, load_bars, correct=False)
    # print ("output",b_struct, o_struct)
    return (b_struct.data, o_struct.data)


def execute_from_points(points, dict_nodes, support_nodes=None, support_bars=None, 
                        load_bars=None, load=None, check_col=False, pickle_output=False):
    print("execute from points")
    # in millimeter?
    r = 12.5 # 2.0 | 30.0

    b_struct    = Bar_Structure()
    o_struct    = Overall_Structure(b_struct)
    generate_structure_from_points(o_struct, b_struct, r, points, dict_nodes, support_bars,
                         load_bars, correct=True, load=load, check_col=check_col)

    if pickle_output:
        # better to have a to_data json output...
        return pickle.dumps((b_struct, o_struct))
    else:
        return b_struct, o_struct