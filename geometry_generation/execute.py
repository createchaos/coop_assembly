
'''
                                                                                                 
    °***       *****       ******       ****       ******  ******          **           **       
   **  **      **  **      **          **  **        **    **              **           **       
   **          *****       ****        ******        **    ****            **   *****   *****    
   **  **      **  **      **          **  **        **    **              **  **  **   **  **   
    ****   **  **  **  **  ******  **  **  **  **    **    ******          **   ******  *****    
                           
                                           
created on 28.06.2019
author: stefanaparascho
'''



import pickle
from spatial_structures.bar_structure import Bar_Structure
from coop_assembly.data_structure.data_structure_compas import Overall_Structure


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
    generate_structure(o_struct, b_struct, bool_draw, r, support_bars, load_bars, correct=False)

    return (a)