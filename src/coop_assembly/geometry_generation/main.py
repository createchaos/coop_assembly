
'''
                                                                                                 
    ****       *****       ******       ****       ******  ******          **           **       
   **  **      **  **      **          **  **        **    **              **           **       
   **          *****       ****        ******        **    ****            **   *****   *****    
   **  **      **  **      **          **  **        **    **              **  **  **   **  **   
    ****   **  **  **  **  ******  **  **  **  **    **    ******          **   ******  *****    
                           
                                           
created on 28.06.2019
author: stefanaparascho
'''

import pickle
import compas
import platform

from compas.utilities.xfunc import XFunc

from coop_assembly.geometry_generation.execute import execute
from coop_assembly.help_functions.drawing import draw
from coop_assembly.data_structure import Overall_Structure, Bar_Structure

def xfunc_entry(points, tet_node_ids, radius, check_col=False, correct=True, python_path='pythonw'):
    """ghpython entry point, xfunc or rpc call is made here.

    """
    # if use_xfunc:
    xfunc = XFunc(
            'coop_assembly.geometry_generation.execute.execute_from_points', python=python_path)
    xfunc(points, tet_node_ids, radius, check_col=check_col, correct=correct)
    print('xfunc_entry: xfnc error: ', xfunc.error)
    b_struct_data, o_struct_data = xfunc.data
    # else:
    #     print('main_gh_simple: rpc proxy')
    #     from compas.rpc import Proxy
    #     with Proxy('coop_assembly.geometry_generation.execute', python=python_path) as geo_gen_execute:
    #         b_struct_data, o_struct_data = geo_gen_execute.execute_from_points(
    #             points, dict_nodes, radius, check_col=check_col, correct=correct, base_tri_pt_ids=base_tri_pt_ids)
    return b_struct_data, o_struct_data