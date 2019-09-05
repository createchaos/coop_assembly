
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
# import coop_assembly
import compas
import platform

# if platform.python_implementation() == "CPython":
#     import importlib
#     importlib.reload(coop_assembly)
#     importlib.reload(compas)
# else:
#     reload(coop_assembly)
#     reload(compas)

import compas_fab
from compas.utilities.xfunc import XFunc

from coop_assembly.geometry_generation.execute import execute
from coop_assembly.help_functions.drawing import draw
from coop_assembly.data_structure.data_structure_compas import Overall_Structure

from spatial_structures.bar_structure import Bar_Structure


def main():
    run_python = True
    bool_draw = True
    draw_meshes = False
    draw_analysis = True

    try:
        #import compas_rhino
        import Rhino
    except:
        bool_draw = False

    if run_python:
        print("calling function in python")

        xfunc = XFunc(
            'coop_assembly.geometry_generation.execute.execute') #, python=r'C:\Users\Stefana\Anaconda2\envs\py36\python'
        xfun()
        print("error", xfunc.error)
        data = xfunc.data

        # geo_gen_execute.execute()
    else:
        data = pickle.loads(execute())

    b = Bar_Structure.from_data(data[0])
    
    o = Overall_Structure(b)
    o.data = data[1]

    if bool_draw:
        import Rhino.RhinoDoc
        import compas_rhino.helpers
        
        draw(b, o, 0, colors_b=((0, 255, 0), (70, 70, 255)),
             colors_o=((0, 255, 0), (170, 170, 255)))
        
        # if data[2] and data[3]:
        #     draw(b_n, o_n, 0, colors_b=((0, 200, 255), (0, 100, 255)),
        #          colors_o=((255, 0, 0), (255, 0, 0)))
        # if draw_meshes:
        #     if path:
        #         draw_robots(structure, building_member, rob_num,
        #                     rob_num_b, path, config_b, pick_up_config)
        #     else:
        #         for m in ms:
        #             compas_rhino.helpers.mesh.draw_mesh(m)
        # if draw_analysis:
        #     draw_structure(b, 12.5, "stresses")
        #     if data[2] and data[3]:
        #         draw_structure(b_n, 12.5, "stresses")

    return data


def main_gh_simple(points, dict_nodes, sup_nodes=None, sup_bars=None, l_bars=None, load=None, check_col=False, use_xfunc=True):
    """ghpython entry point, xfunc or rpc call is made here.
    
    Parameters
    ----------
    points : [type]
        [description]
    dict_nodes : dict
        [description]
    sup_nodes : [type], optional
        [description], by default None
    sup_bars : [type], optional
        support bars, by default None
    l_bars : [type], optional
        [description], by default None
    load : list of float, optional
        [description], by default None
    check_col : bool, optional
        [description], by default False
    
    Returns
    -------
    [type]
        [description]
    """
    save_struct_info = True

    if sup_nodes:
        for i, s in enumerate(sup_nodes):
            sup_nodes[i] = int(s)

    if use_xfunc:
        xfunc = XFunc(
                'coop_assembly.geometry_generation.execute.execute_from_points')
        xfunc(points, dict_nodes, support_nodes=sup_nodes,
            support_bars=sup_bars, load_bars=l_bars, load=load, check_col=check_col)
        print('main_gh_simple, xfnc error: ', xfunc.error)
        b_struct, o_struct = pickle.loads(xfunc.data)
    else:
        from compas.rpc import Proxy
        with Proxy('coop_assembly.geometry_generation.execute') as geo_gen_execute:
            b_struct, o_struct = geo_gen_execute.execute_from_points(
                points, dict_nodes, support_nodes=sup_nodes, 
                support_bars=sup_bars, load_bars=l_bars, load=load, check_col=check_col)

    return b_struct, o_struct