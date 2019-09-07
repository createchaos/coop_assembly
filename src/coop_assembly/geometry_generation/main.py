
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
        xfunc()
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


def main_gh_simple(points, dict_nodes, radius, sup_nodes=None, sup_bars=None, l_bars=None, load=None, 
    check_col=False, correct=True, use_xfunc=True):
    """ghpython entry point, xfunc or rpc call is made here.
    
    Parameters
    ----------
    points : [type]
        [description]
    dict_nodes : dict
        [description]
    radius : float
        radius of the rod's cross section
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
    if use_xfunc:
        print('main_gh_simple: xfunc')
        xfunc = XFunc(
                'coop_assembly.geometry_generation.execute.execute_from_points')
        xfunc(points, dict_nodes, radius, support_nodes=sup_nodes,
            support_bars=sup_bars, load_bars=l_bars, load=load, check_col=check_col, correct=correct)
        print('main_gh_simple: xfnc error: ', xfunc.error)
        b_struct_data, o_struct_data = xfunc.data
    else:
        print('main_gh_simple: rpc proxy')
        from compas.rpc import Proxy
        with Proxy('coop_assembly.geometry_generation.execute') as geo_gen_execute:
            b_struct_data, o_struct_data = geo_gen_execute.execute_from_points(
                points, dict_nodes, radius, support_nodes=sup_nodes, 
                support_bars=sup_bars, load_bars=l_bars, load=load, check_col=check_col, correct=correct)
    return b_struct_data, o_struct_data