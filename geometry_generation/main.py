
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

from compas.utilities.xfunc import XFunc
from coop_assembly.geometry_generation.execute import execute


def main():
    print("running main")

    run_python = True
    bool_draw = False
    draw_meshes = False
    draw_analysis = True


    try:
        #import compas_rhino
        import Rhino
    except:
        bool_draw = False

    if run_python:
        print("calling function in python")
        # xfunc = XFunc(
        #     'lws_geometry.geometry_generation.execute_compas.execute_optimise')
        xfunc = XFunc(
            'coop_assembly.geometry_generation.execute.execute')
        xfunc()
        print("error", xfunc.error)
        # data = pickle.loads(xfunc.data)
        data = xfunc.data

    else:
        data = execute()
   
    print(data)

    return data


main()