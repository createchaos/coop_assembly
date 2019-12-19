
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

from compas.utilities.xfunc import XFunc
from coop_assembly.data_structure import OverallStructure, BarStructure
from coop_assembly.geometry_generation.generate_triangles import generate_structure
from coop_assembly.geometry_generation.generate_tetrahedra import generate_first_tri, generate_structure_from_points
from coop_assembly.help_functions.helpers_geometry import update_bar_lengths

def execute_from_points(points, tet_node_ids, radius, check_collision=False, correct=True):
    """Main entry point for the design system, for direct, xfunc or rpc call

    Parameters
    ----------
    points : list of float lists
        [[x,y,z], ...]
    tet_node_ids : list
        [[(base triangle vertex ids), new vertex id], ...]
    radius : float
        rod radius in millimeter
    check_col : bool, optional
        [description], by default False
    correct : bool, optional
        [description], by default True

    Returns
    -------
    (Overall_Structure.data, Bar_Structure.data)
        Serialized version of the overall structure and bar structure
    """
    b_struct    = BarStructure()
    o_struct    = OverallStructure(b_struct)
    generate_structure_from_points(o_struct, b_struct, radius, points, tet_node_ids,
        correct=correct, check_collision=check_collision)

    return (b_struct.data, o_struct.data)


def xfunc_execute_from_points(points, tet_node_ids, radius, check_collision=False, correct=True, python_path='pythonw'):
    """ghpython entry point, xfunc call is made here.

    """
    xfunc = XFunc(
            'coop_assembly.geometry_generation.execute.execute_from_points', python=python_path)
    xfunc(points, tet_node_ids, radius, check_collision=check_collision, correct=correct)
    print('xfunc_entry: xfnc error: ', xfunc.error)
    b_struct_data, o_struct_data = xfunc.data
    return b_struct_data, o_struct_data
