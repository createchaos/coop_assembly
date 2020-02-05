import os
import pytest
from itertools import combinations

from compas.datastructures import Network

from coop_assembly.help_functions import find_point_id
from coop_assembly.help_functions import find_point_id, tet_surface_area, \
    tet_volume, distance_point_triangle
from coop_assembly.geometry_generation.tet_sequencing import \
    compute_distance_from_grounded_node
from coop_assembly.geometry_generation.tet_sequencing import \
    get_pt2tri_search_heuristic_fn, \
    point2point_shortest_distance_tet_sequencing, \
    point2triangle_tet_sequencing
from coop_assembly.geometry_generation.execute import execute_from_points
from coop_assembly.assembly_info_generation import calculate_gripping_plane, calculate_offset
from coop_assembly.help_functions.parsing import export_structure_data, parse_saved_structure_data

@pytest.fixture
def save_dir():
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.join(here, 'test_data')

@pytest.mark.gen_from_pts
# @pytest.mark.parametrize('test_set_name', [('single_cube'), ('YJ_12_bars')])
@pytest.mark.parametrize('test_set_name', [('YJ_12_bars')])
@pytest.mark.parametrize('radius', [(3.17), ])
# @pytest.mark.parametrize('pt_search_method', [('point2point'), ])
@pytest.mark.parametrize('pt_search_method', [('point2triangle'), ])
# @pytest.mark.parametrize('pt_search_method', [('point2point'), ('point2triangle')])
def test_generate_from_points(points_library, test_set_name, radius, pt_search_method, save_dir, write):
    points, base_tri_pts = points_library[test_set_name]
    print('\n' + '#'*10)
    print('Testing generate from point for set: {}, total # of pts: {}'.format(test_set_name, len(points)))

    start_tri_ids = [find_point_id(base_pt, points) for base_pt in base_tri_pts]
    assert len(start_tri_ids) == 3, 'start triangle should only have three points!'
    print('base triangle ids: {}'.format(start_tri_ids))

    if pt_search_method == 'point2point':
        cost_from_node = {}
        all_pt_ids = list(range(len(points)))
        elements = list(combinations(all_pt_ids, 2))
        cost_from_node = compute_distance_from_grounded_node(elements, points, start_tri_ids)
        tet_node_ids = point2point_shortest_distance_tet_sequencing(points, cost_from_node)
    elif pt_search_method == 'point2triangle':
        ordering_heuristic = 'tet_surface_area'
        penalty_cost = 2.0
        print('pt search strategy: {} | heuristic: {} | penalty cost: {}'.format(pt_search_method, ordering_heuristic, penalty_cost))
        heuristic_fn = get_pt2tri_search_heuristic_fn(points, penalty_cost, ordering_heuristic)
        tet_node_ids = point2triangle_tet_sequencing(points, start_tri_ids)
    else:
        raise NotImplementedError('search method not implemented!')

    b_struct_data, o_struct_data = execute_from_points(points, tet_node_ids, radius, correct=True, check_collision=True)
    if write:
        export_structure_data(save_dir, b_struct_data, o_struct_data, file_name=test_set_name+'_'+pt_search_method+'.json')

@pytest.mark.gen_grasp_planes
@pytest.mark.parametrize('test_file_name', [('YJ_12_bars_point2triangle.json'),])
def test_gen_grasp_planes(points_library, test_file_name, save_dir):
    b_struct_data, o_struct_data, _ = parse_saved_structure_data(os.path.join(save_dir, test_file_name))

    o_struct = Network.from_data(o_struct_data)
    b_struct = Network.from_data(b_struct_data)
    o_struct.struct_bar = b_struct

    offset_d1, offset_d2 = 5, 5
    nb_rot, nb_trans = 4, 4

    seq = [v for v in b_struct.vertex]
    for v in b_struct.vertex:
        calculate_gripping_plane(b_struct, v, b_struct.vertex[v]["mean_point"], nb_rot=nb_rot, nb_trans=nb_trans)
        calculate_offset(o_struct, b_struct, v, offset_d1, offset_d2, seq)
