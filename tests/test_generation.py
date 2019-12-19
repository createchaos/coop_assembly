import pytest
from itertools import combinations
from termcolor import cprint

from coop_assembly.help_functions import find_point_id
from coop_assembly.geometry_generation import compute_distance_from_grounded_node
from coop_assembly.geometry_generation import point2point_shortest_distance_tet_sequencing
from coop_assembly.geometry_generation import point2triangle_tet_sequencing
from coop_assembly.geometry_generation import execute_from_points

@pytest.mark.gen_from_pts
@pytest.mark.parametrize('test_set_name', [('single_cube'), ])
@pytest.mark.parametrize('radius', [(3.17), ])
# @pytest.mark.parametrize('pt_search_method', [('point2point'), ])
# @pytest.mark.parametrize('pt_search_method', [('point2triangle'), ])
@pytest.mark.parametrize('pt_search_method', [('point2point'), ('point2triangle')])
def test_generate_from_points(points_library, test_set_name, radius, pt_search_method):
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
        tet_node_ids = point2triangle_tet_sequencing(points, start_tri_ids)
    else:
        raise NotImplementedError('search method not implemented!')

    b_struct_data, o_struct_data = execute_from_points(points, tet_node_ids, radius, correct=True)
