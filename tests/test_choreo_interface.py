import os
import pytest

from coop_assembly.choreo_interface import sequenced_picknplace_plan

@pytest.mark.choreo_call
# @pytest.mark.parametrize('solve_method', [('ladder_graph'), ('sparse_ladder_graph')])
@pytest.mark.parametrize('solve_method', [('sparse_ladder_graph')])
def test_choreo_seq_plan(viewer, solve_method):
    here = os.path.dirname(os.path.abspath(__file__))
    test_data_dir = os.path.join(here, 'test_data')

    pkg_name = 'dms_ws_tet_bars'
    assembly_json_path = os.path.join(test_data_dir, pkg_name, 'json', pkg_name + '.json')
    sequenced_picknplace_plan(assembly_json_path, solve_method=solve_method, scale=1e-3, viewer=viewer)
