import os
import pytest

from coop_assembly.choreo_interface import sequenced_picknplace_plan

@pytest.mark.choreo_call
def test_choreo_seq_plan():
    here = os.path.dirname(os.path.abspath(__file__))
    # pkg_dir = 'C:\\Users\\yijiangh\\Desktop\\choreo_instances'
    pkg_dir = os.path.join(here, 'choreo_instances')

    pkg_name = 'dms_ws_tet_bars'
    assembly_json_path = os.path.join(pkg_dir, pkg_name, 'json', pkg_name + '.json')
    sequenced_picknplace_plan(assembly_json_path, scale=0.001, enable_viewer=False)