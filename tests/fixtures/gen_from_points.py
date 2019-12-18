import pytest

@pytest.fixture
def points_library():
    # test_set_name : (all_points, base_triangle_points)
    point_lib = {
    'single_cube' : ([[52, 0, 0],
                      [52, 52, 0],
                      [52, 52, 52],
                      [0, 52, 52],
                      [0, 52, 0],
                      [0, 0, 0],
                      [52, 0, 52],
                      [0, 0, 52]],
                     [[52, 0, 0],
                      [0, 52, 0],
                      [0, 0, 0]]),
    }
    return point_lib