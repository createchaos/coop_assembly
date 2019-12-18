import pytest
from fixtures.gen_from_points import points_library

def pytest_addoption(parser):
    parser.addoption('--viewer', action='store_true', help='Enables the pybullet viewer')

@pytest.fixture
def viewer(request):
    return request.config.getoption("--viewer")