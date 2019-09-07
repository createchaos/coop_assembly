import pytest

from compas.datastructures import Network
from coop_assembly.data_structure import Overall_Structure, Bar_Structure
from coop_assembly.geometry_generation import execute_from_points

def test_generate_from_points():
    points = [(866.02540378443905, 500.0, 0.0), (0.0, 0.0, 0.0), (0.0, 1000.0, 0.0), (288.67513720801298, 500.0, 818.08450024563103), \
        (769.63370557686096, 1333.0446812766299, 544.74326601771895), (-160.70791990805799, 1388.88716089995, 907.16026675621299), \
        (117.857296010868, 1981.4332041832699, 151.32258105017999), (-798.20078255574094, 1580.5022372901301, 160.91197525036301), \
        (-560.70093861717601, 2300.5484023076901, 812.92987741903903), (-666.65502975497805, 2519.4354019355501, -176.536520678389), \
        (58.742324089034398, 2934.6909001409299, 527.67951271342201)]

    dict_nodes = {'5': [4, 2, 3], '4': [3, 0, 2], '7': [6, 5, 2], \
                  '6': [2, 4, 5], '10': [8, 6, 9], '3': [0, 2, 1], '9': [6, 7, 8], '8': [5, 7, 6]}
    supports_bars = [(0,1), (1,2), (2,0)]
    supports_nodes = [0, 1, 2, 4]
    load_bars = [(4,5)]
    load = (0, 0, -2000)

    b_struct_data, o_struct_data = execute_from_points(
        points, dict_nodes, support_nodes=supports_nodes, 
        support_bars=supports_bars, load_bars=load_bars, load=load)

    # workaround for "reconstructing" classes in GH 
    b_struct = Network.from_data(b_struct_data)
    o_struct = Network.from_data(o_struct_data)