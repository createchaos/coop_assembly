'''

    ****       *****       ******       ****       ******  ******          **           **
   **  **      **  **      **          **  **        **    **              **           **
   **          *****       ****        ******        **    ****            **   *****   *****
   **  **      **  **      **          **  **        **    **              **  **  **   **  **
    ****   **  **  **  **  ******  **  **  **  **    **    ******          **   ******  *****


created on 28.06.2019
author: stefanaparascho

edited on 17.12.2019 by Yijiang Huang, yijiangh@mit.edu
'''

from compas.datastructures.network import Network
from compas.geometry import is_point_on_line
from coop_assembly.help_functions.helpers_geometry import dropped_perpendicular_points, find_points_extreme


class BarStructure(Network):
    """[summary]

    SP:
    The Bar_Structure is some sort of an "inverted" network.
    # ! It contains bars as vertices and the connections between bars as edges,
    # ! these include the geometric information of each bar (endpoints) and their connecting points.
    However, this does not include information about which bars form a tetrahedron or which
    bars come together within a larger node, they only have information about where two bars
    are connected to one another

    SP dissertation section 3.5.2:
    One bar may be connected to multiple other bars, whereas one welded joint can only bridge two bars.
    The vertices describe the bars, each of which can have multiple joints.
    The edges describe the joints between pairs of bars.
    `BarStructure` includes geometric information about the bars endpoints and the joint positions in the
    space.

    This model is referred as *base data model*.

    """
    # crosec_type     = "rectangle" / "tube" / "circle"
    # crosec_values = "rectangle" : (width, height) - height = dimension in z-axis direction
    #                                                            "tube"   : (outer diameter, thickness)
    #                                                            "circle"    : (diameter)
    def __init__(self):
        super(BarStructure, self).__init__()
        self.support_point_max_key   = 0
        self.__load_point_max_key    = 0
        self.__connector_max_key     = 0
        self.name = "Network_b"

    def add_bar(self, _bar_type, _axis_endpoints, _crosec_type, _crosec_values, _zdir, _bar_parameters=[]):
        v_key = self.add_vertex()
        self.vertex[v_key].update({"bar_type":_bar_type,
                                   "axis_endpoints":_axis_endpoints,
                                   "crosec_type":_crosec_type,
                                   "crosec_values":_crosec_values,
                                   "zdir":_zdir,
                                   "supports":{}, # not used now
                                   "loads":{},    # not used now
                                   "bar_parameters":_bar_parameters,
                                   "exchange_values":{}})
        return v_key

    def connect_bars(self, v_key1, v_key2, _endpoints=[], _connection_type=0, _connection_parameters=[]):
        if self.has_edge(v_key1, v_key2):
            # edge exists already, updating edge attributes
            id = self.edge[v_key1][v_key2]["connections_count"]
            self.edge[v_key1][v_key2]["endpoints"].update( {id : _endpoints} )
            self.edge[v_key1][v_key2]["connection_type"].update( {id : _connection_type} )
            self.edge[v_key1][v_key2]["connection_parameters"].update( {id : _connection_parameters} )
            self.edge[v_key1][v_key2]["exchange_values"].update( {id : {}} )
            id += 1
            self.edge[v_key1][v_key2]["connections_count"] = id
        else:
            # create an new edge
            self.add_edge(v_key1, v_key2, {"connections_count":1,
                                           "endpoints":{0:_endpoints},
                                           "connection_type":{0:_connection_type},
                                           "connection_parameters":{0:_connection_parameters},
                                           "exchange_values":{0:{}}})
        return (v_key1, v_key2)

    def update_bar_lengths(self):
        """[summary]
        """
        for b in self.vertex:
            edges_con = self.vertex_connected_edges(b)
            list_pts = []
            for e, f in edges_con:
                dpp = dropped_perpendicular_points(self.vertex[e]["axis_endpoints"][0],
                                                   self.vertex[e]["axis_endpoints"][1],
                                                   self.vertex[f]["axis_endpoints"][0],
                                                   self.vertex[f]["axis_endpoints"][1])
                self.edge[e][f]["endpoints"][0] = dpp
                points = self.edge[e][f]["endpoints"]
                for p in points.keys():
                    pair_points = points[p]
                    if pair_points !=[]:
                        for pt in pair_points:
                            bool_online = is_point_on_line(pt, self.vertex[b]["axis_endpoints"], 0.1)
                            if bool_online:
                                list_pts.append(pt)

            if len(list_pts) > 0:
                if len(list_pts) > 2:
                    pts_extr = find_points_extreme(list_pts, self.vertex[b]["axis_endpoints"])
                else:
                    pts_extr = list_pts
                self.vertex[b].update({"axis_endpoints":pts_extr})

    # def add_bar_support(self, v_key, _point, _dof=(-1,-1,-1,0,0,0), _support_type=0):
    #     self.vertex[v_key]["supports"].update({self.support_point_max_key+1:{"point":_point,"dof":_dof,"type":_support_type}})
    #     self.support_point_max_key += 1
    #     return self.support_point_max_key

    # def add_bar_load(self, v_key, _point, _load_force, _load_moment=(0.0,0.0,0.0)):
    #     self.vertex[v_key]["loads"].update({self.__load_point_max_key+1:{"point":_point,"force":_load_force,"moment":_load_moment}})
    #     self.__load_point_max_key += 1
    #     return self.__load_point_max_key
