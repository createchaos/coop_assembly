from compas.datastructures.network import Network


class Bar_Structure(Network):
    """[summary]

    The Bar_Structure is some sort of an "inverted" network. It contains bars as vertices 
    and the connections between bars as edges, these include the geometric information of 
    each bar (endpoints) and their connecting points. 
    However, this does not include information about which bars form a tetrahedron or which 
    bars come together within a larger node, they only have information about where two bars 
    are connected to one another
    
    """
    # crosec_type     = "rectangle" / "tube" / "circle"
    # crosec_values = "rectangle" : (width, height) - height = dimension in z-axis direction
    #                                                            "tube"   : (outer diameter, thickness)
    #                                                            "circle"    : (diameter)
    def __init__(self):
        super(Bar_Structure, self).__init__()
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
                                   "zdir":_zdir, "supports":{}, 
                                   "loads":{}, 
                                   "bar_parameters":_bar_parameters, 
                                   "exchange_values":{}})
        return v_key
    
    def add_bar_support(self, v_key, _point, _dof=(-1,-1,-1,0,0,0), _support_type=0):
        self.vertex[v_key]["supports"].update({self.support_point_max_key+1:{"point":_point,"dof":_dof,"type":_support_type}})
        self.support_point_max_key += 1
        return self.support_point_max_key
    
    def add_bar_load(self, v_key, _point, _load_force, _load_moment=(0.0,0.0,0.0)):
        self.vertex[v_key]["loads"].update({self.__load_point_max_key+1:{"point":_point,"force":_load_force,"moment":_load_moment}})
        self.__load_point_max_key += 1
        return self.__load_point_max_key
    
    def connect_bars(self, v_key1, v_key2, _endpoints=[], _connection_type=0, _connection_parameters=[]):
        if self.has_edge(v_key1, v_key2):
            id = self.edge[v_key1][v_key2]["connections_count"]
            self.edge[v_key1][v_key2]["endpoints"].update( {id:_endpoints} )
            self.edge[v_key1][v_key2]["connection_type"].update( {id:_connection_type} )
            self.edge[v_key1][v_key2]["connection_parameters"].update( {id:_connection_parameters} )
            self.edge[v_key1][v_key2]["exchange_values"].update( {id:{}} )
            id += 1
            self.edge[v_key1][v_key2]["connections_count"] = id
        else:
            self.add_edge(v_key1, v_key2, {"connections_count":1, "endpoints":{0:_endpoints}, "connection_type":{0:_connection_type}, "connection_parameters":{0:_connection_parameters}, "exchange_values":{0:{}}})
        return (v_key1, v_key2)