
'''
                                                                                                 
    ****       *****       ******       ****       ******  ******          **           **       
   **  **      **  **      **          **  **        **    **              **           **       
   **          *****       ****        ******        **    ****            **   *****   *****    
   **  **      **  **      **          **  **        **    **              **  **  **   **  **   
    ****   **  **  **  **  ******  **  **  **  **    **    ******          **   ******  *****    
                           
                                           
created on 28.06.2019
author: stefanaparascho
'''

from compas.datastructures.network import Network
from compas.geometry.basic import add_vectors, scale_vector, cross_vectors, subtract_vectors, vector_from_points
from compas.geometry.distance import distance_point_point, closest_point_on_plane
from compas.geometry.average import centroid_points

from coop_assembly.help_functions.helpers_geometry import dropped_perpendicular_points


class Overall_Structure(Network):
    """class defining the overall structure in which a node is represented 
    by a network.vertex and a bar by a network.edge does not include 
    connectors - these are referenced through the additional bar_structure class
    
    The Overall_Structure is a second Network structure in which bars are 
    modelled as edges and nodes (points where multiple bars would ideally 
    come together) as vertices. This does not include the geometric information 
    about the bars' actual position or endpoints, but only an idealised 
    point where tetrahedra edges would be located.

    """
    
    def __init__(self, struct_bar):
        super(Overall_Structure, self).__init__()
        self.struct_bar = struct_bar
        self.name = "Network_o"
        self.update_default_vertex_attributes({"name" : "network_o"})
        self.update_default_vertex_attributes({"velocity_vector": (0.0,0.0,0.0)})
        self.update_default_vertex_attributes({"velocities_all": []})
        self.update_default_vertex_attributes({"fixed": False})
        self.update_default_edge_attributes({"name" : "network_o"})
        self.tetrahedra = {}
        self.t_key_max = 0
        
    def add_node(self, xyz = (0.0, 0.0, 0.0), t_key = None):
        v_key = self.add_vertex()
        self.vertex[v_key].update({"x":xyz[0], "y":xyz[1], "z":xyz[2], "point_xyz":xyz})
        if t_key != None:
            if t_key in self.tetrahedra:
                if v_key not in self.tetrahedra[t_key]["vertices"]: self.tetrahedra[t_key]["vertices"].append(v_key)
            else:
                self.tetrahedra.update({t_key:{"vertices":[v_key]}})
        else:
            if self.t_key_max in self.tetrahedra:
                if v_key not in self.tetrahedra[self.t_key_max]["vertices"]: self.tetrahedra[self.t_key_max]["vertices"].append(v_key)
            else:
                self.tetrahedra.update({self.t_key_max:{"vertices":[v_key]}})
            self.t_key_max += 1
            
        return v_key
    
    def add_bar(self, v_key_1, v_key_2, vertex_bar_struct, t_key = None):
        e_keys = self.add_edge(v_key_1, v_key_2)
        self.edge[v_key_1][v_key_2].update({"vertex_bar":vertex_bar_struct})
        if t_key != None:
            if t_key in self.tetrahedra:
                if v_key_1 not in self.tetrahedra[t_key]["vertices"]: self.tetrahedra[t_key]["vertices"].append(v_key_1)
                if v_key_2 not in self.tetrahedra[t_key]["vertices"]: self.tetrahedra[t_key]["vertices"].append(v_key_2)
            else:
                self.tetrahedra.update({t_key:{"vertices":[v_key_1, v_key_2]}})
        else:
            if self.t_key_max in self.tetrahedra:
                if v_key_1 not in self.tetrahedra[self.t_key_max]["vertices"]: self.tetrahedra[self.t_key_max]["vertices"].append(v_key_1)
                if v_key_2 not in self.tetrahedra[self.t_key_max]["vertices"]: self.tetrahedra[self.t_key_max]["vertices"].append(v_key_2)
            else:
                self.tetrahedra.update({self.t_key_max:{"vertices":[v_key_1, v_key_2]}})
        self.struct_bar.vertex[vertex_bar_struct].update({"o_edge":(v_key_1, v_key_2)})
        return e_keys
    
    def calculate_point(self, v_key):
        cons = self.connectors(v_key)
        pts = []
        for con in cons:
            points_1 = self.struct_bar.vertex[con[0]]["axis_endpoints"]
            points_2 = self.struct_bar.vertex[con[1]]["axis_endpoints"]
            dpp = dropped_perpendicular_points(points_1[0], points_1[1], points_2[0], points_2[1])
            pts.append(centroid_points(dpp))
        pt_mean = centroid_points(pts)   
        self.vertex[v_key].update({"x":pt_mean[0], "y":pt_mean[1], "z":pt_mean[2], "point_xyz":pt_mean})
        return pt_mean
        
    def connectors(self, n_key):
        #self.vertex[n_key]
        o_edges = self.vertex_connected_edges(n_key)
        b_vertices = [self.edge[e[0]][e[1]]["vertex_bar"] for e in o_edges]
        
        b_edges = []
        for b_vert in b_vertices:
            b_edges.append(self.struct_bar.vertex_connected_edges(b_vert))
        
        common_e = []
        for i, e_1 in enumerate(b_edges):
            for j, e_2 in enumerate(b_edges):
                if j>i:
                    for e_1_1 in e_1:
                        for e_2_1 in e_2:
                            if (e_1_1[0] == e_2_1[0] and e_1_1[1] == e_2_1[1]) or (e_1_1[0] == e_2_1[1] and e_1_1[1] == e_2_1[0]):
                                common_e.append(e_1_1)
        return common_e
    
    def nodes_vic(self, n_key, dist):
        # find nodes in a certain distance to node n_key
        
        point_base = self.vertex_coordinates(n_key)
        points = [self.vertex_coordinates(v_key) for v_key, args in self.vertices_iter(True)]
        n_key_new = n_key
        nodes_used = []
        nodes_vic = []
        nodes_vic, nodes_used = self.check_distance(n_key_new, nodes_used, nodes_vic, dist, point_base, n_key)
        
        return nodes_vic    

    def check_distance(self, n_key, nodes_used, nodes_vic, dist, point_base, n_base):
        # checks distance for vertices while going through the network structure via neighbours (recursively)
        
        bool_check = True
        for n in self.vertex_neighbours(n_key):
            bool_check_l = True
            p = self.vertex_coordinates(n)
            if n not in nodes_used and n != n_base:
                d = distance_point_point(point_base, p)
                if d < dist:
                    nodes_vic.append(n)
                else:
                    bool_check = False
                    bool_check_l = False
                nodes_used.append(n)
                if bool_check_l: self.check_distance(n, nodes_used, nodes_vic, dist, point_base, n_base)
            if not bool_check: return nodes_vic, nodes_used
             
        return nodes_vic, nodes_used