import heapq
from collections import defaultdict
from itertools import combinations
import random
from random import shuffle
import time

from compas.geometry import distance_point_point, distance_point_line, distance_point_plane
from compas.geometry import is_coplanar

TOL = 1e-4
INF = 1e10

def adjacent_from_edges(edges):
    """get a dict for graph connectivity {node_id : neighnor node ids}
    
    Parameters
    ----------
    edges : a list of 2-int-tuples
        [(node_1, node_2), ...]
    
    Returns
    -------
    undirected_edges : dict
        {node_id : neighnor node ids}
    """
    undirected_edges = defaultdict(set)
    for v1, v2 in edges:
        undirected_edges[v1].add(v2)
        undirected_edges[v2].add(v1)
    return undirected_edges

def compute_distance_from_grounded_node(elements, node_points, ground_node_ids):
    """Use the dijkstra algorithm to compute the graph distance from each node to the grounded nodes.
    Graph edge cost is set to be the Euclidean distance between two vertices.
    
    Parameters
    ----------
    elements : list of 2-int-tuples
        [(node_1, node_2), ...]
    node_points : list of points
        [[x,y,z], ...]
    ground_node_ids : list of int
        grounded node indices
    
    Returns
    -------
    cost_from_node : dict
        {node_id (int) : cost (float)}
    """
    neighbors = adjacent_from_edges(elements)
    edge_costs = {edge: distance_point_point(node_points[edge[0]], node_points[edge[1]])
                  for edge in elements}
    # undirected edges
    edge_costs.update({edge[::-1]: distance for edge, distance in edge_costs.items()})

    cost_from_node = {}
    queue = []
    for node in ground_node_ids:
        cost = 0
        cost_from_node[node] = cost
        heapq.heappush(queue, (cost, node))

    while queue:
        cost1, node1 = heapq.heappop(queue)
        if cost_from_node[node1] < cost1:
            continue
        for node2 in neighbors[node1]:
            cost2 = cost1 + edge_costs[node1, node2]
            if cost2 < cost_from_node.get(node2, INF): # if no key found, return INF
                cost_from_node[node2] = cost2
                heapq.heappush(queue, (cost2, node2))
    return cost_from_node

def find_point_id(query_pt, pts, tol=TOL):
    ids = []
    for id, pt in enumerate(pts):
        if query_pt.DistanceTo(pt) < tol:
            ids.append(id)
    assert len(ids) == 1, 'duplicated pts!'
    return ids[0]

def point2point_shortest_distance_tet_sequencing(points, cost_from_node):
    """generate a tet sequence by adding node one by one based on cost_from_node (ascending order)
    The base triangle for each node is chosen based on its Euclidean distance to the already added nodes (closest three).
    
    Parameters
    ----------
    points : a list of 3-float lists
        [description]
    cost_from_node : dict
        {node_id : cost}
    
    Returns
    -------
    tet_node_ids : list of 4-int lists
        [[base_tri node_id_0, base_tri node_id_1, base_tri node_id_2, newly added node_id], ...]
    """
    # TODO: add a penalty cost for coplanar nodes?
    added_pt_ids = []
    tet_node_ids = []
    for pt_id, score in sorted(cost_from_node.items(), key=lambda item: item[1]):
        # if pt_id not in start_tri_ids and len(added_pt_ids) >= 3:
        if len(added_pt_ids) >= 3:
            dists = [points[pt_id].DistanceTo(points[prev_node]) for prev_node in added_pt_ids]
            # use distance to previous nodes to sort previous nodes' indices
            sorted_added_ids = [added_id for dist, added_id in sorted(zip(dists, added_pt_ids)) \
                if added_id != pt_id]
            assert len(sorted_added_ids) >= 3
            # take the top three nodes
            closest_node_ids = sorted_added_ids[0:3]
            tet_node_ids.append(closest_node_ids + [pt_id])
        assert pt_id not in added_pt_ids, 'pt_id: {} already in previously added pts: {}'.format(
            pt_id, added_pt_ids)
        added_pt_ids.append(pt_id)
    return tet_node_ids

#######################################

def distance_point_triangle(point, tri_end_pts):
    lines = [(pt_1, pt_2) for pt_1, pt_2 in combinations(tri_end_pts, 2)]
    return sum([distance_point_line(point, line) for line in lines])

def compute_candidate_nodes(all_nodes, grounded_nodes, built_nodes):
    if grounded_nodes <= built_nodes:
        nodes = built_nodes | set(grounded_nodes) # union
    else:
        # build grounded ones first
        nodes = grounded_nodes
    return {node for node in set(nodes) - set(built_nodes)}

def add_successors(queue, all_nodes, grounded_nodes, heuristic_fn, built_nodes, built_triangles):
    remaining = all_nodes - built_nodes
    num_remaining = len(remaining) - 1
    assert 0 <= num_remaining
    for node_id in shuffle(compute_candidate_nodes(all_nodes, grounded_nodes, built_nodes)):
        # compute bias
        bias, tri_node_ids = heuristic_fn(built_nodes, node_id, built_triangles)
        priority = (num_remaining, bias, random.random())
        heapq.heappush(queue, (priority, built_nodes, built_triangles, node_id, tri_node_ids))

def get_heuristic_fn(points):
    def h_fn(built_nodes, node_id, built_triangles):
        # iterate through all existing triangles and return the minimal distance
        # TODO: this is wasteful...
        dist_to_tri = [distance_point_triangle(points[node_id], [points[tri_id] for tri_id in tri]) for tri in list(built_triangles)]
        sort_tris = sorted(zip(dist_to_tri, built_triangles))
        return sort_tris[0]
    return h_fn

def retrace_tet_sequence(visited, next_built_nodes):
    # recursive plan retrace
    pass

def point2triangle_shortest_distance_tet_sequencing(points, ground_nodes):
    all_nodes = frozenset(list(range(len(points))))
    assert len(ground_nodes) == 3, 'the grounded nodes need to form a triangle.'
    heuristic_fn = get_heuristic_fn(points)

    initial_built_nodes = frozenset(ground_nodes)
    initial_built_triangles = set(frozenset(ground_nodes))
    queue = []
    visited = [initial_built_nodes]
    add_successors(queue, all_nodes, ground_nodes, heuristic_fn, initial_built_nodes, initial_built_triangles)

    min_remaining = len(points)
    num_evaluated = max_backtrack = 0
    max_time = 10
    start_time = time.time()
    while queue and (time.time() - start_time < max_time) :
        num_evaluated += 1
        bias, built_nodes, built_triangles, node_id, tri_node_ids = heapq.heappop(queue)
        num_remaining = len(all_nodes) - len(built_nodes)
        num_evaluated += 1
        if num_remaining < min_remaining:
            min_remaining = num_remaining

        print('Iteration: {} | Best: {} | Built: {}/{} | Node: {} | Triangle: {}'.format(
            num_evaluated, min_remaining, len(built_nodes), len(all_nodes), node_id, tri_node_ids))
        next_built_nodes = built_nodes | {node_id}
        if all_nodes <= next_built_nodes:
            min_remaining = 0
            plan = retrace_tet_sequence(visited, next_built_nodes)
            break

        add_successors(queue, all_nodes, ground_nodes, heuristic_fn, built_nodes, built_triangles)
    return plan