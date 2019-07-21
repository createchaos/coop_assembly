
'''
                                                                                                 
    ****       *****       ******       ****       ******  ******          **           **       
   **  **      **  **      **          **  **        **    **              **           **       
   **          *****       ****        ******        **    ****            **   *****   *****    
   **  **      **  **      **          **  **        **    **              **  **  **   **  **   
    ****   **  **  **  **  ******  **  **  **  **    **    ******          **   ******  *****    
                           
                                           
created on 28.06.2019
author: stefanaparascho
'''

from compas.geometry.basic import normalize_vector, subtract_vectors, cross_vectors, vector_from_points, add_vectors, scale_vector
from compas.geometry.average import centroid_points
from compas.geometry.intersections import intersection_line_plane
from compas.geometry.angles import angle_vectors
from compas.geometry.distance import distance_point_point
from compas.geometry.queries import is_point_on_line


def calculate_coord_sys(end_pts, pt_mean):

    vec_x = normalize_vector(subtract_vectors(end_pts[1], end_pts[0]))
    # print("end points", end_pts)
    vec_n = normalize_vector(subtract_vectors(pt_mean, centroid_points(end_pts)))
    vec_y = normalize_vector(cross_vectors(vec_n, vec_x))
    vec_z = normalize_vector(cross_vectors(vec_x, vec_y))

    return tuple(vec_x), tuple(vec_y), tuple(vec_z)


def calculate_bar_z(points):

    vec_x = subtract_vectors(points[1], points[0])
    vec_y_temp = (1,0,0)
    vec_z = cross_vectors(vec_x, vec_y_temp)

    return vec_z


def dropped_perpendicular_points(line_point_1_1, line_point_1_2, line_point_2_1, line_point_2_2):
    
    # print("dpp input check", line_point_1_1, line_point_1_2, line_point_2_1, line_point_2_2)
    line_unity_vector_1 = normalize_vector(vector_from_points(line_point_1_1, line_point_1_2))
    line_unity_vector_2 = normalize_vector(vector_from_points(line_point_2_1, line_point_2_2))
    d_vector = cross_vectors(line_unity_vector_1, line_unity_vector_2)

    normal_1 = cross_vectors(line_unity_vector_1, d_vector)
    normal_2 = cross_vectors(line_unity_vector_2, d_vector)
    plane_1 = (line_point_1_1, normal_1)
    plane_2 = (line_point_2_1, normal_2)
    # print("dpp second", line_point_1_1, line_point_1_2, plane_2)
    # print("dpp second", line_point_2_1, line_point_2_2, plane_1)
    dp_point_line_1 = intersection_line_plane((line_point_1_1, line_point_1_2), plane_2)
    # print("return", dp_point_line_1)
    dp_point_line_2 = intersection_line_plane((line_point_2_1, line_point_2_2), plane_1)
    # print("return", dp_point_line_2)

    return [dp_point_line_1, dp_point_line_2]


def find_points_extreme(pts_all, pts_init):
    vec_init = normalize_vector(vector_from_points(pts_init[0], pts_init[1]))
    distances = []
    dic_dist = {}
    for pt in pts_all:
        for pt1 in pts_all:
            if pt != pt1:
                distances.append(distance_point_point(pt, pt1))
                dic_dist[distances[-1]] = [pt, pt1]
    max_dist = max(distances)
    pts_draw = dic_dist[max_dist]
    vec_new = normalize_vector(vector_from_points(pts_draw[0], pts_draw[1]))
    ang_test = angle_vectors(vec_init, vec_new, deg=True)
    if ang_test > 90:
        pts_draw = [pts_draw[1], pts_draw[0]]
    
    ext_len = 30
    # pts_draw = (add_vectors(pts_draw[0], scale_vector(normalize_vector(vector_from_points(pts_draw[1], pts_draw[0])), ext_len)), add_vectors(
    #            pts_draw[1], scale_vector(normalize_vector(vector_from_points(pts_draw[0], pts_draw[1])), ext_len)))

    return pts_draw


def check_dir(vec1, vec2):

    ang = angle_vectors(vec1, vec2, deg=True)
    if ang < 90:
        return True
    else:
        return False


# to move to bar structure 
def update_bar_lengths(b_struct):

    # print("input update", b_struct.vertex[0]["axis_endpoints"], b_struct.vertex[1]["axis_endpoints"], b_struct.vertex[2]["axis_endpoints"])
    for b in b_struct.vertex:
        edges_con = b_struct.vertex_connected_edges(b)
        list_pts = []
        for e, f in edges_con:
            dpp = dropped_perpendicular_points(b_struct.vertex[e]["axis_endpoints"][0], b_struct.vertex[e]["axis_endpoints"][1], b_struct.vertex[f]["axis_endpoints"][0], b_struct.vertex[f]["axis_endpoints"][1])
            b_struct.edge[e][f]["endpoints"][0] = dpp
            points = b_struct.edge[e][f]["endpoints"]
            # points2 = b_struct.edge[f][e]["endpoints"]
            # points = points + points2
            # print(points)
            # if points != []:
            for p in points.keys():
                pair_points = points[p]
                # print("pair points", pair_points)
                if pair_points !=[]:
                    for pt in pair_points:
                        # print("input",  b_struct.vertex[b]["axis_endpoints"])
                        bool_online = is_point_on_line(pt, b_struct.vertex[b]["axis_endpoints"], 0.1)
                        if bool_online:
                            # print("yes!", pt)
                            list_pts.append(pt)
                        # else:
                            # print("no!", pt)

        # print("list_points", list_pts)
        if list_pts != []:
            # list_pts.append(b_struct.vertex[b]["axis_endpoints"][0])
            # list_pts.append(b_struct.vertex[b]["axis_endpoints"][1])
            if len(list_pts) > 2:
                pts_extr = find_points_extreme(list_pts, b_struct.vertex[b]["axis_endpoints"])
            else:
                pts_extr = list_pts
            # print("point extr", pts_extr)
            b_struct.vertex[b].update({"axis_endpoints":pts_extr})



