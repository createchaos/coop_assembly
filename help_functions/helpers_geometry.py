
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
    pts_draw = (add_vectors(pts_draw[0], scale_vector(normalize_vector(vector_from_points(pts_draw[1], pts_draw[0])), ext_len)), add_vectors(
               pts_draw[1], scale_vector(normalize_vector(vector_from_points(pts_draw[0], pts_draw[1])), ext_len)))

    return pts_draw


def check_dir(vec1, vec2):

    ang = angle_vectors(vec1, vec2, deg=True)
    if ang < 90:
        return True
    else:
        return False