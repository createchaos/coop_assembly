
'''
                                                                                                 
    ****       *****       ******       ****       ******  ******          **           **       
   **  **      **  **      **          **  **        **    **              **           **       
   **          *****       ****        ******        **    ****            **   *****   *****    
   **  **      **  **      **          **  **        **    **              **  **  **   **  **   
    ****   **  **  **  **  ******  **  **  **  **    **    ******          **   ******  *****    
                           
                                           
created on 28.06.2019
author: stefanaparascho
'''

from compas.geometry.basic import normalize_vector, subtract_vectors, cross_vectors, vector_from_points
from compas.geometry.average import centroid_points
from compas.geometry.intersections import intersection_line_plane


def calculate_coord_sys(end_pts, pt_mean):

    vec_x = normalize_vector(subtract_vectors(end_pts[1], end_pts[0]))
    vec_n = normalize_vector(subtract_vectors(pt_mean, centroid_points((end_pts))))
    vec_y = normalize_vector(cross_vectors(vec_n, vec_x))
    vec_z = normalize_vector(cross_vectors(vec_x, vec_y))

    return tuple(vec_x), tuple(vec_y), tuple(vec_z)


def calculate_bar_z(points):

    vec_x = subtract_vectors(points[1], points[0])
    vec_y_temp = (1,0,0)
    vec_z = cross_vectors(vec_x, vec_y_temp)

    return vec_z


def dropped_perpendicular_points(line_point_1_1, line_point_1_2, line_point_2_1, line_point_2_2):
    
    line_unity_vector_1 = normalize_vector(vector_from_points(line_point_1_1, line_point_1_2))
    line_unity_vector_2 = normalize_vector(vector_from_points(line_point_2_1, line_point_2_2))
    d_vector = cross_vectors(line_unity_vector_1, line_unity_vector_2)

    normal_1 = cross_vectors(line_unity_vector_1, d_vector)
    normal_2 = cross_vectors(line_unity_vector_2, d_vector)
    plane_1 = (line_point_1_1, normal_1)
    plane_2 = (line_point_2_1, normal_2)

    dp_point_line_1 = intersection_line_plane((line_point_1_1, line_point_1_2), plane_2)
    dp_point_line_2 = intersection_line_plane((line_point_2_1, line_point_2_2), plane_1)

    return [dp_point_line_1, dp_point_line_2]