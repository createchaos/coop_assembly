
'''

    ****       *****       ******       ****       ******  ******          **           **
   **  **      **  **      **          **  **        **    **              **           **
   **          *****       ****        ******        **    ****            **   *****   *****
   **  **      **  **      **          **  **        **    **              **  **  **   **  **
    ****   **  **  **  **  ******  **  **  **  **    **    ******          **   ******  *****


created on 28.06.2019
author: stefanaparascho
'''

from compas.geometry.basic import normalize_vector, subtract_vectors, cross_vectors, vector_from_points, \
    add_vectors, scale_vector
from compas.geometry.average import centroid_points
from compas.geometry.intersections import intersection_line_plane
from compas.geometry.angles import angle_vectors
from compas.geometry.distance import distance_point_point, distance_point_plane
from compas.geometry.queries import is_point_on_line
from compas.geometry.transformations import project_point_plane, translate_points
from coop_assembly.help_functions.shared_const import EPS


###############################################

def find_point_id(query_pt, pts, tol=EPS):
    ids = []
    for id, pt in enumerate(pts):
        if distance_point_point(query_pt, pt) < tol:
            ids.append(id)
    assert len(ids) == 1, 'duplicated pts!'
    return ids[0]

###############################################

def calculate_coord_sys(end_pts, pt_mean):
    """construct local coordinate system for a line connecting two end_pts.
    Local x axis: along the element from st to end
    Local y axis: center of the element to the given pt_mean

    Parameters
    ----------
    end_pts : list of two points
        start and end point specifying the element
    pt_mean : list of three floats
        A given pt to help constructing the local y axis

    Returns
    -------
    tuple of three lists
        a tuple of three vectors for the local coordinate system
    """
    vec_x = normalize_vector(subtract_vectors(end_pts[1], end_pts[0]))
    vec_n = normalize_vector(subtract_vectors(pt_mean, centroid_points(end_pts)))
    vec_y = normalize_vector(cross_vectors(vec_n, vec_x))
    vec_z = normalize_vector(cross_vectors(vec_x, vec_y))
    return tuple(vec_x), tuple(vec_y), tuple(vec_z)


def calculate_bar_z(points):
    """compute cross product between the vector formed by points and the global z axis.

    """
    vec_x = subtract_vectors(points[1], points[0])
    vec_y_temp = (1,0,0)
    vec_z = cross_vectors(vec_x, vec_y_temp)
    return vec_z


def dropped_perpendicular_points(line_point_1_1, line_point_1_2, line_point_2_1, line_point_2_2):
    """compute the projected tangent point on axis defined by [L1_pt1, L1_pt2] and [L2_pt1, L2_pt2]

    See figure 'perpendicular_bar_tangent_to_two_existing_bars.png' in the docs/images
    or Fig. 3.7. We are computing the point pair (P1, P_{C1}) here, given the axis endpoints of
    bar b_{e1} and b_{n1}

    """
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


def update_bar_lengths(b_struct):
    raise ImportError('Moved to Bar_Structure class function.')


def correct_point(b_struct, o_struct, o_v_key, pt_new, bars_1, bars_2, bars_3):

    pt_n = pt_new
    for i in range(1):
        lins_all    = []
        lin_1   = calc_correction_vector(b_struct, pt_n, bars_1)
        if lin_1 != None:   lins_all.append(lin_1)
        lin_2   = calc_correction_vector(b_struct, pt_n, bars_2)
        if lin_2 != None:   lins_all.append(lin_2)
        lin_3   = calc_correction_vector(b_struct, pt_n, bars_3)
        if lin_3 != None:   lins_all.append(lin_3)
        pt_base_1   = intersection_bars_base(b_struct, bars_1)
        pt_base_2   = intersection_bars_base(b_struct, bars_2)
        pt_base_3   = intersection_bars_base(b_struct, bars_3)

        if lins_all != []:
            if len(lins_all) == 1: pt_n = lins_all[0][1]
            else: pt_n = calculate_new_point(lins_all)

        pt_4    = calc_correction_vector_tip(b_struct, pt_n, pt_base_1, pt_base_2, pt_base_3)
        if pt_4:
            pt_n = pt_4

    if o_v_key:
        o_struct.vertex[o_v_key]["x"], o_struct.vertex[o_v_key]["y"], o_struct.vertex[o_v_key]["z"] = pt_n

    return pt_n


def calc_correction_vector(b_struct, pt_new, bars_1):
    bar_11  = b_struct.vertex[bars_1[0]]
    bar_12  = b_struct.vertex[bars_1[1]]

    pts_int = dropped_perpendicular_points(bar_11["axis_endpoints"][0], bar_11["axis_endpoints"][1], bar_12["axis_endpoints"][0], bar_12["axis_endpoints"][1])

    pt_int  = centroid_points(pts_int)
    vec_x   = normalize_vector(vector_from_points(bar_11["axis_endpoints"][0], bar_11["axis_endpoints"][1]))
    vec_y   = normalize_vector(vector_from_points(bar_12["axis_endpoints"][0], bar_12["axis_endpoints"][1]))
    vec_z   = normalize_vector(cross_vectors(vec_x, vec_y))
    pl_test = (pt_int, vec_z)
    vec_m   = correct_angle(pt_new, pt_int, pl_test)

    return vec_m


def correct_angle(pt_new, pt_int, pl_test):

    pt_proj = project_point_plane(pt_new, pl_test)
    sin_ang = distance_point_point(pt_new, pt_proj)/distance_point_point(pt_new, pt_int)
    if sin_ang < 0.4:
        dist_n  = 0.3 * distance_point_point(pt_new, pt_int)
        pt_m    = add_vectors(pt_proj, scale_vector(normalize_vector(vector_from_points(pt_proj, pt_new)), dist_n))        #vec_m   = vector_from_points(pt_new, pt_m)
        #return vec_m
        lin_c   = (pt_int, pt_m)
        return lin_c
    else:
        return None


def calc_correction_vector_tip(b_struct, pt_new, pt_base_1, pt_base_2, pt_base_3):
    # print("correcting")
    vec_x   = normalize_vector(vector_from_points(pt_base_1, pt_base_2))
    vec_y   = normalize_vector(vector_from_points(pt_base_1, pt_base_3))
    vec_z   = normalize_vector(cross_vectors(vec_x, vec_y))
    pl_test = (pt_base_1, vec_z)
#     pt_int  = centroid_points([pt_base_1, pt_base_2, pt_base_3])
#     vec_m   = correct_angle(pt_new, pt_int, pl_test)
    dist_p  = distance_point_plane(pt_new, pl_test)
    pt_proj = project_point_plane(pt_new, pl_test)

    if dist_p < 80:
        vec_m   = scale_vector(normalize_vector(vector_from_points(pt_proj, pt_new)), 80)
        pt_n    = add_vectors(pt_proj, vec_m)
    else: pt_n = None

    return pt_n


def calculate_new_point(lines):

    pts_all = []
    for i, l1 in enumerate(lines):
        for j, l2 in enumerate(lines):
            if j > i:
                pts = dropped_perpendicular_points(l1[0], l1[1], l2[0], l2[1])
                pts_all.append(centroid_points(pts))
    pt = centroid_points(pts_all)
    return pt


def intersection_bars_base(b_struct, bars_1):

    bar_11  = b_struct.vertex[bars_1[0]]
    bar_12  = b_struct.vertex[bars_1[1]]
    pts_int = dropped_perpendicular_points(bar_11["axis_endpoints"][0], bar_11["axis_endpoints"][1], bar_12["axis_endpoints"][0], bar_12["axis_endpoints"][1])
    pt_fin  = centroid_points(pts_int)

    return pt_fin


def adjust_gripping_plane(pt_bar, pt_new, b_struct, b_v0):

    if distance_point_point(pt_bar[0], pt_bar[1]) > 10000:
    ### hardcoded asjustments for prototype node - to be implemented in overall iteration
    #if b_v0 == 27 or b_v0 == 20 or b_v0 == 12 or b_v0 == 9 or b_v0 == 15 or b_v0 == 24:
        pt_o    = pt_new
        d_1     = distance_point_point(pt_o, pt_bar[0])
        d_2     = distance_point_point(pt_o, pt_bar[1])
        if d_1 < d_2:
            vec_move    = normalize_vector(vector_from_points(pt_bar[1], pt_bar[0]))
        else:
            vec_move    = normalize_vector(vector_from_points(pt_bar[0], pt_bar[1]))
        len_bar = distance_point_point(pt_bar[0], pt_bar[1])
        d_move  = len_bar/5
        # if b_v0 == 12 or b_v0 == 9 or b_v0 == 15 or b_v0 == 24:
        #     d_move = len_bar/7

        gp      = b_struct.vertex[b_v0]["gripping_plane_no_offset"]
        gp_o_n  = translate_points([gp[0]], scale_vector(vec_move, d_move))[0]
        b_struct.vertex[b_v0]["gripping_plane_no_offset"] = (gp_o_n, gp[1], gp[2], gp[3])

    #hard coded gripping plane shifting
    # if b_v0 == 32:
    #     pt_o    = pt_new
    #     d_1     = distance_point_point(pt_o, pt_bar[0])
    #     d_2     = distance_point_point(pt_o, pt_bar[1])
    #     if d_1 < d_2:
    #         vec_move    = normalize_vector(vector_from_points(pt_bar[1], pt_bar[0]))
    #     else:
    #         vec_move    = normalize_vector(vector_from_points(pt_bar[0], pt_bar[1]))
    #     len_bar = distance_point_point(pt_bar[0], pt_bar[1])
    #     d_move  = -len_bar/5
    #     gp      = b_struct.vertex[b_v0]["gripping_plane_no_offset"]
    #     gp_o_n  = translate_points([gp[0]], scale_vector(vec_move, d_move))[0]
    #     b_struct.vertex[b_v0]["gripping_plane_no_offset"] = (gp_o_n, gp[1], gp[2], gp[3])

    # if b_v0 == 35:
    #     vec_move    = normalize_vector(vector_from_points(pt_bar[0], pt_bar[1]))
    #     len_bar = distance_point_point(pt_bar[0], pt_bar[1])
    #     d_move  = -len_bar/4.5
    #     gp      = b_struct.vertex[b_v0]["gripping_plane_no_offset"]
    #     gp_o_n  = translate_points([gp[0]], scale_vector(vec_move, d_move))[0]
    #     b_struct.vertex[b_v0]["gripping_plane_no_offset"] = (gp_o_n, gp[1], gp[2], gp[3])

    # if b_v0 == 37:
    #     vec_move    = normalize_vector(vector_from_points(pt_bar[0], pt_bar[1]))
    #     len_bar = distance_point_point(pt_bar[0], pt_bar[1])
    #     d_move  = -len_bar/5
    #     gp      = b_struct.vertex[b_v0]["gripping_plane_no_offset"]
    #     gp_o_n  = translate_points([gp[0]], scale_vector(vec_move, d_move))[0]
    #     b_struct.vertex[b_v0]["gripping_plane_no_offset"] = (gp_o_n, gp[1], gp[2], gp[3])



def find_bar_ends(b_struct, b, b_key):
    pts_all_b = []
    b_vert_n = b_struct.vertex_neighbors(b_key)
    for n in b_vert_n:
        pts_all_b.append(dropped_perpendicular_points(b["axis_endpoints"][0], b["axis_endpoints"][1], b_struct.vertex[n]["axis_endpoints"][0], b_struct.vertex[n]["axis_endpoints"][1])[0])
    pts_b = find_points_extreme(pts_all_b, b["axis_endpoints"])
    b_struct.vertex[b_key].update({"axis_endpoints":pts_b})
