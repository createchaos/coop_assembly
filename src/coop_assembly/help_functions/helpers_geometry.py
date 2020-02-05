
'''

    ****       *****       ******       ****       ******  ******          **           **
   **  **      **  **      **          **  **        **    **              **           **
   **          *****       ****        ******        **    ****            **   *****   *****
   **  **      **  **      **          **  **        **    **              **  **  **   **  **
    ****   **  **  **  **  ******  **  **  **  **    **    ******          **   ******  *****


created on 28.06.2019
author: stefanaparascho
'''

from itertools import combinations

from compas.geometry import Plane
from compas.geometry import normalize_vector, subtract_vectors, cross_vectors, vector_from_points, \
    add_vectors, scale_vector, angle_vectors
from compas.geometry import centroid_points
from compas.geometry import intersection_line_plane
from compas.geometry import is_point_on_line, is_point_infront_plane, is_coplanar
from compas.geometry import project_point_plane, translate_points
from compas.geometry import distance_point_point, distance_point_line, distance_point_plane, \
    area_triangle, volume_polyhedron

from coop_assembly.help_functions.shared_const import EPS, NODE_CORRECTION_TOP_DISTANCE, NODE_CORRECTION_SINE_ANGLE


###############################################

def find_point_id(query_pt, pts, tol=EPS):
    ids = []
    for id, pt in enumerate(pts):
        if distance_point_point(query_pt, pt) < tol:
            ids.append(id)
    assert len(ids) == 1, 'duplicated pts!'
    return ids[0]

def distance_point_triangle(point, tri_end_pts):
    lines = [(pt_1, pt_2) for pt_1, pt_2 in combinations(tri_end_pts, 2)]
    return sum([distance_point_line(point, line) for line in lines])

def tet_surface_area(tet_end_points):
    assert len(tet_end_points) == 4, 'input points must be four!'
    faces = [face_pts for face_pts in combinations(tet_end_points, 3)]
    return sum([area_triangle(face_pts) for face_pts in faces])

def tet_from_points(tet_end_points):
    assert len(tet_end_points) == 4, 'input points must be four!'
    ids = frozenset([0,1,2,3])
    faces = []
    if is_coplanar(tet_end_points):
        return None, None
    for face in combinations(ids, 3):
        left_id, = ids - set(face)
        if is_point_infront_plane(tet_end_points[left_id], Plane.from_three_points(*[tet_end_points[i] for i in face])):
            face = [face[0], face[2], face[1]]
        faces.append(list(face) + [left_id])
    return tet_end_points, faces

def tet_volume(tet_end_points):
    assert len(tet_end_points) == 4, 'input points must be four!'
    verts, faces = tet_from_points(tet_end_points)
    if verts and faces:
        return volume_polyhedron((verts, faces))
    else:
        return None

###############################################

def Frame_to_plane_data(frame):
    data = frame.to_data()
    return (data['point'], data['xaxis'], data['yaxis'], cross_vectors(data['xaxis'], data['yaxis']))

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

    Parameters
    ----------
    points : [type]
        [description]

    Returns
    -------
    [type]
        [description]
    """
    vec_x = subtract_vectors(points[1], points[0])
    vec_y_temp = (1,0,0)
    vec_z = cross_vectors(vec_x, vec_y_temp)
    return vec_z


def dropped_perpendicular_points(line_point_1_1, line_point_1_2, line_point_2_1, line_point_2_2):
    """compute the projected tangent point on axis defined by [L1_pt1, L1_pt2] and [L2_pt1, L2_pt2]

    See Fig. 3.7 in SP's dissertaion (below). We are computing the point pair (P1, P_{C1}) here, given the axis endpoints of bar b_{e1} and b_{n1}

    .. image:: ../images/perpendicular_bar_tangent_to_two_existing_bars.png
        :scale: 80 %
        :alt: perpendicular_bar_tangent_to_two_existing_bars
        :align: center

    Parameters
    ----------
    line_point_1_1 : [type]
        [description]
    line_point_1_2 : [type]
        [description]
    line_point_2_1 : [type]
        [description]
    line_point_2_2 : [type]
        [description]

    Returns
    -------
    list of two points
       representing the contact line segment between the two axes
    """
    line_unity_vector_1 = normalize_vector(vector_from_points(line_point_1_1, line_point_1_2))
    line_unity_vector_2 = normalize_vector(vector_from_points(line_point_2_1, line_point_2_2))
    d_vector = cross_vectors(line_unity_vector_1, line_unity_vector_2)

    normal_1 = cross_vectors(line_unity_vector_1, d_vector)
    normal_2 = cross_vectors(line_unity_vector_2, d_vector)
    plane_1 = (line_point_1_1, normal_1)
    plane_2 = (line_point_2_1, normal_2)
    line_1_dp_point = intersection_line_plane((line_point_1_1, line_point_1_2), plane_2)
    line_2_dp_point = intersection_line_plane((line_point_2_1, line_point_2_2), plane_1)

    return [line_1_dp_point, line_2_dp_point]


def find_points_extreme(pts_all, pts_init):
    """update a bar's axis end point based on all the contact projected points specified in `pts_all`

    Parameters
    ----------
    pts_all : list of points
        all the contact points projected on the axis (specified by pts_init)
    pts_init : list of two points
        the initial axis end points

    Returns
    -------
    [type]
        [description]
    """
    vec_init = normalize_vector(vector_from_points(*pts_init))
    # * find the pair of points with maximal distance
    sorted_pt_pairs = sorted(combinations(pts_all, 2), key=lambda pt_pair: distance_point_point(*pt_pair))
    pts_draw = sorted_pt_pairs[-1]

    vec_new = normalize_vector(vector_from_points(*pts_draw))
    if angle_vectors(vec_init, vec_new, deg=True) > 90:
        # angle can only be 0 or 180
        pts_draw = pts_draw[::-1]

    # ext_len = 30
    # pts_draw = (add_vectors(pts_draw[0], scale_vector(normalize_vector(vector_from_points(pts_draw[1], pts_draw[0])), ext_len)), add_vectors(pts_draw[1], scale_vector(normalize_vector(vector_from_points(pts_draw[0], pts_draw[1])), ext_len)))

    return pts_draw


def check_dir(vec1, vec2):
    ang = angle_vectors(vec1, vec2, deg=True)
    if ang < 90:
        return True
    else:
        return False


def update_bar_lengths(b_struct):
    raise ImportError('Moved to Bar_Structure class function.')

###################################################

def correct_point(b_struct, o_struct, pt_new, bar_pairs, o_v_key=None):
    """vertex position correction following an angle/distance heuristic to improve the success rate of a feasible three-bar group.
        Performing vertex correction wrt individual bar bases (fig.1 below) and three-bar base (fig.2 below)
        See SP dissertation 3.1.3.f. (p.80)

    .. image:: ../images/vertex_correction_to_base.png
        :scale: 80 %
        :align: center

    .. image:: ../images/vertex_correction_to_top_connection.png
        :scale: 80 %
        :align: center

    Parameters
    ----------
    b_struct : [type]
        [description]
    o_struct : [type]
        [description]
    pt_new : [type]
        [description]
    bars_1 : list of two-int tuples
        BarS vertex tuple pairs, representing three pairs of touching bars.
    o_v_key : int, optional
        if specified, o_struct's corresponding vertex pt will be updated, by default None

    Returns
    -------
    [type]
        [description]
    """
    # correction of node position in regards to the base bars (angle)
    lins_all    = []
    for bar_pair in bar_pairs:
        new_base2vert_line = calc_correction_vector(b_struct, pt_new, bar_pair)
        if new_base2vert_line is not None:
            lins_all.append(new_base2vert_line)
    if len(lins_all) > 0:
        pt_new = calculate_new_point(lins_all)

    # correction of node position in regards to the top connection (distance)
    base_pts = [intersection_bars_base(b_struct, bar_pair) for bar_pair in bar_pairs]
    pt_4 = calc_correction_vector_tip(pt_new, base_pts)
    if pt_4:
        pt_new = pt_4

    if o_v_key:
        print('correct_point: OverallS #{} vertex position updated.'.format(o_v_key))
        o_struct.vertex[o_v_key]["x"], o_struct.vertex[o_v_key]["y"], o_struct.vertex[o_v_key]["z"] = pt_new

    return pt_new


def calc_correction_vector(b_struct, pt_new, bar_pair):
    """Computing correction vector to meet the angle threshold.
        return vector P-P_c (figure below).

    .. image:: ../images/vertex_correction_to_base.png
        :scale: 80 %
        :align: center

    Parameters
    ----------
    b_struct : [type]
        [description]
    pt_new : [type]
        [description]
    bar_pair : list of two int
        BarS vertex key, representing two bars

    Returns
    -------
    list of two points
        return None if feasible (bigger than the angle threshold), otherwise return the line connecting pt_int and modified pt
    """
    bar1  = b_struct.vertex[bar_pair[0]]
    bar2  = b_struct.vertex[bar_pair[1]]
    pt_int = intersection_bars_base(b_struct, bar_pair)

    vec_x   = normalize_vector(vector_from_points(bar1["axis_endpoints"][0], bar1["axis_endpoints"][1]))
    vec_y   = normalize_vector(vector_from_points(bar2["axis_endpoints"][0], bar2["axis_endpoints"][1]))
    # contact vector
    vec_z   = normalize_vector(cross_vectors(vec_x, vec_y))
    # test plane
    pl_test = (pt_int, vec_z)
    vec_m   = correct_angle(pt_new, pt_int, pl_test)
    return vec_m


def correct_angle(pt_new, pt_int, pl_test):
    """Computing correction vector to meet the angle threshold.
        return vector P-P_c (figure below).

    .. image:: ../images/vertex_correction_to_base.png
        :scale: 80 %
        :align: center

    Parameters
    ----------
    pt_new : point
        new point P
    pt_int : point
        contact point between two bars, i.e. point N in the image above
    pl_test : tuple (point, vector)
        the grey plane shown in the image above

    Returns
    -------
    tuple of two points
        return None if feasible (bigger than the angle threshold), otherwise return the line connecting pt_int and modified pt
    """

    pt_proj = project_point_plane(pt_new, pl_test)
    sin_ang = distance_point_point(pt_new, pt_proj)/distance_point_point(pt_new, pt_int)
    if sin_ang < NODE_CORRECTION_SINE_ANGLE:
        # ? why 0.3 here?
        # length of the triangle's hypotenuse
        # dist_n = 0.3 * distance_point_point(pt_new, pt_int)
        dist_n = NODE_CORRECTION_SINE_ANGLE * distance_point_point(pt_new, pt_int)
        # modified point Pc
        pt_m = add_vectors(pt_proj, scale_vector(normalize_vector(vector_from_points(pt_proj, pt_new)), dist_n))
        lin_c = (pt_int, pt_m)
        return lin_c
    else:
        return None


def calc_correction_vector_tip(pt_new, base_pts):
    """Computing correction vector to meet the distance threshold.
        return vector P-P_c (figure below).

    .. image:: ../images/vertex_correction_to_top_connection.png
        :scale: 80 %
        :align: center

    Parameters
    ----------
    pt_new : [type]
        [description]
    base_pts : list of three points
        contact base points for the base bars
    """
    assert len(base_pts) == 3
    vec_x   = normalize_vector(vector_from_points(base_pts[0], base_pts[1]))
    vec_y   = normalize_vector(vector_from_points(base_pts[0], base_pts[2]))
    vec_z   = normalize_vector(cross_vectors(vec_x, vec_y))
    pl_test = (base_pts[0], vec_z)
    dist_p  = distance_point_plane(pt_new, pl_test)
    pt_proj = project_point_plane(pt_new, pl_test)

    if dist_p < NODE_CORRECTION_TOP_DISTANCE:
        vec_m = scale_vector(normalize_vector(vector_from_points(pt_proj, pt_new)), NODE_CORRECTION_TOP_DISTANCE)
        pt_n = add_vectors(pt_proj, vec_m)
    else:
        pt_n = None
    return pt_n


def calculate_new_point(lines):
    """calculate average point of the contact points between the given list of lines.

    Parameters
    ----------
    lines : list of (pt_0, pt_1)
        list of lines, each line: (bar contact (base) point, new vertex point)

    Returns
    -------
    point
        point coordinate
    """
    if len(lines) == 1:
        return lines[0][1]
    pts_all = []
    for l1, l2 in combinations(lines, 2):
        pts = dropped_perpendicular_points(l1[0], l1[1], l2[0], l2[1])
        pts_all.append(centroid_points(pts))
    pt = centroid_points(pts_all)
    return pt


def intersection_bars_base(b_struct, bar_pair):
    """find contact point between two connected bars
        Find point N, given two bars b_e1 and b_e2 (see figure below).

    .. image:: ../images/vertex_correction_to_base.png
        :scale: 80 %
        :align: center

    Parameters
    ----------
    b_struct : [type]
        [description]
    bar_pair : list of two int
        BarS vertex keys, representing two bars.

    Returns
    -------
    list of three float
        point N
    """
    bar1 = b_struct.vertex[bar_pair[0]]
    bar2  = b_struct.vertex[bar_pair[1]]
    pts_int = dropped_perpendicular_points(bar1["axis_endpoints"][0], bar1["axis_endpoints"][1],
                                           bar2["axis_endpoints"][0], bar2["axis_endpoints"][1])
    return centroid_points(pts_int)

###################################################

def find_bar_ends(b_struct, b_key):
    """Update bar's end points according to the contact points

    Parameters
    ----------
    b_struct : [type]
        [description]
    b_key : int
        BarStructure vertex key
    """
    bar = b_struct.vertex[b_key]
    bar_all_contact_pts = []
    b_vert_n = b_struct.vertex_neighbors(b_key)
    for neighbor_bar_key in b_vert_n:
        bar_contact_pt, _ = dropped_perpendicular_points(bar["axis_endpoints"][0],
                                                         bar["axis_endpoints"][1],
                                                         b_struct.vertex[neighbor_bar_key]["axis_endpoints"][0],
                                                         b_struct.vertex[neighbor_bar_key]["axis_endpoints"][1])
        bar_all_contact_pts.append(bar_contact_pt)
    bar_end_pts_new = find_points_extreme(bar_all_contact_pts, bar["axis_endpoints"])
    b_struct.vertex[b_key].update({"axis_endpoints" : bar_end_pts_new})
