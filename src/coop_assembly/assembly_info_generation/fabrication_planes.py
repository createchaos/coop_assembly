
'''                                         
created on 28.08.2019
'''

import random
import itertools
import math

from compas.geometry.basic import add_vectors, normalize_vector, vector_from_points, scale_vector, cross_vectors, subtract_vectors, length_vector, length_vector
from compas.geometry.distance import distance_point_point, distance_point_line, distance_line_line
from compas.geometry.transformations import rotate_points
from compas.geometry.angles import angle_vectors
from compas.geometry.average import centroid_points
from compas.geometry import translate_points, rotate_points_xy, Frame

from coop_assembly.help_functions.helpers_geometry import calculate_bar_z, calculate_coord_sys

def generate_planes_no_glue(b_struct, r, iterations, pickup_station_0, mill_station_0, start_position, end_position):

    frames = []
    for i in range(6, iterations+1):

        frames.append(start_position)

        # assumed to be centerpoint of gripper at center of bar far end 
        bar_pickup = pickup(b_struct, i, pickup_station_0)
        for frame in bar_pickup:
            frames.append(frame)

        # assumed to be centerpoint of gripper at center of mill bit
        mill_notch_1 = mill_path(b_struct, i, mill_station_0, 0, r)
        for frame in mill_notch_1:
            frames.append(frame)

        regrip = bar_regrip(b_struct, i, pickup_station_0)
        for frame in regrip:
            frames.append(frame)

        mill_notch_2 = mill_path(b_struct, i, mill_station_0, 1, r)
        for frame in mill_notch_2:
            frames.append(frame)

        # regrip again? potentially to a better position for placing the bar? how do we want to do this 
        place_planes = b_struct.vertex[i]["final_regrip_place"]
        for frame in place_planes:
            frames.append(frame)

        final_position = gripping_planes(b_struct, i, r, iterations, two_robots=True)
        final_pickup = pickup_glue(b_struct, i, pickup_station_0, final_position)
        for frame in final_pickup:
            frames.append(frame)

        # path to position (tbd)

        frames.append(final_position)

        frames.append(end_position)


    return frames

# things to fix:  pickup rotation angle, why is the starting position already rotated

def generate_planes_glue(b_struct, r, pickup_station_0, iterations, start_position, end_position):

    frames = []
    for i in range(iterations+1):
        grip = gripping_planes(b_struct, i, r, iterations)

        frames.append(start_position)

        pickup = pickup_glue(b_struct, i, pickup_station_0, grip)
        for frame in pickup:
            frames.append(frame)
        # print("grip")
        # frames.append("grip")
        frames.append(grip)

        frames.append(end_position)

    
    return frames

def gripping_planes(b_struct, index, r, iterations, two_robots=False):

    multiple_planes = True
    nb = 8

    if index == iterations or index <= 5:
        bar_endpoints = b_struct.vertex[index]["axis_endpoints"]
        gripping_point = centroid_points(bar_endpoints)
        bar_z = calculate_bar_z(bar_endpoints)
        # temporary subs
        cp1 = bar_endpoints[0]
        c1 = cross_vectors(subtract_vectors(bar_endpoints[1], bar_endpoints[0]), bar_z)
        c2 = cross_vectors(subtract_vectors(bar_endpoints[1], bar_endpoints[0]), bar_z)

    else:
        bar_connection = b_struct.vertex[index]["connection_vectors"]
        cp1 = bar_connection[0][1]
        c1 = subtract_vectors(bar_connection[0][1], bar_connection[0][0])
        cp2 = bar_connection[1][1]
        c2 = subtract_vectors(bar_connection[1][1], bar_connection[1][0])
        if two_robots is not False:
            # two robots
            conneciton_pts = [cp1, cp2]
            gripping_point = centroid_points(conneciton_pts)
            bar_endpoints = b_struct.vertex[index]["axis_endpoints"]
            bar_z = calculate_bar_z(bar_endpoints)
        else: 
            # one robot
            next_index = index+1
            next_bar_connections = b_struct.vertex[next_index]["connection_vectors"]
            third_connection = next_bar_connections[0][0]
            bar_z = subtract_vectors(third_connection, next_bar_connections[0][1])
            gripping_point = third_connection
            

    # temporarily generic z for no
    # gripping_angle = cross_vectors(c1, subtract_vectors(bar_endpoints[1], bar_endpoints[0]))

    multiple_gripping_points = [gripping_point, gripping_point]

    plane_x_vec = subtract_vectors(cp1, gripping_point)
    print(plane_x_vec)
    plane_y_vec = cross_vectors(plane_x_vec, bar_z)
    point_xaxis = translate_points(multiple_gripping_points, plane_x_vec)
    point_xyplane = translate_points(multiple_gripping_points, plane_y_vec)

    # gripping_frame = Frame.from_points(gripping_point, point_xaxis[0], point_xyplane[0])
    gripping_frame = [gripping_point, point_xaxis[0], point_xyplane[0]]

    if multiple_planes:
        ang = math.radians(360/nb)
        frames_all = []
        for n in range(nb):
            frame_n = rotate_points(gripping_frame, angle=n*ang, axis=subtract_vectors(bar_endpoints[1], bar_endpoints[0]), origin=bar_endpoints[0])
            frames_all.append(frame_n)
        b_struct.vertex[index].update({"gripping_planes_all":frames_all})
    
    b_struct.vertex[index].update({"gripping_plane":gripping_frame})
    print("gripping frame", gripping_frame)

    return gripping_frame


def calculate_gripping_plane(b_struct, v, pt_mean, nb_rot=8, nb_trans=8, planes_rot=True, planes_trans=True, planes_flip=True):

    end_pts_0 = b_struct.vertex[v]["axis_endpoints"]
    vec_x, vec_y, vec_z = calculate_coord_sys(end_pts_0, pt_mean)
    pt_o        = centroid_points(end_pts_0)

    b_struct.vertex[v].update({"gripping_plane":(pt_o, vec_x, vec_y, vec_z)})
    gripping_plane = b_struct.vertex[v]["gripping_plane"]

    frames_all = []
    
    if planes_trans == True:
        vec_bar = scale_vector(normalize_vector(subtract_vectors(end_pts_0[1], end_pts_0[0])), 30)
        pt1 = add_vectors(end_pts_0[0], vec_bar)
        vec_bar = scale_vector(vec_bar, -1)
        pt2 = add_vectors(end_pts_0[1], vec_bar)
        vec_n = subtract_vectors(pt2, pt1)
        len_vec = length_vector(vec_n)
        len_new = len_vec/(nb_trans-1)

        for i in range(nb_trans):
            origin = add_vectors(pt1, scale_vector(normalize_vector(vec_n), len_new*i))
            frame_n = [origin, gripping_plane[1], gripping_plane[2]]
            if planes_rot == False:
                frames_all.append(frame_n)
                if planes_flip == True:
                    frame_n = [frame_n[0], scale_vector(frame_n[1], -1), scale_vector(frame_n[2], -1)]
                    frames_all.append(frame_n)

            if planes_rot == True:
                ang = math.radians(360/nb_rot)
                for n in range(nb_rot):
                    gripping_plane = frame_n
                    vecs_n = rotate_points([gripping_plane[1], gripping_plane[2]], angle=n*ang, axis=subtract_vectors(end_pts_0[1], end_pts_0[0]), origin=(0,0,0))
                    frame_n = [gripping_plane[0], vecs_n[0], vecs_n[1]]
                    frames_all.append(frame_n)
                    
                    if planes_flip == True:
                        frame_n = [frame_n[0], scale_vector(frame_n[1], -1), scale_vector(frame_n[2], -1)]
                        frames_all.append(frame_n)

    elif planes_rot == True:
        ang = math.radians(360/nb_rot)
        for n in range(nb_rot):
            vecs_n = rotate_points([gripping_plane[1], gripping_plane[2]], angle=n*ang, axis=subtract_vectors(end_pts_0[1], end_pts_0[0]), origin=(0,0,0))
            frame_n = [gripping_plane[0], vecs_n[0], vecs_n[1]]
            frames_all.append(frame_n)
            
            if planes_flip == True:
                frame_n = [frame_n[0], scale_vector(frame_n[1], -1), scale_vector(frame_n[2], -1)]
                frames_all.append(frame_n)
    
    b_struct.vertex[v].update({"gripping_planes_all":frames_all})


def pickup_glue(b_struct, index, pickup_station_0, gripping_planes):
    #pickup position
    bar_endpoints = b_struct.vertex[index]["axis_endpoints"]
    pickup_position_distance = distance_point_point(bar_endpoints[0], gripping_planes[0])
    pickup_position_translation_vector = scale_vector(normalize_vector(subtract_vectors(pickup_station_0[0], pickup_station_0[1])), pickup_position_distance)
    pickup_position = translate_points(pickup_station_0, pickup_position_translation_vector)

    offset_vector = subtract_vectors([0,0,200], [0,0,0])
    offset_pickup_plane = translate_points(pickup_position, offset_vector)

    b_struct.vertex[index].update({"pickup_position": [offset_pickup_plane, pickup_position, offset_pickup_plane]})

    return offset_pickup_plane, pickup_position, offset_pickup_plane
    
def pickup(b_struct, index, pickup_station_0):
    # pickup station
    bar_endpoints = b_struct.vertex[index]["axis_endpoints"]
    bar_notch = b_struct.vertex[index]["connection_vectors"]
    bar_notch_distance_1 = length_vector(subtract_vectors(bar_notch[0][0], bar_endpoints[0]))
    offset_vector = subtract_vectors([0,0,200], [0,0,0])
    bar_notch_location_1 = subtract_vectors([bar_notch_distance_1, 0, 0], [0,0,0])

    pickup_plane_1 = translate_points(pickup_station_0, bar_notch_location_1)
    offset_pickup_plane_1 = translate_points(pickup_plane_1, offset_vector)

    b_struct.vertex[index].update({"pickup_planes":[offset_pickup_plane_1, pickup_plane_1, offset_pickup_plane_1]})


    return offset_pickup_plane_1, pickup_plane_1, offset_pickup_plane_1

def mill_path(b_struct, index, mill_station_0, bar_0_or_1, r):
    # bar to be milled
    bar_endpoints_0 = b_struct.vertex[index]["axis_endpoints"]
    bar_0 = subtract_vectors(bar_endpoints_0[1], bar_endpoints_0[0])

    # connected bars
    connected_bars = b_struct.vertex[index]["connected_bars"]
    bar_endpoints_1 = b_struct.vertex[connected_bars[bar_0_or_1]]["axis_endpoints"]
    bar_1 = subtract_vectors(bar_endpoints_1[0], bar_endpoints_1[1])

    # angle between bars (checks for vector directions)
    angle_1 = angle_vectors(bar_0, bar_1)
    cross_vec = normalize_vector(cross_vectors(bar_0, bar_1))
    connection_vector_pts = b_struct.vertex[index]["connection_vectors"]
    connection_vector = subtract_vectors(connection_vector_pts[bar_0_or_1][1], connection_vector_pts[bar_0_or_1][0])
    direction_check = add_vectors(cross_vec, connection_vector)
    if length_vector(direction_check) > length_vector(connection_vector):
        angle = angle_1
    else:
        angle = -1*angle_1
    # gives you an anlge relative to the bar to be milled (do this in radians)

    start = [0,0,0]
    end   = [0,200,0]
    start_end = [start, end]
    rotation_center = centroid_points(start_end)
    mill_path = rotate_points_xy([start, end], angle, rotation_center)
    translation_vector = subtract_vectors(mill_station_0[0], rotation_center)
    mill_path = translate_points(mill_path, translation_vector)

    step_down = subtract_vectors([0,0,0], [0,0, r/3])
    
    # mill path
    translation_to_start = subtract_vectors(mill_path[0], mill_station_0[0])
    translation_from_start_to_end = subtract_vectors(mill_path[1], mill_path[0])
    translation_from_end_to_start = subtract_vectors(mill_path[0], mill_path[1])
    final_offset_translation = subtract_vectors([0,0,200], [0,0,0])
    # starting position
    mill_1 = translate_points(mill_station_0, translation_to_start)
    # first pass
    mill_2 = translate_points(mill_1, translation_from_start_to_end)
    # step down
    mill_3 = translate_points(mill_2, step_down)
    # second pass
    mill_4 = translate_points(mill_3, translation_from_end_to_start)
    # step down
    mill_5 = translate_points(mill_4, step_down)
    # third pass
    mill_6 = translate_points(mill_5, translation_from_start_to_end)
    # offset up
    mill_7 = translate_points(mill_6, final_offset_translation)

    if bar_0_or_1 == 0:
        b_struct.vertex[index].update({"mill_planes_0":[mill_1, mill_2, mill_3, mill_4, mill_5, mill_6, mill_7]})
    else:
        b_struct.vertex[index].update({"mill_planes_1":[mill_1, mill_2, mill_3, mill_4, mill_5, mill_6, mill_7]})


    return mill_1, mill_2, mill_3, mill_4, mill_5, mill_6, mill_7

def bar_regrip(b_struct, index, pickup_station_0):  
    # rotation and regrip for second notch
    bar_connections = b_struct.vertex[index]["connection_vectors"]
    bc1 = subtract_vectors(bar_connections[0][1], bar_connections[0][0])
    bc2 = subtract_vectors(bar_connections[1][1], bar_connections[1][0])
    angle_connections = angle_vectors(bc1, bc2)
    bar_endpoints = b_struct.vertex[index]["axis_endpoints"]
    bar_notch_distance_1 = length_vector(subtract_vectors(bar_connections[0][0], bar_endpoints[0]))
    bar_notch_distance_2 = length_vector(subtract_vectors(bar_connections[1][0], bar_endpoints[0]))

    bar_notch_location_1 = subtract_vectors([bar_notch_distance_1, 0, 0], [0,0,0])
    bar_notch_location_2 = subtract_vectors([bar_notch_distance_2, 0, 0], [0,0,0])

    # rotates gripper around an axis (possibly needs to change), to 1/2 the difference between connection angles
    cross_vec = normalize_vector(cross_vectors(subtract_vectors(bar_connections[0][1], bar_connections[0][0]), subtract_vectors(bar_connections[1][1], bar_connections[1][0])))
    bar_vector = subtract_vectors(bar_endpoints[1], bar_endpoints[0])
    direction_check = add_vectors(bar_vector, cross_vec)

    if length_vector(direction_check) > length_vector(bar_vector):
        angle_connections = 2*3.14159-(angle_connections*-1)

    else:
        angle_connections = 2*3.14159-(angle_connections*1)

    print("pickup_station",pickup_station_0)

    # need to find a way to decide which direction of rotation is better (the smaller angle faces down), possible crossvectors
    rotation_axis = normalize_vector(subtract_vectors(pickup_station_0[1], pickup_station_0[0]))
    rotated_pickup_station_1 = rotate_points(pickup_station_0, angle_connections/2, axis = rotation_axis, origin= pickup_station_0[0])

    # initial offset plane, and place into pickup station
    offset_vector = subtract_vectors([0,0,200], [0,0,0])
    place_plane_1 = translate_points(rotated_pickup_station_1, bar_notch_location_1)
    offset_plane_1 = translate_points(place_plane_1, offset_vector)

    # after release, back out along tool axis
    back_out_pts = rotate_points(([0,0,200], [0,0,0]), angle_connections/2, axis = rotation_axis, origin= pickup_station_0[0])
    back_out = subtract_vectors(back_out_pts[1], back_out_pts[0])
    offset_tool_1 = translate_points(place_plane_1, back_out)

    # move to notch 2 at neutral offset
    rotated_pickup_station_2 = rotate_points(pickup_station_0, -angle_connections/2, axis= rotation_axis, origin= pickup_station_0[0])
    pick_plane_2 = translate_points(rotated_pickup_station_2, bar_notch_location_2)
    offset_plane_2 = translate_points(pick_plane_2, offset_vector)

    # offset position, then back in to regrip
    back_in_pts  = rotate_points(([0,0,200], [0,0,0]), -angle_connections/2, axis = rotation_axis, origin= pickup_station_0[0])
    back_in      = subtract_vectors(back_in_pts[1], back_in_pts[0])
    offset_tool_2 = translate_points(pick_plane_2, back_in)
    
    b_struct.vertex[index].update({"regrip_planes":[offset_plane_1, place_plane_1, offset_tool_1, offset_plane_1, offset_plane_2, offset_tool_2, pick_plane_2, offset_plane_2]})
    b_struct.vertex[index].update({"final_regrip_place":[offset_plane_2, pick_plane_2, offset_tool_2]})

    return offset_plane_1, place_plane_1, offset_tool_1, offset_plane_1, offset_plane_2, offset_tool_2, pick_plane_2, offset_plane_2