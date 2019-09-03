
'''                                         
created on 28.08.2019
'''

import random
import itertools
import math

from compas.geometry.basic import add_vectors, normalize_vector, vector_from_points, scale_vector, cross_vectors, subtract_vectors, length_vector
from compas.geometry.distance import distance_point_point, distance_point_line, distance_line_line
from compas.geometry.transformations import rotate_points
from compas.geometry.angles import angle_vectors
from compas.geometry.average import centroid_points
from compas.geometry import translate_points, rotate_points_xy, Frame

from coop_assembly.geometry_generation.generate_triangles import generate_structure_no_points

def generate_planes_no_glue(b_struct, r, pickup_station_0, mill_station_0):

    frames = []
    for index in b_struct:

        start_position = []
        frames.append(start_position)

        
        # assumed to be centerpoint of gripper at center of bar far end 
        bar_pickup = pickup(b_struct, index, pickup_station_0)
        for frame in bar_pickup:
            frames.append(frame)

        
        # assumed to be centerpoint of gripper at center of mill bit
        mill_notch_1 = mill_path(b_struct, index, mill_station_0, 0, r)
        for frame in mill_notch_1:
            frames.append(frame)

        regrip = bar_regrip(b_struct, index, pickup_station_0)
        for frame in regrip:
            frames.append(frame)

        mill_notch_2 = mill_path(b_struct, index, mill_station_0, 1, r)
        for frame in mill_notch_2:
            frames.append(frame)

        # regrip again? potentially to a better position for placing the bar? how do we want to do this 

        # path to position (tbd)

        # final position
            # offset (stefana?)
                # otherwise find center between notches (normal vector to both connections), rotate around this point
        

        end_position = []


    return frames


def generate_planes_glue(b_struct, r, pickup_station_0, iterations):

    frames = []
    for i in range(6, iterations+1):
        grip = gripping_planes(b_struct, i, r, iterations)

        pickup = pickup_glue(b_struct, i, pickup_station_0, grip)
        for frame in pickup:
            frames.append(frame)
        # print("grip")
        # frames.append("grip")
        frames.append(grip)

    
    return frames


def gripping_planes(b_struct, index, r, iterations, two_robots=False):

    bar_connection = b_struct.vertex[index]["connection_vectors"]
    cp1 = bar_connection[0][0]
    c1 = subtract_vectors(bar_connection[0][1], bar_connection[0][0])
    cp2 = bar_connection[1][0]
    c2 = subtract_vectors(bar_connection[1][1], bar_connection[1][0])


    if index == iterations:
            bar_endpoints = b_struct.vertex[index]["axis_endpoints"]
            gripping_point = centroid_points(bar_endpoints)
    else:
        if two_robots is not False:
            # two robots
            conneciton_pts = [cp1, cp2]
            gripping_point = centroid_points(conneciton_pts)
        else: 
            # one robot
            
            next_index = index+1
            next_bar_connections = b_struct.vertex[next_index]["connection_vectors"]
            third_connection = next_bar_connections[0][1]
            gripping_point = third_connection
            
    
    conenction_middle_angle = add_vectors(c1, c2)

    bar_endpoints = b_struct.vertex[index]["axis_endpoints"]

    multiple_gripping_points = [gripping_point, gripping_point]

    plane_x_vec = subtract_vectors(cp1, gripping_point)
    print(plane_x_vec)
    plane_y_vec = cross_vectors(plane_x_vec, conenction_middle_angle)
    point_xaxis = translate_points(multiple_gripping_points, plane_x_vec)
    point_xyplane = translate_points(multiple_gripping_points, plane_y_vec)

    # gripping_frame = Frame.from_points(gripping_point, point_xaxis[0], point_xyplane[0])
    gripping_frame = [gripping_point, point_xaxis[0], point_xyplane[0]]
    
    b_struct.vertex[index].update({"gripping_plane":gripping_frame})

    return gripping_frame

def pickup_glue(b_struct, index, pickup_station_0, gripping_planes):
#pickup position
    bar_endpoints = b_struct.vertex[index]["axis_endpoints"]
    pickup_position_distance = distance_point_point(bar_endpoints[0], gripping_planes[0])
    pickup_position_translation_vector = scale_vector(normalize_vector(subtract_vectors(gripping_planes[1], gripping_planes[0])), pickup_position_distance)
    pickup_position = translate_points(gripping_planes, pickup_position_translation_vector)

    offset_vector = subtract_vectors([0,0,20], [0,0,0])
    offset_pickup_plane = translate_points(pickup_position, offset_vector)

    b_struct.vertex[index].update({"pickup_position":pickup_position})
    b_struct.vertex[index].update({"pickup_offset":offset_pickup_plane})

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


    return [offset_pickup_plane_1, pickup_plane_1, offset_pickup_plane_1]


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
        angle = -1*angle
    # gives you an anlge relative to the bar to be milled (do this in radians)

    start = [0,0,0]
    end   = [0,200,0]
    rotation_center = centroid_points(start, end)
    mill_path = rotate_points_xy([start, end], angle, rotation_center)
    translation_vector = subtract_vectors(rotation_center, mill_station_0[0])
    mill_path = translate_points(mill_path, translation_vector)

    step_down = subtract_vectors([0,0,0], [0,0, r/3])
    
    # mill path
    translation_to_start = subtract_vectors(mill_path[0], mill_station_0[0])
    translation_from_start_to_end = subtract_vectors(mill_path[1], mill_path[0])
    translation_from_end_to_start = subtract_vectors(mill_path[1], mill_path[0])
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

    b_struct.vertex[index].update({"mill_planes":[mill_1, mill_2, mill_3, mill_4, mill_5, mill_6, mill_7]})

    return mill_1, mill_2, mill_3, mill_4, mill_5, mill_6, mill_7


def bar_regrip(b_struct, index, pickup_station_0):
# rotation and regrip for second notch
    bar_connections = b_struct.vertex[index]["connection_vectors"]
    angle_connections = angle_vectors(bar_connections[0], bar_connections[1])
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
        angle_connections = angle_connections*-1
    else:
        angle_connections = angle_connections

    # need to find a way to decide which direction of rotation is better (the smaller angle faces down), possible crossvectors
    rotation_axis = subtract_vectors(pickup_station_0[1], pickup_station_0[0])
    rotated_pickup_station_1 = rotate_points(pickup_station_0, angle_connections/2, rotation_axis)

    # initial offset plane, and place into pickup station
    offset_vector = subtract_vectors([0,0,200], [0,0,0])
    place_plane_1 = translate_points(rotated_pickup_station_1, bar_notch_location_1)
    offset_plane_1 = translate_points(place_plane_1, offset_vector)

    # after release, back out along tool axis
    back_out_pts = rotate_points(([0,0,200], [0,0,0]), angle_connections/2, rotation_axis)
    back_out = subtract_vectors(back_out_pts[1], back_out_pts[0])
    offset_tool_1 = translate_points(place_plane_1, back_out)

    # move to notch 2 at neutral offset
    rotated_pickup_station_2 = rotate_points(pickup_station_0, -angle_connections/2, rotation_axis)
    pick_plane_2 = translate_points(rotated_pickup_station_2, bar_notch_location_2)
    offset_plane_2 = translate_points(pick_plane_2, offset_vector)

    # offset position, then back in to regrip
    back_in_pts  = rotate_points(([0,0,200], [0,0,0]), -angle_connections/2, rotation_axis)
    back_in      = subtract_vectors(back_in_pts[1], back_in_pts[0])
    offset_tool_2 = translate_points(pick_plane_2, back_in)
    
    b_struct.vertex[index].update({"regrip_planes":[offset_plane_1, place_plane_1, offset_tool_1, offset_plane_1, offset_plane_2, offset_tool_2, pick_plane_2, offset_plane_2]})

    return offset_plane_1, place_plane_1, offset_tool_1, offset_plane_1, offset_plane_2, offset_tool_2, pick_plane_2, offset_plane_2

