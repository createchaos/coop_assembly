
'''
                                                                                                 
    ****       *****       ******       ****      ******  ******          **           **       
   **  **      **  **      **          **  **       **    **              **           **       
   **          *****       ****        ******       **    ****            **   *****   *****    
   **  **      **  **      **          **  **       **    **              **  **  **   **  **   
    ****   **  **  **  **  ******  **  **  **  **   **    ******          **   ******  *****    
                           
                                           
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
from compas.geometry import translate_points

from coop_assembly.geometry_generation.generate_triangles import generate_structure_no_points

def generate_planes(b_struct):
    pass
    

# starting position

def pickup(b_struct, index, pickup_station_0):
# pickup station
    bar_endpoints = b_struct.vertex[index]["axis_endpoints"]
    bar_notch = b_struct.vertex[index]["connection_vectors"]
    bar_notch_location_1 = subtract_vectors(bar_notch[0], bar_endpoints[0])
    bar_notch_location_2 = subtract_vectors(bar_notch[0], bar_endpoints[0])
    offset_vector = [tbd]

    pickup_plane_1 = translate_points(pickup_station_0, bar_notch_location_1)
    offset_pickup_plane_1 = translate_points(pickup_plane_1, offset_vector)

    # pickup position
        # distance of first notch from end of pickup station
            # distance along axis of bar from a vertex endpoint to the connection vector
        #  offset
            # hard-coded offset from pickup
    return [offset_pickup_plane_1, pickup_plane_1, offset_pickup_plane_1]

# mill path
    # notch angle from dictionary (follow the axis of bar connection)
        # start
        # end
        # step
        # repeat

# rotation
    # pickup station offset (do along tool coordinate sysetm)
        # at rotation of -1/2 angle_vectors(connection 1, connection 2) 
    # place
    # offset (allong tool coordinate system, aka backing out)
    # neutral offset (above)?
    # rotation + 1/2 angle_vectors
        # at distance along axis of bar from the same vertex endpoint to the connection vector

# mill path

# regrip again? potentially to a better position for placing the bar? how do we want to do this 

# path to position (tbd)

# final position
    # offset (stefana?)
        # otherwise find center between notches (normal vector to both connections), rotate around this point

# end position

