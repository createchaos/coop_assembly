
'''
                                                                                                 
    ****       *****       ******       ****      ******  ******          **           **       
   **  **      **  **      **          **  **       **    **              **           **       
   **          *****       ****        ******       **    ****            **   *****   *****    
   **  **      **  **      **          **  **       **    **              **  **  **   **  **   
    ****   **  **  **  **  ******  **  **  **  **   **    ******          **   ******  *****    
                           
                                           
created on 30.06.2019
author: stefanaparascho
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

from coop_assembly.help_functions.helpers_geometry import calculate_coord_sys, calculate_bar_z, dropped_perpendicular_points, update_bar_lengths
from coop_assembly.help_functions.tangents import tangent_from_point, check_length_sol_one, tangent_through_two_points, check_colisions
from coop_assembly.geometry_generation.generate_tetrahedra import add_tetra


def generate_first_tetra(o_struct, b_struct, r, points = None):
# generates tetrahedron base structure
    pt_0  = points[0]
    pt_1  = points[1]
    pt_2  = points[2]
    pt_3  = points[3]

    vec_0   = normalize_vector(vector_from_points(pt_0, pt_1))
    vec_1   = normalize_vector(vector_from_points(pt_1, pt_2))
    vec_2   = normalize_vector(vector_from_points(pt_2, pt_0))

    c_0     = scale_vector(normalize_vector(cross_vectors(vec_0, vec_2)), 2*r)
    c_1     = scale_vector(normalize_vector(cross_vectors(vec_1, vec_0)), 2*r)
    c_2     = scale_vector(normalize_vector(cross_vectors(vec_2, vec_1)), 2*r)

    end_pts_0   = (pt_0, add_vectors(pt_1, c_1))
    end_pts_1   = (pt_1, add_vectors(pt_2, c_2))
    end_pts_2   = (pt_2, add_vectors(pt_0, c_0))

    vec_0   = normalize_vector(vector_from_points(end_pts_0[0], end_pts_0[1]))
    vec_1   = normalize_vector(vector_from_points(end_pts_1[0], end_pts_1[1]))
    vec_2   = normalize_vector(vector_from_points(end_pts_2[0], end_pts_2[1]))

    pt_int = centroid_points((end_pts_0[0], end_pts_0[1], end_pts_1[0], end_pts_1[1], end_pts_2[0], end_pts_2[1]))

    vec_x_0, vec_y_0, vec_z_0 = calculate_coord_sys(end_pts_0, pt_int)
    vec_x_1, vec_y_1, vec_z_1 = calculate_coord_sys(end_pts_1, pt_int)
    vec_x_2, vec_y_2, vec_z_2 = calculate_coord_sys(end_pts_2, pt_int)


    vec_z_0     = calculate_bar_z(end_pts_0)
    vec_z_1     = calculate_bar_z(end_pts_1)
    vec_z_2     = calculate_bar_z(end_pts_2)


    b_v0    = b_struct.add_bar(0, end_pts_0, "tube", (25.0, 2.0), vec_z_0)
    b_v1    = b_struct.add_bar(0, end_pts_1, "tube", (25.0, 2.0), vec_z_1)
    b_v2    = b_struct.add_bar(0, end_pts_2, "tube", (25.0, 2.0), vec_z_2)

    b_struct.connect_bars(b_v0, b_v1)
    b_struct.connect_bars(b_v1, b_v2)
    b_struct.connect_bars(b_v2, b_v0)


    add_tetra(o_struct, b_struct, (b_v0, b_v1), (b_v1, b_v2), (b_v2, b_v0), pt_3, r)



    return b_struct, o_struct



def generate_structure(o_struct, b_struct, bool_draw, r, points = None, supports=None, loads=None, correct=True):

    if points: 
        iterations = len(points)
    else:
        iterations = 1
    print("iterations", iterations)
        
    # repeat for each point
    for i in range(iterations):

        # pick two bars - ??
        bars_rnd  = [i*2, i*2+1]

        # identify input parameters for tangent generation function
        bar1        = b_struct.vertex[bars_rnd[0]]["axis_endpoints"]
        bp1         = bar1[0]
        lv1         = subtract_vectors(bar1[1], bar1[0])
        bar2        = b_struct.vertex[bars_rnd[1]]["axis_endpoints"]
        bp2         = bar2[0]
        lv2         = subtract_vectors(bar2[1], bar2[0])
        dist1       = r
        dist2       = r

        # if points:
        pt_test = points[i]
        # else:
        #     if i == 0: 
        #         pt_test     = (300,300,1000)
        #     else:
        #         pt_test = (1200, 500, 700)

        rp          = pt_test

        # calculate tangent
        bar_index   = add_tangent(b_struct, bp1, lv1, bp2, lv2, rp, dist1, dist2, bars_rnd)

        # pick 2 new bars - last one added and another one
        bars_rnd = [bar_index]
        bars_rnd.append(bar_index-1)

        # move input point outside of the last bar's volume
        vec_move = normalize_vector(b_struct.vertex[bar_index]["zdir"])
        vec_axis = normalize_vector(subtract_vectors(b_struct.vertex[bar_index]["axis_endpoints"][1], b_struct.vertex[bar_index]["axis_endpoints"][0]))
        ang_rnd = math.radians(180)
        vec_new = scale_vector(rotate_points([vec_move], ang_rnd, vec_axis)[0], 30)

        rp          = (rp[0] + vec_new[0], rp[1] + vec_new[1], rp[2] + vec_new[2])

        # identify input parameters for tangent generation
        bar1        = b_struct.vertex[bars_rnd[0]]["axis_endpoints"]
        bp1         = bar1[0]
        lv1         = subtract_vectors(bar1[1], bar1[0])

        bar2        = b_struct.vertex[bars_rnd[1]]["axis_endpoints"]
        bp2         = bar2[0]
        lv2         = subtract_vectors(bar2[1], bar2[0])
        dist1       = r
        dist2       = r

        # calculate second tangent
        bar_index   = add_tangent(b_struct, bp1, lv1, bp2, lv2, rp, dist1, dist2, bars_rnd)

        # update lengths of bars of entire structure
        update_bar_lengths(b_struct)
        

    return b_struct

def generate_structure_no_points(o_struct, b_struct, bool_draw, r, iterations, supports=None, loads=None, correct=True):
    
    max_bar_length = 1200
    
    radius1 = r
    radius2 = r

    adj_con = []
    
    # repeat for each bar added to structure
    for i in range(iterations):
        rps = None
        breaker = 0
        while rps is None:
            breaker += 1
            if breaker >= 30:
                print("break_1")
                break

            rp1, rp2, lv1, lv2, bp1, bp2, bar1, bar2, bars_ind = bar_selection(b_struct, max_bar_length, i)

            #  if random bar selected is too far from bar 1, repick random bar
            dll = distance_line_line(bar1, bar2)
            breaker_2 = 0
            while dll > max_bar_length-200:
                breaker_2 +=1
                if breaker_2 > 20:
                    print("break_2")
                    break
                rp1, rp2, lv1, lv2, bp1, bp2, bar1, bar2, bars_ind = bar_selection(b_struct, max_bar_length, i)
                dll =  distance_line_line(bar1, bar2)

            #  checks to make sure the angle between the axis of two connecting bars satisfies a given condition and for tangency issues, repicks points to try for solution
            rps = master_checker(lv1, lv2, rp1, rp2, bar1, bar2, max_bar_length, i, r)
            print("rps1", rps)


            if rps is not None:
                rp1 = rps[0]
                rp2 = rps[1]
                print("test")
                bar_added = add_tangent_no_points(b_struct, bp1, lv1, bp2, lv2, rp1, rp2, radius1, radius2, bars_ind, adj_con, i)
                j = 0
                while bar_added is None:
                    print("bar_added_is_none")
                    j += 1
                    if j == 40:
                        break
                    else:
                        rp1, rp2 = point_selection(bar1, lv1, bar2, lv2, max_bar_length, i)
                        rps = master_checker(lv1, lv2, rp1, rp2, bar1, bar2, max_bar_length, i, r)
                        if rps is not None:
                            rp1 = rps[0]
                            rp2 = rps[1]
                            bar_added = add_tangent_no_points(b_struct, bp1, lv1, bp2, lv2, rp1, rp2, radius1, radius2, bars_ind, adj_con, i)
            if bar_added is None:
                rps = None


    return b_struct


def bar_selection(b_struct, max_bar_length, i):
# selects previous bar added, random second bar, and reference points on each
    bars_ind = []
    if i == 0:
        bar_added = 5
    bars_ind.append(i+5)
    print("bars_ind2", bars_ind)
    bar1 = b_struct.vertex[bars_ind[0]]["axis_endpoints"]
    bp1  = bar1[0]
    lv1  = subtract_vectors(bar1[1], bar1[0])
    second_bar_key = second_bar_rnd(bars_ind[0])
    bars_ind.append(second_bar_key)
    bar2 = b_struct.vertex[bars_ind[1]]["axis_endpoints"]
    bp2  = bar2[0]
    lv2  = subtract_vectors(bar2[1], bar2[0])

    rp1, rp2 = point_selection(bar1, lv1, bar2, lv2, max_bar_length, i)

    return rp1, rp2, lv1, lv2, bp1, bp2, bar1, bar2, bars_ind

def point_selection(bar1, lv1, bar2, lv2, max_bar_length, i):
# select reference points on bar1 and bar2
    if i == 0:
        rp1  = point_on_bar_2(bar1, lv1)
    else:
        rp1  = point_on_bar_1(bar1, lv1, max_bar_length)
    rp2  = point_on_bar_2(bar2, lv2)
    print("repick points")
    return rp1, rp2

def master_checker(lv1, lv2, rp1, rp2, bar1, bar2, max_bar_length, i, r):
# completes all required checks and re-tries with new reference points a fixed number of times
    bca = bar_connection_angle(lv1, lv2, rp1, rp2, bar1, bar2, max_bar_length, i)
    b_ckr = bar_checker(i, bar1, lv1, rp1, bar2, lv2, rp2, max_bar_length, r)
    h = 0
    print(bca, b_ckr)    
    while bca is False or b_ckr is False:
        h += 1
        if h > 10:
            return None
        else:
            rp1, rp2 = point_selection(bar1, lv1, bar2, lv2, max_bar_length, i)
            print("rp11111", rp1)
            bca = bar_connection_angle(lv1, lv2, rp1, rp2, bar1, bar2, max_bar_length, i)
            b_ckr = bar_checker(i, bar1, lv1, rp1, bar2, lv2, rp2, max_bar_length, r)
            print("checker", bca, b_ckr)
            if bca is True and b_ckr is True:
                return [rp1, rp2]
    print("checker success")
    return [rp1, rp2]

def bar_connection_angle(lv1, lv2, rp1, rp2, bar1, bar2, max_bar_length, i):
# checks to make sure the angle between the axis of two connecting bars satisfies a given condition, repicks points  
    bar_new = subtract_vectors(rp2, rp1)
    connection_angle_1 = angle_vectors(bar_new, lv1, deg=True)
    connection_angle_2 = angle_vectors(bar_new, lv2, deg=True)
    print("connection_angle_1-2", connection_angle_1, connection_angle_2)

    if 150 > connection_angle_1 > 30 and 150 > connection_angle_2 > 30:
        return True
    else:
        print("bad connection angleS")
        return False

def point_on_bar_1(bar_end_points, line_vector, max_bar_length):
# selects a reference point on the previous bar added to the structre
    cp      = centroid_points(bar_end_points)
    bar_len = distance_point_point(bar_end_points[0], bar_end_points[1])    
    max_len = max_bar_length - (bar_len/2)
    sv      = scale_vector(normalize_vector(line_vector), random.randrange(bar_len/2, max_len, step=1, _int=float))

    # random direction
    rand_direction = 1 if random.random() < 0.5 else -1
    sv_rand_direction = scale_vector(sv, rand_direction)
    rp      = add_vectors(cp, sv_rand_direction)

    # upward trending: too few solutions, feathering pattern
    # rp_pos  = add_vectors(cp, sv)
    # rp_neg  = add_vectors(cp, scale_vector(sv, -1))
    # if rp_pos[2] > rp_neg[2]:
    #     rp = rp_pos
    # else:
    #     rp = rp_neg

    return rp

def point_on_bar_2(bar_end_points, line_vector):
# selects a reference point on a second, randomly chosen bar
    cp  = centroid_points(bar_end_points)
    vec_len = length_vector(line_vector)
    sv  = scale_vector(normalize_vector(line_vector), random.randrange((-vec_len/3), (vec_len/3), step=1, _int=float))
    rp = add_vectors(cp, sv)

    return rp

def second_bar_rnd(end):
# randomly selects a second bar

    # use to utilize all three tetrahedron bars
    # if end <= 7:
    #     bar_2_key = end-2
    # else:


    if end < 100:
        start = 3
    else:
        start = end - 6
    bar_2_key = random.randrange(start, end)
    print("bar_2_key", bar_2_key)

    return bar_2_key

def bar_checker(i, bar1, lv1, rp1, bar2, lv2, rp2, max_bar_length, r):
# checks to make sure there is a solution that satisfies tangency issues and is less than the maximum bar length
    dpl_1 = distance_point_line(rp1, bar2)
    dpl_2 = distance_point_line(rp2, bar1)
    dpp = distance_point_point(rp1, rp2)
    if dpl_1 >= 5*r and dpl_2 >= 5*r and dpp <= max_bar_length-200:
        return True
    else:        
        return False

def add_tangent_no_points(b_struct, bp1, lv1, bp2, lv2, rp1, rp2, radius1, radius2, bars_ind, adj_con, i):
#   adds an acceptable solution to b_struct 
    sols_two_points     = tangent_through_two_points(bp1, lv1, rp1, bp2, lv2, rp2, radius1, radius2)
    sol = find_sol_interlock(b_struct, bars_ind[0], bars_ind[1], sols_two_points, adj_con, rp1, i, radius1)

    if sol is not None:
        print("bar was added here")
        ec_x, vec_y, vec_z = calculate_coord_sys(sol, (500,500,500))
        b_new_bar = b_struct.add_bar(0, sol, "tube", (25.0, 2.0), vec_z)
        b_struct.connect_bars(b_new_bar, bars_ind[0])
        b_struct.connect_bars(b_new_bar, bars_ind[1])

        update_bar_lengths(b_struct)
        return b_new_bar      
    else:
        return None

def add_tangent(b_struct, bp1, lv1, bp2, lv2, rp, dist1, dist2, bars_rnd):
    
    sol     = tangent_from_point(bp1, lv1, bp2, lv2, rp, dist1, dist2)
    
    b1_key  = bars_rnd[0]
    b2_key  = bars_rnd[1]
    b1      = b_struct.vertex[b1_key]
    b2      = b_struct.vertex[b2_key]

    ind_max_ang = find_sol_interlock(b_struct, b1_key, b2_key, sol, rp)

    # vec_sol = sol[0]
    vec_sol = sol[ind_max_ang]
    end_pts_0 = [rp, add_vectors(rp, vec_sol)]

    # pt_mean_1 = (200,200,500)
    # ret_cls = check_length_sol_one(sol[0], pt_mean_1, rp, b1, b2, b1_key, b2_key, b_struct)

    # vec_sol_1, l1, pts_b1_1, pts_b1_2 = ret_cls
    # # pt1_e = add_vectors(bp1, scale_vector(vec_sol_1, l1))
    # pt1_e = add_vectors(rp, scale_vector(vec_sol_1, l1))
    # # end_pts_0 = (bp1, pt1_e)
    # end_pts_0 = (rp, pt1_e)
    
    # # add extension
    # ext_len = 30
    # # end_pts_0 = (add_vectors(bp1, scale_vector(normalize_vector(vector_from_points(pt1_e, bp1)), ext_len)), add_vectors(
    # #     pt1_e, scale_vector(normalize_vector(vector_from_points(bp1, pt1_e)), ext_len)))


    # end_pts_0 = [map(float, p) for p in end_pts_0]
    # end_pts_0 = [list(p) for p in end_pts_0]
    
    vec_x, vec_y, vec_z = calculate_coord_sys(end_pts_0, (500,500,500))
    pt_o        = centroid_points(end_pts_0)
    
    b_v0 = b_struct.add_bar(0, end_pts_0, "tube", (25.0, 2.0), vec_z)


    # # b_struct.vertex[b_v0].update({"index_sol":[ind]})
    b_struct.vertex[b_v0].update({"gripping_plane_no_offset":(pt_o, vec_x, vec_y, vec_z)})

    # # b1_1.update({"axis_endpoints" :  pts_b1_1})
    # # b1_2.update({"axis_endpoints" :  pts_b1_2})
    b_struct.connect_bars(b_v0, b1_key)
    b_struct.connect_bars(b_v0, b2_key)

    dpp_1   = dropped_perpendicular_points(b_struct.vertex[b_v0]["axis_endpoints"][0], b_struct.vertex[b_v0]["axis_endpoints"][1], b_struct.vertex[b1_key]["axis_endpoints"][0], b_struct.vertex[b1_key]["axis_endpoints"][1])
    dpp_2   = dropped_perpendicular_points(b_struct.vertex[b_v0]["axis_endpoints"][0], b_struct.vertex[b_v0]["axis_endpoints"][1], b_struct.vertex[b2_key]["axis_endpoints"][0], b_struct.vertex[b2_key]["axis_endpoints"][1])

    k_1     = list(b_struct.edge[b_v0][b1_key]["endpoints"].keys())[0]
    k_2     = list(b_struct.edge[b_v0][b2_key]["endpoints"].keys())[0]
    b_struct.edge[b_v0][b1_key]["endpoints"].update({k_1:(dpp_1[0], dpp_1[1])})
    b_struct.edge[b_v0][b2_key]["endpoints"].update({k_2:(dpp_2[0], dpp_2[1])})

    return b_v0

def find_sol_interlock(b_struct, b1_key, b2_key, sol, adj_con, rp1, i, r):
# selects a solution that satisfies connection conditons
    angles = []
    vec_1s = []
    vec_2s = []
    angles_previous = []
    if i == 0:
        for pts in sol:
            pts_b1 = b_struct.vertex[b1_key]["axis_endpoints"] 
            dpp1 = dropped_perpendicular_points(
                pts_b1[0], pts_b1[1], pts[0], pts[1])
            pts_b2 = b_struct.vertex[b2_key]["axis_endpoints"]
            dpp2 = dropped_perpendicular_points(
                pts_b2[0], pts_b2[1], pts[0], pts[1])
            vec_1 = (subtract_vectors(dpp1[0], dpp1[1]))
            vec_2 = (subtract_vectors(dpp2[0], dpp2[1]))

            ang_vec = angle_vectors(vec_1, vec_2, deg=True) 
            angles.append(ang_vec)

            vec_1s.append(vec_1)
            vec_2s.append(vec_2)
        ang_max = max(angles)
        ind = angles.index(ang_max)
        # selects the best solution (if one is available) that satisfies connection directions  
        if 120 < ang_max < 180:
            adj_con.append([vec_1s[ind], vec_2s[ind]])
            solution = sol[ind]
            return solution
        else:
            return None

    else:
        for pts in sol:
            pts_b1 = b_struct.vertex[b1_key]["axis_endpoints"] 
            dpp1 = dropped_perpendicular_points(
                pts_b1[0], pts_b1[1], pts[0], pts[1])
            pts_b2 = b_struct.vertex[b2_key]["axis_endpoints"]
            dpp2 = dropped_perpendicular_points(
                pts_b2[0], pts_b2[1], pts[0], pts[1])
            vec_1 = (subtract_vectors(dpp1[0], dpp1[1]))
            vec_2 = (subtract_vectors(dpp2[0], dpp2[1]))
            vec_previous = adj_con[-1]

            # checks which connection on the previous bar is the closest to rp1
            distance_1= distance_point_point(pts_b1[0], rp1)
            distance_2= distance_point_point(pts_b1[1], rp1)
            if distance_1 < distance_2:
                vec_previous = adj_con[-1][0]
            else:
                vec_previous = adj_con[-1][1]

            ang_vec = angle_vectors(vec_1, vec_2, deg=True) 
            angles.append(ang_vec)

            # this is calculating angles incorrectly, or recieving wrong vectors
            ang_previous = angle_vectors(vec_1, vec_previous, deg=True)
            angles_previous.append(ang_previous)

            vec_2s.append(vec_2)
            vec_1s.append(vec_1)
        
        # creates a list of possible solutions
        possible_sol = []
        for w in range(len(angles)):
            if 120 < angles[w] < 180 and 120 < angles_previous[w] < 180:
                print("anglesw", w, angles[w], angles_previous[w], sol[w])
                possible_sol.append(sol[w])

        print("possible sol", possible_sol)

        # selects a possible solution that has no collisions
        for solution in possible_sol:
            no_collision = check_colisions(b_struct, solution, r)
            if no_collision is True:
                ind = sol.index(solution)
                print("ind", ind)
                adj_con.append([vec_1s[ind], vec_2s[ind]])
                return solution
            
        return None



        # angles_average = []
        # for m in range(len(angles)):
        #     print("all solutions", angles[m], angles_previous[m])
        #     angles_average.append((angles_previous[m] + angles[m]) / 2)
        # ang_av_max = max(angles_average)
        # ind = angles_average.index(ang_av_max)
        # print("ang_previous_max", angles_previous[ind], "ang_max", angles[ind])
        # if 120 < angles[ind] < 180 and 120 < angles_previous[ind] < 180:
        #     adj_con.append([vec_1s[ind], vec_2s[ind]])
        #     solution = sol[ind]
        #     return solution
        # else:
        #     return None

    #     ang_previous_max = max(angles_previous)
    #     ind  = angles_previous.index(ang_previous_max)
    #     if 120 < ang_previous_max < 180:
    #         # ang_max = max(angles)
    #         # ind = angles.index(ang_max)
    #         # print("index max angle", ind, ang_max)
    #         solution = sol[ind]
    #         if 120 < angles[ind] < 180:
    #             adj_con.append(vec_2s[ind])
    #             return solution
    #         else:
    #             return None
