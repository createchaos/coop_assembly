from __future__ import print_function
import os
import time
import math
import argparse
import json

from itertools import product

from compas.geometry import Frame
from compas.geometry import Translation
from compas.datastructures import Mesh
from compas.robots import LocalPackageMeshLoader
from compas.robots import RobotModel

import compas_fab
from compas_fab.backends import RosClient
from compas_fab.robots import Robot as RobotClass
from compas_fab.robots import RobotSemantics
from compas_fab.robots import PlanningScene
from compas_fab.robots import CollisionMesh
from compas_fab.robots import AttachedCollisionMesh
from compas_fab.robots.ur5 import Robot
from compas_fab.robots.time_ import Duration
from compas_fab.robots.configuration import Configuration
from compas_fab.robots import JointTrajectoryPoint, JointTrajectory

from compas_fab.backends.pybullet import attach_end_effector_geometry, \
    convert_mesh_to_pybullet_body, get_TCP_pose, create_pb_robot_from_ros_urdf, \
    convert_meshes_and_poses_to_pybullet_bodies, sanity_check_collisions, \
    pb_pose_from_Transformation

from compas_fab.backends.ros.plugins_choreo import load_assembly_package, display_picknplace_trajectories

from conrob_pybullet import load_pybullet, connect, disconnect, wait_for_user, \
    LockRenderer, has_gui, get_model_info, get_pose, euler_from_quat, draw_pose, \
    get_link_pose, link_from_name, create_attachment, add_fixed_constraint,\
    create_obj, set_pose, joints_from_names, set_joint_positions, get_fixed_constraints, \
    remove_debug, WorldSaver
from choreo import direct_ladder_graph_solve_picknplace, divide_nested_list_chunks
from choreo.choreo_utils import plan_joint_motion, get_collision_fn

import ikfast_ur5
import ikfast_ur3

def main():
    parser = argparse.ArgumentParser()
    # ur_picknplace_multiple_piece
    parser.add_argument('-p', '--problem', default='dms_ws_tet_bars', help='The name of the problem to solve')
    parser.add_argument('-rob', '--robot', default='ur3', help='The type of UR robot to use.')
    parser.add_argument('-m', '--plan_transit', action='store_false', help='Plans motions between each picking and placing')
    parser.add_argument('-v', '--viewer', action='store_true', help='Enables the viewer during planning (slow!)')
    parser.add_argument('-s', '--save_result', action='store_true', help='save planning results as a json file')
    parser.add_argument('-scale', '--model_scale', default=0.001, help='model scale conversion to meter, default 0.001 (from millimeter)')
    parser.add_argument('-vik', '--view_ikfast', action='store_true', help='Visualize each ikfast solutions')
    parser.add_argument('-tres', '--transit_res', default=0.01, help='joint resolution (rad)')
    parser.add_argument('-ros', '--use_ros', action='store_true', help='use ros backend with moveit planners')
    parser.add_argument('-cart_ts', '--cartesian_time_step', default=0.1, help='cartesian time step in trajectory simulation')
    parser.add_argument('-trans_ts', '--transit_time_step', default=0.01, help='transition time step in trajectory simulation')
    parser.add_argument('-per_conf_step', '--per_conf_step', action='store_true', help='stepping each configuration in simulation')
    args = parser.parse_args()
    print('Arguments:', args)

    pick_from_same_rack = True

    VIZ = args.viewer
    VIZ_IKFAST = args.view_ikfast
    TRANSITION_JT_RESOLUTION = float(args.transit_res)
    plan_transition = args.plan_transit
    use_moveit_planner = args.use_ros

    # sim settings
    CART_TIME_STEP = args.cartesian_time_step
    TRANSITION_TIME_STEP = args.transit_time_step
    PER_CONF_STEP = args.per_conf_step

    # transition motion planner settings
    RRT_RESTARTS = 5
    RRT_ITERATIONS = 40

    # choreo pkg settings
    choreo_problem_instance_dir = compas_fab.get('choreo_instances')
    unit_geos, static_obstacles = load_assembly_package(choreo_problem_instance_dir,
                                                        args.problem, scale=args.model_scale)

    result_save_path = os.path.join(choreo_problem_instance_dir, 'results', 'choreo_result.json') if args.save_result else None

    # urdf, end effector settings
    if args.robot == 'ur3':
        urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur3.urdf')
        # urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur3_collision_viz.urdf')
        srdf_filename = compas_fab.get('universal_robot/ur3_moveit_config/config/ur3.srdf')
    else:
        urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
        srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')

    urdf_pkg_name = 'ur_description'

    # ee_filename = compas_fab.get('universal_robot/ur_description/meshes/' +
    #                             'pychoreo_workshop_gripper/collision/victor_gripper_jaw03.obj')
    ee_filename = compas_fab.get('universal_robot/ur_description/meshes/' +
                                'dms_2019_gripper/collision/190907_Gripper_05.obj')
    # define TCP transformation
    tcp_tf = Translation([1e-3 * 80.525, 0, 0]) # in meters
    ur5_start_conf = [0,-1.65715,1.71108,-1.62348,0,0]

    client = RosClient() if use_moveit_planner else None

    # geometry file is not loaded here
    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
    robot = RobotClass(model, semantics=semantics, client=client)

    group = robot.main_group_name
    base_link_name = robot.get_base_link_name()
    ee_link_name = robot.get_end_effector_link_name()
    ik_joint_names = robot.get_configurable_joint_names()
    disabled_link_names = semantics.get_disabled_collisions()

    # parse end effector mesh
    ee_meshes = [Mesh.from_obj(ee_filename)]

    if use_moveit_planner:
        # TODO: attach end effector to the robot in planning scene
        # https://github.com/compas-dev/compas_fab/issues/66
        scene = PlanningScene(robot)
        scene.remove_all_collision_objects()
        client.set_joint_positions(group, ik_joint_names, ur5_start_conf)
    else:
        scene = None

    # add static collision obstacles
    co_dict = {}
    for i, static_obs_mesh in enumerate(static_obstacles):
        # offset the table a bit...
        cm = CollisionMesh(static_obs_mesh, 'so_'+str(i), frame=Frame.from_transformation(Translation([0, 0, -0.02])))
        if use_moveit_planner:
            scene.add_collision_mesh(cm)
        else:
            co_dict[cm.id] = {}
            co_dict[cm.id]['meshes'] = [cm.mesh]
            co_dict[cm.id]['mesh_poses'] = [cm.frame]

    if use_moveit_planner:
        # See: https://github.com/compas-dev/compas_fab/issues/63#issuecomment-519525879
        time.sleep(1)
        co_dict = scene.get_collision_meshes_and_poses()

    # ======================================================
    # ======================================================
    # start pybullet environment & load pybullet robot
    connect(use_gui=VIZ)
    pb_robot = create_pb_robot_from_ros_urdf(urdf_filename, urdf_pkg_name,
                                                planning_scene=scene,
                                                ee_link_name=ee_link_name)
    ee_attachs = attach_end_effector_geometry(ee_meshes, pb_robot, ee_link_name)

    # update current joint conf and attach end effector
    pb_ik_joints = joints_from_names(pb_robot, ik_joint_names)
    pb_end_effector_link = link_from_name(pb_robot, ee_link_name)
    if not use_moveit_planner:
        set_joint_positions(pb_robot, pb_ik_joints, ur5_start_conf)
    for e_at in ee_attachs: e_at.assign()

    # draw TCP frame in pybullet
    if has_gui():
        TCP_pb_pose = get_TCP_pose(pb_robot, ee_link_name, tcp_tf, return_pb_pose=True)
        handles = draw_pose(TCP_pb_pose, length=0.04)
        # wait_for_user()

    # deliver ros collision meshes to pybullet
    static_obstacles_from_name = convert_meshes_and_poses_to_pybullet_bodies(co_dict)
    # for now...
    for so_key, so_val in static_obstacles_from_name.items():
        static_obstacles_from_name[so_key] = so_val[0]

    for unit_name, unit_geo in unit_geos.items():
        geo_bodies = []
        for sub_id, mesh in enumerate(unit_geo.mesh):
            geo_bodies.append(convert_mesh_to_pybullet_body(mesh))
        unit_geo.pybullet_bodies = geo_bodies

    # check collision between obstacles and element geometries
    assert not sanity_check_collisions(unit_geos, static_obstacles_from_name)

    # from random import shuffle
    seq_assignment = list(range(len(unit_geos)))
    # shuffle(seq_assignment)
    element_seq = {seq_id : e_id for seq_id, e_id in enumerate(seq_assignment)}

    # for key, val in element_seq.items():
    #     # element_seq[key] = 'e_' + str(val)
    #     element_seq[key] = val

    if has_gui():
        for e_id in element_seq.values():
            # for e_body in brick_from_index[e_id].body: set_pose(e_body, brick_from_index[e_id].goal_pose)
            handles.extend(draw_pose(unit_geos[e_id].initial_pb_pose, length=0.02))
            handles.extend(draw_pose(unit_geos[e_id].goal_pb_pose, length=0.02))
            for e_body in unit_geos[e_id].pybullet_bodies:
                set_pose(e_body, unit_geos[e_id].initial_pb_pose)
        print('pybullet env loaded.')
        # wait_for_user()
        for h in handles:
            remove_debug(h)

    saved_world = WorldSaver()

    ik_fn = ikfast_ur3.get_ik if args.robot == 'ur3' else ikfast_ur5.get_ik
    tot_traj, graph_sizes = \
    direct_ladder_graph_solve_picknplace(pb_robot, ik_joint_names, base_link_name, ee_link_name, ik_fn,
        unit_geos, element_seq, static_obstacles_from_name, 
        pick_from_same_rack=pick_from_same_rack,
        tcp_transf=pb_pose_from_Transformation(tcp_tf),
        ee_attachs=ee_attachs, disabled_collision_link_names=disabled_link_names, viz=VIZ_IKFAST, st_conf=ur5_start_conf)

    picknplace_cart_plans = divide_nested_list_chunks(tot_traj, graph_sizes)

    saved_world.restore()
    print('Cartesian planning finished.')

    # reset robot and parts for better visualization
    set_joint_positions(pb_robot, pb_ik_joints, ur5_start_conf)
    for ee in ee_attachs: ee.assign()
    for e_id in element_seq.values():
        for e_body in unit_geos[e_id].pybullet_bodies:
            set_pose(e_body, unit_geos[e_id].initial_pb_pose)

    # if has_gui():
    #     wait_for_user()

    def flatten_unit_geos_bodies(in_dict):
        out_list = []
        for ug in in_dict.values():
            out_list.extend(ug.pybullet_bodies)
        return out_list

    if plan_transition:
        print('Transition planning started.')
        disabled_collision_links = [(link_from_name(pb_robot, pair[0]), link_from_name(pb_robot, pair[1])) \
                for pair in disabled_link_names]

        for seq_id, unit_picknplace in enumerate(picknplace_cart_plans):
            print('----\ntransition seq#{}'.format(seq_id))
            e_id = element_seq[seq_id]

            if seq_id != 0:
                tr_start_conf = picknplace_cart_plans[seq_id-1]['place_retreat'][-1]
            else:
                tr_start_conf = ur5_start_conf

            # obstacles=static_obstacles + cur_mo_list
            place2pick_st_conf = list(tr_start_conf)
            place2pick_goal_conf = list(picknplace_cart_plans[seq_id]['pick_approach'][0])
            # assert not client.is_joint_state_colliding(group, ik_joint_names, place2pick_st_conf)
            # assert not client.is_joint_state_colliding(group, ik_joint_names, place2pick_goal_conf)

            if use_moveit_planner:
                # TODO: add collision objects

                st_conf = Configuration.from_revolute_values(place2pick_st_conf)
                goal_conf = Configuration.from_revolute_values(place2pick_goal_conf)
                goal_constraints = robot.constraints_from_configuration(goal_conf, [math.radians(1)]*6, group)
                place2pick_jt_traj = robot.plan_motion(goal_constraints, st_conf, group, planner_id='RRTConnect')
                place2pick_path = [jt_pt['values'] for jt_pt in place2pick_jt_traj.to_data()['points']]

            else:
                saved_world = WorldSaver()

                set_joint_positions(pb_robot, pb_ik_joints, place2pick_st_conf)
                for ee_a in ee_attachs: ee_a.assign()

                built_obstacles = []
                ignored_pairs = []
                if pick_from_same_rack:
                    built_obstacles =  flatten_unit_geos_bodies({element_seq[prev_seq_id] : \
                        unit_geos[element_seq[prev_seq_id]] for prev_seq_id in range(seq_id)})
                    # if seq_id > 0:
                    #     ignored_pairs = list(product([ee_attach.child for ee_attach in ee_attachs], unit_geos[element_seq[seq_id-1]].pybullet_bodies))
                else:
                    built_obstacles = flatten_unit_geos_bodies(unit_geos)
                place2pick_obstacles = list(static_obstacles_from_name.values()) + built_obstacles

                place2pick_path = plan_joint_motion(pb_robot, pb_ik_joints,
                                    place2pick_goal_conf,
                                    attachments=ee_attachs,
                                    obstacles=place2pick_obstacles,
                                    self_collisions=True,
                                    resolutions=[TRANSITION_JT_RESOLUTION]*len(pb_ik_joints),
                                    restarts=RRT_RESTARTS, iterations=RRT_ITERATIONS,
                                    ignored_pairs=ignored_pairs)
                saved_world.restore()

                if not place2pick_path:
                    saved_world = WorldSaver()

                    print('****\nseq #{} cannot find place2pick transition'.format(seq_id))
                    print('Diagnosis...')

                    cfn = get_collision_fn(pb_robot, pb_ik_joints, \
                        obstacles=place2pick_obstacles,
                        attachments=ee_attachs, self_collisions=True, diagnosis=True)

                    print('start pose:')
                    cfn(place2pick_st_conf)

                    print('end pose:')
                    cfn(place2pick_goal_conf)

                    saved_world.restore()
                    print('Diagnosis over')

            pick2place_st_conf = picknplace_cart_plans[seq_id]['pick_retreat'][-1]
            pick2place_goal_conf = picknplace_cart_plans[seq_id]['place_approach'][0]

            if use_moveit_planner:
                st_conf = Configuration.from_revolute_values(picknplace_cart_plans[seq_id]['pick_retreat'][-1])
                goal_conf = Configuration.from_revolute_values(picknplace_cart_plans[seq_id]['place_approach'][0])
                goal_constraints = robot.constraints_from_configuration(goal_conf, [math.radians(1)]*6, group)
                pick2place_jt_traj = robot.plan_motion(goal_constraints, st_conf, group, planner_id='RRTConnect')
                pick2place_path = [jt_pt['values'] for jt_pt in pick2place_jt_traj.to_data()['points']]
            else:
                saved_world = WorldSaver()

                # create attachement without needing to keep track of grasp...
                set_joint_positions(pb_robot, pb_ik_joints, picknplace_cart_plans[seq_id]['pick_retreat'][0])
                # attachs = [Attachment(robot, tool_link, invert(grasp.attach), e_body) for e_body in brick.body]
                element_attachs = [create_attachment(pb_robot, pb_end_effector_link, e_body) \
                    for e_body in unit_geos[e_id].pybullet_bodies]

                set_joint_positions(pb_robot, pb_ik_joints, pick2place_st_conf)
                for ee_a in ee_attachs: ee_a.assign()
                for e_a in element_attachs: e_a.assign()

                built_obstacles = []
                if pick_from_same_rack:
                    built_obstacles =  flatten_unit_geos_bodies({element_seq[prev_seq_id] : \
                        unit_geos[element_seq[prev_seq_id]] for prev_seq_id in range(seq_id)})
                else:
                    built_obstacles = flatten_unit_geos_bodies(unit_geos)
                pick2place_obstacles = list(static_obstacles_from_name.values()) + built_obstacles

                pick2place_path = plan_joint_motion(pb_robot, pb_ik_joints, pick2place_goal_conf,
                    obstacles=pick2place_obstacles,
                    attachments=ee_attachs + element_attachs, self_collisions=True,
                    resolutions=[TRANSITION_JT_RESOLUTION]*len(pb_ik_joints),
                    restarts=RRT_RESTARTS, iterations=RRT_ITERATIONS,)

                saved_world.restore()

                if not pick2place_path:
                    saved_world = WorldSaver()

                    print('****\nseq #{} cannot find pick2place transition'.format(seq_id))
                    print('Diagnosis...')

                    cfn = get_collision_fn(pb_robot, pb_ik_joints,
                        obstacles=pick2place_obstacles, \
                        attachments=ee_attachs + element_attachs, self_collisions=True, diagnosis=True)

                    print('start pose:')
                    cfn(pick2place_st_conf)

                    print('end pose:')
                    cfn(pick2place_goal_conf)

                    saved_world.restore()

                    print('Diagnosis over')

            picknplace_cart_plans[seq_id]['place2pick'] = place2pick_path
            picknplace_cart_plans[seq_id]['pick2place'] = pick2place_path

            for e_body in unit_geos[e_id].pybullet_bodies:
                set_pose(e_body, unit_geos[e_id].goal_pb_pose)

            if seq_id == len(picknplace_cart_plans)-1:
                saved_world = WorldSaver()

                set_joint_positions(pb_robot, pb_ik_joints, picknplace_cart_plans[seq_id]['place_retreat'][-1])
                for ee_a in ee_attachs: ee_a.assign()

                return2idle_path = plan_joint_motion(pb_robot, pb_ik_joints, ur5_start_conf,
                    obstacles=list(static_obstacles_from_name.values()) + flatten_unit_geos_bodies(unit_geos),
                    attachments=ee_attachs, self_collisions=True,
                    resolutions=[TRANSITION_JT_RESOLUTION]*len(pb_ik_joints),
                    restarts=RRT_RESTARTS, iterations=RRT_ITERATIONS,)

                saved_world.restore()
                picknplace_cart_plans[seq_id]['return2idle'] = return2idle_path

        print('Transition planning finished.')

    # convert to ros JointTrajectory
    traj_json_data = []
    traj_time_count = 0.0
    for i, element_process in enumerate(picknplace_cart_plans):
        e_proc_data = {}
        for sub_proc_name, sub_process in element_process.items():
            sub_process_jt_traj_list =[]
            for jt_sol in sub_process:
                sub_process_jt_traj_list.append(
                    JointTrajectoryPoint(values=jt_sol, types=[0] * 6, time_from_start=Duration(traj_time_count, 0)))
                traj_time_count += 1.0 # meaningless timestamp
            e_proc_data[sub_proc_name] = JointTrajectory(trajectory_points=sub_process_jt_traj_list,
                                                            start_configuration=sub_process_jt_traj_list[0]).to_data()
        traj_json_data.append(e_proc_data)

    if result_save_path:
        with open(result_save_path, 'w+') as outfile:
            json.dump(traj_json_data, outfile, indent=4)
            print('planned trajectories saved to {}'.format(result_save_path))

    print('\n*************************\nplanning completed. Simulate?')
    if has_gui():
        wait_for_user()

    for e_id in element_seq.values():
        for e_body in unit_geos[e_id].pybullet_bodies:
            set_pose(e_body, unit_geos[e_id].initial_pb_pose)

    display_picknplace_trajectories(pb_robot, ik_joint_names, ee_link_name,
                                    unit_geos, traj_json_data, \
                                    ee_attachs=ee_attachs,
                                    cartesian_time_step=CART_TIME_STEP, transition_time_step=TRANSITION_TIME_STEP, step_sim=True, per_conf_step=PER_CONF_STEP)

    if use_moveit_planner: scene.remove_all_collision_objects()


if __name__ == '__main__':
    main()
