from __future__ import print_function
import os
import time
import math
import json
import numpy as np

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
    remove_debug, WorldSaver, set_camera_pose
from choreo import direct_ladder_graph_solve_picknplace, divide_nested_list_chunks, \
    quick_check_place_feasibility
from choreo.choreo_utils import plan_joint_motion, get_collision_fn

import ikfast_ur3
import ikfast_ur5


def single_place_check(
    seq_id, assembly_json_path, customized_sequence=[],
    num_cart_steps=10,
    robot_model='ur3', enable_viewer=True, view_ikfast=False,
    tcp_tf_list=[1e-3 * 80.525, 0, 0], scale=1, diagnosis=False):

    # # rescaling
    # # TODO: this should be done when the Assembly object is made
    unit_geos, static_obstacle_meshes = load_assembly_package(assembly_json_path, scale=scale)
    assert seq_id < len(unit_geos)

    if customized_sequence:
        unit_geo = unit_geos[customized_sequence[seq_id]]
    else:
        unit_geo = unit_geos[seq_id]

    # urdf, end effector settings
    if robot_model == 'ur3':
        urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur3.urdf')
        srdf_filename = compas_fab.get('universal_robot/ur3_moveit_config/config/ur3.srdf')
    else:
        urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
        srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')
    urdf_pkg_name = 'ur_description'

    ee_filename = compas_fab.get('universal_robot/ur_description/meshes/' +
                                'dms_2019_gripper/collision/190907_Gripper_05.obj')

    # geometry file is not loaded here
    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
    robot = RobotClass(model, semantics=semantics)

    base_link_name = robot.get_base_link_name()
    ee_link_name = robot.get_end_effector_link_name()
    ik_joint_names = robot.get_configurable_joint_names()
    disabled_link_names = semantics.get_disabled_collisions()

    # parse end effector mesh
    ee_meshes = [Mesh.from_obj(ee_filename)]
    tcp_tf = Translation(tcp_tf_list)

    # ======================================================
    # ======================================================
    # start pybullet environment & load pybullet robot
    connect(use_gui=enable_viewer)
    camera_base_pt = (0,0,0)
    camera_pt = np.array(camera_base_pt) + np.array([1, 0, 0.5])
    set_camera_pose(tuple(camera_pt), camera_base_pt)

    pb_robot = create_pb_robot_from_ros_urdf(urdf_filename, urdf_pkg_name,
                                             ee_link_name=ee_link_name)
    ee_attachs = attach_end_effector_geometry(ee_meshes, pb_robot, ee_link_name)

    # update current joint conf and attach end effector
    pb_ik_joints = joints_from_names(pb_robot, ik_joint_names)
    pb_end_effector_link = link_from_name(pb_robot, ee_link_name)
    robot_start_conf = [0,-1.65715,1.71108,-1.62348,0,0]
    set_joint_positions(pb_robot, pb_ik_joints, robot_start_conf)
    for e_at in ee_attachs: e_at.assign()

    # viz handles
    handles = []

    # convert mesh into pybullet bodies
    # TODO: this conversion should be moved into UnitGeometry
    geo_bodies = []
    for mesh in unit_geo.mesh:
        geo_bodies.append(convert_mesh_to_pybullet_body(mesh))
    unit_geo.pybullet_bodies = geo_bodies

    static_obstacles = []
    for so_mesh in static_obstacle_meshes:
        static_obstacles.append(convert_mesh_to_pybullet_body(so_mesh))

    if customized_sequence:
        assembled_element_ids = [customized_sequence[seq_iter] for seq_iter in range(seq_id)]
    else:
        assembled_element_ids = list(range(seq_id))

    print('assembled_ids: ', assembled_element_ids)

    for prev_e_id in assembled_element_ids:
        other_unit_geo = unit_geos[prev_e_id]
        print('other unit geo: ', other_unit_geo)
        for _, mesh in enumerate(other_unit_geo.mesh):
            pb_e = convert_mesh_to_pybullet_body(mesh)
            set_pose(pb_e, other_unit_geo.goal_pb_pose)
            static_obstacles.append(pb_e)

    # check collision between obstacles and element geometries
    # assert not sanity_check_collisions([unit_geo], static_obstacles_from_name)

    ik_fn = ikfast_ur3.get_ik if robot_model == 'ur3' else ikfast_ur5.get_ik
    
    return quick_check_place_feasibility(pb_robot, ik_joint_names, base_link_name, ee_link_name, ik_fn,
                unit_geo,
                num_cart_steps=num_cart_steps,
                static_obstacles=static_obstacles, self_collisions=True,
                mount_link_from_tcp_pose=pb_pose_from_Transformation(tcp_tf), 
                ee_attachs=ee_attachs, viz=view_ikfast, 
                disabled_collision_link_names=disabled_link_names, diagnosis=diagnosis)

def sequenced_picknplace_plan(assembly_json_path,
    robot_model='ur3', pick_from_same_rack=True, 
    customized_sequence=[],
    from_seq_id=0, to_seq_id=None,
    num_cart_steps=10,
    enable_viewer=True, plan_transit=True, transit_res=0.01, view_ikfast=False,
    tcp_tf_list=[1e-3 * 80.525, 0, 0], robot_start_conf = [0,-1.65715,1.71108,-1.62348,0,0],
    scale=1, result_save_path='',
    sim_traj=True, cart_ts=0.1, trans_ts=0.01, per_conf_step=False):

    # parser.add_argument('-vik', '--view_ikfast', action='store_true', help='Visualize each ikfast solutions')
    # parser.add_argument('-per_conf_step', '--per_conf_step', action='store_true', help='stepping each configuration in simulation')

    # transition motion planner settings
    RRT_RESTARTS = 5
    RRT_ITERATIONS = 40

    # rescaling
    # TODO: this should be done when the Assembly object is made
    unit_geos, static_obstacles = load_assembly_package(assembly_json_path, scale=scale)

    # urdf, end effector settings
    if robot_model == 'ur3':
        urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur3.urdf')
        # urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur3_collision_viz.urdf')
        srdf_filename = compas_fab.get('universal_robot/ur3_moveit_config/config/ur3.srdf')
    else:
        urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
        srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')

    urdf_pkg_name = 'ur_description'

    ee_filename = compas_fab.get('universal_robot/ur_description/meshes/' +
                                'dms_2019_gripper/collision/190907_Gripper_05.obj')

    # geometry file is not loaded here
    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
    robot = RobotClass(model, semantics=semantics)

    base_link_name = robot.get_base_link_name()
    ee_link_name = robot.get_end_effector_link_name()
    ik_joint_names = robot.get_configurable_joint_names()
    disabled_link_names = semantics.get_disabled_collisions()

    # parse end effector mesh
    ee_meshes = [Mesh.from_obj(ee_filename)]
    tcp_tf = Translation(tcp_tf_list)

    # add static collision obstacles
    co_dict = {}
    for i, static_obs_mesh in enumerate(static_obstacles):
        cm = CollisionMesh(static_obs_mesh, 'so_'+str(i))
        co_dict[cm.id] = {}
        co_dict[cm.id]['meshes'] = [cm.mesh]
        co_dict[cm.id]['mesh_poses'] = [cm.frame]

    # ======================================================
    # ======================================================
    # start pybullet environment & load pybullet robot
    connect(use_gui=enable_viewer)

    camera_base_pt = (0,0,0)
    camera_pt = np.array(camera_base_pt) + np.array([1, 0, 0.5])
    set_camera_pose(tuple(camera_pt), camera_base_pt)

    pb_robot = create_pb_robot_from_ros_urdf(urdf_filename, urdf_pkg_name,
                                             ee_link_name=ee_link_name)
    ee_attachs = attach_end_effector_geometry(ee_meshes, pb_robot, ee_link_name)

    # update current joint conf and attach end effector
    pb_ik_joints = joints_from_names(pb_robot, ik_joint_names)
    pb_end_effector_link = link_from_name(pb_robot, ee_link_name)
    set_joint_positions(pb_robot, pb_ik_joints, robot_start_conf)
    for e_at in ee_attachs: e_at.assign()

    # draw TCP frame in pybullet
    handles = []
    if has_gui() and view_ikfast:
        TCP_pb_pose = get_TCP_pose(pb_robot, ee_link_name, tcp_tf, return_pb_pose=True)
        handles = draw_pose(TCP_pb_pose, length=0.04)
        wait_for_user()

    # deliver ros collision meshes to pybullet
    so_lists_from_name = convert_meshes_and_poses_to_pybullet_bodies(co_dict)
    static_obstacles_from_name = {}
    for so_key, so_val in so_lists_from_name.items():
        for so_i, so_item in enumerate(so_val):
            static_obstacles_from_name[so_key + '_' + str(so_i)] = so_item

    for unit_name, unit_geo in unit_geos.items():
        geo_bodies = []
        for sub_id, mesh in enumerate(unit_geo.mesh):
            geo_bodies.append(convert_mesh_to_pybullet_body(mesh))
        unit_geo.pybullet_bodies = geo_bodies

    # check collision between obstacles and element geometries
    assert not sanity_check_collisions(unit_geos, static_obstacles_from_name)

    # from random import shuffle
    seq_assignment = customized_sequence or list(range(len(unit_geos))) 
    element_seq = {seq_id : e_id for seq_id, e_id in enumerate(seq_assignment)}

    to_seq_id = to_seq_id or len(element_seq)-1
    assert 0 <= from_seq_id and from_seq_id < len(element_seq)
    assert from_seq_id <= to_seq_id and to_seq_id < len(element_seq)

    if has_gui():
        for e_id in element_seq.values():
            handles.extend(draw_pose(unit_geos[e_id].initial_pb_pose, length=0.02))
            handles.extend(draw_pose(unit_geos[e_id].goal_pb_pose, length=0.02))
            for e_body in unit_geos[e_id].pybullet_bodies:
                set_pose(e_body, unit_geos[e_id].initial_pb_pose)
        print('pybullet env loaded.')
        # wait_for_user()
        for h in handles:
            remove_debug(h)

    saved_world = WorldSaver()

    ik_fn = ikfast_ur3.get_ik if robot_model == 'ur3' else ikfast_ur5.get_ik
    tot_traj, graph_sizes = \
    direct_ladder_graph_solve_picknplace(pb_robot, ik_joint_names, base_link_name, ee_link_name, ik_fn,
        unit_geos, element_seq, static_obstacles_from_name, 
        from_seq_id=from_seq_id, to_seq_id=to_seq_id,
        pick_from_same_rack=pick_from_same_rack,
        tcp_transf=pb_pose_from_Transformation(tcp_tf),
        ee_attachs=ee_attachs, disabled_collision_link_names=disabled_link_names, viz=view_ikfast, st_conf=robot_start_conf)

    picknplace_cart_plans = divide_nested_list_chunks(tot_traj, graph_sizes)

    saved_world.restore()
    print('Cartesian planning finished.')

    # reset robot and parts for better visualization
    set_joint_positions(pb_robot, pb_ik_joints, robot_start_conf)
    for ee in ee_attachs: ee.assign()
    for e_id in element_seq.values():
        for e_body in unit_geos[e_id].pybullet_bodies:
            set_pose(e_body, unit_geos[e_id].initial_pb_pose)

    def flatten_unit_geos_bodies(in_dict):
        out_list = []
        for ug in in_dict.values():
            out_list.extend(ug.pybullet_bodies)
        return out_list

    if plan_transit:
        print('Transition planning started.')
        disabled_collision_links = [(link_from_name(pb_robot, pair[0]), link_from_name(pb_robot, pair[1])) \
                for pair in disabled_link_names]

        for seq_id in range(0, from_seq_id):
            e_id = element_seq[seq_id]
            for e_body in unit_geos[e_id].pybullet_bodies:
                set_pose(e_body, unit_geos[e_id].goal_pb_pose)

        for seq_id in range(from_seq_id, to_seq_id + 1):
            e_id = element_seq[seq_id]
            print('----\ntransition seq#{} element #{}'.format(seq_id, e_id))

            if seq_id != from_seq_id:
                tr_start_conf = picknplace_cart_plans[seq_id-1-from_seq_id]['place_retreat'][-1]
            else:
                tr_start_conf = robot_start_conf

            place2pick_st_conf = list(tr_start_conf)
            assert picknplace_cart_plans[seq_id-from_seq_id]['pick_approach'], 'pick approach not found!'
            place2pick_goal_conf = list(picknplace_cart_plans[seq_id-from_seq_id]['pick_approach'][0])

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
                                disabled_collisions=disabled_collision_links,
                                self_collisions=True,
                                resolutions=[transit_res]*len(pb_ik_joints),
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

            assert picknplace_cart_plans[seq_id-from_seq_id]['pick_retreat'], 'pick retreat not found!'
            assert picknplace_cart_plans[seq_id-from_seq_id]['place_approach'], 'place approach not found!'
            pick2place_st_conf = picknplace_cart_plans[seq_id-from_seq_id]['pick_retreat'][-1]
            pick2place_goal_conf = picknplace_cart_plans[seq_id-from_seq_id]['place_approach'][0]

            saved_world = WorldSaver()

            # create attachement without needing to keep track of grasp...
            set_joint_positions(pb_robot, pb_ik_joints, picknplace_cart_plans[seq_id-from_seq_id]['pick_retreat'][0])
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
                disabled_collisions=disabled_collision_links,
                obstacles=pick2place_obstacles,
                attachments=ee_attachs + element_attachs, self_collisions=True,
                resolutions=[transit_res]*len(pb_ik_joints),
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

            picknplace_cart_plans[seq_id-from_seq_id]['place2pick'] = place2pick_path
            picknplace_cart_plans[seq_id-from_seq_id]['pick2place'] = pick2place_path

            for e_body in unit_geos[e_id].pybullet_bodies:
                set_pose(e_body, unit_geos[e_id].goal_pb_pose)

            if seq_id == to_seq_id:
                saved_world = WorldSaver()
                return2idle_st_conf = picknplace_cart_plans[seq_id-from_seq_id]['place_retreat'][-1]
                return2idle_goal_conf = robot_start_conf

                set_joint_positions(pb_robot, pb_ik_joints, return2idle_st_conf)
                for ee_a in ee_attachs: ee_a.assign()

                built_obstacles =  flatten_unit_geos_bodies({element_seq[prev_seq_id] : \
                    unit_geos[element_seq[prev_seq_id]] for prev_seq_id in range(seq_id+1)})
                return2idle_obstacles = list(static_obstacles_from_name.values()) + built_obstacles
                return2idle_path = plan_joint_motion(pb_robot, pb_ik_joints, return2idle_goal_conf,
                    disabled_collisions=disabled_collision_links,
                    obstacles=return2idle_obstacles,
                    attachments=ee_attachs, self_collisions=True,
                    resolutions=[transit_res]*len(pb_ik_joints),
                    restarts=RRT_RESTARTS, iterations=RRT_ITERATIONS,)

                if not return2idle_path:
                    saved_world = WorldSaver()

                    print('****\nseq #{} cannot find return2idle transition'.format(seq_id))
                    print('Diagnosis...')

                    cfn = get_collision_fn(pb_robot, pb_ik_joints, \
                        obstacles=return2idle_obstacles,
                        attachments=ee_attachs, self_collisions=True, diagnosis=True)

                    print('start pose:')
                    cfn(return2idle_st_conf)

                    print('end pose:')
                    cfn(return2idle_goal_conf)

                    saved_world.restore()
                    print('Diagnosis over')

                saved_world.restore()
                picknplace_cart_plans[seq_id-from_seq_id]['return2idle'] = return2idle_path

        print('Transition planning finished.')

    # convert to ros JointTrajectory
    traj_json_data = []
    traj_time_count = 0.0
    for i, element_process in enumerate(picknplace_cart_plans):
        e_proc_data = {}
        for sub_proc_name, sub_process in element_process.items():
            sub_process_jt_traj_list =[]
            if not sub_process:
                continue
            for jt_sol in sub_process:
                sub_process_jt_traj_list.append(
                    JointTrajectoryPoint(values=jt_sol, types=[0] * 6, time_from_start=Duration(traj_time_count, 0)))
                traj_time_count += 1.0 # meaningless timestamp
            e_proc_data[sub_proc_name] = JointTrajectory(trajectory_points=sub_process_jt_traj_list,
                                                            start_configuration=sub_process_jt_traj_list[0]).to_data()
        traj_json_data.append(e_proc_data)

    if result_save_path:
        if not os.path.exists(os.path.dirname(result_save_path)):
            os.mkdir(os.path.dirname(result_save_path))
        with open(result_save_path, 'w+') as outfile:
            json.dump(traj_json_data, outfile)
            print('planned trajectories saved to {}'.format(result_save_path))

    print('\n*************************\nplanning completed.')

    if sim_traj and has_gui():
        if has_gui():
            wait_for_user()

        for e_id in element_seq.values():
            for e_body in unit_geos[e_id].pybullet_bodies:
                set_pose(e_body, unit_geos[e_id].initial_pb_pose)

        display_picknplace_trajectories(pb_robot, ik_joint_names, ee_link_name,
                                        unit_geos, traj_json_data, \
                                        element_seq=element_seq,
                                        from_seq_id=from_seq_id, to_seq_id=to_seq_id,
                                        ee_attachs=ee_attachs,
                                        cartesian_time_step=cart_ts, 
                                        transition_time_step=trans_ts, step_sim=True, per_conf_step=per_conf_step)

    return traj_json_data


def viz_saved_traj(traj_save_path, assembly_json_path, 
    element_seq,
    from_seq_id, to_seq_id,
    cart_ts=0.01, trans_ts=0.01,
    scale=0.001,  robot_model='ur3', per_conf_step=False):

    assert os.path.exists(traj_save_path) and os.path.exists(assembly_json_path)
    # rescaling
    # TODO: this should be done when the Assembly object is made
    unit_geos, static_obstacle_meshes = load_assembly_package(assembly_json_path, scale=scale)

    # urdf, end effector settings
    if robot_model == 'ur3':
        urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur3.urdf')
        # urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur3_collision_viz.urdf')
        srdf_filename = compas_fab.get('universal_robot/ur3_moveit_config/config/ur3.srdf')
    else:
        urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
        srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')

    urdf_pkg_name = 'ur_description'

    ee_filename = compas_fab.get('universal_robot/ur_description/meshes/' +
                                'dms_2019_gripper/collision/190907_Gripper_05.obj')

    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
    robot = RobotClass(model, semantics=semantics)
    ik_joint_names = robot.get_configurable_joint_names()
    ee_link_name = robot.get_end_effector_link_name()

    connect(use_gui=True)
    camera_base_pt = (0,0,0)
    camera_pt = np.array(camera_base_pt) + np.array([1, 0, 0.5])
    set_camera_pose(tuple(camera_pt), camera_base_pt)

    pb_robot = create_pb_robot_from_ros_urdf(urdf_filename, urdf_pkg_name,
                                            ee_link_name=ee_link_name)
    ee_meshes = [Mesh.from_obj(ee_filename)]
    ee_attachs = attach_end_effector_geometry(ee_meshes, pb_robot, ee_link_name)

    static_obstacles = []
    for so_mesh in static_obstacle_meshes:
        static_obstacles.append(convert_mesh_to_pybullet_body(so_mesh))

    for unit_name, unit_geo in unit_geos.items():
        geo_bodies = []
        for sub_id, mesh in enumerate(unit_geo.mesh):
            geo_bodies.append(convert_mesh_to_pybullet_body(mesh))
        unit_geo.pybullet_bodies = geo_bodies

    if os.path.exists(traj_save_path):
        with open(traj_save_path, 'r') as f:
            traj_json_data = json.loads(f.read())

        for e_id in element_seq:
            for e_body in unit_geos[e_id].pybullet_bodies:
                set_pose(e_body, unit_geos[e_id].initial_pb_pose)

        display_picknplace_trajectories(pb_robot, ik_joint_names, ee_link_name,
                                        unit_geos, traj_json_data, \
                                        element_seq=element_seq,
                                        from_seq_id=from_seq_id, to_seq_id=to_seq_id,
                                        ee_attachs=ee_attachs,
                                        cartesian_time_step=cart_ts, 
                                        transition_time_step=trans_ts, step_sim=True, per_conf_step=per_conf_step)
    else:
        print('no saved traj found at: ', traj_save_path)