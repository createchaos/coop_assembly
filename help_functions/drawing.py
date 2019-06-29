
'''
                                                                                                 
    ****       *****       ******       ****       ******  ******          **           **       
   **  **      **  **      **          **  **        **    **              **           **       
   **          *****       ****        ******        **    ****            **   *****   *****    
   **  **      **  **      **          **  **        **    **              **  **  **   **  **   
    ****   **  **  **  **  ******  **  **  **  **    **    ******          **   ******  *****    
                           
                                           
created on 28.06.2019
author: stefanaparascho
'''

from compas.utilities.colors import color_to_colordict
from lws_geometry.help_functions.help_functions_compas import dropped_perpendicular_points,\
    dropped_perpendicular_points_ipy
from compas.geometry.basic import add_vectors, scale_vector

def draw(b_struct, o_struct, j, colors_b=(None, None), colors_o=(None, None)):

    # from draw_compas import draw_network_inv
    # from compas_rhino.helpers.network import draw_network
    from compas_rhino.artists import NetworkArtist
    import rhinoscriptsyntax as rs
    import Rhino
    import time

    bool_export = False
    bool_debug = False

    Rhino.RhinoApp.Wait()
    draw_network_inv(b_struct, None, False, vertexcolor=colors_b[0], edgecolor=colors_b[1])
    Rhino.RhinoApp.Wait()
    artist = NetworkArtist(o_struct)
    # draw_network(o_struct, None, False, vertexcolor=colors_o[0], edgecolor=colors_o[1])
    rs.Redraw()
    Rhino.RhinoApp.Wait()
    # if bool_export:
    #     capture_to_file("C:\Users\Stefana\Documents\imgs_lws\img9999_" + str(j) + ".jpg",  "network_1", 1)

    if bool_debug:
        for b in b_struct.edge:
            for c in b_struct.edge[b]:
                key     = b_struct.edge[b][c]["endpoints"].keys()[0]
                pt_0    = b_struct.edge[b][c]["endpoints"][key][0]
                pt_1    = b_struct.edge[b][c]["endpoints"][key][1]
                pt_0    = Rhino.Geometry.Point3d(pt_0[0], pt_0[1], pt_0[2])
                pt_1    = Rhino.Geometry.Point3d(pt_1[0], pt_1[1], pt_1[2])
                rs.AddSphere(pt_0, 10)
                rs.AddSphere(pt_1, 10)


def draw_network_inv(network,
                     layer=None,
                     clear_layer=False,
                     vertexcolor=None,
                     edgecolor=None):
  

    import compas_rhino
    from compas_rhino.artists import Artist

    artist = Artist()

    vertexcolor = color_to_colordict(vertexcolor,
                                     network.vertices(),
                                     default=network.attributes['color.vertex'],
                                     colorformat='rgb',
                                     normalize=False)

    edgecolor = color_to_colordict(edgecolor,
                                   network.edges(),
                                   default=network.attributes['color.edge'],
                                   colorformat='rgb',
                                   normalize=False)

    points = []
    for key, attr in network.vertices(True):
        points.append({
            "pos": network.vertex[key]["axis_endpoints"][0],
            "name": '{0}.vertex.{1}'.format(network.attributes['name'], key),
            "color": vertexcolor[key]
        })
        points.append({
            "pos": network.vertex[key]["axis_endpoints"][1],
            "name": '{0}.vertex.{1}'.format(network.attributes['name'], key),
            "color": vertexcolor[key]
        })

    lines = []
    for key, attr in network.vertices(True):
        lines.append({
            "start": network.vertex[key]["axis_endpoints"][0],
            "end": network.vertex[key]["axis_endpoints"][1],
            "name": '{0}.vertex.{1}'.format(network.attributes['name'], key),
            "color": vertexcolor[(key)]
        })

    for u, v, attr in network.edges(True):

        dpp = dropped_perpendicular_points_ipy(network.vertex[u]["axis_endpoints"][0],
                                               network.vertex[u]["axis_endpoints"][1],
                                               network.vertex[v]["axis_endpoints"][0],
                                               network.vertex[v]["axis_endpoints"][1])
        lines.append({
            "start": dpp[0],
            "end": dpp[1],
            "name": '{0}.edge.{1}-{2}'.format(network.attributes['name'], u, v),
            "color": edgecolor[(u, v)]
        })

    lines_n = []
    for v in network.vertex:
        if "gripping_plane" in network.vertex[v].keys():
            st_p = network.vertex[v]["gripping_plane"][0]
            end_p = add_vectors(network.vertex[v]["gripping_plane"][0], scale_vector(
                network.vertex[v]["gripping_plane"][1], 100))
            lines_n.append({
                "start": st_p,
                "end": end_p,
                "name": '{0}.gripping.{1}'.format(network.attributes['name'], v),
                "color": (255, 0, 0)
            })

            st_p = network.vertex[v]["gripping_plane"][0]
            end_p = add_vectors(network.vertex[v]["gripping_plane"][0], scale_vector(
                network.vertex[v]["gripping_plane"][3], 100))
            lines_n.append({
                "start": st_p,
                "end": end_p,
                "name": '{0}.gripping.{1}'.format(network.attributes['name'], v),
                "color": (255, 0, 0)
            })

    guids = compas_rhino.get_objects(
        name='{0}.*'.format(network.attributes['name']))
    compas_rhino.delete_objects(guids)

    # compas_rhino.xdraw_points(
    #     points,
    #     layer=layer,
    #     clear=clear_layer,
    #     redraw=False
    # )

    artist.draw_points(
        points,
        layer=layer,
        clear_layer=False,
        redraw=True
    )

    # compas_rhino.xdraw_lines(
    #     lines,
    #     layer=layer,
    #     clear=False,
    #     redraw=True
    # )

    artist.draw_lines(
        lines,
        layer=layer,
        clear_layer=False,
        redraw=True
    )

    # compas_rhino.xdraw_lines(
    #     lines_n,
    #     layer=layer,
    #     clear=False,
    #     redraw=True
    # )

    artist.draw_lines(
        lines_n,
        layer=layer,
        clear_layer=False,
        redraw=True
    )


def draw_network_inv_gh(network):
    """Draw a network data structure in Grasshopper.

    Parameters:
        network (compas.datastructures.network.Network): The network object.

    Example:

        .. code-block:: python
            :emphasize-lines: 7

            import compas
            from compas.datastructures.network import Network
            import compas_rhino

            network = Network.from_obj(compas.get_data('lines.obj'))

            compas_rhino.draw_network(network)

    """

    from compas_fab.fab.grasshopper.utilities import drawing

    points = []
    for key, attr in network.vertices(True):
        points.append({
            "pos": network.vertex[key]["axis_endpoints"][0]
        })
        points.append({
            "pos": network.vertex[key]["axis_endpoints"][1]
        })

    lines = []
    for key, attr in network.vertices(True):
        lines.append({
            "start": network.vertex[key]["axis_endpoints"][0],
            "end": network.vertex[key]["axis_endpoints"][1]
        })

    lines_c = []
    for u, v, attr in network.edges(True):

        # dpp = dropped_perpendicular_points_ipy(network.vertex[u]["axis_endpoints"][0],
        #                              network.vertex[u]["axis_endpoints"][1],
        #                              network.vertex[v]["axis_endpoints"][0],
        #                              network.vertex[v]["axis_endpoints"][1])
        if network.edge[u][v]:
            dpp = network.edge[u][v]["endpoints"][0]
        else:
            dpp = network.edge[v][u]["endpoints"][0]
        lines_c.append({
            "start": dpp[0],
            "end": dpp[1]
        })

    lines_n = []
    for v in network.vertex:
        if "gripping_plane" in network.vertex[v].keys():
            st_p = network.vertex[v]["gripping_plane"][0]
            end_p = add_vectors(network.vertex[v]["gripping_plane"][0], scale_vector(
                network.vertex[v]["gripping_plane"][1], 100))
            lines_n.append({
                "start": st_p,
                "end": end_p
            })

            st_p = network.vertex[v]["gripping_plane"][0]
            end_p = add_vectors(network.vertex[v]["gripping_plane"][0], scale_vector(
                network.vertex[v]["gripping_plane"][3], 100))
            lines_n.append({
                "start": st_p,
                "end": end_p
            })

    points_gh = drawing.xdraw_points(points)

    lines_gh = drawing.xdraw_lines(lines)

    lines_c_gh = drawing.xdraw_lines(lines_c)

    return points_gh, lines_gh, lines_c_gh


def create_rhino_mesh(vertices, faces):
    from Rhino.Geometry import Mesh as RhinoMesh
    # print vertices

    mesh = RhinoMesh()
    for a, b, c in vertices:
        mesh.Vertices.Add(a, b, c)
    for face in faces:
        mesh.Faces.AddFace(face[0], face[1], face[2])

    return mesh


def draw_robots(structure, building_member, rob_num, rob_num_b, path, config_b, pick_up_config):

    import compas_rhino.helpers
    import scriptcontext

    # print "structure", structure

    r_m = []
    for m in structure:
        # compas_rhino.helpers.mesh.draw_mesh(m)
        vertices_m, faces_m = m.to_vertices_and_faces()
        r_m.append(create_rhino_mesh(vertices_m, faces_m))

    from compas_fab.fabrication.grasshopper.path_planning import PathVisualizer
    vertices_m, faces_m = building_member.to_vertices_and_faces()
    bm = create_rhino_mesh(vertices_m, faces_m)

    with Simulator(debug=True) as simulator:
        robot = Robot(rob_num, client=simulator)
        rob_11 = Robot(11, client=simulator)
        rob_12 = Robot(12, client=simulator)
        robot_b = Robot(rob_num_b, client=simulator)

        start_config_11 = Configuration.from_joints_and_external_axes(
            [0, 0, 0, 0, 0, 0], [0, -1000, -3500])
        start_config_12 = Configuration.from_joints_and_external_axes(
            [0, 0, 0, 0, 0, 0], [0, -10000, -3500])

        simulator.set_robot_config(rob_11, start_config_11)
        simulator.set_robot_config(rob_12, start_config_12)
        simulator.set_robot_config(robot_b, config_b)
        # simulator.add_meshes(structure)

        pv = PathVisualizer(simulator, robot, bm, pick_up_config)
        meshes = pv.get_frame_meshes(path, len(path)-1, {})

    for m in meshes:
        scriptcontext.doc.Objects.AddMesh(m)

    for m in r_m:
        scriptcontext.doc.Objects.AddMesh(m)
    scriptcontext.doc.Views.Redraw()


def draw_structure(network, r, values="stresses"):

    from compas_rhino import xdraw_cylinders
    from spatial_structures.rhino_visualization import draw_inner_forces_bar,\
    get_inner_force_value_points, draw_force_line, get_inner_force_line_points

    cylinders = []
    for key, attr in network.vertices_iter(True):
        if values == "stresses":
            # print "utils draw", max(network.vertex[key]["exchange_values"]["utils_stresses"])
            if max(network.vertex[key]["exchange_values"]["utils_stresses"]) < 1:
                color_struct = (max(network.vertex[key]["exchange_values"]["utils_stresses"])
                                * 255, 0, (1 - max(network.vertex[key]["exchange_values"]["utils_stresses"])) * 255)
            else:
                color_struct = (255, 0, 0)
        elif values == "deformations":
            # print "utils draw", max(network.vertex[key]["exchange_values"]["utils_deformation"])
            if max(network.vertex[key]["exchange_values"]["utils_deformation"]) < 1:
                color_struct = (max(network.vertex[key]["exchange_values"]["utils_deformation"])
                                * 255, 0, (1 - max(network.vertex[key]["exchange_values"]["utils_deformation"])) * 255)
            else:
                color_struct = (255, 0, 0)
        if values == "stresses" or values == "deformations":
            cylinders.append({
                "start": network.vertex[key]["axis_endpoints"][0],
                "end": network.vertex[key]["axis_endpoints"][1],
                "name": '{0}.vertex_pipe.{1}'.format(network.attributes['name'], key),
                "radius": r,
                "color": color_struct
            })

    if values == "stresses":
        for key1, key2, attr in network.edges_iter(True):
            # print network.edge[key1][key2]["exchange_values"][0]["utils_stresses"]
            # print "utils draw", max(network.edge[key1][key2]["exchange_values"][0]["utils_stresses"])
            if max(network.edge[key1][key2]["exchange_values"][0]["utils_stresses"]) < 1:
                color_struct = (max(network.edge[key1][key2]["exchange_values"][0]["utils_stresses"])
                                * 255, 0, (1 - max(network.edge[key1][key2]["exchange_values"][0]["utils_stresses"])) * 255)
            else:
                color_struct = (255, 0, 0)

            cylinders.append({
                "start": network.edge[key1][key2]["endpoints"][0][0],
                "end": network.edge[key1][key2]["endpoints"][0][1],
                "name": '{0}.edge_pipe.{1}'.format(network.attributes['name'], key1, key2),
                "radius": r,
                "color": color_struct
            })

    if values == "stresses" or values == "deformations":
        xdraw_cylinders(cylinders, cap=True)
    elif values == "forces":
        for v in network.vertex:
            forces = network.vertex[v]["exchange_values"]["forces"]
            #draw_inner_forces_bar(sBar, 0)
            draw_forces_bar(forces, force_scale=0.0001, index_force=0)


def draw_structure_gh(network, r, values="stresses"):

    # print "drawing..."
    from compas_fab.fab.grasshopper.utilities import drawing

    cylinders = []
    colours = []
    for key, attr in network.vertices(True):
        if values == "stresses":
            # print "utils draw", max(network.vertex[key]["exchange_values"]["utils_stresses"])
            if network.vertex[key]["exchange_values"]["utils_stresses"] == []:
                color_struct = (255, 255, 255)
            elif max(network.vertex[key]["exchange_values"]["utils_stresses"]) < 0.5:
                color_struct = (0, (1 - 2*max(network.vertex[key]["exchange_values"]["utils_stresses"])) * 255, 2*max(
                    network.vertex[key]["exchange_values"]["utils_stresses"])
                    * 255)
            elif max(network.vertex[key]["exchange_values"]["utils_stresses"]) < 1.0:
                color_struct = (2*(max(network.vertex[key]["exchange_values"]["utils_stresses"])-0.5)
                                * 255, 0, (1 - 2*(max(network.vertex[key]["exchange_values"]["utils_stresses"])-0.5)) * 255)
            else:
                color_struct = (255, 0, 0)
            colours.append(color_struct)
        elif values == "deformations":
            max_val = 20
            # print "utils draw", max(network.vertex[key]["exchange_values"]["utils_deformation"])
            if max(network.vertex[key]["exchange_values"]["utils_deformation"]) < max_val/2:
                # color_struct = (max(network.vertex[key]["exchange_values"]["utils_deformation"])
                #                 * 255, 0, (1 - max(network.vertex[key]["exchange_values"]["utils_deformation"])) * 255)
                color_struct = (0, (1 - max(network.vertex[key]["exchange_values"]["utils_deformation"])/(
                    max_val/2)) * 255, max(network.vertex[key]["exchange_values"]["utils_deformation"])/(max_val/2) * 255)
            elif max(network.vertex[key]["exchange_values"]["utils_deformation"]) < max_val:
                color_struct = (((max(network.vertex[key]["exchange_values"]["utils_deformation"]) - max_val/2)/(
                    max_val/2)) * 255, 0, (1 - (max(network.vertex[key]["exchange_values"]["utils_deformation"])-max_val/2)/(max_val/2)) * 255)
            else:
                color_struct = (255, 0, 0)
            colours.append(color_struct)
        if values == "stresses" or values == "deformations":
            cylinders.append({
                "start": network.vertex[key]["axis_endpoints"][0],
                "end": network.vertex[key]["axis_endpoints"][1],
                "name": '{0}.vertex_pipe.{1}'.format(network.attributes['name'], key),
                "radius": r
            })

    # if values == "stresses":
    #     for key1, key2, attr in network.edges_iter(True):
    #         # print network.edge[key1][key2]["exchange_values"][0]["utils_stresses"]
    #         # print "utils draw", max(network.edge[key1][key2]["exchange_values"][0]["utils_stresses"])
    #         key_e = network.edge[key1][key2]["exchange_values"].keys()[0]
    #         if max(network.edge[key1][key2]["exchange_values"][key_e]["utils_stresses"]) < 1:
    #             color_struct = (max(network.edge[key1][key2]["exchange_values"][key_e]["utils_stresses"])
    #                             * 255, 0, (1 - max(network.edge[key1][key2]["exchange_values"][key_e]["utils_stresses"])) * 255)
    #         else:
    #             color_struct = (255, 0, 0)
    #         colours.append(color_struct)
    #         cylinders.append({
    #             "start": network.edge[key1][key2]["endpoints"][key_e][0],
    #             "end": network.edge[key1][key2]["endpoints"][key_e][1],
    #             "radius": r
    #         })

    if values == "stresses" or values == "deformations":
        cylinders_gh = drawing.xdraw_cylinders(cylinders, cap=True)
    elif values == "forces":
        for v in network.vertex:
            forces = network.vertex[v]["exchange_values"]["forces"]
            #draw_inner_forces_bar(sBar, 0)
            #draw_forces_bar(forces, force_scale=0.0001, index_force=0)
    # print "cylinders", cylinders_gh

    return cylinders_gh, colours


def draw_forces_bar(forces, index_force=0, mVect=(0.0, 0.0, 0.0), bar_scale=1000.0, force_scale=1.0, color_pos=(226, 0, 26), color_neg=(0, 118, 189)):
    dirIndexes = [2, 1, 2, 2, 2, 1]
#     for iStates in sBar.iStatesMembers:
#         for i in range(len(iStates)-1):
#             istat_p1 = iStates[i]
#             istat_p2 = iStates[i+1]
#             fline_tupel1 = get_inner_force_value_points(istat_p1.abs_Pos, force_scale*istat_p1.iForceList[iForceIndex], istat_p1.basis[dirIndexes[iForceIndex]])
#             fline_tupel2 = get_inner_force_value_points(istat_p2.abs_Pos, force_scale*istat_p2.iForceList[iForceIndex], istat_p2.basis[dirIndexes[iForceIndex]])
#             sline = draw_force_line(fline_tupel1, mVect, bar_scale, color_pos, color_neg)
#             eline = draw_force_line(fline_tupel2, mVect, bar_scale, color_pos, color_neg)
#             if sline or eline:
#                 for flt in get_inner_force_line_points(fline_tupel1, fline_tupel2): draw_force_line(flt, mVect, bar_scale, color_pos, color_neg)
#
    for i in range(len(forces.keys())-1):
        f1 = forces.keys()[i]
        f2 = forces.keys()[i+1]
        fline_tupel1 = get_inner_force_value_points(
            forces[f1][0], force_scale*forces[f1][1][index_force], forces[f1][2][index_force])
        fline_tupel2 = get_inner_force_value_points(
            forces[f2][0], force_scale*forces[f2][1][index_force], forces[f2][2][index_force])
        sline = draw_force_line(fline_tupel1, mVect,
                                bar_scale, color_pos, color_neg)
        eline = draw_force_line(fline_tupel2, mVect,
                                bar_scale, color_pos, color_neg)
        if sline or eline:
            for flt in get_inner_force_line_points(fline_tupel1, fline_tupel2):
                draw_force_line(flt, mVect, bar_scale, color_pos, color_neg)
