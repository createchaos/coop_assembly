
'''
                                                                                                 
    ****       *****       ******       ****       ******  ******          **           **       
   **  **      **  **      **          **  **        **    **              **           **       
   **          *****       ****        ******        **    ****            **   *****   *****    
   **  **      **  **      **          **  **        **    **              **  **  **   **  **   
    ****   **  **  **  **  ******  **  **  **  **    **    ******          **   ******  *****    
                           
                                           
created on 28.06.2019
author: stefanaparascho
'''


def draw(b_struct, o_struct, j, colors_b=(None, None), colors_o=(None, None)):

    from draw_compas import draw_network_inv
    from compas_rhino.helpers.network import draw_network
    import rhinoscriptsyntax as rs
    import Rhino
    import time

    bool_export = False
    bool_debug = False

    Rhino.RhinoApp.Wait()
    draw_network_inv(b_struct, None, False, vertexcolor=colors_b[0], edgecolor=colors_b[1])
    Rhino.RhinoApp.Wait()
    draw_network(o_struct, None, False, vertexcolor=colors_o[0], edgecolor=colors_o[1])
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