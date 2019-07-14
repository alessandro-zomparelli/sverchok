# ##### BEGIN GPL LICENSE BLOCK #####
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# ##### END GPL LICENSE BLOCK #####

# made by: Alessandro Zomparelli
# url: www.alessandrozomparelli.com

import io
import itertools
import pprint
import sverchok

import bpy, os, mathutils
from mathutils import Vector
import numpy as np
import operator
from math import pi

from bpy.props import BoolProperty, EnumProperty, StringProperty, FloatProperty, IntProperty

from sverchok.node_tree import SverchCustomTreeNode#, StringsSocket
from sverchok.data_structure import node_id, multi_socket, updateNode
from sverchok.utils.sv_itertools import sv_zip_longest2, flatten, list_of_lists, recurse_verts_fxy, match_longest_lists

from sverchok.utils.sv_text_io_common import (
    FAIL_COLOR, READY_COLOR, TEXT_IO_CALLBACK,
    get_socket_type,
    new_output_socket,
    name_dict,
    text_modes
)

def convert_to_text(list):
    while True:
        if type(list) is str: break
        elif type(list) in (tuple, list):
            try:
                list = '\n'.join(list)
                break
            except: list = list[0]
        else: break
    return list

class SvExportGcodeNode(bpy.types.Node, SverchCustomTreeNode):
    """
    Triggers: Export gcode from vertices position
    Tooltip: Generate a gcode file from a list of vertices
    """
    bl_idname = 'SvExportGcodeNode'
    bl_label = 'Export Gcode'
    bl_icon = 'COPYDOWN'

    #def update_socket(self, context):
    #    self.update()

    last_e : FloatProperty(name="Pull", default=5.0, min=0, soft_max=10)
    path_length : FloatProperty(name="Pull", default=5.0, min=0, soft_max=10)

    folder : StringProperty(name="File", default="", subtype='FILE_PATH')
    pull : FloatProperty(name="Pull", default=5.0, min=0, soft_max=10, update = updateNode)
    push : FloatProperty(name="Push", default=5.0, min=0, soft_max=10, update = updateNode)
    dz : FloatProperty(name="dz", default=2.0, min=0, soft_max=20, update = updateNode)
    flow_mult : FloatProperty(name="Flow Mult", default=1.0, min=0, soft_max=3, update = updateNode)
    feed : IntProperty(name="Feed Rate (F)", default=1000, min=0, soft_max=20000, update = updateNode)
    feed_horizontal : IntProperty(name="Feed Horizontal", default=2000, min=0, soft_max=20000, update = updateNode)
    feed_vertical : IntProperty(name="Feed Vertical", default=500, min=0, soft_max=20000, update = updateNode)
    feed : IntProperty(name="Feed Rate (F)", default=1000, min=0, soft_max=20000, update = updateNode)
    esteps : FloatProperty(name="E Steps/Unit", default=5, min=0, soft_max=100, update = updateNode)
    start_code : StringProperty(name="Start", default='')
    end_code : StringProperty(name="End", default='')
    auto_sort_layers : BoolProperty(name="Auto Sort Layers", default=True, update = updateNode)
    auto_sort_points : BoolProperty(name="Auto Sort Points", default=False, update = updateNode)
    close_all : BoolProperty(name="Close Shapes", default=False, update = updateNode)
    nozzle : FloatProperty(name="Nozzle", default=0.4, min=0, soft_max=10, update = updateNode)
    layer_height : FloatProperty(name="Layer Height", default=0.1, min=0, soft_max=10, update = updateNode)
    filament : FloatProperty(name="Filament (\u03A6)", default=1.75, min=0, soft_max=120, update = updateNode)

    gcode_mode : EnumProperty(items=[
            ("CONT", "Continuous", ""),
            ("RETR", "Retraction", "")
        ], default='CONT', name="Mode", update = updateNode)

    retraction_mode : EnumProperty(items=[
            ("FIRMWARE", "Firmware", ""),
            ("GCODE", "Gcode", "")
        ], default='GCODE', name="Retraction mode", update = updateNode)

    def sv_init(self, context):
        self.inputs.new('StringsSocket', 'Layer Height',).prop_name = 'layer_height'
        self.inputs.new('StringsSocket', 'Flow Mult',).prop_name = 'flow_mult'
        self.inputs.new('VerticesSocket', 'Vertices',)

        self.outputs.new('StringsSocket', 'Info',)
        self.outputs.new('VerticesSocket', 'Vertices',)
        self.outputs.new('StringsSocket', 'Printed Edges',)
        self.outputs.new('StringsSocket', 'Travel Edges',)

    def draw_buttons(self, context, layout):

        #addon = context.user_preferences.addons.get(sverchok.__name__)
        #over_sized_buttons = addon.preferences.over_sized_buttons

        col = layout.column(align=True)
        row = col.row()
        row.prop(self, 'folder', toggle=True, text='')
        col = layout.column(align=True)
        row = col.row()
        row.prop(self, 'gcode_mode', expand=True, toggle=True)
        #col = layout.column(align=True)
        col = layout.column(align=True)
        col.label(text="Extrusion:", icon='MOD_FLUIDSIM')
        #col.prop(self, 'esteps')
        col.prop(self, 'filament')
        col.prop(self, 'nozzle')
        col.separator()
        col.label(text="Speed (Feed Rate F):", icon='DRIVER')
        col.prop(self, 'feed', text='Print')
        if self.gcode_mode == 'RETR':
            col.prop(self, 'feed_vertical', text='Z Lift')
            col.prop(self, 'feed_horizontal', text='Travel')
        col.separator()
        if self.gcode_mode == 'RETR':
            col = layout.column(align=True)
            col.label(text="Retraction mode:", icon='PREFERENCES')
            row = col.row()
            row.prop(self, 'retraction_mode', expand=True, toggle=True)
            if self.retraction_mode == 'GCODE':
                col.label(text="Retraction:", icon='NOCURVE')
                col.prop(self, 'pull', text='Retraction')
                col.prop(self, 'dz', text='Z Hop')
                col.prop(self, 'push', text='Preload')
                col.separator()
            #col.label(text="Layers options:", icon='ALIGN_JUSTIFY')
            col.separator()
            col.prop(self, 'auto_sort_layers', text="Sort Layers (Z)")
            col.prop(self, 'auto_sort_points', text="Sort Points (XY)")
            col.prop(self, 'close_all')
            col.separator()
        col.label(text='Custom Code:', icon='TEXT')
        col.prop_search(self, 'start_code', bpy.data, 'texts')
        col.prop_search(self, 'end_code', bpy.data, 'texts')
        col.separator()
        row = col.row(align=True)
        row.scale_y = 4.0
        row.operator(TEXT_IO_CALLBACK, text='Export Gcode').fn_name = 'export'

    def process(self, export=False):
        # manage data
        feed = self.feed
        feed_v = self.feed_vertical
        feed_h = self.feed_horizontal
        layer = self.layer_height
        layer = self.inputs['Layer Height'].sv_get()
        vertices = self.inputs['Vertices'].sv_get()
        flow_mult = self.inputs['Flow Mult'].sv_get()

        # data matching
        vertices = list_of_lists(vertices, True)
        flow_mult = list_of_lists(flow_mult)
        layer = list_of_lists(layer)
        vertices, flow_mult, layer = match_longest_lists([vertices, flow_mult, layer])
        vertices = list(vertices)
        flow_mult = list(flow_mult)
        layer = list(layer)

        if len(vertices) == 1: self.gcode_mode = 'CONT'

        # open file
        if(export):
            if self.folder == '':
                folder = '//' + os.path.splitext(bpy.path.basename(bpy.context.blend_data.filepath))[0]
            else:
                folder = self.folder
            if '.gcode' not in folder: folder += '.gcode'
            path = bpy.path.abspath(folder)
            file = open(path, 'w')
            try:
                for line in bpy.data.texts[self.start_code].lines:
                    file.write(line.body + '\n')
            except:
                pass

        if self.gcode_mode == 'RETR':

            # sort layers (Z)
            if self.auto_sort_layers:
                sorted_verts = []
                for curve in vertices:
                    # mean z
                    listz = [v[2] for v in curve]
                    meanz = np.mean(listz)
                    # store curve and meanz
                    sorted_verts.append((curve, meanz))
                vertices = [data[0] for data in sorted(sorted_verts, key=lambda height: height[1])]

            # sort vertices (XY)
            if self.auto_sort_points:
                # curves median point
                median_points = [np.mean(verts,axis=0) for verts in vertices]

                # chose starting point for each curve
                for j, curve in enumerate(vertices):
                    kd = mathutils.kdtree.KDTree(len(vertices[j]))
                    for i, v in enumerate(vertices[j]):
                        kd.insert(v, i)
                    kd.balance()

                    if j==0:
                        # close to next two curves median point
                        co_find = np.mean(median_points[j+1:j+3],axis=0)
                    elif j < len(vertices)-1:
                        co_find = np.mean([median_points[j-1],median_points[j+1]],axis=0)
                    else:
                        co_find = np.mean(median_points[j-2:j],axis=0)

                    co, index, dist = kd.find(co_find)
                    vertices[j] = vertices[j][index:]+vertices[j][:index]
                    flow_mult[j] = flow_mult[j][index:]+flow_mult[j][:index]
                    layer[j] = layer[j][index:]+layer[j][:index]

            #  close shapes
            if self.close_all:
                for i in range(len(vertices)):
                    vertices[i].append(vertices[i][0])
                    flow_mult[i].append(flow_mult[i][0])
                    layer[i].append(layer[i][0])

        # calc bounding box
        min_corner = np.min(vertices[0],axis=0)
        max_corner = np.max(vertices[0],axis=0)
        for i in range(1,len(vertices)):
            eval_points = vertices[i] + [min_corner]
            min_corner = np.min(eval_points,axis=0)
            eval_points = vertices[i] + [max_corner]
            max_corner = np.max(eval_points,axis=0)

        # initialize variables
        e = 0
        last_vert = Vector((0,0,0))
        maxz = 0
        path_length = 0
        travel_length = 0

        printed_verts = []
        printed_edges = []
        travel_verts = []
        travel_edges = []

        # write movements
        for i in range(len(vertices)):
            curve = vertices[i]
            first_id = len(printed_verts)
            for j in range(len(curve)):
                v = curve[j]
                v_flow_mult = flow_mult[i][j]
                v_layer = layer[i][j]

                # record max z
                maxz = np.max((maxz,v[2]))
                #maxz = max(maxz,v[2])

                # first point of the gcode
                if i == j == 0:
                    printed_verts.append(v)
                    if(export):
                        file.write('G92 E0 \n')
                        params = v[:3] + (feed,)
                        to_write = 'G1 X{0:.4f} Y{1:.4f} Z{2:.4f} F{3:.0f}\n'.format(*params)
                        file.write(to_write)
                else:
                    # start after retraction
                    if j == 0 and self.gcode_mode == 'RETR':
                        if(export):
                            params = v[:2] + (maxz+self.dz,) + (feed_h,)
                            to_write = 'G1 X{0:.4f} Y{1:.4f} Z{2:.4f} F{3:.0f}\n'.format(*params)
                            file.write(to_write)
                            params = v[:3] + (feed_v,)
                            to_write = 'G1 X{0:.4f} Y{1:.4f} Z{2:.4f} F{3:.0f}\n'.format(*params)
                            file.write(to_write)
                            if self.retraction_mode == 'GCODE':
                                e += self.push
                                file.write( 'G1 E' + format(e, '.4f') + '\n')
                            else:
                                file.write('G11\n')
                        printed_verts.append((v[0], v[1], maxz+self.dz))
                        travel_edges.append((len(printed_verts)-1, len(printed_verts)-2))
                        travel_length += (Vector(printed_verts[-1])-Vector(printed_verts[-2])).length
                        printed_verts.append(v)
                        travel_edges.append((len(printed_verts)-1, len(printed_verts)-2))
                        travel_length += maxz+self.dz - v[2]
                    # regular extrusion
                    else:
                        printed_verts.append(v)
                        v1 = Vector(v)
                        v0 = Vector(curve[j-1])
                        dist = (v1-v0).length
                        area = v_layer * self.nozzle + pi*(v_layer/2)**2 # rectangle + circle
                        cylinder = pi*(self.filament/2)**2
                        flow = area / cylinder
                        e += dist * v_flow_mult * flow
                        params = v[:3] + (e,)
                        if(export):
                            to_write = 'G1 X{0:.4f} Y{1:.4f} Z{2:.4f} E{3:.4f}\n'.format(*params)
                            file.write(to_write)
                        path_length += dist
                        printed_edges.append([len(printed_verts)-1, len(printed_verts)-2])
            if self.gcode_mode == 'RETR':
                v0 = Vector(curve[-1])
                if self.close_all and False:
                    #printed_verts.append(v0)
                    printed_edges.append([len(printed_verts)-1, first_id])

                    v1 = Vector(curve[0])
                    dist = (v0-v1).length
                    area = v_layer * self.nozzle + pi*(v_layer/2)**2 # rectangle + circle
                    cylinder = pi*(self.filament/2)**2
                    flow = area / cylinder
                    e += dist * v_flow_mult * flow
                    params = v1[:3] + (e,)
                    if(export):
                        to_write = 'G1 X{0:.4f} Y{1:.4f} Z{2:.4f} E{3:.4f}\n'.format(*params)
                        file.write(to_write)
                    path_length += dist
                    v0 = v1
                if i < len(vertices)-1:
                    if(export):
                        if self.retraction_mode == 'GCODE':
                            e -= self.pull
                            file.write('G0 E' + format(e, '.4f') + '\n')
                        else:
                            file.write('G10\n')
                        params = v0[:2] + (maxz+self.dz,) + (feed_v,)
                        to_write = 'G1 X{0:.4f} Y{1:.4f} Z{2:.4f} F{3:.0f}\n'.format(*params)
                        file.write(to_write)
                    printed_verts.append(v0.to_tuple())
                    printed_verts.append((v0.x, v0.y, maxz+self.dz))
                    travel_edges.append((len(printed_verts)-1, len(printed_verts)-2))
                    travel_length += maxz+self.dz - v0.z
        if(export):
            # end code
            try:
                for line in bpy.data.texts[self.end_code].lines:
                    file.write(line.body + '\n')
            except:
                pass
            file.close()
            print("Saved gcode to " + path)
        bb = list(min_corner) + list(max_corner)
        info = 'Bounding Box:\n'
        info += '\tmin\tX: {0:.1f}\tY: {1:.1f}\tZ: {2:.1f}\n'.format(*bb)
        info += '\tmax\tX: {3:.1f}\tY: {4:.1f}\tZ: {5:.1f}\n'.format(*bb)
        info += 'Extruded Filament: ' + format(e, '.2f') + '\n'
        info += 'Extruded Volume: ' + format(e*pi*(self.filament/2)**2, '.2f') + '\n'
        info += 'Printed Path Length: ' + format(path_length, '.2f') + '\n'
        info += 'Travel Length: ' + format(travel_length, '.2f')
        self.outputs[0].sv_set(info)
        self.outputs[1].sv_set([printed_verts])
        self.outputs[2].sv_set([printed_edges])
        if self.gcode_mode == 'RETR': self.outputs[3].sv_set([travel_edges])

    def export(self):
        self.process(export=True)

def register():
    bpy.utils.register_class(SvExportGcodeNode)

def unregister():
    bpy.utils.unregister_class(SvExportGcodeNode)
