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
from numpy import mean
import operator
from math import pi

from bpy.props import BoolProperty, EnumProperty, StringProperty, FloatProperty, IntProperty

from sverchok.node_tree import SverchCustomTreeNode, StringsSocket
from sverchok.data_structure import node_id, multi_socket, updateNode

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

class SvExportGcodeNnode(bpy.types.Node, SverchCustomTreeNode):
    """
    Triggers: Export gcode from vertices position
    Tooltip: Generate a gcode file from a list of vertices
    """
    bl_idname = 'SvExportGcodeNnode'
    bl_label = 'Export Gcode'
    bl_icon = 'COPYDOWN'

    folder = StringProperty(name="File", default="", subtype='FILE_PATH')
    pull = FloatProperty(name="Pull", default=5.0, min=0, soft_max=10)
    push = FloatProperty(name="Push", default=4.0, min=0, soft_max=10)
    dz = FloatProperty(name="dz", default=2.0, min=0, soft_max=20)
    #flow_mult = FloatProperty(name="Flow Mult", default=1.0, min=0, soft_max=3)
    feed = IntProperty(name="Feed Rate (F)", default=1000, min=0, soft_max=20000)
    feed_horizontal = IntProperty(name="Feed Horizontal", default=1000, min=0, soft_max=20000)
    feed_vertical = IntProperty(name="Feed Vertical", default=1000, min=0, soft_max=20000)
    feed = IntProperty(name="Feed Rate (F)", default=1000, min=0, soft_max=20000)
    esteps = FloatProperty(name="E Steps/Unit", default=5, min=0, soft_max=100)
    start_code = StringProperty(name="Start", default='')
    end_code = StringProperty(name="End", default='')
    auto_sort = BoolProperty(name="Auto Sort", default=True)
    close_all = BoolProperty(name="Close Shapes", default=False)
    nozzle = FloatProperty(name="Nozzle", default=0.4, min=0, soft_max=10)
    layer_height = FloatProperty(name="Layer Height", default=0.1, min=0, soft_max=10)
    filament = FloatProperty(name="Filament (\u03A6)", default=1.75, min=0, soft_max=120)

    gcode_mode = EnumProperty(items=[
            ("CONT", "Continuous", ""),
            ("RETR", "Retraction", "")
        ], default='CONT', name="Mode")

    def sv_init(self, context):
        #self.inputs.new('StringsSocket', 'Flow', 'Flow').prop_name = 'flow'
        #self.inputs.new('StringsSocket', 'Start Code', 'Start Code').prop_name = 'start_code'
        self.inputs.new('StringsSocket', 'Layer Height', 'Layer Height').prop_name = 'layer_height'
        self.inputs.new('VerticesSocket', 'Vertices', 'Vertices')

    def draw_buttons(self, context, layout):

        addon = context.user_preferences.addons.get(sverchok.__name__)
        over_sized_buttons = addon.preferences.over_sized_buttons

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
            col.label(text="Retraction:", icon='NOCURVE')
            col.prop(self, 'pull', text='Retraction')
            col.prop(self, 'dz', text='Z Hop')
            col.prop(self, 'push', text='Preload')
            col.prop(self, 'auto_sort', text="Sort Layers (z)")
            col.prop(self, 'close_all')
            col.separator()
        #col.prop(self, 'flow_mult')
        col.label(text='Custom Code:', icon='SCRIPT')
        col.prop_search(self, 'start_code', bpy.data, 'texts')
        col.prop_search(self, 'end_code', bpy.data, 'texts')
        col.separator()
        row = col.row(align=True)
        row.scale_y = 4.0
        row.operator(TEXT_IO_CALLBACK, text='Export Gcode').fn_name = 'process'


    def update_socket(self, context):
        self.update()

    def process(self):
        # manage data
        feed = self.feed
        feed_v = self.feed_vertical
        feed_h = self.feed_horizontal
        flow = self.layer_height * self.nozzle / ((self.filament/2)**2 * pi)
        print("flow: " + str(flow))
        vertices = self.inputs['Vertices'].sv_get()
        #start_code = '\n'.join(self.inputs['Start Code'].sv_get()[0])
        #end_code = '\n'.join(self.inputs['End Code'].sv_get()[0])

        # open file
        if self.folder == '':
            folder = '//' + os.path.splitext(bpy.path.basename(bpy.context.blend_data.filepath))[0]
        else:
            folder = self.folder
        if '.gcode' not in folder: folder += '.gcode'
        path = bpy.path.abspath(folder)
        file = open(path, 'w')
        #file.write('M92 E' + format(self.esteps, '.4f') + '\n')
        try:
            for line in bpy.data.texts[self.start_code].lines:
                file.write(line.body + '\n')
        except:
            pass

        # sort vertices
        if self.auto_sort and self.gcode_mode == 'RETR':
            sorted_verts = []
            for curve in vertices:
                # mean z
                listz = [v[2] for v in curve]
                meanz = mean(listz)
                # store curve and meanz
                sorted_verts.append((curve, meanz))
            vertices = [data[0] for data in sorted(sorted_verts, key=lambda height: height[1])]

        # initialize variables
        e = 0
        first_point = True
        count = 0
        last_vert = mathutils.Vector((0,0,0))
        maxz = 0

        # write movements
        for curve in vertices:
            #path = path[0]
            #print(curve)
            for v in curve:
                #print(v)
                new_vert = mathutils.Vector(v)
                dist = (new_vert-last_vert).length

                # record max z
                maxz = max(maxz,v[2])

                # first point of the gcode
                if first_point:
                    file.write('G92 E0 \n')
                    file.write('G1 X' + format(v[0], '.4f') + ' Y' + format(v[1], '.4f') + ' Z' + format(v[2], '.4f') + ' F' + format(feed, '.0f') + '\n')
                    #file.write('G0 E0.5 \n')
                    first_point = False
                else:
                    # start after retraction
                    if v == curve[0] and self.gcode_mode == 'RETR':
                        file.write('G1 X' + format(v[0], '.4f') + ' Y' + format(v[1], '.4f') + ' Z' + format(maxz+self.dz, '.4f') + ' F' + format(feed_h, '.0f') + '\n')
                        e += self.push
                        file.write('G1 X' + format(v[0], '.4f') + ' Y' + format(v[1], '.4f') + ' Z' + format(v[2], '.4f') + ' E' + format(e, '.4f') + ' F' + format(feed_v, '.0f') + '\n')
                        file.write( 'G1 F' + format(feed, '.0f') + '\n')
                    # regular extrusion
                    else:
                        e += dist*flow
                        file.write('G1 X' + format(v[0], '.4f') + ' Y' + format(v[1], '.4f') + ' Z' + format(v[2], '.4f') + ' E' + format(e, '.4f') + '\n')
                count+=1
                last_vert = new_vert
            if curve != vertices[-1] and self.gcode_mode == 'RETR':
                if self.close_all:
                    new_vert = mathutils.Vector(curve[0])
                    dist = (new_vert-last_vert).length
                    e += dist*flow
                    file.write('G1 X' + format(new_vert[0], '.4f') + ' Y' + format(new_vert[1], '.4f') + ' Z' + format(new_vert[2], '.4f') + ' E' + format(e, '.4f') + '\n')
                e -= self.pull
                file.write('G0 E' + format(e, '.4f') + '\n')
                file.write('G1 X' + format(last_vert[0], '.4f') + ' Y' + format(last_vert[1], '.4f') + ' Z' + format(maxz+self.dz, '.4f') + ' F' + format(feed_v, '.0f') + '\n')

        # end code
        try:
            for line in bpy.data.texts[self.end_code].lines:
                file.write(line.body + '\n')
        except:
            pass
        #file.write(end_code)
        file.close()
        print("Saved gcode to " + path)

def register():
    bpy.utils.register_class(SvExportGcodeNnode)


def unregister():
    bpy.utils.unregister_class(SvExportGcodeNnode)
