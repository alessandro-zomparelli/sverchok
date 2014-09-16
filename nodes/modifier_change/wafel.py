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

# nikitron made

import bpy

from mathutils import Vector, Euler
from mathutils.geometry import distance_point_to_plane as D2P
from mathutils.geometry import intersect_line_line as IL2L
from mathutils.geometry import intersect_line_plane as IL2P
from mathutils.geometry import normal as NM
from mathutils import kdtree as KDT
from data_structure import Vector_generate, Vector_degenerate, fullList, \
                           SvSetSocketAnyType, SvGetSocketAnyType, dataCorrect
from math import sin, atan, cos, degrees, radians
from bpy.props import FloatProperty, BoolProperty, EnumProperty
from node_tree import SverchCustomTreeNode


class SvWafelNode(bpy.types.Node, SverchCustomTreeNode):
    '''Making vertical wafel - much raw node'''
    bl_idname = 'SvWafelNode'
    bl_label = 'Wafel'
    bl_icon = 'OUTLINER_OB_EMPTY'

    thick = FloatProperty(name='thick', description='thickness of material',
                           default=0.01)

    circle_rad = FloatProperty(name='radius', description='radius of circle',
                           default=0.01)

    tube_radius = FloatProperty(name='tube_radius', description='radius of tube',
                           default=0.05)

    threshold = FloatProperty(name='threshold', description='threshold for interect edge',
                           default=16)

    circle = BoolProperty(name='circle', description='circle for leyer',
                           default=False)

    circl_place = EnumProperty(name='place', items=[('Up','Up','Up'),('Midl','Midl','Midl'),('Down','Down','Down')],
                           description='circle placement', default='Up')

    rounded = BoolProperty(name='rounded', description='making rounded edges',
                           default = False)

    def init(self, context):
        self.inputs.new('VerticesSocket', 'vec', 'vec')
        self.inputs.new('VerticesSocket', 'vecplan', 'vecplan')
        self.inputs.new('StringsSocket', 'edgplan', 'edgplan')
        self.inputs.new('VerticesSocket', 'vecont', 'vecont')
        self.inputs.new('StringsSocket', 'edgcont', 'edgcont')
        self.inputs.new('StringsSocket', 'thick').prop_name = 'thick'
        self.inputs.new('StringsSocket', 'circl rad').prop_name = 'circle_rad'
        self.inputs.new('VerticesSocket', 'vec_tube', 'vec_tube')
        self.inputs.new('StringsSocket', 'tube_radius').prop_name = 'tube_radius'
        
        self.outputs.new('VerticesSocket', 'vupper', 'vupper')
        self.outputs.new('StringsSocket', 'outeup', 'outeup')
        self.outputs.new('VerticesSocket', 'vlower', 'vlower')
        self.outputs.new('StringsSocket', 'outelo', 'outelo')
        self.outputs.new('VerticesSocket', 'centers', 'centers')

    def draw_buttons(self, context, layout):
        row = layout.row(align=True)
        row.prop(self, 'rounded')
        row.prop(self, 'circle')
        row = layout.row(align=True)
        row.prop(self, 'threshold')
        row = layout.row(align=True)
        row.prop(self, 'circl_place', expand=True)

    def rotation_on_axis(self, p,v,a):
        '''
        rotate one point 'p' over axis normalized 'v' on degrees 'a'
        '''
        Xp,Yp,Zp = p[:]
        Xv,Yv,Zv = v[:]
        Temp = 1 - cos(a)
        Nx = Xp * (Xv * Temp * Xv + cos(a)) + \
             Yp * (Yv * Temp * Xv - sin(a) * Zv) + \
             Zp * (Zv * Temp * Xv + sin(a) * Yv)

        Ny = Xp * (Xv * Temp * Yv + sin(a) * Zv) + \
             Yp * (Yv * Temp * Yv + cos(a)) + \
             Zp * (Zv * Temp * Yv - sin(a) * Xv)

        Nz = Xp * (Xv * Temp * Zv - sin(a) * Yv) + \
             Yp * (Yv * Temp * Zv + sin(a) * Xv) + \
             Zp * (Zv * Temp * Zv + cos(a))
        return Vector(( Nx,Ny,Nz ))

    def calc_indexes(self, edgp, near):
        '''
        find binded edges and vertices, prepare to delete edges
        '''
        q = []
        deledges = []
        for i in edgp:
            if near in i:
                for t in i:
                    if t != near:
                        q.append(t)
                deledges.append(list(i))
        q.append(deledges)
        return q

    def interpolation(self, vecp, vec, en0, en1, diry, dirx):
        '''
        shifting on height
        '''
        out = []
        k = False
        for en in [en0, en1]:
            if k:
                out.append(IL2L(vec,vecp[en],vec-dirx,vec-dirx-diry)[0])
            else:
                out.append(IL2L(vec,vecp[en],vec+dirx,vec+dirx-diry)[0])
            k = True
        return out

    def calc_leftright(self, vecp, vec, dirx, en0, en1, thick, diry):
        '''
        calc left right from defined point and direction to join vertices
        oriented on given indexes
        left right - points
        l r - indexes of this nearest points
        lz rz - height difference to compensate
        '''
        a,b = vecp[en0]-vec+dirx, vecp[en0]-vec-dirx
        if a.length > b.length:
            l =  en0
            r =  en1
            rz, lz = self.interpolation(vecp, vec, l, r, diry, dirx)
        else:
            l =  en1
            r =  en0
            rz, lz = self.interpolation(vecp, vec, l, r, diry, dirx)
        return l, r, lz, rz

    def get_coplanar(self,vec, loc_cont, norm_cont,vec_cont):
        '''
        if coplanar - than make flip cutting up-bottom
        '''
        for locon, nocon, vecon in zip(loc_cont,norm_cont,vec_cont):
            x = [i[0] for i in vecon]
            y = [i[1] for i in vecon]
            con_domein = vec[0]<max(x) and vec[0]>min(x) and vec[1]<max(y) and vec[1]>min(y)
            if con_domein:
                a = abs(D2P(vec,locon[0],nocon[0]))
                if a < 0.001:
                    return True
        return False

    def update(self):
        if not 'centers' in self.outputs:
            return
        if 'vec' in self.inputs and 'vecplan' in self.inputs \
                and 'edgplan' in self.inputs:
            print(self.name, 'is starting')
            if self.inputs['vec'].links and self.inputs['vecplan'].links \
                    and self.inputs['edgplan'].links:
                if self.circle:
                    circle = [ (Vector((sin(radians(i)),cos(radians(i)),0))*self.circle_rad)/4 \
                              for i in range(0,360,30) ]
                vec = self.inputs['vec'].sv_get()
                vecplan = self.inputs['vecplan'].sv_get()
                edgplan = self.inputs['edgplan'].sv_get()
                thick = self.inputs['thick'].sv_get()[0][0]
                sinuso60 = 0.8660254037844386
                sinuso60_minus = 0.133974596
                sinuso30 = 0.5
                sinuso45 = 0.7071067811865475
                thick_2 = thick/2
                thick_3 = thick/3
                thick_6 = thick/6
                threshold = self.threshold
                if 'vecont' in self.inputs and self.inputs['vecont'].links:
                    vecont = self.inputs['vecont'].sv_get()
                    edgcont = self.inputs['edgcont'].sv_get()
                    vec_cont = Vector_generate(vecont)
                    loc_cont = [ [ i[0] ] for i in vec_cont ]
                    norm_cont = [ [ NM(i[0],i[len(i)//2], i[-1]) ] for i in vec_cont ] # довести до ума
                else:
                    vec_cont = []
                if 'vec_tube' in self.inputs and self.inputs['vec_tube'].links:
                    vectube = self.inputs['vec_tube'].sv_get()
                    vec_tube = Vector_generate(vectube)
                    tube_radius = self.inputs['tube_radius'].sv_get()[0][0]
                    circle_tube = [ (Vector((sin(radians(i)),cos(radians(i)),0))*tube_radius) \
                              for i in range(0,360,15) ]
                else:
                    vec_tube = []
                outeup = []
                outelo = []
                vupper = []
                vlower = []
                centers = []
                vec_ = Vector_generate(vec)
                vecplan_ = Vector_generate(vecplan)
                for centersver, vecp, edgp in zip(vecplan,vecplan_,edgplan):
                    tubes_flag_bed_solution_i_know = False
                    newinds1 = edgp.copy()
                    newinds2 = edgp.copy()
                    vupperob = vecp.copy()
                    vlowerob = vecp.copy()
                    deledges1 = []
                    deledges2 = []
                    # to define bounds
                    x = [i[0] for i in vecp]
                    y = [i[1] for i in vecp]
                    z = [i[2] for i in vecp]
                    m1x,m2x,m1y,m2y,m1z,m2z = max(x), min(x), max(y), min(y), max(z), min(z)
                    l = Vector((sum(x)/len(x),sum(y)/len(y),sum(z)/len(z)))
                    n_select = [vecp[0],vecp[len(vecp)//2], vecp[-1]] # довести до ума
                    n_select.sort(key=lambda x: sum(x[:]), reverse=False)
                    n_ = NM(n_select[0],n_select[1],n_select[2])
                    n_.normalize()
                    # а виновта ли нормаль?
                    if n_[0] < 0:
                        n = n_ * -1
                    else:
                        n = n_
                    cen = [sum(i) for i in zip(*centersver)]
                    centers.append(Vector(cen)/len(centersver))
                    k = 0
                    lenvep = len(vecp)
                    # KDtree collections closest to join edges to sockets
                    tree = KDT.KDTree(lenvep)
                    for i,v in enumerate(vecp):
                        tree.insert(v,i)
                    tree.balance()
                    # vertical edges iterations
                    # every edge is object - two points, one edge
                    for v in vec_:
                        # sort vertices by Z value
                        # find two vertices - one lower, two upper
                        vlist = [v[0],v[1]]
                        vlist.sort(key=lambda x: x[2], reverse=False)
                        # flip if coplanar to enemy plane
                        # flip plane coplanar
                        if vec_cont:
                            fliped = self.get_coplanar(v[0], loc_cont,norm_cont, vec_cont)
                        else:
                            fliped = False
                        shortedge = (vlist[1]-vlist[0]).length
                        if fliped:
                            two, one = vlist
                        else:
                            one, two = vlist
                        # coplanar to owner
                        cop = abs(D2P(one,l,n))
                        # defining bounds
                        inside = one[0]<m1x and one[0]>m2x and one[1]<m1y and one[1]>m2y \
                                 and one[2]<=m1z and one[2]>=m2z
                        # if in bounds and coplanar do:
                        #print(self.name,l, cop, inside)
                        if cop < 0.001 and inside and shortedge > thick*threshold:
                            '''
                            huge calculations. if we can reduce...
                            '''
                            # find shift for thickness in sockets
                            diry = two - one
                            diry.normalize()
                            # solution for vertical wafel - cool but not in diagonal case
                            # angle = radians(degrees(atan(n.y/n.x))+90)
                            dirx_ = self.rotation_on_axis(diry, n, radians(90))
                            dirx = dirx_*thick_2
                            # вектор, индекс, расстояние
                            # запоминаем порядок находим какие удалить рёбра
                            # делаем выборку левая-правая точка
                            nearv_1, near_1 = tree.find(one)[:2]
                            nearv_2, near_2 = tree.find(two)[:2]
                            # indexes of two nearest points
                            # удалить рёбра что мешают спать заодно
                            en_0, en_1, de1 = self.calc_indexes(edgp, near_1)
                            deledges1.extend(de1)
                            en_2, en_3, de2 = self.calc_indexes(edgp, near_2)
                            deledges2.extend(de2)
                            # print(vecp, one, dirx, en_0, en_1)
                            # left-right indexes and vectors
                            # с учётом интерполяций по высоте
                            l1, r1, lz1, rz1 = \
                                    self.calc_leftright(vecp, one, dirx, en_0, en_1, thick_2, diry)
                            l2, r2, lz2, rz2 = \
                                    self.calc_leftright(vecp, two, dirx, en_2, en_3, thick_2, diry)
                            # print(left2, right2, l2, r2, lz2, rz2)
                            # средняя точка и её смещение по толщине материала
                            three = (one-two)/2 + two
                            if self.rounded:
                                '''рёбра'''
                                # пазы формируем независимо от верх низ

                                outeob1 = [[lenvep+k+8,lenvep+k],[lenvep+k+1,lenvep+k+2],
                                          [lenvep+k+2,lenvep+k+3],[lenvep+k+3,lenvep+k+4],
                                          [lenvep+k+4,lenvep+k+5],[lenvep+k+5,lenvep+k+6],
                                          [lenvep+k+6,lenvep+k+7],[lenvep+k+7,lenvep+k+8],
                                          [lenvep+k+9,lenvep+k+1]]

                                outeob2 = [[lenvep+k,lenvep+k+1],[lenvep+k+1,lenvep+k+2],
                                          [lenvep+k+2,lenvep+k+3],[lenvep+k+3,lenvep+k+4],
                                          [lenvep+k+4,lenvep+k+5],[lenvep+k+5,lenvep+k+6],
                                          [lenvep+k+6,lenvep+k+7],[lenvep+k+7,lenvep+k+8],
                                          [lenvep+k+8,lenvep+k+9]]
                                # наполнение списков lenvep = length(vecp)
                                newinds1.extend([[l1, lenvep+k], [lenvep+k+9, r1]])
                                newinds2.extend([[l2, lenvep+k+9], [lenvep+k, r2]])
                                '''Вектора'''
                                round1 = diry*thick_3
                                round2 = diry*thick_3*sinuso30
                                round2_= dirx/3 + dirx*(2*sinuso60/3)
                                round3 = diry*thick_3*sinuso60_minus
                                round3_= dirx/3 + dirx*(2*sinuso30/3)
                                round4 = dirx/3
                                vupperob.extend([lz2,
                                                 three+round1-dirx, three+round2-round2_,
                                                 three+round3-round3_, three-round4,
                                                 three+round4, three+round3+round3_,
                                                 three+round2+round2_, three+round1+dirx,
                                                 rz2])
                                vlowerob.extend([rz1,
                                                 three-round1-dirx, three-round2-round2_,
                                                 three-round3-round3_, three-round4,
                                                 three+round4, three-round3+round3_,
                                                 three-round2+round2_, three-round1+dirx,
                                                 lz1])
                                k += 10
                            else:
                                '''рёбра'''
                                # пазы формируем независимо от верх низ
                                outeob1 = [[lenvep+k,lenvep+k+1],[lenvep+k+1,lenvep+k+2],[lenvep+k+2,lenvep+k+3]]
                                outeob2 = [[lenvep+k,lenvep+k+1],[lenvep+k+1,lenvep+k+2],[lenvep+k+2,lenvep+k+3]]
                                # наполнение списков lenvep = length(vecp)
                                newinds1.extend([[l1, lenvep+k], [lenvep+k+3, r1]])
                                newinds2.extend([[l2, lenvep+k+3], [lenvep+k, r2]])
                                '''Вектора'''
                                vupperob.extend([lz2, three-dirx, 
                                                 three+dirx, rz2])
                                vlowerob.extend([rz1, three+dirx,
                                                 three-dirx, lz1])
                                k += 4
                            newinds1.extend(outeob1)
                            newinds2.extend(outeob2)
                            if self.circle:
                                CP = self.circl_place
                                if CP == 'Midl':
                                    crcl_cntr = IL2P(one, two, Vector((0,0,0)), Vector((0,0,-1)))
                                elif CP == 'Up' and not fliped:
                                    crcl_cntr = two - diry*self.circle_rad*2
                                elif CP == 'Down' and not fliped:
                                    crcl_cntr = one + diry*self.circle_rad*2
                                elif CP == 'Up' and fliped:
                                    crcl_cntr = one + diry*self.circle_rad*2
                                elif CP == 'Down' and fliped:
                                    crcl_cntr = two - diry*self.circle_rad*2
                                # forgot howto 'else' in line iteration?
                                outeob1 = [ [lenvep+k+i,lenvep+k+i+1] for i in range(0,11) ]
                                outeob1.append([lenvep+k,lenvep+k+11])
                                outeob2 = [ [lenvep+k+i,lenvep+k+i+1] for i in range(12,23) ]
                                outeob2.append([lenvep+k+12,lenvep+k+23])
                                newinds1.extend(outeob1+outeob2)
                                newinds2.extend(outeob1+outeob2)
                                mat_rot_cir = n.rotation_difference(Vector((0,0,1))).to_matrix().to_4x4()
                                circle_to_add_1 = [vecir*mat_rot_cir+crcl_cntr+ \
                                        dirx_*self.circle_rad for vecir in circle ]
                                circle_to_add_2 = [vecir*mat_rot_cir+crcl_cntr- \
                                        dirx_*self.circle_rad for vecir in circle ]
                                vupperob.extend(circle_to_add_1+circle_to_add_2)
                                vlowerob.extend(circle_to_add_1+circle_to_add_2)
                                k += 24
                            if vec_tube and not tubes_flag_bed_solution_i_know:
                                for v in vec_tube:
                                    crcl_cntr = IL2P(v[0], v[1], l, n)
                                    if crcl_cntr:
                                        inside = crcl_cntr[0]<m1x and crcl_cntr[0]>m2x and crcl_cntr[1]<m1y \
                                             and crcl_cntr[1]>m2y and crcl_cntr[2]<=m1z and crcl_cntr[2]>=m2z
                                        if inside:
                                            outeob = [ [lenvep+k+i,lenvep+k+i+1] for i in range(0,23) ]
                                            outeob.append([lenvep+k,lenvep+k+23])
                                            newinds1.extend(outeob)
                                            newinds2.extend(outeob)
                                            mat_rot_cir = n.rotation_difference(Vector((0,0,1))).to_matrix().to_4x4()
                                            circle_to_add = [ vecir*mat_rot_cir+crcl_cntr for vecir in circle_tube ]
                                            vupperob.extend(circle_to_add)
                                            vlowerob.extend(circle_to_add)
                                            k += 24
                                tubes_flag_bed_solution_i_know = True
                        elif cop < 0.001 and inside and shortedge <= thick*threshold:
                            vupperob.extend([one,two])
                            vlowerob.extend([one,two])
                            newinds1.append([lenvep+k,lenvep+k+1])
                            newinds2.append([lenvep+k,lenvep+k+1])
                            k += 2
                    del tree
                    for e in deledges1:
                        if e in newinds1:
                            newinds1.remove(e)
                    for e in deledges2:
                        if e in newinds2:
                            newinds2.remove(e)
                    if vupperob or vlowerob:
                        outeup.append(newinds2)
                        outelo.append(newinds1)
                        vupper.append(vupperob)
                        vlower.append(vlowerob)
                vupper = Vector_degenerate(vupper)
                vlower = Vector_degenerate(vlower)
                centers = Vector_degenerate([centers])
                
                if 'vupper' in self.outputs and self.outputs['vupper'].links:
                    out = dataCorrect(vupper)
                    SvSetSocketAnyType(self, 'vupper', out)
                if 'outeup' in self.outputs and self.outputs['outeup'].links:
                    SvSetSocketAnyType(self, 'outeup', outeup)
                if 'vlower' in self.outputs and self.outputs['vlower'].links:
                    SvSetSocketAnyType(self, 'vlower', vlower)
                if 'outelo' in self.outputs and self.outputs['outelo'].links:
                    SvSetSocketAnyType(self, 'outelo', outelo)
                if 'centers' in self.outputs and self.outputs['centers'].links:
                    SvSetSocketAnyType(self, 'centers', centers)
                print(self.name, 'is finishing')
        

    def update_socket(self, context):
        self.update()


def register():
    bpy.utils.register_class(SvWafelNode)


def unregister():
    bpy.utils.unregister_class(SvWafelNode)


if __name__ == '__main__':
    register()






