#!/usr/bin/env python3
"""Rebuild the RobotRacers visual from the final RevQ EAGLE board.

Only visual meshes are generated here. Collision primitives, mass, inertia and
wheel articulation belong to the simulator model. All mesh coordinates are SI.
No Fusion API, proprietary CAD converter or runtime download is required.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / 'simulation' / 'assets'
SOURCES = ASSETS / 'sources'
OUTPUT = ASSETS / 'robot'
MM = .001
PCB_BOTTOM, PCB_TOP = .004, .0056
CAD_CENTRE = (65.15, 34.95)
AXLE_X, AXLE_Z = -.01315, .0115
DNP = {'C5', 'C6', 'J40', 'J41'}

# rgb, metalness, roughness. The exported MTL and Webots include share colours.
MATERIALS = {
    'mask': ((.018, .18, .11), .05, .38),
    'mask_top': ((.018, .18, .11), .05, .38),
    'fr4_edge': ((.39, .40, .22), .0, .65),
    'chip': ((.026, .030, .035), .05, .48),
    'resistor': ((.037, .040, .042), .02, .52),
    'ceramic': ((.57, .39, .20), .0, .50),
    'silver': ((.62, .65, .68), .85, .25),
    'aluminium': ((.72, .74, .77), .85, .23),
    'gold': ((.72, .55, .20), .78, .24),
    'white': ((.88, .88, .82), .0, .46),
    'black': ((.017, .019, .022), .0, .65),
    'rubber': ((.019, .021, .024), .0, .91),
    'inductor': ((.25, .27, .28), .40, .55),
    'yellow': ((.83, .66, .08), .0, .45),
    'red': ((.50, .025, .025), .0, .55),
    'blue': ((.03, .13, .40), .0, .55),
    'green': ((.04, .30, .09), .0, .55),
    'orange': ((.79, .30, .035), .0, .55),
    'battery': ((.10, .11, .13), .12, .50),
    'battery_label': ((.72, .73, .69), .15, .55),
    'lens': ((.20, .12, .14), .18, .16),
    'led': ((.04, .70, .31), .0, .28),
}


def fmt(v):
    return f'{v:.9g}'


def cad_xyz(x, y, z=PCB_TOP):
    return ((y-CAD_CENTRE[1])*MM, (CAD_CENTRE[0]-x)*MM, z)


class Mesh:
    def __init__(self):
        self.vertices = []
        self.faces = []
        self.uvs = []
        self.part = 'visual'
        self.parts = {}
        self.geometry_stats = {'input_polygons':0, 'triangulated_polygons':0,
                               'degenerate_triangles_dropped':0,
                               'crossed_quad_projections':0, 'crossed_quad_details':[]}

    def face(self, points, material, uv=None):
        self.geometry_stats['input_polygons'] += 1
        original_uv = uv or [(0,0)]*len(points)
        clean, clean_uv = [], []
        for point, texcoord in zip(points, original_uv):
            # Nanometre quantisation precedes triangulation so the validity test
            # uses the same coordinates that Webots will parse from the file.
            point=tuple(round(float(v),9)for v in point)
            if not clean or point != clean[-1]:
                clean.append(point); clean_uv.append(texcoord)
        if len(clean)>1 and clean[0] == clean[-1]:
            clean.pop(); clean_uv.pop()
        if len(clean)<3: return
        def cross(a,b,c):
            u=tuple(b[i]-a[i]for i in range(3));v=tuple(c[i]-a[i]for i in range(3))
            return (u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0])
        def area2(ids):
            n=cross(*(clean[i]for i in ids));return math.sqrt(sum(x*x for x in n))
        if len(clean)>3:
            self.geometry_stats['triangulated_polygons'] += 1
        if len(clean)==4:
            normal=max([cross(clean[0],clean[i],clean[i+1])for i in [1,2]],key=lambda n:sum(v*v for v in n))
            drop=max(range(3),key=lambda i:abs(normal[i]));xy=[tuple(p[i]for i in range(3)if i!=drop)for p in clean]
            def orient(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
            def crosses(a,b,c,d):
                return orient(a,b,c)*orient(a,b,d)<-1e-28 and orient(c,d,a)*orient(c,d,b)<-1e-28
            if crosses(xy[0],xy[1],xy[2],xy[3])or crosses(xy[1],xy[2],xy[3],xy[0]):
                self.geometry_stats['crossed_quad_projections'] += 1
                self.geometry_stats['crossed_quad_details'].append({'part':self.part,'points':clean})
            choices=[[(0,1,2),(0,2,3)],[(0,1,3),(1,2,3)]]
            triangles=max(choices,key=lambda candidate:min(area2(t)for t in candidate))
        else:
            triangles=[(0,i,i+1)for i in range(1,len(clean)-1)]
        for triangle in triangles:
            # Slabs narrower than meaningful visual precision can collapse at
            # the tangent of a PCB drill. They must not become invalid faces.
            if area2(triangle)<1e-14:
                self.geometry_stats['degenerate_triangles_dropped'] += 1
                continue
            start=len(self.vertices)
            self.vertices.extend(clean[i]for i in triangle)
            self.uvs.extend(clean_uv[i]for i in triangle)
            self.faces.append(((start,start+1,start+2),material,self.part))

    def box(self, centre, size, material, angle=0):
        x, y, z = centre
        a, b, c = (v/2 for v in size)
        co, si = math.cos(angle), math.sin(angle)
        v = [(x+co*i-si*j, y+si*i+co*j, z+k)
             for i, j, k in [(-a,-b,-c),(a,-b,-c),(a,b,-c),(-a,b,-c),
                             (-a,-b,c),(a,-b,c),(a,b,c),(-a,b,c)]]
        for face in [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]:
            self.face([v[i] for i in face], material)

    def cylinder(self, centre, radius, length, material, axis='z', segments=32, inner=0):
        x, y, z = centre
        def p(a, r, h):
            q = (math.cos(a)*r, math.sin(a)*r, h)
            if axis == 'y': q = (q[0], q[2], -q[1])
            if axis == 'x': q = (q[2], q[0], q[1])
            return (x+q[0], y+q[1], z+q[2])
        for i in range(segments):
            a, b = 2*math.pi*i/segments, 2*math.pi*(i+1)/segments
            self.face([p(a,radius,-length/2),p(b,radius,-length/2),p(b,radius,length/2),p(a,radius,length/2)],material)
            if inner:
                self.face([p(b,inner,-length/2),p(a,inner,-length/2),p(a,inner,length/2),p(b,inner,length/2)],material)
                self.face([p(a,inner,length/2),p(a,radius,length/2),p(b,radius,length/2),p(b,inner,length/2)],material)
                self.face([p(b,inner,-length/2),p(b,radius,-length/2),p(a,radius,-length/2),p(a,inner,-length/2)],material)
            else:
                self.face([p(0,0,length/2),p(a,radius,length/2),p(b,radius,length/2)],material)
                self.face([p(0,0,-length/2),p(b,radius,-length/2),p(a,radius,-length/2)],material)

    def tube(self, path, radius, material, segments=7):
        # Coherent central tangents and parallel-transported frames prevent the
        # initial ring from reversing relative to subsequent rings (bow ties).
        # Smooth the unmeasured cable route before sweeping it. Joining just
        # four sharply bent rings can fold a side quad even with coherent
        # tangents; a sampled C1 centreline keeps adjacent rings near-parallel.
        sampled=[]
        for i in range(len(path)-1):
            p0,p1=path[max(0,i-1)],path[i]
            p2,p3=path[i+1],path[min(len(path)-1,i+2)]
            for j in range(6):
                t=j/6;tt=t*t;ttt=tt*t
                sampled.append(tuple(.5*((2*p1[k])+(-p0[k]+p2[k])*t+(2*p0[k]-5*p1[k]+4*p2[k]-p3[k])*tt+(-p0[k]+3*p1[k]-3*p2[k]+p3[k])*ttt)for k in range(3)))
        path=sampled+[tuple(path[-1])]
        rings = []
        previous_u=None
        for i, c in enumerate(path):
            before,after=path[max(0,i-1)],path[min(i+1,len(path)-1)]
            t = tuple(after[k]-before[k] for k in range(3))
            n = math.sqrt(sum(v*v for v in t)) or 1
            t = tuple(v/n for v in t)
            if previous_u is None:
                ref=(0,0,1)if abs(t[2])<.8 else(1,0,0)
                u=(t[1]*ref[2]-t[2]*ref[1],t[2]*ref[0]-t[0]*ref[2],t[0]*ref[1]-t[1]*ref[0])
            else:
                dot=sum(previous_u[k]*t[k]for k in range(3))
                u=tuple(previous_u[k]-t[k]*dot for k in range(3))
            n = math.sqrt(sum(v*v for v in u)) or 1
            u = tuple(v/n for v in u)
            previous_u=u
            v = (t[1]*u[2]-t[2]*u[1],t[2]*u[0]-t[0]*u[2],t[0]*u[1]-t[1]*u[0])
            rings.append([tuple(c[k]+radius*(u[k]*math.cos(a*2*math.pi/segments)+v[k]*math.sin(a*2*math.pi/segments)) for k in range(3)) for a in range(segments)])
        for a,b in zip(rings,rings[1:]):
            for j in range(segments):
                self.face([a[j],a[(j+1)%segments],b[(j+1)%segments],b[j]],material)

    def start_part(self, name):
        self.part = name
        self.parts[name] = {'first_vertex': len(self.vertices)}

    def end_part(self, **metadata):
        d = self.parts[self.part]
        d.update(metadata)
        d['vertex_count'] = len(self.vertices)-d['first_vertex']
        vertices = self.vertices[d['first_vertex']:]
        if vertices:
            d['bounds_m'] = [[min(v[i] for v in vertices) for i in range(3)],
                             [max(v[i] for v in vertices) for i in range(3)]]

    def write(self, name):
        OUTPUT.mkdir(parents=True, exist_ok=True)
        used = sorted({f[1] for f in self.faces})
        mtl = []
        for material in used:
            rgb, metal, rough = MATERIALS[material]
            if material == 'mask_top': rgb = (1,1,1)
            mtl += [f'newmtl {material}', 'Kd '+' '.join(map(fmt,rgb)),
                    'Ks .35 .35 .35',f'Ns {fmt(max(2,200*(1-rough)))}',
                    f'Pm {metal}',f'Pr {rough}','d 1','illum 2']
            if material == 'mask_top': mtl.append('map_Kd pcb_top.png')
            mtl.append('')
        (OUTPUT/f'{name}.mtl').write_text('\n'.join(mtl))
        obj = [f'# RobotRacers RevQ; metres; X forward, Y left, Z up',f'mtllib {name}.mtl']
        obj += ['v '+' '.join(map(fmt,v)) for v in self.vertices]
        obj += ['vt '+' '.join(map(fmt,v)) for v in self.uvs]
        last_part = last_material = None
        for indices,material,part in self.faces:
            if part != last_part: obj.append('g '+part);last_part=part
            if material != last_material:obj.append('usemtl '+material);last_material=material
            obj.append('f '+' '.join(f'{i+1}/{i+1}' for i in indices))
        (OUTPUT/f'{name}.obj').write_text('\n'.join(obj)+'\n')
        # Include is intentionally free of PROTO/header so the world generator
        # can insert its Group literally, preserving all PBR appearance fields.
        chunks = ['Group { children [']
        for material in used:
            rgb, metal, rough = MATERIALS[material]
            appearance = 'PBRAppearance { baseColor '+' '.join(map(fmt,rgb))+f' metalness {metal} roughness {rough}'
            if material == 'mask_top':
                appearance = 'PBRAppearance { baseColor 1 1 1 metalness .08 roughness .38 baseColorMap ImageTexture { url [ "../../assets/robot/pcb_top.png" ] filtering 5 }'
            appearance += ' }'
            vertices,uvs,faces=[],[],[]
            for ids,mat,_ in self.faces:
                if mat != material:continue
                base=len(vertices);vertices.extend(self.vertices[i] for i in ids);uvs.extend(self.uvs[i] for i in ids)
                faces.append(tuple(range(base,base+len(ids))))
            chunks += ['Shape { appearance '+appearance+' geometry IndexedFaceSet {',
                       'coord Coordinate { point [ '+', '.join(' '.join(map(fmt,v)) for v in vertices)+' ] }',
                       'coordIndex [ '+', '.join(' '.join(map(str,f))+' -1' for f in faces)+' ]',
                       'texCoord TextureCoordinate { point [ '+', '.join(' '.join(map(fmt,v)) for v in uvs)+' ] }',
                       'ccw TRUE creaseAngle .35 } }']
        chunks += ['] }']
        include='\n'.join(chunks)+'\n'
        (OUTPUT/f'{name}.inc.wbt').write_text(include)
        (OUTPUT/f'{name}.wrl').write_text('#VRML_SIM R2025a utf8\n'+include)
        return {'files':[f'robot/{name}.{x}'for x in ['obj','mtl','wrl','inc.wbt']],
                'vertices':len(self.vertices),'faces':len(self.faces),
                'geometry_stats':self.geometry_stats,
                'bounds_m':[[min(v[i]for v in self.vertices)for i in range(3)],[max(v[i]for v in self.vertices)for i in range(3)]],
                'parts':self.parts}


def polygon_circle(x,y,r,n=48):
    return [(x+r*math.cos(2*math.pi*i/n),y+r*math.sin(2*math.pi*i/n))for i in range(n)]


def board(mesh, board_xml):
    """Extrude rectangle minus all drill polygons, using vertical slab meshing."""
    holes=[polygon_circle(65.15,38.95,16,96)]
    holes += [polygon_circle(float(h.get('x')),float(h.get('y')),float(h.get('drill'))/2,24)for h in board_xml.findall('./plain/hole')]
    # The large XT30/switch holes are visible through the final board as well.
    holes += [polygon_circle(x,y,r,20) for x,y,r in [(23.65,3.35,.9),(28.65,3.35,.9),(24.05,10.6,1.15),(28.75,10.6,1.15),(77.85,66.05,1.15),(82.55,66.05,1.15)]]
    xmin,xmax,ymin,ymax=.15,130.15,-.05,69.95
    xs=sorted(set(round(x,9)for poly in holes for x,y in poly)|{xmin,xmax})
    def cut(poly,x):
        yy=[]
        for a,b in zip(poly,poly[1:]+poly[:1]):
            if min(a[0],b[0])-1e-8 <= x <= max(a[0],b[0])+1e-8:
                if abs(a[0]-b[0])<1e-10: yy.extend([a[1],b[1]])
                else: yy.append(a[1]+(x-a[0])*(b[1]-a[1])/(b[0]-a[0]))
        return min(yy),max(yy)
    def top_uv(p):
        x,y,_=p
        # Source render has a 13-pixel border; bake crops it before this map.
        return ((.065-y)/.130,(x+.035)/.070)
    mesh.start_part('PCB_RevQ')
    for x1,x2 in zip(xs,xs[1:]):
        active=[poly for poly in holes if min(x for x,y in poly)<(x1+x2)/2<max(x for x,y in poly)]
        active.sort(key=lambda poly:sum(y for x,y in poly)/len(poly))
        low1=low2=ymin
        for poly in active+[None]:
            if poly is None: high1=high2=ymax;next1=next2=ymax
            else:
                high1,next1=cut(poly,x1);high2,next2=cut(poly,x2)
            if high1-low1>1e-8 or high2-low2>1e-8:
                points=[cad_xyz(x1,low1),cad_xyz(x2,low2),cad_xyz(x2,high2),cad_xyz(x1,high1)]
                mesh.face(points,'mask_top',[top_uv(p)for p in points])
                mesh.face([(x,y,PCB_BOTTOM)for x,y,z in reversed(points)],'mask')
            low1,low2=next1,next2
    edges=[[(xmin,ymin),(xmax,ymin),(xmax,ymax),(xmin,ymax)]]+holes
    for poly in edges:
        for a,b in zip(poly,poly[1:]+poly[:1]):
            mesh.face([cad_xyz(*a,PCB_BOTTOM),cad_xyz(*b,PCB_BOTTOM),cad_xyz(*b,PCB_TOP),cad_xyz(*a,PCB_TOP)],'fr4_edge')
    mesh.end_part(provenance='RevQ board layer20 and holes',board_dimensions_m=[.130,.070,.0016],central_cutout_diameter_m=.032,mounting_hole_count=10)


def rotation(element):
    return math.radians(float(element.get('rot','R0').split('R')[-1]))


def component(mesh, e, package):
    name,pkg,value=e.get('name'),e.get('package'),e.get('value','')
    ex,ey=float(e.get('x')),float(e.get('y'));theta=rotation(e)
    co,si=math.cos(theta),math.sin(theta)
    def pt(x,y,z): return cad_xyz(ex+co*x-si*y,ey+si*x+co*y,PCB_TOP+z*MM)
    def box(x,y,z,dx,dy,dz,mat):mesh.box(pt(x,y,z),(dx*MM,dy*MM,dz*MM),mat,theta-math.pi/2)
    def cylinder(x,y,z,r,h,mat,n=20,inner=0):mesh.cylinder(pt(x,y,z),r*MM,h*MM,mat,segments=n,inner=inner*MM)
    mesh.start_part(name)
    # Solder joints and actual footprint coordinates, including non-centred ESP32.
    for pad in package.findall('smd'):
        if pad.get('layer')!='1':continue
        x,y,dx,dy=(float(pad.get(k))for k in ['x','y','dx','dy'])
        a=theta+rotation(pad)-math.pi/2
        mesh.box(pt(x,y,.065),(dx*MM*.92,dy*MM*.92,.13*MM),'silver',a)
    if pkg.startswith(('RESC','CAPC')):
        dims={'1608':(1.6,.8,.6 if pkg.startswith('RES')else .85),'2012':(2,1.2,1.1),'3216':(3.2,1.6,.7 if pkg.startswith('RES')else 1.35),'1005':(1,.5,.5)}
        dx,dy,dz=next((v for k,v in dims.items()if k in pkg),(1.6,.8,.8))
        box(0,0,dz/2+.12,dx,dy,dz,'resistor'if pkg.startswith('RES')else'ceramic')
        for x in [-dx*.40,dx*.40]:box(x,0,dz/2+.14,dx*.2,dy*1.01,dz*1.01,'silver')
    elif pkg=='ESP32-S3-WROOM-1':
        box(9,12.75,.40,18,25.5,.65,'mask')
        box(9,9.75,1.65,17.9,19.4,2.3,'silver')
        # RF antenna is the bare PCB end, never a second metal cover.
        points=[(1.2,20.6),(1.2,24.2),(4.5,24.2),(4.5,21.3),(7.5,21.3),(7.5,24.2),(10.4,24.2),(10.4,21.3),(13.1,21.3),(13.1,24.2),(16.6,24.2)]
        for a,b in zip(points,points[1:]):
            mx,my=(a[0]+b[0])/2,(a[1]+b[1])/2
            box(mx,my,.78,max(.28,abs(a[0]-b[0])),max(.28,abs(a[1]-b[1])),.045,'gold')
        box(9,9.5,2.82,8,.18,.025,'chip');box(9,11,2.82,5,.12,.025,'chip')
    elif pkg=='CAPAE660X540N':
        box(0,0,.35,6.6,6.6,.55,'chip');cylinder(0,0,2.9,3.15,4.9,'aluminium',40)
        cylinder(0,0,5.37,2.7,.05,'silver',40)
        box(0,0,5.405,4.2,.09,.025,'chip');box(0,0,5.405,.09,4.2,.025,'chip')
    elif pkg in ['JST-6-SMD-VERT-1.0MM','JST_SH_4','JST_SH_10','JST_XH_B2B_VERTICAL']:
        count=6 if pkg.startswith('JST-6')else 4 if pkg=='JST_SH_4'else 10 if pkg=='JST_SH_10'else 2
        dx=8 if count==6 else 6 if count==4 else 12 if count==10 else 7.4
        dy=3.0 if count!=2 else 5.75;h=4.25 if count!=2 else 7.0;cy=-.675 if count!=2 else -.525
        box(0,cy,.45,dx,dy,.75,'white');box(0,cy,h-.45,dx,dy,.35,'white')
        # Open socket: walls surround a dark recessed well; lid is not solid.
        box(0,cy,h-.38,dx-1.0,dy-1.0,.37,'black')
        for x in [-dx/2+.3,dx/2-.3]:box(x,cy,h/2,.6,dy,h,'white')
        for y in [cy-dy/2+.25,cy+dy/2-.25]:box(0,y,h/2,dx,.5,h,'white')
        pitch=1 if count!=2 else 2.5
        for i in range(count):cylinder((i-(count-1)/2)*pitch,cy,h-.45,.13,.75,'gold',8)
    elif pkg=='XT30UPB-M':
        box(0,0,5.35,10.2,5.6,10.7,'yellow')
        for x in [-2.5,2.5]:
            cylinder(x,0,10.76,1.65,.10,'black',24)
            cylinder(x,0,10.85,1.42,.18,'gold',24,inner=1.0)
    elif pkg=='12401832E402A':
        box(0,0,1.65,9.2,10.5,3.1,'silver')
        box(0,-5.28,1.65,7.8,.04,2.35,'black')
        box(0,-5.33,1.65,6.1,.08,.50,'chip')
        for x in [-2.65,-2.15,-1.65,-1.15,-.65,-.15,.35,.85,1.35,1.85,2.35,2.85]:box(x,-5.39,1.90,.22,.025,.035,'gold')
    elif pkg=='B3U-1000P(M)':
        box(0,0,.55,3,2.5,1.0,'white');box(0,0,1.22,1.55,1.4,.38,'chip')
        for x in [-1.35,1.35]:box(x,0,.75,.30,2.5,.65,'silver')
    elif pkg=='MLT5030':
        box(0,0,1.6,5.2,5.2,3,'chip');cylinder(0,0,3.12,.9,.035,'black',24)
    elif pkg=='SK6805_EC15':
        box(0,0,.40,1.5,1.5,.65,'white');box(0,0,.75,.9,.85,.09,'led')
    elif 'IHLP4040' in pkg:
        box(0,0,2.15,10.7,10.1,4,'inductor')
        for x in [-4.5,4.5]:box(x,0,.5,3.12,3.25,.7,'silver')
        box(0,0,4.20,3,.24,.025,'chip')
    elif pkg=='YSPI0530_REVN':
        box(0,0,1.65,5.7,5.4,3,'inductor');box(0,0,3.18,2.2,.20,.025,'chip')
    else:
        # Nominal manufacturer body sizes, not oversized silk courtyard boxes.
        dims={'TI_RYQ0021A':(5,3,1.2),'TI_PWP0016J':(4.4,5,1.2),'TI_DRL0008A':(2.1,1.6,1.0),'TSSOP16_PW':(4.4,5,1.2),'SOP65P640X120-14':(4.4,5,1.2),'TO252':(6.5,6.1,2.35),'SMC_DO214AB':(6.9,5.9,2.4),'SMBJ6.5A':(4.6,3.6,2.45),'SS14':(4.3,2.7,2.6),'SOT95P280X145-5N':(1.6,2.9,1.2),'SOT95P280X145-6N':(1.6,2.9,1.2),'SOT95P237X112-3N':(1.3,2.1,1.0)}
        dx,dy,dz=dims.get(pkg,(2.5,2,1.2));box(0,0,dz/2+.15,dx,dy,dz,'chip')
        if pkg=='TO252':box(0,3.25,.35,5.8,1.5,.4,'silver')
        if pkg in ['SS14','SMBJ6.5A','SMC_DO214AB']:box(-dx*.33,0,dz+.17,.35,dy,.025,'silver')
        elif name.startswith('U'):cylinder(-dx*.32,dy*.32,dz+.17,.12,.035,'white',10)
    mesh.end_part(kind='PCB component',package=pkg,value=value,position_cad_mm=[ex,ey],rotation_cad_deg=math.degrees(theta),origin='final RevQ XML; dimensional visual reconstruction',physical_model=False)


def additions(mesh):
    # These mechanical assemblies are dimensioned prototypes where the final
    # assembled CAD, cable lengths and mounting measurements were not supplied.
    for side,sign in [('left',1),('right',-1)]:
        mesh.start_part('Pololu5178_'+side)
        mesh.box((AXLE_X,sign*.0505,AXLE_Z),(.012,.022,.010),'silver')
        mesh.box((AXLE_X,sign*.062,AXLE_Z),(.012,.006,.010),'gold')
        for x in [AXLE_X-.0045,AXLE_X+.0045]:
            mesh.cylinder((x,sign*.0651,AXLE_Z+.0032),.00065,.0003,'silver','y',12)
        mesh.cylinder((AXLE_X,sign*.070,AXLE_Z),.0015,.010,'silver','y',24)
        mesh.box((AXLE_X,sign*.0365,AXLE_Z),(.0125,.006,.0105),'chip')
        mesh.box((AXLE_X,sign*.0332,AXLE_Z+.001),(.008,.001,.0045),'white')
        mesh.end_part(kind='dimensioned reconstruction',source='https://www.pololu.com/product/5178',mass_kg_manufacturer=.011,shaft_axis='Y',shaft_centre_m=[AXLE_X,sign*.065,AXLE_Z])
        mesh.start_part('Pololu1089_'+side)
        mesh.box((AXLE_X,sign*.060,PCB_TOP+.0006),(.024,.010,.0012),'black')
        for x in [AXLE_X-.0067,AXLE_X+.0067]:mesh.box((x,sign*.0595,.0118),(.0014,.013,.0115),'black')
        for x in [AXLE_X-.009,AXLE_X+.009]:
            mesh.cylinder((x,sign*.060,.0072),.0017,.0015,'silver',segments=16)
            mesh.box((x,sign*.060,.0080),(.002,.00028,.00003),'black')
        mesh.end_part(kind='dimensioned bracket reconstruction',source='Pololu1089 STEP and final RevQ mounting holes',hole_pitch_m=.018)
        mesh.start_part('Encoder_cable_'+side)
        start=(-.0093,sign*.027,.011)
        for k,mat in enumerate(['green','white','yellow','blue','black','red']):
            off=(k-2.5)*.00065
            mesh.tube([(start[0]+off,start[1],start[2]),(-.002+off,sign*.030,.018),(-.008+off,sign*.034,.020),(AXLE_X+off,sign*.033,.014)],.00027,mat)
        mesh.end_part(kind='JST SH 6-wire visual',wire_diameter_m=.00054,length='illustrative; not measured')
    mesh.start_part('Turbine_32mm')
    centre=(.004,0,.012)
    mesh.cylinder(centre,.016,.008,'black',segments=64,inner=.014)
    mesh.cylinder((.004,0,.011),.0045,.007,'silver',segments=24)
    for i in range(12):
        a=i*math.pi/6;r1=.004;r2=.0134
        p=[(.004+r1*math.cos(a),r1*math.sin(a),.0085),(.004+r2*math.cos(a+.34),r2*math.sin(a+.34),.0085),(.004+r2*math.cos(a+.34),r2*math.sin(a+.34),.013),(.004+r1*math.cos(a),r1*math.sin(a),.013)]
        mesh.face(p,'inductor');mesh.face(list(reversed(p)),'inductor')
    mesh.cylinder((.004,0,.022),.005,.020,'silver',segments=32)
    mesh.cylinder((.004,0,.0323),.005,.001,'chip',segments=32)
    for mat,y in [('red',-.0012),('black',.0012)]:mesh.tube([(.004,-.022+y,.012),(.006,-.019+y,.022),(.007,-.012+y,.034),(.005,y,.033)],.00045,mat)
    mesh.end_part(kind='unmeasured 1020/32mm turbine visual',rotor_diameter_m=.027,suction='not inferred from geometry; physics configurable separately')
    mesh.start_part('Battery_2S_400mAh_estimated')
    mesh.box((.005,.039,.0265),(.036,.020,.013),'battery')
    mesh.box((.005,.039,.0331),(.027,.015,.00018),'battery_label')
    for x in [-.007,.017]:mesh.box((x,.039,.0335),(.0025,.021,.0008),'black')
    for mat,dy in [('red',-.001),('black',.001)]:mesh.tube([(-.013,.039+dy,.026),(-.022,.041+dy,.026),(-.033,.039+dy,.022),(-.0315,.039+dy,.016)],.00055,mat)
    mesh.end_part(kind='explicitly estimated visual',dimensions_m=[.036,.020,.013],nominal_voltage_v=7.4,capacity_mah=400,mounting='unconfirmed; elevated for component clearance',not_mass_measurement=True)
    for side,y in [('left',.052),('right',-.052)]:
        mesh.start_part('QTRX_HD_01A_'+side)
        mesh.box((.047,y,.0092),(.020,.005,.001),'mask')
        for x in [.0405,.0445]:mesh.cylinder((x,y,.00815),.0012,.0011,'lens',segments=16)
        for k,mat in enumerate(['white','red','black','orange']):
            dy=(k-1.5)*.0008
            mesh.tube([(.031,y+dy,.010),(.035,y+dy,.014),(.040,y+dy,.011)],.00023,mat)
        mesh.end_part(kind='Pololu4401 dimensional reconstruction',source='https://www.pololu.com/product/4401',dimensions_m=[.020,.005,.0044],mounting='provisional forward/down-facing; not a rangefinder')
    mesh.start_part('Sensor_bar_reference')
    mesh.box((.061,0,.0085),(.018,.064,.0012),'mask')
    for i in range(16):
        y=(i-7.5)*.004
        for x in [.058,.064]:mesh.cylinder((x,y,.00725),.0008,.0011,'lens',segments=10)
    for k in range(10):
        y=(k-4.5)*.0008;mesh.tube([(.033,y,.010),(.042,y,.014),(.055,y,.010)],.00023,'black')
    mesh.end_part(kind='provisional custom sensor bar visual',channels=16,source='Pololu4415 is a dimensional/electrical reference only',not_actual_manufacturer_part=True,multiplexer='external on bar; model not identified')
    mesh.start_part('Front_skid_estimated')
    # Provisional forward support, aligned to the physical support at base X=.065.
    # Its attachment and height have not been measured on the assembled robot.
    contact_x=.05185
    mesh.box((contact_x-.010,0,.0040),(.028,.013,.002),'black')
    mesh.cylinder((contact_x,0,.0026),.0018,.0012,'black',segments=20)
    mesh.cylinder((contact_x,0,.0010),.0045,.0020,'white',segments=28)
    mesh.end_part(kind='estimated low-friction support visual',not_measured=True,
                  contact_centre_m=[contact_x,0,0],physical_base_contact_centre_m=[.065,0,0],
                  mounting='provisional forward support; not measured assembled CAD')


def wheel():
    m=Mesh();m.start_part('Silicone_tire_23x40')
    m.cylinder((0,0,0),.0115,.040,'rubber','y',96,inner=.0075)
    # Thin edge rings and an asymmetric seam make wheel rotation observable.
    for y in [-.0198,.0198]:m.cylinder((0,y,0),.01145,.0004,'black','y',96,inner=.0108)
    for a in [0,math.pi/2,math.pi,3*math.pi/2]:
        r=.0115001
        points=[(r*math.cos(t),y,r*math.sin(t))for t,y in [(a-.009,-.018),(a+.009,-.018),(a+.009,.018),(a-.009,.018)]]
        m.face(list(reversed(points)),'black')
    m.end_part(kind='nominal Ja-Bots silicone geometry',diameter_m=.023,width_m=.040,hardness='10A per vendor, not a calibrated friction coefficient')
    m.start_part('Aluminium_hub')
    m.cylinder((0,0,0),.0075,.0396,'chip','y',64,inner=.0015)
    for y in [-.0200,.0200]:
        m.cylinder((0,y,0),.0074,.00045,'aluminium','y',64,inner=.0015)
        m.cylinder((0,y*1.025,0),.0032,.0006,'chip','y',32,inner=.0015)
        for i in range(4):
            a=i*math.pi/2+.25
            m.cylinder((.005*math.cos(a),y*1.018,.005*math.sin(a)),.0008,.0001,'black','y',12)
        # Set-screw head, offset from axle, provides an orientation cue.
        m.cylinder((.0036,y*1.035,.0021),.0008,.0004,'silver','y',12)
    m.end_part(kind='dimensioned hub reconstruction',rim_diameter_m=.015,shaft_diameter_m=.003,source='https://ja-bots.com/producto/llantas-seguidor-de-linea-largas-23x40-negro/')
    return m


def prepare_sources(source_root):
    files={
        'pcb/RobotRacers_RevQ.brd':source_root/'outputs/RobotRacers_RevQ/RobotRacers_RevQ.brd',
        'pcb/BOM_RevQ.csv':source_root/'outputs/RobotRacers_RevQ/Fabricacion/RobotRacers_RevQ_BOM_JLCPCB.csv',
        'pcb/Serigrafia_Top.png':source_root/'outputs/RobotRacers_RevQ/Serigrafia_Top.png',
        'reference/ESP32-S3-WROOM-1_oficial.step':source_root/'outputs/ESP32S3_Core_RevB/modelos_3D/ESP32-S3-WROOM-1_oficial.step',
        'reference/LICENSE_Espressif_KiCad.md':source_root/'outputs/ESP32S3_Core_RevB/modelos_3D/LICENSE_Espressif_KiCad.md',
    }
    for name,src in files.items():
        dst=SOURCES/name
        if not dst.exists():
            if not src.exists():raise FileNotFoundError(f'Missing source {src}; provide --source-root')
            dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
    reference = SOURCES/'reference'/'Pololu1089_reference.json'
    reference.parent.mkdir(parents=True,exist_ok=True)
    reference.write_text(json.dumps({'source':'https://www.pololu.com/file/0J1712/micro-metal-gearmotor-extended-with-1089-extended-bracket.step','dimensions_used_mm':{'mount_hole_pitch':18,'PCB_hole_edge_distance':4.25,'motor_cross_section':[10,12]},'licence':'Manufacturer CAD redistribution licence not established; STEP is not redistributed. Only dimensions inform the original generated mesh.','note':'Reference STEP depicts extended-shaft motor; it is not the exact encoder-back5178 model.'},indent=2)+'\n')
    OUTPUT.mkdir(parents=True,exist_ok=True)
    # This is a deterministic CAD texture bake, not an alteration of an image
    # of the real robot. Only the render's blue solder mask becomes quote Green.
    try:
        from PIL import Image
        im=Image.open(SOURCES/'pcb/Serigrafia_Top.png').convert('RGB').crop((13,13,2035,1103))
        pix=im.load()
        for y in range(im.height):
            for x in range(im.width):
                r,g,b=pix[x,y]
                if b>r*1.5 and b>g*1.25 and r<60:pix[x,y]=(8,62,38)
        # Webots/Wren expects power-of-two texture dimensions. UVs cover the
        # full CAD board, so resampling the raster does not change its scale.
        im=im.resize((2048,1024),Image.Resampling.LANCZOS)
        im.save(OUTPUT/'pcb_top.png',optimize=True)
    except ImportError:
        if not (OUTPUT/'pcb_top.png').exists():raise RuntimeError('Install Pillow to rebuild the PCB texture; runtime does not need Pillow.')
    return [{'path':'sources/'+str(p.relative_to(SOURCES)),'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size}for p in sorted(SOURCES.rglob('*'))if p.is_file()]


def verify_assets(manifest):
    """Validate serialized meshes against Webots R2025a geometry restrictions."""
    checks={
        'mounted_count_210':manifest['mounted_components']==210,
        'removed_parts_absent':not(set(manifest['body']['parts'])&{'C5','C6','U30','S5','S6','J40','J41'}),
        'units_m':manifest['units']=='m',
        'PCB_dimensions_correct':manifest['body']['parts']['PCB_RevQ']['board_dimensions_m']==[.13,.07,.0016],
        'turbine_hole_32mm':manifest['body']['parts']['PCB_RevQ']['central_cutout_diameter_m']==.032,
        'wheel_radius_11_5mm':manifest['wheel_radius_m']==.0115,
        'wheel_width_40mm':manifest['wheel_width_m']==.040,
        'wheel_axis_y':manifest['wheel_axis']==[0,1,0],
        'battery_over_component_height':manifest['clearances']['battery_bottom_m']>manifest['clearances']['maximum_PCB_component_top_m'],
        'texture_path_resolves':(ROOT/'simulation/webots/protos/../../assets/robot/pcb_top.png').resolve().is_file(),
    }
    exported={}
    for name in ['body','wheel']:
        vertices=[];faces=[]
        for line in (OUTPUT/f'{name}.obj').read_text().splitlines():
            if line.startswith('v '):vertices.append(tuple(map(float,line.split()[1:])))
            elif line.startswith('f '):faces.append(tuple(int(x.split('/')[0])-1 for x in line.split()[1:]))
        checks[name+'_finite_vertices']=all(math.isfinite(x)for p in vertices for x in p)
        checks[name+'_triangles_only']=all(len(f)==3 for f in faces)
        checks[name+'_indices_valid']=all(len(set(f))==3 and min(f)>=0 and max(f)<len(vertices)for f in faces)
        checks[name+'_face_count']=len(faces)==manifest[name]['faces']
        min_area2=math.inf
        for f in faces:
            a,b,c=(vertices[i]for i in f)
            u=[b[i]-a[i]for i in range(3)];v=[c[i]-a[i]for i in range(3)]
            cross=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
            min_area2=min(min_area2,math.sqrt(sum(x*x for x in cross)))
        checks[name+'_no_degenerate_triangles']=min_area2>=1e-14
        checks[name+'_no_crossed_quad_projections']=manifest[name]['geometry_stats']['crossed_quad_projections']==0
        include=(OUTPUT/f'{name}.inc.wbt').read_text()
        checks[name+'_Webots_R2025a_fields']=not re.search(r'\bsolid\b',include) and 'ccw TRUE' in include
        vrml_faces=[f.strip().replace(',',' ').split()for block in re.findall(r'coordIndex\s*\[([^\]]+)\]',include)for f in block.split('-1')if f.strip().strip(',')]
        checks[name+'_VRML_triangles_only']=len(vrml_faces)==len(faces) and all(len(f)==3 for f in vrml_faces)
        exported[name]={'triangle_count':len(faces),'minimum_twice_triangle_area_m2':min_area2,'sha256':hashlib.sha256((OUTPUT/f'{name}.inc.wbt').read_bytes()).hexdigest()}
        if name=='wheel':checks['wheel_radius_geometry']=max(math.hypot(x,z)for x,y,z in vertices)<=.0115002
    checks['files_under_GitHub_limit']=max(p.stat().st_size for p in ASSETS.rglob('*')if p.is_file())<100*1024*1024
    report={'checks':checks,'passed':sum(checks.values()),'failed':sum(not x for x in checks.values()),'exported_geometry':exported,
            'schema_reference':'Webots R2025a Contents/Resources/nodes/IndexedFaceSet.wrl: ccw supported; solid is not a field',
            'scope':'Serialized visual geometry and source registration, not a physical calibration or PCBA verification'}
    (ASSETS/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
    if not all(checks.values()):raise ValueError('Asset validation failed: '+', '.join(k for k,v in checks.items()if not v))
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root',type=Path,default=ROOT.parent)
    args=parser.parse_args()
    sources=prepare_sources(args.source_root)
    r=ET.parse(SOURCES/'pcb/RobotRacers_RevQ.brd').getroot().find('./drawing/board')
    packages={(l.get('name'),p.get('name')):p for l in r.findall('./libraries/library')for p in l.findall('./packages/package')}
    mounted=[e for e in r.findall('./elements/element')if e.get('name')not in DNP]
    assert len(mounted)==210,len(mounted)
    assert not any(e.get('name')in {'U30','S5','S6'}for e in mounted)
    assert all('M'not in e.get('rot','R0')for e in mounted)
    body=Mesh();board(body,r)
    for e in mounted:component(body,e,packages[(e.get('library'),e.get('package'))])
    additions(body)
    body_data=body.write('body');wheel_data=wheel().write('wheel')
    manifest={
        'revision':'RobotRacers PCB RevQ / visual reconstruction 1',
        'units':'m','frame':{'x':'forward; CAD+y','y':'left; CAD-x','z':'up'},
        'cad_centre_mm':CAD_CENTRE,'pcb_bottom_m':PCB_BOTTOM,'pcb_top_m':PCB_TOP,
        'mounted_components':210,'pcb_elements':214,'excluded_bodies':sorted(DNP|{'U30','S5','S6'}),
        'body':body_data,'wheel':wheel_data,
        'wheel_placement_in_body_frame_m':{'left':[AXLE_X,.095,AXLE_Z],'right':[AXLE_X,-.095,AXLE_Z]},
        'wheel_axis':[0,1,0],'wheel_radius_m':.0115,'wheel_width_m':.040,
        'body_translation_for_axle_origin_m':[.01315,0,0],
        'marker_platform':{'generated_by':'supervisor/world, not body mesh','centre_in_body_frame_m':[-.040,0,.023],'size_m':.040,'carrier_size_m':.052},
        'physical_colliders':'NONE: visual only; root simulator owns separate simplified collision geometry',
        'clearances':{'battery_bottom_m':.020,'maximum_PCB_component_top_m':max(d['bounds_m'][1][2]for d in body.parts.values()if d.get('kind')=='PCB component'),'wheel_inner_edges_y_m':[-.075,.075],'motor_case_inner_edge_abs_y_m':.033,'motor_case_bottom_m':.0065},
        'source_files':sources,
        'provenance':[
            {'item':'PCB positions/pads/silkscreen','source':'RobotRacers RevQ .brd and source render','licence':'Project design; repository MIT'},
            {'item':'Motor and encoder dimensions','source':'https://www.pololu.com/product/5178/specs','notes':'Dimensioned visual reconstruction, not converted official CAD'},
            {'item':'Bracket dimensional reference','source':'https://www.pololu.com/file/0J1712/micro-metal-gearmotor-extended-with-1089-extended-bracket.step','notes':'Reference for bracket and holes; old extended-shaft motor is not presented as exact5178 CAD','licence':'Manufacturer reference; keep attribution; not relicensed MIT'},
            {'item':'ESP32 STEP reference','source':'https://github.com/espressif/kicad-libraries','licence':'KiCad library licence supplied in sources/reference'},
            {'item':'Rim/tire','source':'https://ja-bots.com/producto/llantas-seguidor-de-linea-largas-23x40-negro/','notes':'Nominal23x40 geometry; no CAD or mass supplied'},
            {'item':'Reflectance sensors','source':'https://www.pololu.com/product/4401','notes':'Reconstructed dimensions; positions provisional; no fictitious range sensor'},
            {'item':'Battery, turbine, cables, support, custom sensor bar','source':'Project discussion and dimensioned provisional reconstruction','licence':'Generated project meshes; MIT','notes':'Not measured final assembled geometry, mass or aerodynamic calibration'}
        ],
        'unknown_mechanical_parameters':['total mass','centre of gravity','inertia tensor','actual battery dimensions/mass/mounting','real custom sensor bar and its mounting','cable lengths/routes','turbine housing/impeller final CAD','tyre friction/rolling resistance/compression','support final CAD'],
        'rebuild':'python3 simulation/tools/build_assets.py; Pillow needed only for CAD texture bake',
        'integration':'Insert body.inc.wbt Group literally under Transform translation0.01315 0 0 in Robot PROTO. Insert wheel.inc.wbt at each joint; wheel local axis Y. Texture URL is relative to simulation/webots/protos.'
    }
    (ASSETS/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')
    verification=verify_assets(manifest)
    print(json.dumps({'mounted_components':210,'body_vertices':body_data['vertices'],'body_faces':body_data['faces'],'wheel_faces':wheel_data['faces'],'body_bounds_m':body_data['bounds_m'],'validation_passed':verification['passed'],'validation_failed':verification['failed'],'output':str(OUTPUT)},indent=2))


if __name__=='__main__':main()
