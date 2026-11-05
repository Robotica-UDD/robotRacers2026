#!/usr/bin/env python3
"""Generate a fully local Webots world from the checked-in RevQ geometry."""
from pathlib import Path
import json
import math
import sys

ROOT = Path(__file__).resolve().parents[2]
SIM = ROOT/'simulation'

def marker_texture(identifier, path, size=512):
    import cv2
    import numpy as np
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    tag=cv2.aruco.generateImageMarker(dictionary, identifier, size)
    # The white carrier is wider than the encoded 40/48mm tag.
    cv2.imwrite(str(path),tag)

def plane_tag(identifier, cx,cy,z,side,robot=False):
    # Image's top edge corresponds to robot +X or fixed-world +Y.
    if robot:
        points=[(cx+side/2,cy+side/2,z),(cx+side/2,cy-side/2,z),(cx-side/2,cy-side/2,z),(cx-side/2,cy+side/2,z)]
    else:
        points=[(cx-side/2,cy+side/2,z),(cx+side/2,cy+side/2,z),(cx+side/2,cy-side/2,z),(cx-side/2,cy-side/2,z)]
    coords=', '.join(' '.join(map(str,p)) for p in points)
    # Positive texture V points upward; image PNG is automatically flipped by the renderer.
    return f'''Pose {{ translation {cx} {cy} {z-.0005} children [
      Shape {{ appearance PBRAppearance {{ baseColor 1 1 1 roughness 1 metalness 0 }} geometry Box {{ size {side*1.3} {side*1.3} .0006 }} }}
    ] }}
    Shape {{ appearance PBRAppearance {{ baseColorMap ImageTexture {{ url ["../../assets/markers/tag_{identifier}.png"] filtering 0 }} roughness 1 metalness 0 }}
      geometry IndexedFaceSet {{ coord Coordinate {{ point [{coords}] }} coordIndex [0,3,2,1,-1]
      texCoord TextureCoordinate {{ point [0 1,1 1,1 0,0 0] }} texCoordIndex [0,3,2,1,-1] ccw TRUE }} }}'''

def generate():
    (SIM/'webots/protos').mkdir(parents=True,exist_ok=True)
    markers=SIM/'assets/markers';markers.mkdir(parents=True,exist_ok=True)
    for i in range(5): marker_texture(i, markers/f'tag_{i}.png')
    p=json.loads((SIM/'config/robot.json').read_text())
    radius=p['wheel_radius_m'];width=p['wheel_width_m'];track=p['track_width_m'];wheel_mass=p['wheel_mass_kg']
    wheel_inertia_radial=wheel_mass*(3*radius**2+width**2)/12
    wheel_inertia_axial=wheel_mass*radius**2/2
    center=' '.join(map(str,p['body_center_of_mass_m']))
    inertia=' '.join(map(str,p['body_inertia_kg_m2']))
    body_path=SIM/'assets/robot/body.inc.wbt'
    wheel_path=SIM/'assets/robot/wheel.inc.wbt'
    if not body_path.exists() or not wheel_path.exists():
        raise SystemExit('Generate RevQ assets first: python simulation/tools/build_assets.py')
    body=body_path.read_text();wheel=wheel_path.read_text()
    # Native child fragments carry exactly the generated CAD geometry, including materials.
    body='\n'.join(x for x in body.splitlines() if not x.startswith('#VRML'))
    wheel='\n'.join(x for x in wheel.splitlines() if not x.startswith('#VRML'))
    wheels=[]
    for name,y in [('left',track/2),('right',-track/2)]:
        wheels.append(f'''HingeJoint {{ jointParameters HingeJointParameters {{ axis 0 1 0 anchor 0 {y} {radius} dampingConstant 0 staticFriction .00001 }}
        device [ RotationalMotor {{ name "{name}_motor" maxVelocity 800 maxTorque .015 }} PositionSensor {{ name "{name}_encoder" }} ]
        endPoint Solid {{ translation 0 {y} {radius} children [Transform {{ scale {radius/.0115} {width/.04} {radius/.0115} children [{wheel}] }}] name "{name}_wheel" contactMaterial "rubber"
          boundingObject Pose {{ rotation 1 0 0 -1.57079632679 children [ Cylinder {{ radius {radius} height {width} subdivision 32 }} ] }}
          physics Physics {{ density -1 mass {wheel_mass} centerOfMass [0 0 0] inertiaMatrix [{wheel_inertia_radial} {wheel_inertia_axial} {wheel_inertia_radial} 0 0 0] }} }} }}''')
    tag=plane_tag(0,-.02685,0,.023,.04,True)
    line_sensors='\n'.join(f'''DistanceSensor {{ translation .079 {(.028-i*.003733333):.8f} .0045 rotation 0 1 0 1.57079632679 name "line_reflectance_{i}" type "infra-red" lookupTable [0 1000 0,.01 500 .02,.03 0 .05] }}''' for i in range(16))
    proto=f'''#VRML_SIM R2025a utf8
# Copyright Robotica UDD. MIT. Generated from RevQ; dynamics are provisional.
PROTO RobotRacers [ field SFVec3f translation -.6 .55 0 field SFRotation rotation 0 0 1 -1.57079632679 field SFFloat bodyMass {p['mass_kg']-2*wheel_mass} field MFVec3f bodyInertia [{inertia},0 0 0] field SFVec3f markerShift 0 0 0 ] {{
  Robot {{ translation IS translation rotation IS rotation name "RobotRacers" controller "robot_driver"
    contactMaterial "skid" synchronization TRUE
    children [
      Pose {{ translation .01315 0 0 children [ {body} ] }}
      DEF ROBOT_MARKER Pose {{ translation IS markerShift children [{tag}] }}
      LED {{ translation -.018 .039 .012 name "status_rgb" color [] children [ Shape {{ appearance PBRAppearance {{ baseColor .02 .15 .04 emissiveColor .01 .8 .1 metalness 0 roughness .5 }} geometry Sphere {{ radius .0016 subdivision 1 }} }} ] }}
      DistanceSensor {{ translation .052 .055 .005 rotation 0 1 0 1.57079632679 name "reflectance_left" type "infra-red" lookupTable [0 1000 0,.01 500 .02,.03 0 .05] numberOfRays 3 aperture .15 }}
      DistanceSensor {{ translation .052 -.055 .005 rotation 0 1 0 1.57079632679 name "reflectance_right" type "infra-red" lookupTable [0 1000 0,.01 500 .02,.03 0 .05] numberOfRays 3 aperture .15 }}
      {line_sensors}
      {''.join(wheels)}
    ]
    boundingObject Group {{ children [
      # Conservative envelope of PCB, motors, connectors, battery and sensor bar.
      # Its lower surface stays above the floor: only wheels and skid support weight.
      Pose {{ translation .01515 0 .01965 children [Box {{ size .136 .15 .0313 }}] }}
      Pose {{ translation .065 0 .001 children [Cylinder {{ radius .0045 height .002 subdivision 16 }}] }}
    ] }}
    physics Physics {{ density -1 mass IS bodyMass centerOfMass [{center}] inertiaMatrix IS bodyInertia }}
  }}
}}
'''
    (SIM/'webots/protos/RobotRacers.proto').write_text(proto)
    scale=1.0
    # Boundary of the free H-shaped corridor. Each edge is a static wall.
    vertices=[(-.85,-.8),(-.35,-.8),(-.35,-.25),(.35,-.25),(.35,-.8),(.85,-.8),(.85,.8),(.35,.8),(.35,.25),(-.35,.25),(-.35,.8),(-.85,.8)]
    walls=[]
    for i,(a,b) in enumerate(zip(vertices,vertices[1:]+vertices[:1])):
        x=(a[0]+b[0])/2;y=(a[1]+b[1])/2
        sx=max(abs(a[0]-b[0]),.02);sy=max(abs(a[1]-b[1]),.02)
        walls.append(f'''DEF WALL_{i} Solid {{ translation {x} {y} .05 children [Shape {{ appearance {"DEF WALL_MATERIAL PBRAppearance { baseColor .018 .023 .025 roughness .5 metalness .02 }" if i==0 else "USE WALL_MATERIAL"} geometry DEF WALL_GEOM_{i} Box {{ size {sx+.01 if sx>.02 else sx} {sy+.01 if sy>.02 else sy} .1 }} }}] name "wall_{i}" contactMaterial "wall" boundingObject USE WALL_GEOM_{i} }}''')
    fixed=[]
    for i,(x,y) in enumerate([(-1.05,.75),(1.05,.75),(1.05,-.75),(-1.05,-.75)],1):
        # Worlds and protos have the same directory depth for marker resources.
        fixed.append(f'DEF REFERENCE_TAG_{i} Group {{ children [{plane_tag(i,x,y,.0006,.048)}] }}')
    world=f'''#VRML_SIM R2025a utf8
EXTERNPROTO "../protos/RobotRacers.proto"
WorldInfo {{ title "RobotRacers" basicTimeStep 2 FPS 60 coordinateSystem "ENU" gravity 9.81
  contactProperties [
    DEF RUBBER_CONTACT ContactProperties {{ material1 "rubber" material2 "floor" coulombFriction [.8 .8] bounce 0 rollingFriction .000005 .000005 0 softERP .6 softCFM .00001 maxContactJoints 8 }}
    DEF SKID_CONTACT ContactProperties {{ material1 "skid" material2 "floor" coulombFriction .12 bounce 0 softERP .6 softCFM .00001 }}
    ContactProperties {{ material1 "wall" material2 "rubber" coulombFriction .5 bounce 0 }}
    ContactProperties {{ material1 "wall" material2 "skid" coulombFriction .4 bounce 0 }}
  ] }}
Viewpoint {{ position 1.9 -2.6 2.8 orientation -.3448517026 .1751115083 .9221785417 2.2657049875 near .001 far 30 }}
Background {{ skyColor [.94 .96 .95] }}
DirectionalLight {{ direction -.3 -.4 -1 intensity 1.2 ambientIntensity .75 castShadows TRUE }}
PointLight {{ location -1 0 2 intensity .8 ambientIntensity 0 attenuation 0 0 1 radius 5 }}
DEF FLOOR Solid {{ translation 0 0 -.025 children [Shape {{ appearance PBRAppearance {{ baseColor .985 .985 .975 roughness .75 metalness 0 }} geometry DEF FLOOR_BOX Box {{ size 2.4 1.8 .05 }} }}] name "floor" contactMaterial "floor" boundingObject USE FLOOR_BOX }}
{''.join(walls)}
{''.join(fixed)}
DEF ROBOT RobotRacers {{ }}
DEF ARENA_SUPERVISOR Robot {{ name "arena_camera" controller "arena_camera" supervisor TRUE synchronization TRUE children [
  DEF OVERHEAD_CAMERA Camera {{ translation 0 0 2.6 rotation -.577350269 .577350269 .577350269 2.09439510239
    name "overhead_camera" width 1920 height 1440 fieldOfView .98 near .02 far 20 antiAliasing TRUE }}
] }}
'''
    (SIM/'webots/worlds/robotracers.wbt').write_text(world)
    print('World generated:',SIM/'webots/worlds/robotracers.wbt')

if __name__=='__main__': generate()
