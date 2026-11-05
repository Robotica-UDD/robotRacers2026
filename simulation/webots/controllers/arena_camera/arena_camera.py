"""Camera producer and scene supervisor. Ground truth is evaluation-only."""
from controller import Supervisor
from pathlib import Path
import json, math, mmap, os, socket, struct, time

robot=Supervisor()
robot.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)
runtime=Path(os.environ.get('ROBOTRACERS_RUNTIME_DIR',Path(__file__).resolve().parents[3]/'.runtime'))
runtime.mkdir(parents=True,exist_ok=True)
physical=json.loads((Path(__file__).resolve().parents[3]/'config/robot.json').read_text())
camera=robot.getDevice('overhead_camera');camera.enable(20)
body=robot.getFromDef('ROBOT')
marker=body.getFromProtoDef('ROBOT_MARKER')
cam_node=robot.getFromDef('OVERHEAD_CAMERA')
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
destination=('127.0.0.1',int(os.environ.get('ROBOTRACERS_TELEMETRY_PORT','9201')))
header=struct.Struct('<8sIIIIQdd16x')
frame_file=None;shared=None;width=0;height=0;sequence=0;command_seq=-1;scale=1.
diagnostics={'occluded':False,'image_delay_ms':0.}
initial_wall=[]
for i in range(12):
    n=robot.getFromDef(f'WALL_{i}')
    initial_wall.append((n,n.getField('translation').getSFVec3f(),robot.getFromDef(f'WALL_GEOM_{i}').getField('size').getSFVec3f()))
tag_positions=[(-1.05,.75),(1.05,.75),(1.05,-.75),(-1.05,-.75)]

def send(data):
    try:sock.sendto(json.dumps(data,separators=(',',':')).encode(),destination)
    except OSError:pass

def atomic_json(path,data):
    temp=path.with_suffix('.tmp');temp.write_text(json.dumps(data));temp.replace(path)

def calibration():
    fx=camera.getWidth()/(2*math.tan(camera.getFov()/2))
    references=[]
    for i,(x,y) in enumerate(tag_positions,1):
        x*=scale;y*=scale;s=.048/2
        references.append({'id':i,'corners_world':[[x-s,y+s,.0006],[x+s,y+s,.0006],[x+s,y-s,.0006],[x-s,y-s,.0006]]})
    data={'width':camera.getWidth(),'height':camera.getHeight(),'camera_matrix':[[fx,0,camera.getWidth()/2],[0,fx,camera.getHeight()/2],[0,0,1]],
          'dist_coeffs':[0,0,0,0,0],'camera_position':[0,0,2.6*scale],'camera_target':[0,0,0],
          'marker_size_m':.04,'marker_height_m':.023,'marker_offset_m':[-.02685,0],'arena_scale':scale,'reference_tags':references}
    atomic_json(runtime/'camera_calibration.json',data)

def publish_frame():
    global frame_file,shared,width,height,sequence
    w,h=camera.getWidth(),camera.getHeight()
    image=camera.getImage()
    if not image or len(image)!=w*h*4:return
    if shared is None or (w,h)!=(width,height):
        if shared:shared.close();frame_file.close()
        width,height=w,h
        temporary=runtime/'camera.frame.next'
        frame_file=open(temporary,'w+b');frame_file.truncate(header.size+w*h*4)
        shared=mmap.mmap(frame_file.fileno(),0)
        temporary.replace(runtime/'camera.frame')
        calibration()
    sequence+=2;stamp=time.monotonic()-float(diagnostics['image_delay_ms'])/1000;sim=robot.getTime()
    shared[:header.size]=header.pack(b'RRFRAME1',w,h,4,0,sequence-1,sim,stamp)
    shared[header.size:]=image
    shared[:header.size]=header.pack(b'RRFRAME1',w,h,4,0,sequence,sim,stamp)

def configure(command):
    global scale
    new_scale=max(.8,min(1.5,float(command.get('scale',scale))))
    if new_scale!=scale:
        old_scale=scale;scale=new_scale
        floor_size=robot.getFromDef('FLOOR_BOX').getField('size');floor_size.setSFVec3f([2.4*scale,1.8*scale,.05])
        for i,(n,pos,size) in enumerate(initial_wall):
            n.getField('translation').setSFVec3f([pos[0]*scale,pos[1]*scale,pos[2]])
            robot.getFromDef(f'WALL_GEOM_{i}').getField('size').setSFVec3f([size[0]*scale,size[1]*scale,.1])
        # The tag Group coordinates are generated absolute, so use a scale transform for positioning only.
        for i,(x,y) in enumerate(tag_positions,1):
            n=robot.getFromDef(f'REFERENCE_TAG_{i}')
            # Move all vertices and carrier centres without scaling the encoded marker itself.
            children=n.getField('children')
            carrier=children.getMFNode(0)
            carrier.getField('translation').setSFVec3f([x*scale,y*scale,.0001])
            shape=children.getMFNode(1)
            coords=shape.getField('geometry').getSFNode().getField('coord').getSFNode().getField('point')
            s=.024
            for k,p in enumerate([[x*scale-s,y*scale+s,.0006],[x*scale+s,y*scale+s,.0006],[x*scale+s,y*scale-s,.0006],[x*scale-s,y*scale-s,.0006]]):coords.setMFVec3f(k,p)
        cam_node.getField('translation').setSFVec3f([0,0,2.6*scale])
        # Resolution is fixed in Webots at world load; moving the camera and
        # recalibrating preserves the native 1280 x 960 stream at every scale.
        calibration()
    mass=max(.05,min(.6,float(command.get('robot_mass_kg',.18))))
    # PROTO field updates also update the Physics node.
    body_mass=mass-2*physical['wheel_mass_kg']
    body.getField('bodyMass').setSFFloat(body_mass)
    inertia_scale=body_mass/(physical['mass_kg']-2*physical['wheel_mass_kg'])
    body.getField('bodyInertia').setMFVec3f(0,[value*inertia_scale for value in physical['body_inertia_kg_m2']])
    robot.getFromDef('RUBBER_CONTACT').getField('coulombFriction').setMFFloat(0,float(command.get('friction',.8)))
    robot.getFromDef('RUBBER_CONTACT').getField('coulombFriction').setMFFloat(1,float(command.get('friction',.8)))
    robot.getFromDef('SKID_CONTACT').getField('coulombFriction').setMFFloat(0,float(command.get('skid_friction',.12)))
    start=command.get('start',{'x':-.6*scale,'y':.55*scale,'yaw':-math.pi/2})
    body.getField('translation').setSFVec3f([start['x'],start['y'],0])
    body.getField('rotation').setSFRotation([0,0,1,start.get('yaw',-math.pi/2)])
    body.resetPhysics()

fan_force=0.;contacts=0
try:
    calibration()
    while robot.step(2 if fan_force else 10)!=-1:
        now=robot.getTime();tick=round(now/.002)
        if tick%10==0:publish_frame()
        if tick%20==0:
            try:
                # Simulator-only fault injection for reproducible acceptance tests.
                # This file never exists in normal use and cannot control hardware.
                fault_path=runtime/'diagnostics.json'
                fault=json.loads(fault_path.read_text()) if fault_path.exists() else {}
                occluded=bool(fault.get('occluded',False))
                if occluded!=diagnostics['occluded']:
                    marker.getField('translation').setSFVec3f([0,0,-.15 if occluded else 0])
                diagnostics.update(occluded=occluded,image_delay_ms=max(0,min(2000,float(fault.get('image_delay_ms',0)))))
                path=runtime/'scene_command.json'
                if path.exists():
                    cmd=json.loads(path.read_text())
                    if cmd.get('seq',-1)!=command_seq:
                        configure(cmd);fan_force=max(0,min(3,float(cmd.get('fan_force_n',0))))
                        command_seq=cmd['seq'];send({'type':'scene_ack','seq':command_seq})
            except (ValueError,KeyError,OSError) as exc:print('Scene command:',exc,flush=True)
        if fan_force:body.addForce([0,0,-fan_force],False)
        if tick%25==0:
            contacts=sum(1 for c in body.getContactPoints(True) if c.point[2]>.003)
        if tick%5==0:
            p=body.getPosition();o=body.getOrientation()
            send({'type':'evaluation','sim_time':now,'x':p[0],'y':p[1],'yaw':math.atan2(o[3],o[0]),'contacts':contacts})
        if tick%500==0:
            # An optional image exported by the simulator helps inspect actual camera input.
            if os.environ.get('ROBOTRACERS_CAPTURE_DEBUG')=='1':camera.saveImage(str(runtime/'camera-debug.png'),100)
finally:
    if shared:shared.close();frame_file.close()
    sock.close()
