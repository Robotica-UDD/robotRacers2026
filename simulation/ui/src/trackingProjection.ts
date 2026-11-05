/** Projection of estimated positions only. Never reads the simulator robot node. */
import type { CameraTracking, SimulationState, Goal } from './types';
import type { TrackingBox, TrackingPoint } from './trackingGeometry';

export type Vector3 = { x: number; y: number; z: number };
export type ViewCamera = { position: Vector3; orientation: Vector3 & { w: number }; fieldOfView: number; aspectRatio: number; near?: number; far?: number };
export type ViewRect = { left: number; top: number; width: number; height: number };
export type VisualBounds = { min: [number,number,number]; max: [number,number,number] };
export const defaultVisualBounds: VisualBounds = { min:[-.05285,-.115,0],max:[.08315,.115,.0353] };
export function topViewAltitude(width:number,height:number,aspect:number,fieldOfView:number) {
  const ty=Math.tan(fieldOfView/2)/(aspect>=1?aspect:1),tx=aspect*ty;
  return Math.max(width/(2*tx),height/(2*ty))*1.14;
}
export type OverlayGeometry = { width:number; height:number; box:TrackingBox|null; robot:TrackingPoint|null; target:{id:string;pixel:TrackingPoint}|null;tracking:'visible'|'predicted'|'lost';label?:string };

const dot=(a:Vector3,b:Vector3)=>a.x*b.x+a.y*b.y+a.z*b.z;
function rotate(v:Vector3,axis:Vector3 & {w:number}):Vector3 {
  const length=Math.hypot(axis.x,axis.y,axis.z);
  if(length<1e-10)return {...v};
  const x=axis.x/length,y=axis.y/length,z=axis.z/length,c=Math.cos(axis.w),s=Math.sin(axis.w),t=1-c,d=x*v.x+y*v.y+z*v.z;
  return {x:v.x*c+(y*v.z-z*v.y)*s+x*d*t,y:v.y*c+(z*v.x-x*v.z)*s+y*d*t,z:v.z*c+(x*v.y-y*v.x)*s+z*d*t};
}

/** Webots FLU camera: X forward, Z up; its fieldOfView is on the larger axis. */
export function projectEstimatedPoint(point:Vector3,camera:ViewCamera,canvas:ViewRect,overlay:ViewRect):TrackingPoint|null {
  const forward=rotate({x:1,y:0,z:0},camera.orientation),up=rotate({x:0,y:0,z:1},camera.orientation);
  const right={x:forward.y*up.z-forward.z*up.y,y:forward.z*up.x-forward.x*up.z,z:forward.x*up.y-forward.y*up.x};
  const d={x:point.x-camera.position.x,y:point.y-camera.position.y,z:point.z-camera.position.z};
  const depth=dot(d,forward),aspect=camera.aspectRatio;
  if(!Number.isFinite(depth)||depth<=Math.max(.0001,camera.near??.001)||((camera.far??0)>0&&depth>=camera.far!)
    ||!Number.isFinite(aspect)||aspect<=0||!Number.isFinite(camera.fieldOfView)||camera.fieldOfView<=0||camera.fieldOfView>=Math.PI)return null;
  const ty=Math.tan(camera.fieldOfView/2)/(aspect>=1?aspect:1),tx=aspect*ty;
  const result={x:canvas.left-overlay.left+(dot(d,right)/(depth*tx)+1)*canvas.width/2,
    y:canvas.top-overlay.top+(1-dot(d,up)/(depth*ty))*canvas.height/2};
  return Number.isFinite(result.x)&&Number.isFinite(result.y)?result:null;
}

export function selectTrackingTarget(state:SimulationState,selectedId:string):Goal|null {
  if(state.status==='completed')return null;
  const active=typeof state.mission.active_goal==='number'?state.mission.goals[state.mission.active_goal]?.id:state.mission.active_goal;
  const id=(state.status==='running'||state.status==='paused')&&active?active:selectedId;
  return state.mission.goals.find(goal=>goal.id===id)??null;
}

/** Align the estimate with the displayed W3D clock, using wheel odometry for <=80ms. */
export function estimatedRenderPose(state:SimulationState,renderTime:number|undefined,track:number) {
  const pose={...state.pose};
  const dt=renderTime===undefined?0:Math.max(-.08,Math.min(.08,renderTime-state.sim_time));
  const v=state.robot.speed,w=(state.robot.right_speed-state.robot.left_speed)/Math.max(.01,track);
  if(Number.isFinite(dt)&&Number.isFinite(v)&&Number.isFinite(w)){
    const angle=w*dt;pose.x+=v*dt*Math.cos(pose.yaw+angle/2);pose.y+=v*dt*Math.sin(pose.yaw+angle/2);pose.yaw+=angle;
  }
  return pose;
}

export function boundsForConfig(robot:Record<string,unknown>|undefined):VisualBounds {
  const raw=robot?.visual_bounds_m as {x?:number[];y?:number[];z?:number[]}|undefined;
  const candidate=raw&&[raw.x,raw.y,raw.z].every(v=>Array.isArray(v)&&v.length===2)?
    {min:[raw.x![0],raw.y![0],raw.z![0]],max:[raw.x![1],raw.y![1],raw.z![1]]} as VisualBounds:undefined;
  return candidate&&Array.isArray(candidate.min)&&Array.isArray(candidate.max)&&candidate.min.length===3&&candidate.max.length===3
    &&candidate.min.every((v,i)=>Number.isFinite(v)&&Number.isFinite(candidate.max[i])&&v<candidate.max[i])?candidate:defaultVisualBounds;
}

export function worldTracking(state:SimulationState,target:Goal|null,camera:ViewCamera,canvas:ViewRect,viewport:ViewRect,
  bounds:VisualBounds=defaultVisualBounds,renderTime?:number,track=.19):OverlayGeometry {
  const result:OverlayGeometry={width:viewport.width,height:viewport.height,box:null,robot:null,target:null,tracking:'lost'};
  if(target){const pixel=projectEstimatedPoint({...target,z:.001},camera,canvas,viewport);if(pixel)result.target={id:target.id,pixel};}
  if(!state.connected||!state.vision.visible||state.scene_pending||state.pose.source==='sin localización'||!Number.isFinite(state.pose.x)||!Number.isFinite(state.pose.y)||!Number.isFinite(state.pose.yaw)
    ||!Number.isFinite(state.pose.age_ms)||state.pose.age_ms>300)return result;
  const pose=estimatedRenderPose(state,renderTime,track),c=Math.cos(pose.yaw),s=Math.sin(pose.yaw);
  const corners:TrackingPoint[]=[];
  for(const x of [bounds.min[0],bounds.max[0]])for(const y of [bounds.min[1],bounds.max[1]])for(const z of [bounds.min[2],bounds.max[2]]){
    const pixel=projectEstimatedPoint({x:pose.x+c*x-s*y,y:pose.y+s*x+c*y,z},camera,canvas,viewport);
    if(!pixel)return result;corners.push(pixel);
  }
  result.robot=projectEstimatedPoint({x:pose.x,y:pose.y,z:.0115},camera,canvas,viewport);
  if(!result.robot)return result;
  const left=Math.min(...corners.map(p=>p.x)),top=Math.min(...corners.map(p=>p.y));
  result.box={x:left-3,y:top-3,width:Math.max(...corners.map(p=>p.x))-left+6,height:Math.max(...corners.map(p=>p.y))-top+6};
  result.tracking=state.vision.visible&&state.pose.source==='cámara + encoders'?'visible':'predicted';
  if(target)result.label=`${Math.hypot(target.x-pose.x,target.y-pose.y).toFixed(2).replace('.',',')} m`;
  return result;
}

/** Match the pixel coordinate system to object-fit:contain, including letterboxing. */
export function cameraTracking(tracking:CameraTracking|null,target:Goal|null,width:number,height:number,connected:boolean,now=performance.now()):OverlayGeometry {
  const result:OverlayGeometry={width,height,box:null,robot:null,target:null,tracking:'lost'};
  if(!tracking||!Number.isFinite(tracking.width)||!Number.isFinite(tracking.height)||tracking.width<=0||tracking.height<=0)return result;
  const scale=Math.min(width/tracking.width,height/tracking.height),ox=Math.max(0,(width-tracking.width*scale)/2),oy=Math.max(0,(height-tracking.height*scale)/2);
  const point=(pixel:[number,number]):TrackingPoint=>({x:ox+pixel[0]*scale,y:oy+pixel[1]*scale});
  const projected=tracking.goals.find(goal=>goal.id===target?.id);
  if(projected)result.target={id:projected.id,pixel:point(projected.pixel)};
  const age=(tracking.age_ms??0)+Math.max(0,now-(tracking.received_ms??now));
  if(!connected||age>250||tracking.tracking==='lost'||!tracking.robot_center||!tracking.robot_box)return result;
  result.robot=point(tracking.robot_center);
  result.box={x:ox+tracking.robot_box.x*scale-3,y:oy+tracking.robot_box.y*scale-3,width:tracking.robot_box.width*scale+6,height:tracking.robot_box.height*scale+6};
  result.tracking=tracking.tracking;
  return result;
}
