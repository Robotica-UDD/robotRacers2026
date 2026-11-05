import type { CameraTracking } from './types';
const point=(value:unknown):value is [number,number]=>Array.isArray(value)&&value.length===2&&value.every(Number.isFinite);
export function parseCameraTracking(header:string|null):CameraTracking|null {
  if(!header||header.length>12000)return null;
  try {
    const t=JSON.parse(header) as CameraTracking;
    if(!Number.isInteger(t.seq)||t.seq<0||!Number.isFinite(t.sim_time)||t.sim_time<0
      ||!Number.isInteger(t.width)||!Number.isInteger(t.height)||t.width<16||t.height<16||t.width>4096||t.height>4096
      ||!['visible','predicted','lost'].includes(t.tracking)||!Array.isArray(t.goals)||t.goals.length>8)return null;
    if(t.robot_center!==null&&!point(t.robot_center))return null;
    const box=t.robot_box;
    if(box!==null&&(!box||![box.x,box.y,box.width,box.height].every(Number.isFinite)||box.width<=0||box.height<=0))return null;
    if(!t.goals.every(goal=>typeof goal.id==='string'&&goal.id.length>0&&goal.id.length<=16&&point(goal.pixel)))return null;
    if(t.age_ms!==undefined&&(!Number.isFinite(t.age_ms)||t.age_ms<0))return null;
    if(t.age_ms!==undefined&&t.age_ms>250)return {...t,tracking:'lost',robot_center:null,robot_box:null,marker_corners:null};
    return {...t,received_ms:performance.now()};
  } catch {return null;}
}
