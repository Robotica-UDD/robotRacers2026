import {describe,it,expect} from 'vitest';
import {cameraTracking,projectEstimatedPoint,selectTrackingTarget,worldTracking,estimatedRenderPose,topViewAltitude} from './trackingProjection';
import {parseCameraTracking} from './cameraTrackingMetadata';
import {disconnectedState} from './types';
import type {SimulationState,CameraTracking} from './types';
const rect={left:0,top:0,width:800,height:800};
const camera={position:{x:0,y:0,z:0},orientation:{x:0,y:0,z:1,w:0},fieldOfView:Math.PI/2,aspectRatio:1,near:.001,far:20};
const state:SimulationState={...disconnectedState,connected:true,status:'running',sim_time:1,
  pose:{x:2,y:0,yaw:0,source:'cámara + encoders',age_ms:0},robot:{speed:0,left_speed:0,right_speed:0,battery_voltage:7.4},vision:{visible:true,latency_ms:0},
  mission:{...disconnectedState.mission,goals:[{id:'A',x:2,y:-1},{id:'B',x:3,y:0}],active_goal:'A'}};

describe('projection of estimated tracking in the displayed viewport',()=>{
  it('fits the entire top view on portrait and landscape panels',()=>{
    for(const aspect of [.5,2]){
      const fov=Math.PI/4,altitude=topViewAltitude(3.6,2.7,aspect,fov);
      const ty=Math.tan(fov/2)/(aspect>=1?aspect:1);
      expect(2*altitude*ty*aspect).toBeGreaterThan(3.6);
      expect(2*altitude*ty).toBeGreaterThan(2.7);
    }
  });
  it('matches Webots FLU axes and discards geometry behind the camera',()=>{
    expect(projectEstimatedPoint({x:1,y:0,z:0},camera,rect,rect)).toEqual({x:400,y:400});
    expect(projectEstimatedPoint({x:1,y:1,z:0},camera,rect,rect)!.x).toBeCloseTo(0);
    expect(projectEstimatedPoint({x:1,y:0,z:1},camera,rect,rect)!.y).toBeCloseTo(0);
    expect(projectEstimatedPoint({x:-1,y:0,z:0},camera,rect,rect)).toBeNull();
  });
  it('accounts for an offset canvas and the larger-axis FOV in portrait',()=>{
    const result=projectEstimatedPoint({x:2,y:.5,z:0},{...camera,aspectRatio:.5},{left:70,top:90,width:400,height:800},{left:20,top:40,width:500,height:900});
    expect(result!.x).toBeCloseTo(150);expect(result!.y).toBeCloseTo(450);
  });
  it('uses the estimated pose and active goal, and hides stale tracking',()=>{
    expect(selectTrackingTarget(state,'B')!.id).toBe('A');
    expect(selectTrackingTarget({...state,status:'idle'},'B')!.id).toBe('B');
    const geometry=worldTracking(state,state.mission.goals[0],camera,rect,rect);
    expect(geometry.robot!.x).toBeCloseTo(400);expect(geometry.box!.width).toBeGreaterThan(40);
    expect(geometry.tracking).toBe('visible');
    expect(worldTracking({...state,connected:false},null,camera,rect,rect).robot).toBeNull();
    expect(worldTracking({...state,vision:{...state.vision,visible:false}},null,camera,rect,rect).tracking).toBe('lost');
    expect(worldTracking({...state,pose:{...state.pose,age_ms:500}},null,camera,rect,rect).tracking).toBe('lost');
  });
  it('bounds odometric alignment to the displayed clock rather than teleporting',()=>{
    const sample={...state,robot:{...state.robot,speed:1,left_speed:1,right_speed:1}};
    expect(estimatedRenderPose(sample,1.02,.19).x).toBeCloseTo(2.02);
    expect(estimatedRenderPose(sample,20,.19).x).toBeCloseTo(2.08);
  });
});

const metadata:CameraTracking={seq:2,sim_time:1,width:1920,height:1440,tracking:'visible',robot_center:[960,720],
  robot_box:{x:920,y:680,width:80,height:80},goals:[{id:'B',pixel:[0,0]}],age_ms:10,received_ms:100};
describe('camera frame and image fit',()=>{
  it('places photo coordinates within the actual letterbox, not the entire panel',()=>{
    const geometry=cameraTracking(metadata,{id:'B',x:0,y:0},1000,1000,true,100);
    expect(geometry.robot!.x).toBeCloseTo(500);expect(geometry.robot!.y).toBeCloseTo(500);
    expect(geometry.target!.pixel).toEqual({x:0,y:125});
    expect(geometry.box!.width).toBeCloseTo(80*1000/1920+6);
  });
  it('stops showing an old photo box when fetching the next frame stalls',()=>{
    expect(cameraTracking(metadata,null,1000,1000,true,400).tracking).toBe('lost');
    expect(cameraTracking({...metadata,tracking:'lost',robot_center:null,robot_box:null},null,1000,1000,true,100).robot).toBeNull();
  });
  it('rejects malformed headers and marks old frames lost',()=>{
    expect(parseCameraTracking('{bad')).toBeNull();
    expect(parseCameraTracking(JSON.stringify({...metadata,robot_center:[1,null]}))).toBeNull();
    expect(parseCameraTracking(JSON.stringify({...metadata,width:Infinity}))).toBeNull();
    expect(parseCameraTracking(JSON.stringify({...metadata,age_ms:600}))!.tracking).toBe('lost');
  });
});
