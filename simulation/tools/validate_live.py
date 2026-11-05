#!/usr/bin/env python3
"""Record real Webots missions and timing; never substitute a kinematic model.

Run against an already launched local application. The resulting CSV and JSON
contain measured values, including unmet targets. Only evaluation metrics read
simulator truth; navigation continues to use camera images and encoders.
"""
from pathlib import Path
import argparse,csv,datetime,json,math,statistics,time,urllib.request

BASE='http://127.0.0.1:8000'
ROOT=Path(__file__).resolve().parents[2]

def api(path,body=None):
    request=urllib.request.Request(BASE+path,data=None if body is None else json.dumps(body).encode(),
        headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(request,timeout=5) as response:return json.load(response)

def wait_ready(timeout=30):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        state=api('/api/state')
        if state['connected'] and not state['scene_pending'] and state['vision']['visible'] and state['pose']['age_ms'] is not None:return state
        time.sleep(.1)
    raise RuntimeError('La escena no confirmó localización y conexión.')

def percentile(values,p):
    values=sorted(x for x in values if x is not None and math.isfinite(x))
    if not values:return None
    position=(len(values)-1)*p;low=math.floor(position);high=math.ceil(position)
    return values[low]+(values[high]-values[low])*(position-low)

def distribution(values):
    clean=[x for x in values if x is not None and math.isfinite(x)]
    return {'samples':len(clean),'min':min(clean) if clean else None,'median':percentile(clean,.5),
        'p95':percentile(clean,.95),'max':max(clean) if clean else None}

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--seconds',type=float,default=600)
    parser.add_argument('--output',type=Path,default=ROOT/'simulation/validation')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    api('/api/mission/stop',{});time.sleep(.4)
    scenarios=[('nominal',1.,.30),('compacto',.8,.15),('amplio',1.5,.50),('rapido',1.,1.5)]
    rows=[];missions=[];scenario_index=0;active=None;failures=[];client={};next_health=0
    started=time.monotonic();start_utc=datetime.datetime.now(datetime.timezone.utc).isoformat()
    fields=['elapsed_s','sim_time','status','phase','goal','x','y','yaw','speed','contacts','position_error_m',
        'heading_error_deg','camera_fps','vision_processing_ms','command_applied_latency_ms','render_fps','renderer_visible','real_time_factor']
    with (args.output/'rendimiento.csv').open('w',newline='') as file:
        writer=csv.DictWriter(file,fieldnames=fields);writer.writeheader()
        while time.monotonic()-started<args.seconds:
            stamp=time.monotonic();elapsed=stamp-started
            state=api('/api/state')
            if active is None:
                name,scale,speed=scenarios[scenario_index%len(scenarios)];scenario_index+=1
                api('/api/settings',{'scale':scale,'speed':speed});wait_ready();time.sleep(.2)
                state=api('/api/mission/start',{});active={'scenario':name,'scale':scale,'speed':speed,'start_s':time.monotonic()-started}
                print('Prueba',name,scale,speed,flush=True)
            if stamp>=next_health:
                client=api('/api/health').get('client_metrics',{});next_health=stamp+1
            metrics=state['metrics'];pose=state['pose'];mission=state['mission']
            valid_client=client.get('visible',False) and stamp-client.get('received_stamp',0)<2.5
            rtf=None
            if rows:
                window=rows[max(0,len(rows)-50)]
                dt=elapsed-window['elapsed_s']
                if dt>.1:rtf=(state['sim_time']-window['sim_time'])/dt
            row={'elapsed_s':elapsed,'sim_time':state['sim_time'],'status':state['status'],'phase':mission['phase'],
                'goal':mission['active_goal'],'x':pose['x'],'y':pose['y'],'yaw':pose['yaw'],'speed':state['robot']['speed'],
                'render_fps':client.get('render_fps') if valid_client else None,'renderer_visible':valid_client,'real_time_factor':rtf,
                **{key:metrics.get(key) for key in fields if key in metrics}}
            if row.get('heading_error_deg') is not None:row['heading_error_deg']=abs(row['heading_error_deg'])
            writer.writerow(row);file.flush();rows.append(row)
            if state['status'] in ('completed','stopped','paused','disconnected'):
                active.update(end_s=elapsed,status=state['status'],message=state['message'],
                    reached_goals=mission['reached_goals'],arrivals=mission['goal_measurements'])
                missions.append(active);print('Resultado',json.dumps(active,ensure_ascii=False),flush=True)
                if state['status']!='completed':failures.append(active)
                active=None
                api('/api/mission/stop',{});time.sleep(.3)
            time.sleep(max(.001,.1-(time.monotonic()-stamp)))
    # An unfinished final mission is stopped explicitly and is not reported as completed.
    state=api('/api/mission/stop',{});time.sleep(.5)
    duration=time.monotonic()-started
    moving=[r for r in rows if r['status']=='running']
    calibration=json.loads((ROOT/'simulation/.runtime/camera_calibration.json').read_text())
    report={'date_utc':start_utc,'wall_duration_s':duration,'requested_duration_s':args.seconds,
        'physics_step_ms':2,'controller_period_ms':10,'camera_period_ms':20,'camera_native_resolution':[calibration['width'],calibration['height']],
        'missions':missions,'incomplete_at_end':active,'failures':failures,
        'metrics':{key:distribution([r.get(key) for r in moving]) for key in
            ['position_error_m','heading_error_deg','camera_fps','vision_processing_ms','command_applied_latency_ms','render_fps','real_time_factor']},
        'max_contacts':max(r.get('contacts') or 0 for r in rows),'samples':len(rows),
        'criteria':{'duration_10min':duration>=600,'completed_missions':sum(m['status']=='completed' for m in missions),
            'no_mission_failure':not failures,'no_contacts':all(not r.get('contacts') for r in rows),
            'position_p95_10mm':(percentile([r.get('position_error_m') for r in moving],.95) or math.inf)<.01,
            'heading_p95_3deg':(percentile([r.get('heading_error_deg') for r in moving],.95) or math.inf)<3,
            'latency_p95_80ms':(percentile([r.get('command_applied_latency_ms') for r in moving],.95) or math.inf)<80}}
    (args.output/'rendimiento.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(report,indent=2,ensure_ascii=False),flush=True)
    api('/api/settings',{'scale':1.,'speed':.3});wait_ready()

if __name__=='__main__':main()
