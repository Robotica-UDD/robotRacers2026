#!/usr/bin/env python3
"""Exercise real rendered occlusion, stale images and the Webots driver watchdog.

Simulator only. The optional watchdog check requires launching with
ROBOTRACERS_TEST_TELEMETRY_PORT=9203; it freezes only our local Python service,
receives independent driver telemetry, then stops the mission before cleanup.
"""
from pathlib import Path
import argparse,json,os,signal,socket,time
from simulation.tools.validate_live import api,wait_ready,ROOT

RUNTIME=ROOT/'simulation/.runtime'

def fault(**values):
    path=RUNTIME/'diagnostics.json';temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(values));temporary.replace(path)

def ready_mission():
    api('/api/mission/stop',{});time.sleep(.5)
    api('/api/settings',{'scale':1.,'speed':.3});wait_ready();time.sleep(.3)
    api('/api/mission/start',{});time.sleep(1)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--watchdog',action='store_true');args=parser.parse_args()
    checks=[];suspended=None
    try:
        ready_mission()
        fault(occluded=True);time.sleep(.10);fault();time.sleep(.15)
        s=api('/api/state');checks.append({'test':'occlusion_100ms','status':s['status'],'passed':s['status']=='running'})
        fault(occluded=True);time.sleep(.95)
        s=api('/api/state');checks.append({'test':'occlusion_950ms','status':s['status'],'speed':s['robot']['speed'],
            'passed':s['status']=='paused' and abs(s['robot']['speed'])<.03})
        fault();wait_ready();time.sleep(.2)
        s=api('/api/state');checks.append({'test':'recovery_does_not_restart','passed':s['status']=='paused'})
        api('/api/mission/start',{});time.sleep(.3)
        fault(image_delay_ms=900);time.sleep(.95)
        s=api('/api/state');checks.append({'test':'stale_image_900ms','status':s['status'],'passed':s['status']=='paused'})
        fault();wait_ready();time.sleep(.2)
        api('/api/mission/start',{});time.sleep(.6)
        api('/api/mission/pause',{});time.sleep(.8)
        s=api('/api/state');checks.append({'test':'pause','passed':s['status']=='paused' and abs(s['robot']['speed'])<.03})
        api('/api/mission/start',{});time.sleep(.5)
        api('/api/mission/stop',{});time.sleep(.8)
        s=api('/api/state');checks.append({'test':'stop','passed':s['status']=='stopped' and abs(s['robot']['speed'])<.03})
        if args.watchdog:
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as receiver:
                receiver.bind(('127.0.0.1',9203));receiver.settimeout(2)
                ready_mission()
                session=json.loads((RUNTIME/'session.json').read_text())
                suspended=session['service'];os.kill(suspended,signal.SIGSTOP)
                started=time.monotonic();observations=[]
                while time.monotonic()-started<1.0:
                    data=json.loads(receiver.recv(32768))
                    if data.get('stamp',0)>started+.55:observations.append(data)
                os.kill(suspended,signal.SIGCONT);suspended=None
                api('/api/mission/stop',{})
                checks.append({'test':'driver_watchdog_250ms','samples':len(observations),
                    'max_abs_pwm':max((max(abs(d['left_pwm']),abs(d['right_pwm'])) for d in observations),default=None),
                    'max_abs_velocity':max((max(abs(d['left_velocity']),abs(d['right_velocity'])) for d in observations),default=None),
                    'passed':bool(observations) and all(d['left_pwm']==0 and d['right_pwm']==0 for d in observations)})
    finally:
        if suspended:os.kill(suspended,signal.SIGCONT)
        fault();api('/api/mission/stop',{});time.sleep(.6)
        api('/api/settings',{'scale':1.,'speed':.3});wait_ready()
    report={'checks':checks,'passed':all(c['passed'] for c in checks)}
    output=ROOT/'simulation/validation';output.mkdir(exist_ok=True)
    (output/'fallos.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)
    if not report['passed']:raise SystemExit(1)

if __name__=='__main__':main()
