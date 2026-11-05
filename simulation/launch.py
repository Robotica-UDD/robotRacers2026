#!/usr/bin/env python3
"""Run the local product, keeping all child processes and logs in one session."""
from pathlib import Path
import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import time
import urllib.request
import webbrowser

ROOT=Path(__file__).resolve().parents[1]
RUNTIME=ROOT/'simulation/.runtime'

def available(port):
    with socket.socket() as s:
        s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        try:s.bind(('127.0.0.1',port));return True
        except OSError:return False

def main():
    parser=argparse.ArgumentParser(description='RobotRacers — simulación local')
    parser.add_argument('--no-browser',action='store_true')
    parser.add_argument('--visible-webots',action='store_true',
        help='Abre la ventana de Webots y activa también su vista 3D local.')
    parser.add_argument('--debug-camera',action='store_true')
    args=parser.parse_args()
    RUNTIME.mkdir(exist_ok=True)
    for port in (8000,1234):
        if not available(port):
            raise SystemExit(f'El puerto local {port} ya está ocupado. Cierra la sesión anterior de RobotRacers antes de iniciar otra.')
    home=Path(os.environ.get('WEBOTS_HOME','/Applications/Webots.app/Contents'))
    executable=home/'MacOS/webots' if sys.platform=='darwin' else home/'webots'
    if not executable.exists():raise SystemExit('Falta Webots. Ejecuta simulation/setup.sh.')
    required_viewer=['WebotsView.js','webots.js','wrenjs.js','dependencies/assimpjs.js',
                     'dependencies/assimpjs.wasm','images/missing_texture.png','images/webots-logo.png']
    if any(not (RUNTIME/'wwi'/name).is_file() for name in required_viewer):
        raise SystemExit('El visor local está incompleto. Ejecuta simulation/setup.sh antes de iniciar.')
    # A fresh session uses defaults; preserve downloaded viewer resources.
    for path in ('scene_command.json','camera.frame','camera_calibration.json','diagnostics.json'):
        (RUNTIME/path).unlink(missing_ok=True)
    env=os.environ.copy()
    env.update({'WEBOTS_HOME':str(home),'ROBOTRACERS_ROOT':str(ROOT),'ROBOTRACERS_RUNTIME_DIR':str(RUNTIME),'PYTHONUNBUFFERED':'1'})
    if args.debug_camera:env['ROBOTRACERS_CAPTURE_DEBUG']='1'
    runtime_ini=ROOT/'simulation/webots/controllers/arena_camera/runtime.ini'
    runtime_ini.write_text('[python]\nCOMMAND = '+str(ROOT/'.venv/bin/python')+'\n')
    subprocess.run([sys.executable,'simulation/tools/build_controller.py'],cwd=ROOT,env=env,check=True)
    subprocess.run([sys.executable,'simulation/tools/build_world.py'],cwd=ROOT,env=env,check=True)
    children=[];files=[]
    def shutdown(*_):
        for child in reversed(children):
            if child.poll() is None:child.terminate()
        for child in children:
            try:child.wait(timeout=8)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        for file in files:file.close()
        (RUNTIME/'session.json').unlink(missing_ok=True)
    signal.signal(signal.SIGTERM,lambda *_:sys.exit(0))
    signal.signal(signal.SIGINT,lambda *_:sys.exit(0))
    try:
        for name,command in [
            ('service',[sys.executable,'-m','simulation.service']),
            ('webots',[str(executable),'--batch','--stdout','--stderr','--mode=realtime','--stream=w3d','--port=1234']
             +([] if args.visible_webots else ['--no-rendering','--minimize'])+[str(ROOT/'simulation/webots/worlds/robotracers.wbt')])]:
            log=open(RUNTIME/f'{name}.log','w');files.append(log)
            child=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
            children.append(child)
        (RUNTIME/'session.json').write_text(json.dumps({'root':str(ROOT),'launcher':os.getpid(),'service':children[0].pid,'webots':children[1].pid}))
        ready=False
        for _ in range(180):
            if any(p.poll() is not None for p in children):raise RuntimeError('Un proceso no pudo arrancar; consulta simulation/.runtime/*.log')
            try:
                with urllib.request.urlopen('http://127.0.0.1:8000/api/health',timeout=1) as f:health=json.load(f)
                if health.get('connected') and health.get('camera'):
                    ready=True;break
            except OSError:pass
            time.sleep(.5)
        if not ready:raise RuntimeError('El simulador no confirmó el arranque. Consulta simulation/.runtime/*.log')
        print('RobotRacers: http://127.0.0.1:8000',flush=True)
        if not args.no_browser:webbrowser.open('http://127.0.0.1:8000')
        while all(child.poll() is None for child in children):time.sleep(.5)
        raise RuntimeError('La sesión terminó. Consulta simulation/.runtime para los registros.')
    finally:shutdown()

if __name__=='__main__':main()
