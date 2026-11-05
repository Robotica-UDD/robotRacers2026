import { useEffect, useRef, useState } from 'react';
import type { AppConfig, CameraTracking, SimulationState, ViewMode } from './types';
import { Icon } from './icons';
import { readRenderMetrics } from './renderMetrics';
import { TrackingOverlay } from './TrackingOverlay';
import { boundsForConfig, cameraTracking, selectTrackingTarget, topViewAltitude, worldTracking } from './trackingProjection';
import type { OverlayGeometry, ViewCamera } from './trackingProjection';

type WebotsElement = HTMLElement & {
  connect: (url: string, mode: string, broadcast: boolean, mobile: boolean) => void;
  close: () => void; hideToolbar: () => void; resetViewpoint: () => void; resize: () => void;
  sendMessage: (message: string) => void;
  onready?: () => void; ondisconnect?: () => void;
  setWebotsErrorMessageCallback?: (callback: (message: string) => void) => void;
  _view?: { time?: number; w3dScene?: { render: () => void; renderer?: { render: () => void } }; onclose?: () => void; onerror?: (message: string) => void };
  [key: string]: unknown;
};

declare global {
  interface Window {
    robotRacersRenderStats?: { total: number; times: number[]; fps: number };
    robotracersRendererStats?: () => { connected: boolean; frames: number; fps: number; last_frame_age_ms: number | null };
  }
}

export function WebotsScene({ config, mode, scale, cameraImage, cameraAvailable, cameraMetadata, state, selectedGoal }: {
  config: AppConfig | null; mode: ViewMode; scale: number; cameraImage: string | null; cameraAvailable: boolean;
  cameraMetadata: CameraTracking | null; state: SimulationState; selectedGoal: string;
}) {
  const stage = useRef<HTMLDivElement>(null);
  const host = useRef<HTMLDivElement>(null);
  const view = useRef<WebotsElement | null>(null);
  const [connection, setConnection] = useState<'loading' | 'ready' | 'error'>('loading');
  const [retry, setRetry] = useState(0);
  const currentMode = useRef(mode); currentMode.current = mode;
  const latest = useRef({state,selectedGoal,cameraMetadata,cameraAvailable});
  latest.current={state,selectedGoal,cameraMetadata,cameraAvailable};
  const [tracking,setTracking]=useState<OverlayGeometry|null>(null);

  useEffect(() => {
    let disposed=false,frame=0,lastDraw=0,lastSimTime=Number.NaN,lastStateStamp=performance.now();
    let world: {instance?:{viewpoint?:ViewCamera}} | undefined;
    if(mode!=='camera'&&connection==='ready'&&config){
      const base=config.webots.module.replace(/WebotsView\.js(?:\?.*)?$/,'');
      void import(/* @vite-ignore */ `${base}nodes/WbWorld.js`).then(module=>{if(!disposed)world=module.default;}).catch(()=>{});
    }
    const draw=(stamp:number)=>{
      if(disposed)return;
      frame=requestAnimationFrame(draw);
      if(stamp-lastDraw<30||document.visibilityState!=='visible'||!stage.current)return;
      lastDraw=stamp;
      const viewport=stage.current.getBoundingClientRect(),input=latest.current;
      if(input.state.sim_time!==lastSimTime){lastSimTime=input.state.sim_time;lastStateStamp=stamp;}
      const target=selectTrackingTarget(input.state,input.selectedGoal);
      if(mode==='camera'){
        setTracking(cameraTracking(input.cameraAvailable?input.cameraMetadata:null,target,viewport.width,viewport.height,input.state.connected));
      }else{
        const canvas=host.current?.querySelector<HTMLCanvasElement>('#canvas');
        if(connection!=='ready'||!world?.instance?.viewpoint||!canvas){setTracking(null);return;}
        const track=Number(config?.robot.track_width_m??.19);
        const freshState=stamp-lastStateStamp<=500?input.state:{...input.state,connected:false};
        setTracking(worldTracking(freshState,target,world.instance.viewpoint,canvas.getBoundingClientRect(),viewport,
          boundsForConfig(config?.robot),typeof view.current?._view?.time==='number'?view.current._view.time/1000:undefined,track));
      }
    };
    frame=requestAnimationFrame(draw);
    return()=>{disposed=true;cancelAnimationFrame(frame);};
  },[mode,connection,config?.webots.module,config?.robot]);

  useEffect(() => {
    if (!config || !host.current) return;
    let disposed = false;
    let timeout: ReturnType<typeof setTimeout>;
    let observer: ResizeObserver | undefined;
    const measurements = { connected: false };
    const rendererStats = () => readRenderMetrics(window.robotRacersRenderStats, measurements.connected, performance.now());
    window.robotracersRendererStats = rendererStats;
    let reporting = false;
    const reportController = new AbortController();
    const reportMetrics = async () => {
      if (reporting || disposed) return;
      reporting = true;
      try {
        const sample = rendererStats();
        await fetch('/api/client-metrics', {
          method: 'POST', headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            render_fps: sample.fps, total_frames: sample.frames,
            visible: sample.connected && sample.available
              && document.visibilityState === 'visible' && currentMode.current !== 'camera',
          }),
          signal: AbortSignal.any([reportController.signal, AbortSignal.timeout(2500)]),
        });
      } catch { /* Diagnostics must never interfere with the scene or mission. */ }
      finally { reporting = false; }
    };
    const reporter = setInterval(() => void reportMetrics(), 1000);
    const connect = async () => {
      setConnection('loading');
      try {
        const moduleUrl = config.webots.module;
        await import(/* @vite-ignore */ moduleUrl);
        if (disposed || !host.current) return;
        const element = document.createElement('webots-view') as WebotsElement;
        for (const key of ['showIde', 'showInfo', 'showPlay', 'showQuit', 'showReload', 'showReset', 'showRobotWindow', 'showRun', 'showStep', 'showTerminal', 'showWorldSelection']) element[key] = false;
        element.onready = () => {
          if (disposed) return;
          clearTimeout(timeout); element.hideToolbar(); element.resize();
          // W3dScene R2025a pauses the world while loading a model, even in
          // broadcast mode. Keep the simulator clock running; mission pause/stop
          // remains exclusively in the service and does not move the robot.
          element.sendMessage('real-time:-1');
          measurements.connected = true;
          // The official stream calls onclose, while WebotsView only exposes onquit.
          // Observe transport loss explicitly so a disconnected scene never stays blank.
          if (element._view) {
            element._view.onclose = () => { measurements.connected = false; if (!disposed) setConnection('error'); };
            const report = element._view.onerror;
            element._view.onerror = (message) => { report?.(message); if (!disposed) setConnection('error'); };
          }
          setConnection('ready');
        };
        element.ondisconnect = () => { measurements.connected = false; if (!disposed) setConnection('error'); };
        host.current.replaceChildren(element); view.current = element;
        element.connect(config.webots.url, 'w3d', true, false);
        element.hideToolbar();
        observer = new ResizeObserver(() => element.resize()); observer.observe(host.current);
        timeout = setTimeout(() => { if (!disposed) setConnection('error'); }, 25000);
      } catch { if (!disposed) setConnection('error'); }
    };
    void connect();
    return () => {
      disposed = true; measurements.connected = false; clearTimeout(timeout); observer?.disconnect();
      clearInterval(reporter); reportController.abort();
      view.current?.close(); view.current = null; host.current?.replaceChildren();
      if (window.robotracersRendererStats === rendererStats) delete window.robotracersRendererStats;
    };
  }, [config?.webots.module, config?.webots.url, retry]);

  useEffect(() => {
    if (connection !== 'ready' || mode === 'camera' || !view.current) return;
    let disposed = false;
    const applyView = async () => {
      try {
        const base = config?.webots.module.replace(/WebotsView\.js(?:\?.*)?$/, '') ?? '/webots/wwi/';
        const [worldModule, vector3Module, vector4Module] = await Promise.all([
          import(/* @vite-ignore */ `${base}nodes/WbWorld.js`),
          import(/* @vite-ignore */ `${base}nodes/utils/WbVector3.js`),
          import(/* @vite-ignore */ `${base}nodes/utils/WbVector4.js`),
        ]);
        if (disposed || !view.current) return;
        const viewpoint = worldModule.default.instance?.viewpoint;
        if (!viewpoint) return;
        if (mode === 'orbital') {
          // Look at the arena origin with FLU camera axes and world Z up.
          // This is independent of the native editor's previous viewpoint.
          viewpoint.position = new vector3Module.default(1.9 * scale, -2.6 * scale, 2.8 * scale);
          viewpoint.orientation = new vector4Module.default(-.3448517026320715, .17511150831188757, .9221785417415095, 2.2657049874880206);
          viewpoint.fieldOfView = 1.05;
          viewpoint.updatePosition(); viewpoint.updateOrientation(); viewpoint.updateFieldOfView();
        } else {
          const width = (config?.arena.width_m ?? 2.4) * scale;
          const height = (config?.arena.height_m ?? 1.8) * scale;
          const aspect = (host.current?.clientWidth ?? 900) / Math.max(1, host.current?.clientHeight ?? 600);
          const fieldOfView = Math.PI / 4;
          const altitude = topViewAltitude(width,height,aspect,fieldOfView);
          viewpoint.position = new vector3Module.default(0, 0, altitude);
          // Webots uses FLU camera axes: forward=-Z, up=+Y in this top view.
          viewpoint.orientation = new vector4Module.default(-1 / Math.sqrt(3), 1 / Math.sqrt(3), 1 / Math.sqrt(3), 2 * Math.PI / 3);
          viewpoint.fieldOfView = fieldOfView;
          viewpoint.updatePosition(); viewpoint.updateOrientation(); viewpoint.updateFieldOfView();
        }
        view.current._view?.w3dScene?.render();
      } catch { /* The live scene remains available if a camera shortcut is unsupported. */ }
    };
    void applyView();
    const observer=new ResizeObserver(()=>{if(mode==='top')void applyView();});
    if(host.current)observer.observe(host.current);
    return () => { disposed = true; observer.disconnect(); };
  }, [mode, scale, connection, config]);

  return <div ref={stage} className={`rr-scene rr-view-${mode}`}>
    <div className="rr-webots-host" ref={host} aria-label="Escena física tridimensional del robot" style={{ visibility: mode === 'camera' ? 'hidden' : 'visible' }} />
    {mode === 'camera' && <div className="rr-camera-stage">
      {cameraImage && <img src={cameraImage} alt="Imagen de la cámara cenital utilizada por el sistema de visión" className={!cameraAvailable ? 'rr-stale' : ''} />}
      {!cameraAvailable && <div className="rr-scene-cover"><Icon name="camera" size={30} /><h2>Esperando imagen</h2><p>La cámara cenital aparecerá al conectar el entorno.</p></div>}
      <span className="rr-camera-live"><span className={`rr-dot ${cameraAvailable ? 'rr-dot-live' : ''}`} />Cámara cenital</span>
    </div>}
    {tracking && (mode==='camera'||connection==='ready') && <TrackingOverlay {...tracking} />}
    {connection !== 'ready' && mode !== 'camera' && <div className="rr-scene-cover">
      <div className={`rr-scene-symbol ${config && connection === 'loading' ? 'rr-loading-symbol' : ''}`}><Icon name="orbit" size={30} /></div>
      <h2>{!config ? 'Esperando el entorno' : connection === 'loading' ? 'Preparando el entorno' : 'Escena sin conexión'}</h2>
      <p>{!config ? 'La aplicación se conectará al iniciar el servicio local.' : connection === 'loading' ? 'Conectando el robot y el laberinto.' : 'Comprueba que Webots esté abierto para ver la simulación.'}</p>
      {config && connection === 'error' && <button className="rr-button rr-button-light" onClick={() => setRetry(v => v + 1)}><Icon name="reset" size={15} />Reconectar escena</button>}
    </div>}
  </div>;
}
