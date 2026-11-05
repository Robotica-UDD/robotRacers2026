import { useEffect, useRef, useState } from 'react';
import type { CSSProperties } from 'react';
import { BrandMark, Icon } from './icons';
import { RouteMap } from './RouteMap';
import { WebotsScene } from './WebotsScene';
import { useCameraPreview, useSimulation } from './useSimulation';
import { arenaGeometry } from './geometry';
import { formatNumber, formatTime, initialSettings, metricValue, statusLabels } from './types';
import type { Goal, Settings, ViewMode } from './types';

function RangeControl({ title, value, min, max, step, display, disabled, onChange, onCommit, hint }: {
  title: string; value: number; min: number; max: number; step: number; display: string;
  disabled?: boolean; onChange: (value: number) => void; onCommit: (value: number) => void; hint?: string;
}) {
  return <label className={`rr-range-control ${disabled ? 'rr-control-disabled' : ''}`}>
    <span className="rr-range-label"><span>{title}</span><strong>{display}</strong></span>
    <input type="range" min={min} max={max} step={step} value={value} disabled={disabled} aria-label={title}
      style={{ '--rr-fill': `${(value - min) / (max - min) * 100}%` } as CSSProperties}
      onChange={event => onChange(Number(event.currentTarget.value))}
      onPointerUp={event => onCommit(Number(event.currentTarget.value))}
      onKeyUp={event => { if (['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown', 'Home', 'End', 'PageUp', 'PageDown'].includes(event.key)) onCommit(Number(event.currentTarget.value)); }} />
    {hint && <span className="rr-range-hint">{hint}</span>}
  </label>;
}

export default function App() {
  const simulation = useSimulation();
  const { state, config, connected, online, busy } = simulation;
  const [viewMode, setViewMode] = useState<ViewMode>('orbital');
  const [selectedGoal, setSelectedGoal] = useState('B');
  const [advanced, setAdvanced] = useState(false);
  const [configuring, setConfiguring] = useState(false);
  const [draft, setDraft] = useState<Settings>(initialSettings);
  const dirty = useRef(new Set<keyof Settings>());
  const versions = useRef<Partial<Record<keyof Settings, number>>>({});
  const speedTimer = useRef<ReturnType<typeof setTimeout> | undefined>(undefined);
  const camera = useCameraPreview(online);
  const moving = state.status === 'running', paused = state.status === 'paused';
  const sceneUpdating = configuring || Boolean(state.scene_pending);
  const editingLocked = moving || paused || Boolean(busy) || sceneUpdating;
  const displayStatus = !connected ? 'disconnected' : state.status;
  const localized = connected && Boolean(state.pose.source) && state.pose.source !== 'sin localización' && Number.isFinite(state.pose.x);
  const geometry = arenaGeometry(config?.arena, state.settings.scale);
  const goals: Goal[] = state.mission.goals?.length ? state.mission.goals : ['A', 'B', 'C', 'D'].map(id => ({ id, ...geometry.nodes[id] }));
  const currentGoal = typeof state.mission.active_goal === 'number' ? goals[state.mission.active_goal]?.id : state.mission.active_goal;
  const selected = goals.find(goal => goal.id === selectedGoal);
  const rawProgress = state.mission.progress || 0;
  const progress = Math.max(0, Math.min(100, rawProgress <= 1 ? rawProgress * 100 : rawProgress));
  const latency = state.vision.latency_ms;
  const contacts = connected ? metricValue(state.metrics, 'collisions', 'contacts', 'collision_count') : null;

  useEffect(() => {
    setDraft(previous => {
      const next = { ...previous };
      for (const key of Object.keys(initialSettings) as (keyof Settings)[]) if (!dirty.current.has(key) && Number.isFinite(state.settings[key])) next[key] = state.settings[key];
      return next;
    });
  }, [state.settings]);
  useEffect(() => () => clearTimeout(speedTimer.current), []);

  const commit = async (key: keyof Settings, value: number) => {
    if (key === 'speed') clearTimeout(speedTimer.current);
    else setConfiguring(true);
    const version = versions.current[key] ?? 0;
    const ok = await simulation.updateSettings({ [key]: value });
    if (version === versions.current[key]) {
      dirty.current.delete(key);
      if (!ok) setDraft(previous => ({ ...previous, [key]: state.settings[key] }));
    }
    if (key !== 'speed') setConfiguring(false);
  };
  const change = (key: keyof Settings, value: number) => {
    dirty.current.add(key); versions.current[key] = (versions.current[key] ?? 0) + 1;
    setDraft(previous => ({ ...previous, [key]: value }));
    if (key === 'speed') {
      clearTimeout(speedTimer.current);
      speedTimer.current = setTimeout(() => void commit(key, value), 140);
    }
  };

  const runOrPause = () => simulation.mission(moving ? 'pause' : 'start');
  const toggleFullscreen = () => document.fullscreenElement ? document.exitFullscreen() : document.documentElement.requestFullscreen();

  return <div className="rr-app">
    <header className="rr-header">
      <a href="/" className="rr-brand" aria-label="RobotRacers, inicio"><BrandMark /><span>RobotRacers<small>Estudio de navegación</small></span></a>
      <div className="rr-header-center"><span>Robot diferencial</span><span className="rr-header-divider" /><span>Visión + encoders</span></div>
      <div className="rr-header-right"><span className={`rr-status rr-status-${displayStatus}`}><span className="rr-dot" />{statusLabels[displayStatus]}</span><button className="rr-icon-button rr-fullscreen" title="Pantalla completa" aria-label="Pantalla completa" onClick={() => void toggleFullscreen()}><Icon name="expand" /></button></div>
    </header>

    <main className="rr-workspace">
      <section className="rr-stage-panel" aria-label="Simulación">
        <div className="rr-stage-topbar">
          <div><span className="rr-eyebrow">ENTORNO</span><h1>Laberinto de doble T<span className="rr-environment-size">{formatNumber(geometry.width, 1)} × {formatNumber(geometry.height, 1)} m</span></h1></div>
          <div className="rr-view-switch" role="group" aria-label="Vista de la simulación">
            {([{ id: 'orbital', label: 'Orbital', icon: 'orbit' }, { id: 'top', label: 'Cenital', icon: 'top' }, { id: 'camera', label: 'Cámara', icon: 'camera' }] as const).map(item =>
              <button key={item.id} className={viewMode === item.id ? 'rr-view-active' : ''} aria-label={`Vista ${item.label.toLowerCase()}`} aria-pressed={viewMode === item.id} onClick={() => setViewMode(item.id)}><Icon name={item.icon} size={15} /><span>{item.label}</span></button>)}
          </div>
        </div>

        <div className="rr-stage-viewport">
          <WebotsScene config={config} mode={viewMode} scale={state.settings.scale} cameraImage={camera.image} cameraAvailable={camera.available && connected} />
          <div className="rr-stage-corner rr-glass"><span className={`rr-dot ${connected && state.vision.visible ? 'rr-dot-live' : ''}`} /><span>{connected && state.vision.visible ? 'Localización activa' : 'Esperando localización'}</span></div>
          {viewMode === 'top' && <div className="rr-stage-compass" aria-hidden="true"><span>N</span><i /><span>S</span></div>}
          <div className="rr-mission-controls rr-glass">
            <button className="rr-start-button" disabled={!connected || Boolean(busy) || sceneUpdating} onClick={() => void runOrPause()}><Icon name={moving ? 'pause' : 'play'} size={17} />{busy === 'start' || busy === 'pause' || sceneUpdating ? 'Aplicando…' : moving ? 'Pausar' : paused ? 'Reanudar' : 'Iniciar recorrido'}</button>
            <button className="rr-icon-button rr-stop-button" title="Detener recorrido" aria-label="Detener recorrido" disabled={!online} onClick={() => void simulation.mission('stop')}><Icon name="stop" size={17} /></button>
            <span className="rr-control-divider" />
            <button className="rr-reset-button" aria-label="Reiniciar recorrido" disabled={!online || Boolean(busy) || sceneUpdating} onClick={() => void simulation.mission('reset')}><Icon name="reset" size={16} /><span>Reiniciar</span></button>
          </div>
        </div>

        <div className="rr-telemetry-bar">
          <div className="rr-telemetry-primary"><Icon name="signal" size={17} /><span>Velocidad<strong>{formatNumber(connected ? state.robot.speed : null)}<small> m/s</small></strong></span></div>
          <div><span>Rueda izquierda<strong>{formatNumber(connected ? state.robot.left_speed : null)}<small> m/s</small></strong></span></div>
          <div><span>Rueda derecha<strong>{formatNumber(connected ? state.robot.right_speed : null)}<small> m/s</small></strong></span></div>
          <div><span>Tiempo<strong>{online ? formatTime(state.sim_time) : '—'}<small> sim.</small></strong></span></div>
          <div className="rr-battery"><Icon name="battery" size={18} /><span>{formatNumber(connected ? state.robot.battery_voltage : null, 1)} V</span></div>
        </div>
      </section>

      <aside className="rr-sidebar" aria-label="Configuración del recorrido">
        <section className="rr-route-card rr-glass">
          <div className="rr-card-heading"><div><span className="rr-eyebrow">MISIÓN</span><h2>Tu recorrido</h2></div><span className="rr-small-pill">4 destinos</span></div>
          <div className="rr-destination-strip">
            {goals.map((goal, index) => {
              const activeIndex = goals.findIndex(item => item.id === currentGoal);
              const done = state.status === 'completed' || (activeIndex >= 0 && index < activeIndex);
              return <div className="rr-destination-node" key={goal.id}><button className={`${selectedGoal === goal.id ? 'rr-selected-destination' : ''} ${currentGoal === goal.id && connected ? 'rr-current-destination' : ''} ${done ? 'rr-destination-done' : ''}`}
                aria-label={`Seleccionar destino ${goal.id}`} aria-pressed={selectedGoal === goal.id} onClick={() => setSelectedGoal(goal.id)}>{done ? <Icon name="check" size={17} /> : goal.id}</button>{index < goals.length - 1 && <span className="rr-destination-link" />}</div>;
            })}
          </div>
          <RouteMap arena={config?.arena} scale={state.settings.scale} goals={goals} state={{ ...state, connected }} editable={online && !editingLocked} selected={selectedGoal} onSelect={setSelectedGoal} onUpdate={simulation.updateGoals} />
          <div className="rr-map-caption"><span>{editingLocked ? 'Recorrido bloqueado durante la misión' : 'Arrastra los destinos sobre los pasillos'}</span><Icon name={editingLocked ? 'route' : 'target'} size={14} /></div>
          <div className="rr-selected-position"><span>Destino <b>{selectedGoal}</b></span><span>X {formatNumber(selected?.x)}<i />Y {formatNumber(selected?.y)}<small> m</small></span></div>
          <div className="rr-route-progress"><div><span>{state.status === 'completed' ? 'Misión completada' : currentGoal && connected ? `Hacia el destino ${currentGoal}` : 'Listo para planificar'}</span><strong>{Math.round(progress)}%</strong></div><div className="rr-progress-track"><span style={{ width: `${progress}%` }} /></div></div>
        </section>

        <section className="rr-settings-card rr-glass">
          <div className="rr-card-heading"><h2>Ajustes de recorrido</h2><Icon name="tune" size={17} /></div>
          <RangeControl title="Velocidad objetivo" value={draft.speed} min={config?.arena.speed_min ?? .05} max={config?.arena.speed_max ?? 1.5} step={.05} display={`${formatNumber(draft.speed)} m/s`} disabled={!online} onChange={value => change('speed', value)} onCommit={value => void commit('speed', value)} />
          <div className="rr-range-extents"><span>Precisión</span><span>Velocidad</span></div>
          <RangeControl title="Tamaño del recorrido" value={draft.scale} min={config?.arena.scale_min ?? .8} max={config?.arena.scale_max ?? 1.5} step={.05} display={`${Math.round(draft.scale * 100)}%`} disabled={!online || editingLocked} onChange={value => change('scale', value)} onCommit={value => void commit('scale', value)} hint={editingLocked ? 'Detén el recorrido para cambiar el entorno.' : undefined} />
          <button className={`rr-advanced-toggle ${advanced ? 'rr-advanced-open' : ''}`} aria-expanded={advanced} onClick={() => setAdvanced(!advanced)}><span>Parámetros físicos</span><Icon name="chevron" size={15} /></button>
          {advanced && <div className="rr-advanced-settings">
            <p>Masa y adherencia estimadas. Ajusta estos valores al medir el robot.</p>
            <RangeControl title="Masa del robot" value={draft.mass_kg} min={.1} max={.5} step={.005} display={`${Math.round(draft.mass_kg * 1000)} g`} disabled={!online || editingLocked} onChange={value => change('mass_kg', value)} onCommit={value => void commit('mass_kg', value)} />
            <RangeControl title="Adherencia de las ruedas" value={draft.friction} min={.1} max={1.5} step={.05} display={formatNumber(draft.friction)} disabled={!online || editingLocked} onChange={value => change('friction', value)} onCommit={value => void commit('friction', value)} />
            <RangeControl title="Rozamiento del apoyo" value={draft.skid_friction} min={.01} max={.6} step={.01} display={formatNumber(draft.skid_friction)} disabled={!online || editingLocked} onChange={value => change('skid_friction', value)} onCommit={value => void commit('skid_friction', value)} />
            <RangeControl title="Fuerza de la turbina" value={draft.fan_force_n} min={0} max={3} step={.1} display={`${formatNumber(draft.fan_force_n, 1)} N`} disabled={!online || editingLocked} onChange={value => change('fan_force_n', value)} onCommit={value => void commit('fan_force_n', value)} hint="Desactivada por defecto; fuerza sin calibrar." />
          </div>}
        </section>

        <section className="rr-vision-card rr-glass">
          <div className="rr-card-heading"><h2>Cámara y localización</h2><span className={`rr-vision-indicator ${connected && state.vision.visible ? 'rr-visible' : ''}`}><span className="rr-dot" />{connected && state.vision.visible ? 'Visible' : 'Sin señal'}</span></div>
          <button className="rr-camera-preview" onClick={() => setViewMode('camera')} aria-label="Ver imagen de la cámara cenital">
            {camera.image ? <img src={camera.image} className={!camera.available || !connected ? 'rr-stale' : ''} alt="Vista previa de la cámara cenital" /> : <div className="rr-camera-placeholder"><Icon name="camera" size={23} /><span>Cámara cenital</span></div>}
            <span className="rr-preview-expand"><Icon name="expand" size={13} /></span>
          </button>
          <div className="rr-vision-metrics"><span>Latencia visual<strong>{formatNumber(connected ? latency : null, 0)}<small> ms</small></strong></span><span>Posición estimada<strong>{formatNumber(localized ? state.pose.x : null)}<small> / </small>{formatNumber(localized ? state.pose.y : null)}<small> m</small></strong></span></div>
        </section>
      </aside>
    </main>

    <footer className="rr-footer"><span><span className={`rr-dot ${online ? 'rr-dot-live' : ''}`} />{online ? 'Entorno local conectado' : 'Esperando el servicio local'}</span><div><span>Orientación <b>{formatNumber(localized ? state.pose.yaw * 180 / Math.PI : null, 1)}°</b></span><i /><span>Contactos <b>{contacts === null ? '—' : Math.round(contacts)}</b></span><i /><span>Navegación por visión</span></div><span className="rr-footer-brand">ROBOTRACERS <b>STUDIO</b></span></footer>
    {simulation.error && <div className="rr-toast" role="alert"><Icon name="info" size={18} /><span>{simulation.error}</span><button aria-label="Cerrar aviso" onClick={simulation.dismissError}><Icon name="close" size={16} /></button></div>}
  </div>;
}
