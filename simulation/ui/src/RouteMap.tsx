import { useEffect, useRef, useState } from 'react';
import type { KeyboardEvent, PointerEvent } from 'react';
import { arenaGeometry, mapToWorld, snapToCorridors, worldToMap } from './geometry';
import type { AppConfig, Goal, SimulationState } from './types';

export function RouteMap({ arena, scale, goals, state, editable, selected, onSelect, onUpdate }: {
  arena?: AppConfig['arena']; scale: number; goals: Goal[]; state: SimulationState;
  editable: boolean; selected: string; onSelect: (id: string) => void; onUpdate: (goals: Goal[]) => Promise<boolean>;
}) {
  const svg = useRef<SVGSVGElement>(null);
  const dragging = useRef<string | null>(null);
  const current = useRef(goals);
  const [draft, setDraft] = useState(goals);
  const geometry = arenaGeometry(arena, scale);
  useEffect(() => { if (!dragging.current) { current.current = goals; setDraft(goals); } }, [goals]);
  const toMap = (p: { x: number; y: number }) => worldToMap(p, geometry.width, geometry.height);
  const segmentPath = geometry.edges.map(([a, b]) => {
    const start = toMap(geometry.nodes[a]), end = toMap(geometry.nodes[b]);
    return `M${start.x},${start.y}L${end.x},${end.y}`;
  }).join(' ');
  const path = state.mission.waypoints?.map(toMap).map(p => `${p.x},${p.y}`).join(' ') ?? '';

  const move = (id: string, point: { x: number; y: number }) => {
    const snapped = snapToCorridors(point, geometry.nodes, geometry.edges);
    const next = current.current.map(goal => goal.id === id ? { ...goal, ...snapped } : goal);
    current.current = next; setDraft(next);
  };
  const pointerMove = (event: PointerEvent<SVGGElement>) => {
    if (!dragging.current || !svg.current) return;
    const matrix = svg.current.getScreenCTM(); if (!matrix) return;
    const p = svg.current.createSVGPoint(); p.x = event.clientX; p.y = event.clientY;
    const relative = p.matrixTransform(matrix.inverse());
    move(dragging.current, mapToWorld(relative, geometry.width, geometry.height));
  };
  const finishDrag = async (event: PointerEvent<SVGGElement>) => {
    if (!dragging.current) return;
    dragging.current = null;
    if (event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId);
    const ok = await onUpdate(current.current);
    if (!ok) { current.current = goals; setDraft(goals); }
  };
  const keyboardMove = (event: KeyboardEvent<SVGGElement>, goal: Goal) => {
    onSelect(goal.id);
    if (!editable) return;
    const step = .025 * scale;
    const directions: Record<string, [number, number]> = { ArrowLeft: [-step, 0], ArrowRight: [step, 0], ArrowUp: [0, step], ArrowDown: [0, -step] };
    if (directions[event.key]) {
      event.preventDefault();
      const [dx, dy] = directions[event.key];
      move(goal.id, { x: goal.x + dx, y: goal.y + dy });
    }
  };
  const poseAvailable = state.connected && Boolean(state.pose.source) && state.pose.source !== 'sin localización'
    && Number.isFinite(state.pose.x) && Number.isFinite(state.pose.y);
  const pose = poseAvailable ? toMap(state.pose) : null;

  return <svg ref={svg} className={`rr-route-map ${editable ? 'rr-map-editable' : ''}`} viewBox="0 0 480 360" role="group" aria-label="Mapa de destinos. Arrastra A, B, C o D sobre los pasillos; también puedes usar las flechas del teclado.">
    <defs>
      <pattern id="rr-map-grid" width="24" height="24" patternUnits="userSpaceOnUse"><circle cx="1" cy="1" r=".8" fill="#c7d0ca" /></pattern>
      <filter id="rr-goal-shadow" x="-50%" y="-50%" width="200%" height="200%"><feDropShadow dx="0" dy="3" stdDeviation="4" floodColor="#203c31" floodOpacity=".16" /></filter>
    </defs>
    <rect x="0" y="0" width="480" height="360" rx="18" fill="#e8eeea" />
    <rect width="480" height="360" rx="18" fill="url(#rr-map-grid)" />
    <path d={segmentPath} fill="none" stroke="#53645c" strokeWidth={geometry.corridor / geometry.width * 480 + 5} strokeLinejoin="miter" />
    <path d={segmentPath} fill="none" stroke="#fafcf9" strokeWidth={geometry.corridor / geometry.width * 480} strokeLinejoin="miter" />
    <path d={segmentPath} fill="none" stroke="#ced7d0" strokeWidth="1.5" strokeDasharray="3 7" />
    {path && <polyline points={path} fill="none" stroke="#789e73" strokeWidth="3" strokeLinejoin="round" strokeLinecap="round" opacity=".75" />}
    {pose && <g transform={`translate(${pose.x} ${pose.y}) rotate(${-state.pose.yaw * 180 / Math.PI})`} className="rr-map-robot">
      <circle r="15" fill="#d0e3b9" opacity=".6" /><path d="m10 0-16-7 3 7-3 7Z" fill="#294d3b" stroke="#fff" strokeWidth="2" />
    </g>}
    {draft.map(goal => {
      const p = toMap(goal);
      return <g key={goal.id} transform={`translate(${p.x} ${p.y})`} className={`rr-map-goal ${goal.id === selected ? 'rr-map-goal-selected' : ''}`}
        tabIndex={0} role="button" aria-label={`Destino ${goal.id}, X ${goal.x.toFixed(2)} metros, Y ${goal.y.toFixed(2)} metros${editable ? ', arrastrable' : ''}`}
        onFocus={() => onSelect(goal.id)}
        onPointerDown={event => {
          onSelect(goal.id);
          if (!editable) return;
          event.preventDefault(); dragging.current = goal.id; event.currentTarget.setPointerCapture(event.pointerId);
        }}
        onPointerMove={pointerMove} onPointerUp={event => void finishDrag(event)}
        onPointerCancel={() => { dragging.current = null; current.current = goals; setDraft(goals); }}
        onKeyDown={event => keyboardMove(event, goal)} onKeyUp={event => { if (editable && event.key.startsWith('Arrow')) void onUpdate(current.current); }}>
        <circle className="rr-goal-halo" r="24" fill="none" stroke="#90ae69" strokeWidth="1.5" />
        <circle r="17" fill={goal.id === selected ? '#2d4d3d' : '#fff'} stroke={goal.id === selected ? '#2d4d3d' : '#d7dfd6'} strokeWidth="1.2" filter="url(#rr-goal-shadow)" />
        <text textAnchor="middle" dominantBaseline="central" fill={goal.id === selected ? '#e7f5cf' : '#425248'} fontSize="14" fontWeight="600">{goal.id}</text>
      </g>;
    })}
    <text x="19" y="338" fontSize="10.5" fill="#849289" letterSpacing="1.5">PLANTA</text>
    <g transform="translate(450 27)"><path d="M0 12V-4m-4 4 4-4 4 4" fill="none" stroke="#7f8f85" strokeWidth="1.2" /><text y="-9" textAnchor="middle" fontSize="9" fill="#7f8f85">N</text></g>
  </svg>;
}
