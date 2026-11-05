import { useId } from 'react';
import type { TrackingBox, TrackingPoint } from './trackingGeometry';
import { pointInTrackingViewport, trackingTether, validTrackingBox, validTrackingViewport } from './trackingGeometry';
import './TrackingOverlay.css';

export type TrackingOverlayProps = {
  width: number;
  height: number;
  box: TrackingBox | null;
  robot: TrackingPoint | null;
  target: { id: string; pixel: TrackingPoint } | null;
  tracking: 'visible' | 'predicted' | 'lost';
  /** Optional compact detail below the goal identifier, for example its distance. */
  label?: string;
};

function boxCorners(box: TrackingBox) {
  const right = box.x + box.width, bottom = box.y + box.height;
  const corner = Math.min(10, box.width * .3, box.height * .3);
  return `M${box.x},${box.y + corner}V${box.y}H${box.x + corner}`
    + `M${right - corner},${box.y}H${right}V${box.y + corner}`
    + `M${right},${bottom - corner}V${bottom}H${right - corner}`
    + `M${box.x + corner},${bottom}H${box.x}V${bottom - corner}`;
}

/** Presentational only: every position is supplied by the projection/localisation layer. */
export function TrackingOverlay({ width, height, box, robot, target, tracking, label }: TrackingOverlayProps) {
  const instance = useId().replace(/[^a-zA-Z0-9_-]/g, '');
  if (!validTrackingViewport(width, height)) return null;
  const clipId = `rr-tracking-clip-${instance}`, arrowId = `rr-tracking-arrow-${instance}`;
  const located = tracking !== 'lost';
  const outline = located && validTrackingBox(box) ? box : null;
  const centre = located && pointInTrackingViewport(robot, width, height) ? robot : null;
  const goal = target && pointInTrackingViewport(target.pixel, width, height) ? target : null;
  const tether = located ? trackingTether(outline, robot, target?.pixel ?? null, width, height) : null;
  if (!outline && !centre && !goal && !tether) return null;
  const textOnLeft = goal ? goal.pixel.x > width - Math.min(88, width / 3) : false;
  const textX = goal ? goal.pixel.x + (textOnLeft ? -13 : 13) : 0;
  const textY = goal ? goal.pixel.y < 29 ? goal.pixel.y + 20 : goal.pixel.y - 11 : 0;
  const accessible = tracking === 'lost' ? 'Sin localización del robot' : tracking === 'predicted' ? 'Posición estimada del robot' : 'Robot localizado';

  return <svg className={`rr-tracking-overlay rr-tracking-${tracking}`} width={width} height={height}
    viewBox={`0 0 ${width} ${height}`} role="img" aria-label={`${accessible}${target ? `. Objetivo ${target.id}` : ''}`}>
    <defs>
      <clipPath id={clipId}><rect width={width} height={height} /></clipPath>
      <marker id={arrowId} markerWidth="7" markerHeight="7" refX="5" refY="3.5" orient="auto" markerUnits="userSpaceOnUse">
        <path d="M1 1 5 3.5 1 6" className="rr-tracking-arrow" />
      </marker>
    </defs>
    <g clipPath={`url(#${clipId})`}>
      {tether && <g className="rr-tracking-tether">
        <line className="rr-tracking-halo" x1={tether.start.x} y1={tether.start.y} x2={tether.end.x} y2={tether.end.y} />
        <line x1={tether.start.x} y1={tether.start.y} x2={tether.end.x} y2={tether.end.y} markerEnd={`url(#${arrowId})`} />
      </g>}
      {outline && <g className="rr-tracking-box">
        <path d={boxCorners(outline)} className="rr-tracking-halo" />
        <path d={boxCorners(outline)} />
      </g>}
      {centre && <g className="rr-tracking-centre" transform={`translate(${centre.x} ${centre.y})`}>
        <path d="M-7 0H-3M3 0H7M0-7V-3M0 3V7" className="rr-tracking-halo" />
        <path d="M-7 0H-3M3 0H7M0-7V-3M0 3V7" />
        <circle r="1.5" />
      </g>}
      {goal && <g className="rr-tracking-goal">
        <circle cx={goal.pixel.x} cy={goal.pixel.y} r="7" className="rr-tracking-halo" />
        <circle cx={goal.pixel.x} cy={goal.pixel.y} r="7" />
        <circle cx={goal.pixel.x} cy={goal.pixel.y} r="2" className="rr-tracking-goal-dot" />
        <text x={textX} y={textY} textAnchor={textOnLeft ? 'end' : 'start'}>
          <tspan className="rr-tracking-goal-id">{goal.id}</tspan>
          {label && <tspan className="rr-tracking-detail" x={textX} dy="13">{label}</tspan>}
        </text>
      </g>}
    </g>
  </svg>;
}
