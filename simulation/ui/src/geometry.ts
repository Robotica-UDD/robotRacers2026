import type { AppConfig, Point } from './types';

export const defaultNodes: Record<string, Point> = {
  A: { x: -.6, y: .55 }, B: { x: .6, y: .55 },
  C: { x: -.6, y: -.55 }, D: { x: .6, y: -.55 },
  L: { x: -.6, y: 0 }, R: { x: .6, y: 0 },
};
export const defaultEdges = [['A', 'L'], ['C', 'L'], ['L', 'R'], ['R', 'B'], ['R', 'D']];

export function arenaGeometry(arena: AppConfig['arena'] | undefined, scale: number) {
  // /api/config exposes nodes at arena.scale; dimensions remain base dimensions.
  const nodeScale = scale / (typeof arena?.scale === 'number' ? arena.scale : 1);
  const nodes = arena?.nodes
    ? Object.fromEntries(Object.entries(arena.nodes).map(([id, xy]) => [id, { x: xy[0] * nodeScale, y: xy[1] * nodeScale }]))
    : Object.fromEntries(Object.entries(defaultNodes).map(([id, p]) => [id, { x: p.x * scale, y: p.y * scale }]));
  const edges = (arena?.edges ?? defaultEdges).filter(([a, b]) => nodes[a] && nodes[b]);
  return {
    nodes, edges,
    width: (arena?.width_m ?? 2.4) * scale,
    height: (arena?.height_m ?? 1.8) * scale,
    corridor: (arena?.corridor_width_m ?? .5) * scale,
  };
}

/** The map only edits targets. Actual routing is always computed by the service. */
export function snapToCorridors(point: Point, nodes: Record<string, Point>, edges: string[][]): Point {
  let best = point, distance = Infinity;
  for (const [a, b] of edges) {
    const start = nodes[a], end = nodes[b];
    if (!start || !end) continue;
    const dx = end.x - start.x, dy = end.y - start.y;
    const length2 = dx * dx + dy * dy;
    const t = length2 === 0 ? 0 : Math.max(0, Math.min(1, ((point.x - start.x) * dx + (point.y - start.y) * dy) / length2));
    const candidate = { x: start.x + t * dx, y: start.y + t * dy };
    const d = (candidate.x - point.x) ** 2 + (candidate.y - point.y) ** 2;
    if (d < distance) { distance = d; best = candidate; }
  }
  return best;
}

export function worldToMap(point: Point, width: number, height: number): Point {
  return { x: (point.x / width + .5) * 480, y: (.5 - point.y / height) * 360 };
}

export function mapToWorld(point: Point, width: number, height: number): Point {
  return { x: (point.x / 480 - .5) * width, y: (.5 - point.y / 360) * height };
}
