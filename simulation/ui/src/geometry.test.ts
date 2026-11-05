import { describe, expect, it } from 'vitest';
import { arenaGeometry, mapToWorld, snapToCorridors, worldToMap } from './geometry';
import { disconnectedState, formatNumber, isSimulationState } from './types';

describe('route editing and live state', () => {
  it('projects targets only onto the five H-corridor segments', () => {
    const { nodes, edges } = arenaGeometry(undefined, 1);
    expect(snapToCorridors({ x: -.9, y: .4 }, nodes, edges)).toEqual({ x: -.6, y: .4 });
    const center = snapToCorridors({ x: .2, y: .15 }, nodes, edges);
    expect(center.x).toBeCloseTo(.2); expect(center.y).toBe(0);
    expect(snapToCorridors({ x: 5, y: 4 }, nodes, edges)).toEqual({ x: .6, y: .55 });
  });
  it('keeps dragging correct when the arena changes scale', () => {
    const { nodes, edges, width, height } = arenaGeometry(undefined, 1.5);
    const point = { x: .9, y: .4 };
    const mapped = mapToWorld(worldToMap(point, width, height), width, height);
    expect(mapped.x).toBeCloseTo(point.x); expect(mapped.y).toBeCloseTo(point.y);
    expect(snapToCorridors({ x: 1.4, y: 1.7 }, nodes, edges).y).toBeCloseTo(.825);
  });
  it('does not scale already-scaled API node coordinates twice after reconnecting', () => {
    const config = { scale: 1.5, nodes: { L: [-.9, 0], R: [.9, 0] }, edges: [['L', 'R']] };
    const geometry = arenaGeometry(config, 1.5);
    expect(geometry.nodes.L.x).toBeCloseTo(-.9);
    expect(geometry.width).toBeCloseTo(3.6);
  });
  it('does not expose a fabricated robot pose or live telemetry on disconnection', () => {
    expect(disconnectedState.connected).toBe(false);
    expect(Number.isNaN(disconnectedState.pose.x)).toBe(true);
    expect(Number.isNaN(disconnectedState.robot.speed)).toBe(true);
    expect(formatNumber(disconnectedState.robot.speed)).toBe('—');
  });
  it('rejects unrelated socket messages instead of treating them as simulation state', () => {
    expect(isSimulationState({ type: 'hello' })).toBe(false);
    expect(isSimulationState(null)).toBe(false);
    expect(isSimulationState({ ...disconnectedState, status: 'idle' })).toBe(true);
  });
});
