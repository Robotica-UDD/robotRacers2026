import { describe, expect, it } from 'vitest';
import { clipTrackingSegment, pointInTrackingViewport, trackingBoxAnchor, trackingTether, validTrackingBox } from './trackingGeometry';

describe('robot tracking geometry', () => {
  const box = { x: 40, y: 30, width: 20, height: 20 };
  const robot = { x: 50, y: 40 };
  it('starts the tether at the box edge instead of crossing the robot', () => {
    expect(trackingTether(box, robot, { x: 90, y: 40 }, 100, 80))
      .toEqual({ start: { x: 60, y: 40 }, end: { x: 90, y: 40 } });
    expect(trackingBoxAnchor(box, { x: 50, y: 0 })).toEqual({ x: 50, y: 30 });
    expect(trackingBoxAnchor(box, { x: 80, y: 70 })).toEqual({ x: 60, y: 50 });
  });
  it('omits the tether when its goal is already inside or on the box', () => {
    expect(trackingBoxAnchor(box, robot)).toBeNull();
    expect(trackingBoxAnchor(box, { x: 60, y: 40 })).toBeNull();
  });
  it('follows the actual projected robot position in an asymmetric box', () => {
    const asymmetricBox = { x: 40, y: 20, width: 40, height: 40 };
    const projectedRobot = { x: 50, y: 25 }, goal = { x: 110, y: 55 };
    expect(trackingBoxAnchor(asymmetricBox, goal, projectedRobot)).toEqual({ x: 80, y: 40 });
    expect(trackingTether(asymmetricBox, projectedRobot, goal, 120, 100))
      .toEqual({ start: { x: 80, y: 40 }, end: goal });
  });
  it('uses the direct exit intersection for an exterior origin and rejects a missed box', () => {
    const projectedBox = { x: 40, y: 20, width: 40, height: 40 };
    expect(trackingBoxAnchor(projectedBox, { x: 110, y: 60 }, { x: 10, y: 10 }))
      .toEqual({ x: 80, y: 45 });
    expect(trackingBoxAnchor(projectedBox, { x: 110, y: 80 }, { x: 10, y: 70 })).toBeNull();
    expect(trackingBoxAnchor(projectedBox, { x: 110, y: 60 }, { x: NaN, y: 10 })).toBeNull();
  });
  it('clips offscreen goals without moving the true goal to the viewport border', () => {
    const goal = { x: 150, y: 40 };
    expect(trackingTether(box, robot, goal, 100, 80))
      .toEqual({ start: { x: 60, y: 40 }, end: { x: 100, y: 40 } });
    expect(pointInTrackingViewport(goal, 100, 80)).toBe(false);
    expect(goal).toEqual({ x: 150, y: 40 });
  });
  it('clips both endpoints when the robot and goal are offscreen', () => {
    expect(clipTrackingSegment({ x: -10, y: 10 }, { x: 110, y: 70 }, 100, 80))
      .toEqual({ start: { x: 0, y: 15 }, end: { x: 100, y: 65 } });
    expect(clipTrackingSegment({ x: -20, y: -10 }, { x: -10, y: 90 }, 100, 80)).toBeNull();
  });
  it('uses the supplied robot point when no projected box exists', () => {
    expect(trackingTether(null, robot, { x: 80, y: 40 }, 100, 80))
      .toEqual({ start: robot, end: { x: 80, y: 40 } });
  });
  it('rejects invalid dimensions, non-finite positions and zero-length segments', () => {
    expect(validTrackingBox({ ...box, width: -1 })).toBe(false);
    expect(clipTrackingSegment(robot, robot, 100, 80)).toBeNull();
    expect(clipTrackingSegment(robot, { x: NaN, y: 10 }, 100, 80)).toBeNull();
    expect(clipTrackingSegment(robot, { x: 80, y: 40 }, 0, 80)).toBeNull();
    expect(trackingTether(box, null, { x: 80, y: 40 }, 100, 80)).toBeNull();
    expect(pointInTrackingViewport({ x: Infinity, y: 1 }, 100, 80)).toBe(false);
  });
});
