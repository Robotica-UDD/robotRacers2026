export type TrackingPoint = { x: number; y: number };
export type TrackingBox = TrackingPoint & { width: number; height: number };
export type TrackingSegment = { start: TrackingPoint; end: TrackingPoint };

export function validTrackingPoint(value: TrackingPoint | null | undefined): value is TrackingPoint {
  return Boolean(value && Number.isFinite(value.x) && Number.isFinite(value.y));
}

export function validTrackingBox(value: TrackingBox | null | undefined): value is TrackingBox {
  return validTrackingPoint(value) && Number.isFinite(value.width) && value.width > 0
    && Number.isFinite(value.height) && value.height > 0
    && Number.isFinite(value.x + value.width) && Number.isFinite(value.y + value.height);
}

export function validTrackingViewport(width: number, height: number) {
  return Number.isFinite(width) && width > 0 && Number.isFinite(height) && height > 0;
}

export function pointInTrackingViewport(point: TrackingPoint | null | undefined, width: number, height: number) {
  return validTrackingViewport(width, height) && validTrackingPoint(point)
    && point.x >= 0 && point.x <= width && point.y >= 0 && point.y <= height;
}

/** Leave the box along the actual robot-to-goal segment; use its centre only without an origin. */
export function trackingBoxAnchor(box: TrackingBox, target: TrackingPoint, origin?: TrackingPoint): TrackingPoint | null {
  if (!validTrackingBox(box) || !validTrackingPoint(target)) return null;
  if (target.x >= box.x && target.x <= box.x + box.width
    && target.y >= box.y && target.y <= box.y + box.height) return null;
  const start = origin ?? { x: box.x + box.width / 2, y: box.y + box.height / 2 };
  if (!validTrackingPoint(start)) return null;
  const dx = target.x - start.x, dy = target.y - start.y;
  if (!Number.isFinite(dx) || !Number.isFinite(dy) || (dx === 0 && dy === 0)) return null;
  let enter = 0, exit = 1;
  for (const [position, delta, lower, upper] of [
    [start.x, dx, box.x, box.x + box.width],
    [start.y, dy, box.y, box.y + box.height],
  ]) {
    if (delta === 0) { if (position < lower || position > upper) return null; continue; }
    const first = (lower - position) / delta, last = (upper - position) / delta;
    enter = Math.max(enter, Math.min(first, last));
    exit = Math.min(exit, Math.max(first, last));
    if (enter > exit) return null;
  }
  // With an exterior origin, the last intersection keeps the visible tether
  // outside the box. A segment that misses the box has no invented anchor.
  const anchor = { x: start.x + exit * dx, y: start.y + exit * dy };
  return validTrackingPoint(anchor) ? anchor : null;
}

/** Liang–Barsky clipping preserves the true goal separately from the visible line. */
export function clipTrackingSegment(start: TrackingPoint, end: TrackingPoint, width: number, height: number): TrackingSegment | null {
  if (!validTrackingViewport(width, height) || !validTrackingPoint(start) || !validTrackingPoint(end)) return null;
  const dx = end.x - start.x, dy = end.y - start.y;
  if (!Number.isFinite(dx) || !Number.isFinite(dy) || (dx === 0 && dy === 0)) return null;
  let first = 0, last = 1;
  for (const [p, q] of [[-dx, start.x], [dx, width - start.x], [-dy, start.y], [dy, height - start.y]]) {
    if (p === 0) { if (q < 0) return null; continue; }
    const fraction = q / p;
    if (p < 0) first = Math.max(first, fraction);
    else last = Math.min(last, fraction);
    if (first >= last) return null;
  }
  const result = {
    start: { x: start.x + first * dx, y: start.y + first * dy },
    end: { x: start.x + last * dx, y: start.y + last * dy },
  };
  return validTrackingPoint(result.start) && validTrackingPoint(result.end) ? result : null;
}

export function trackingTether(box: TrackingBox | null, robot: TrackingPoint | null, target: TrackingPoint | null,
  width: number, height: number): TrackingSegment | null {
  if (!validTrackingPoint(robot) || !validTrackingPoint(target)) return null;
  const start = validTrackingBox(box) ? trackingBoxAnchor(box, target, robot) : robot;
  return start ? clipTrackingSegment(start, target, width, height) : null;
}
