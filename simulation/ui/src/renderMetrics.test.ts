import { describe, expect, it } from 'vitest';
import { readRenderMetrics } from './renderMetrics';

describe('completed Wren renders', () => {
  it('reads the vendor counter without counting animation callbacks', () => {
    expect(readRenderMetrics({ total: 120, times: [800, 900, 980], fps: 42.5 }, true, 1000))
      .toEqual({ available: true, connected: true, frames: 120, fps: 42.5, last_frame_age_ms: 20 });
  });
  it('tolerates partial counters while the renderer is loading', () => {
    for (const counter of [undefined, {}, { total: 1 }, { total: NaN, times: [] }]) {
      expect(readRenderMetrics(counter, true, 1000)).toMatchObject({ available: false, frames: 0, fps: 0 });
    }
  });
  it('reports zero instead of retaining stale or invalid frame rates', () => {
    const counter = { total: 1, times: [100], fps: 60 };
    expect(readRenderMetrics(counter, true, 1000).fps).toBe(0);
    expect(readRenderMetrics(counter, false, 120).fps).toBe(0);
    expect(readRenderMetrics({ ...counter, fps: Infinity }, true, 120).fps).toBe(0);
  });
});
