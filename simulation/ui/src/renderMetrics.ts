type RenderCounter = { total?: unknown; times?: unknown; fps?: unknown };

/** Only completed Wren renders count; a missing or partial counter is no sample. */
export function readRenderMetrics(value: unknown, connected: boolean, now: number) {
  const counter = value && typeof value === 'object' ? value as RenderCounter : undefined;
  const available = Boolean(counter && Array.isArray(counter.times)
    && typeof counter.total === 'number' && Number.isFinite(counter.total) && counter.total >= 0);
  const times = available ? counter!.times as unknown[] : [];
  const last = times.at(-1);
  const age = typeof last === 'number' && Number.isFinite(last) ? now - last : null;
  const rawFps = counter?.fps;
  return {
    available, connected, frames: available ? Math.floor(counter!.total as number) : 0,
    fps: connected && age !== null && age >= 0 && age < 500 && typeof rawFps === 'number'
      && Number.isFinite(rawFps) && rawFps >= 0 && rawFps <= 240 ? rawFps : 0,
    last_frame_age_ms: age,
  };
}
