import { useCallback, useEffect, useRef, useState } from 'react';
import { disconnectedState, isSimulationState } from './types';
import type { AppConfig, Goal, Settings, SimulationState } from './types';

async function requestJson(path: string, method = 'GET', data?: unknown) {
  const response = await fetch(path, {
    method, headers: data === undefined ? undefined : { 'Content-Type': 'application/json' },
    body: data === undefined ? undefined : JSON.stringify(data),
    cache: 'no-store', signal: AbortSignal.timeout(8000),
  });
  if (!response.ok) {
    const body = await response.json().catch(() => ({}));
    throw new Error(typeof body.detail === 'string' ? body.detail : 'No se pudo aplicar el cambio. Inténtalo de nuevo.');
  }
  return response.json();
}

export function useSimulation() {
  const [state, setState] = useState<SimulationState>(disconnectedState);
  const [config, setConfig] = useState<AppConfig | null>(null);
  const [online, setOnline] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [busy, setBusy] = useState<string | null>(null);
  const lastMessage = useRef(0);

  useEffect(() => {
    let closed = false, socket: WebSocket | null = null;
    let retry: ReturnType<typeof setTimeout> | undefined;
    let backoff = 600;
    const apply = (data: unknown) => {
      if (!closed && isSimulationState(data)) { setState(data); lastMessage.current = Date.now(); setOnline(true); }
    };
    const loadConfig = async () => {
      try {
        const nextConfig = await requestJson('/api/config');
        if (!closed) setConfig(nextConfig);
        apply(await requestJson('/api/state'));
      } catch { /* The WebSocket reconnect loop also retries the configuration. */ }
    };
    const connect = () => {
      if (closed) return;
      void loadConfig();
      socket = new WebSocket(`${window.location.protocol === 'https:' ? 'wss' : 'ws'}://${window.location.host}/api/ws`);
      socket.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          apply(data.type === 'state' && data.state ? data.state : data);
          backoff = 600;
        } catch { /* Ignore a malformed packet; it must not invent or advance state. */ }
      };
      socket.onclose = () => {
        if (closed) return;
        setOnline(false);
        retry = setTimeout(connect, backoff);
        backoff = Math.min(backoff * 1.5, 5000);
      };
      socket.onerror = () => socket?.close();
    };
    connect();
    const watchdog = setInterval(() => {
      if (Date.now() - lastMessage.current > 3500) setOnline(false);
    }, 1000);
    return () => { closed = true; clearTimeout(retry); clearInterval(watchdog); socket?.close(); };
  }, []);

  const applyResponse = useCallback((value: unknown) => {
    if (isSimulationState(value)) setState(value);
  }, []);

  const mission = useCallback(async (action: 'start' | 'pause' | 'stop' | 'reset') => {
    setBusy(action); setError(null);
    try { applyResponse(await requestJson(`/api/mission/${action}`, 'POST', {})); }
    catch (e) { setError(e instanceof Error ? e.message : 'No se pudo controlar el recorrido.'); }
    finally { setBusy(null); }
  }, [applyResponse]);

  const updateSettings = useCallback(async (patch: Partial<Settings>) => {
    try { const result = await requestJson('/api/settings', 'POST', patch); applyResponse(result); return true; }
    catch (e) { setError(e instanceof Error ? e.message : 'No se pudo guardar el ajuste.'); return false; }
  }, [applyResponse]);

  const updateGoals = useCallback(async (goals: Goal[]) => {
    setBusy('goals'); setError(null);
    try { applyResponse(await requestJson('/api/goals', 'PUT', { goals })); return true; }
    catch (e) { setError(e instanceof Error ? e.message : 'No se pudieron guardar los destinos.'); return false; }
    finally { setBusy(null); }
  }, [applyResponse]);

  return { state, config, online, connected: online && state.connected, busy, error, dismissError: () => setError(null), mission, updateSettings, updateGoals };
}

/** One human-preview request at a time, maximum 10 Hz, never used for control. */
export function useCameraPreview(enabled: boolean) {
  const [image, setImage] = useState<string | null>(null);
  const [available, setAvailable] = useState(false);
  useEffect(() => {
    if (!enabled) { setAvailable(false); return; }
    let closed = false, url: string | null = null;
    let timer: ReturnType<typeof setTimeout> | undefined;
    let active: AbortController | undefined;
    const load = async () => {
      const started = performance.now();
      active = new AbortController();
      const abort = setTimeout(() => active?.abort(), 2500);
      try {
        const response = await fetch(`/api/camera.jpg?t=${Date.now()}`, { cache: 'no-store', signal: active.signal });
        if (!response.ok) throw new Error('Camera not ready');
        const blob = await response.blob();
        if (!blob.type.startsWith('image/')) throw new Error('Invalid camera image');
        if (!closed) {
          const next = URL.createObjectURL(blob);
          setImage(next); setAvailable(true);
          if (url) URL.revokeObjectURL(url);
          url = next;
        }
      } catch { if (!closed) setAvailable(false); }
      finally { clearTimeout(abort); }
      if (!closed) timer = setTimeout(load, Math.max(0, 100 - (performance.now() - started)));
    };
    void load();
    return () => { closed = true; clearTimeout(timer); active?.abort(); if (url) URL.revokeObjectURL(url); };
  }, [enabled]);
  return { image, available };
}
