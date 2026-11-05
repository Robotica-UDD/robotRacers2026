import type { CSSProperties } from 'react';

const paths: Record<string, string> = {
  play: 'm8 5 11 7-11 7V5Z',
  pause: 'M8 5v14M16 5v14',
  stop: 'M6 6h12v12H6z',
  reset: 'M3 10a9 9 0 1 1 2.6 8.4M3 4v6h6',
  orbit: 'M12 8a4 4 0 1 1 0 8 4 4 0 0 1 0-8ZM3.8 18.2C1 15.4 2.5 9.6 7.2 5s10.5-6.2 13.3-3.4S21.7 10.2 17 15s-10.5 6.2-13.2 3.2Z',
  top: 'M5 5h14v14H5zM9 9h6v6H9z',
  camera: 'M4 7h4l2-3h4l2 3h4v13H4V7Zm8 3a3.5 3.5 0 1 0 0 7 3.5 3.5 0 0 0 0-7Z',
  expand: 'M8 3H3v5M16 3h5v5M21 16v5h-5M8 21H3v-5',
  tune: 'M4 6h7m4 0h5M4 12h12m4 0h0M4 18h3m4 0h9M11 3v6M16 9v6M7 15v6',
  chevron: 'm9 5 7 7-7 7',
  arrow: 'M4 12h15m-6-6 6 6-6 6',
  target: 'M12 3a9 9 0 1 0 0 18 9 9 0 0 0 0-18Zm0 5a4 4 0 1 0 0 8 4 4 0 0 0 0-8Zm0 3v2',
  route: 'M6 4a2 2 0 1 0 0 4 2 2 0 0 0 0-4Zm12 12a2 2 0 1 0 0 4 2 2 0 0 0 0-4ZM6 8v3a2 2 0 0 0 2 2h8a2 2 0 0 1 2 2v1',
  link: 'm10 13 4-4m-7 6-2 2a4 4 0 0 0 6 6l4-4m2-10 2-2a4 4 0 0 0-6-6L9 5',
  close: 'm6 6 12 12M6 18 18 6',
  check: 'm5 12 4 4L19 6',
  signal: 'M4 18v2M9 13v7M14 8v12M19 3v17',
  eye: 'M2 12s4-7 10-7 10 7 10 7-4 7-10 7S2 12 2 12Zm10-3a3 3 0 1 0 0 6 3 3 0 0 0 0-6Z',
  clock: 'M12 3a9 9 0 1 0 0 18 9 9 0 0 0 0-18Zm0 4v6l4 2',
  info: 'M12 3a9 9 0 1 0 0 18 9 9 0 0 0 0-18ZM12 10v7m0-11v1',
  battery: 'M3 6h16v12H3zM21 10v4M6 9v6m3-6v6m3-6v6',
};

export function Icon({ name, size = 18, style }: { name: string; size?: number; style?: CSSProperties }) {
  return <svg width={size} height={size} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true" style={style}><path d={paths[name] ?? paths.info} /></svg>;
}

export function BrandMark() {
  return <svg viewBox="0 0 40 40" fill="none" aria-hidden="true"><rect width="40" height="40" rx="12" fill="currentColor" /><path d="M11 29V12h8c7 0 8 9 1 10l9 7M16 17h6M25 10v5" stroke="#D8F49A" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round" /></svg>;
}
