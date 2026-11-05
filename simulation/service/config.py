"""Configuración y rutas del proyecto, sin dependencias del simulador."""
from __future__ import annotations
import json
import os
from pathlib import Path

SIMULATION_DIR = Path(__file__).resolve().parents[1]

def load_configuration() -> tuple[dict, dict]:
    return tuple(json.loads((SIMULATION_DIR / "config" / name).read_text())
                 for name in ("robot.json", "arena.json"))

def runtime_directory() -> Path:
    return Path(os.getenv("ROBOTRACERS_RUNTIME_DIR", SIMULATION_DIR / ".runtime")).resolve()
