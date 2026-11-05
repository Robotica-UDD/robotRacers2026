"""Lanzador local: python -m simulation.webcam."""
from __future__ import annotations

import argparse
import threading
import webbrowser

import uvicorn


def main():
    parser = argparse.ArgumentParser(description="Prueba del robot con webcam y AprilTag, sin motores ni ROS 2.")
    parser.add_argument("--port", type=int, default=8010, help="Puerto local (predeterminado: 8010).")
    parser.add_argument("--no-browser", action="store_true", help="No abrir el navegador automáticamente.")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("El puerto debe estar entre 1 y 65535.")
    url = f"http://127.0.0.1:{args.port}"
    print(f"RobotRacers · Cámara: {url}\nLa cámara permanece apagada hasta pulsar Iniciar. Ctrl+C para cerrar.")
    opener = None
    if not args.no_browser:
        opener = threading.Timer(1.0, lambda: webbrowser.open(url))
        opener.daemon = True
        opener.start()
    try:
        uvicorn.run("simulation.webcam.app:app", host="127.0.0.1", port=args.port, log_level="info",
                    access_log=False, timeout_graceful_shutdown=2)
    finally:
        if opener is not None:
            opener.cancel()


if __name__ == "__main__":
    main()
