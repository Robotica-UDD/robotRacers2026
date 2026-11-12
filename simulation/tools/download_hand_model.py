"""Instala el modelo de manos local con procedencia y SHA-256 fijados.

Descarga el paquete oficial MediaPipe y su licencia; no instala ejecutables.
La cámara nunca ejecuta este instalador ni hace descargas durante la inferencia.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import tempfile
import urllib.request


MODEL_DIR = Path(__file__).resolve().parents[1] / ".runtime" / "hand-models"
SOURCE = "https://github.com/google-ai-edge/mediapipe"
COMMIT = "1b9800ed7baddd397b888eaaaf0c841fe0358241"
ASSETS = (
    {"name": "hand_landmarker.task",
     "url": "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task",
     "sha256": "fbc2a30080c3c557093b5ddfc334698132eb341044ccee322ccf8bcf3607cde1"},
    {"name": "MediaPipe-LICENSE", "url": f"https://raw.githubusercontent.com/google-ai-edge/mediapipe/{COMMIT}/LICENSE",
     "sha256": "8707eef0533987efc5b155d64761eeb6e20793f50b9bd1a68dad1cf4719d0ed8"},
)


def file_hash(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for chunk in iter(lambda: source.read(512 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def install(model_dir=None, *, opener=urllib.request.urlopen, check_only=False):
    directory = MODEL_DIR if model_dir is None else Path(model_dir)
    if not check_only:
        directory.mkdir(parents=True, exist_ok=True)
    for item in ASSETS:
        destination = directory / item["name"]
        if destination.is_file() and file_hash(destination) == item["sha256"]:
            continue
        if check_only:
            raise ValueError(f"Falta o no coincide la huella SHA-256 de {item['name']}.")
        temporary = None
        try:
            request = urllib.request.Request(item["url"], headers={"User-Agent": "RobotRacers-model-setup"})
            with opener(request, timeout=45) as response, tempfile.NamedTemporaryFile(
                    prefix="modelo-manos-", suffix=".download", dir=directory, delete=False) as output:
                temporary = Path(output.name)
                size = 0
                while chunk := response.read(512 * 1024):
                    size += len(chunk)
                    if size > 64 * 1024 * 1024:
                        raise ValueError("El archivo del modelo excede el tamaño esperado.")
                    output.write(chunk)
            if file_hash(temporary) != item["sha256"]:
                raise ValueError(f"La huella SHA-256 no coincide para {item['name']}; no se instaló.")
            temporary.replace(destination)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
    manifest = {"model": "MediaPipe Hand Landmarker", "author": "Google MediaPipe",
                "source": SOURCE, "source_commit": COMMIT, "assets": list(ASSETS),
                "model_version": "Hand Landmarker full float16 version1", "package": "mediapipe==0.10.31",
                "license": "Apache-2.0; consultar la licencia y ficha del modelo oficial.",
                "model_card": "https://storage.googleapis.com/mediapipe-assets/Model%20Card%20Hand%20Tracking%20%28Lite_Full%29%20with%20Fairness%20Oct%202021.pdf",
                "runtime": "MediaPipe Tasks VIDEO CPU; imágenes procesadas localmente, sin transmisión."}
    if not check_only:
        (directory / "provenance.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description="Instalar o verificar el modelo de manos de RobotRacers.")
    parser.add_argument("--model-dir", type=Path, default=MODEL_DIR)
    parser.add_argument("--check", action="store_true", help="Verificar los archivos existentes sin conexión.")
    args = parser.parse_args(argv)
    try:
        manifest = install(args.model_dir, check_only=args.check)
    except (OSError, ValueError) as exc:
        parser.exit(1, f"No se pudo preparar el modelo: {exc}\n")
    print(f"{manifest['model']}: archivos verificados en {args.model_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
