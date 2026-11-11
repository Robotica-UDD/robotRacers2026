"""Instala el modelo de manos local con procedencia y SHA-256 fijados.

Descarga solo pesos, configuración y licencia desde el repositorio del autor.
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
SOURCE = "https://github.com/cansik/yolo-hand-detection"
COMMIT = "42d038bbc5deed14b11b27a871ca4385b4d880a2"
RELEASE = SOURCE + "/releases/download/pretrained/"
ASSETS = (
    {"name": "cross-hands-yolov4-tiny.cfg", "url": RELEASE + "cross-hands-yolov4-tiny.cfg",
     "sha256": "0effeeb1b14a67ea36fa8f4dcc6c3f9e0988401cbedcc4f543a09fb6697a8dd8"},
    {"name": "cross-hands-yolov4-tiny.weights", "url": RELEASE + "cross-hands-yolov4-tiny.weights",
     "sha256": "bcb27a5772a17d55f6c27b353639a5f5ed0f513ea1763d07ef818e5f786d7b06"},
    {"name": "LICENSE", "url": f"https://raw.githubusercontent.com/cansik/yolo-hand-detection/{COMMIT}/LICENSE",
     "sha256": "0b727f90bb0da0fdde54901c0cab678a0fba03d5b751d8574dfb425315a3198b"},
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
    manifest = {"model": "YOLOv4-tiny Cross-Hands", "author": "Florian Bruggisser (cansik)",
                "source": SOURCE, "source_commit": COMMIT, "assets": list(ASSETS),
                "license": "MIT (repositorio del autor); EgoHands CC BY 4.0.",
                "datasets": "CMU Hand DB, EgoHands e imágenes anotadas por el autor; consultar sus condiciones.",
                "runtime": "OpenCV DNN CPU; archivos locales, sin descarga ni transmisión de imágenes."}
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
