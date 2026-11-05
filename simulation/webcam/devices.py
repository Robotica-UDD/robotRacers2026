"""Nombres de cámara en el mismo orden que OpenCV AVFoundation, sin capturar.

OpenCV 4.14 concatena vídeo y multiplexado, y ordena por NSString.compare(uniqueID):
https://github.com/opencv/opencv/blob/4.14.0/modules/videoio/src/cap_avfoundation_mac.mm

La consulta de AVFoundation solo enumera dispositivos; no crea entradas de
captura, sesiones ni solicita permisos. ctypes evita una dependencia de PyObjC
o de herramientas de compilación. Cada llamada de Objective-C tiene una firma
explícita: punteros de objetos, NSInteger/NSUInteger o el entero transportType;
no se intercambian estructuras ni bloques. Un autorelease pool acota sus objetos.
"""
from __future__ import annotations

import copy
import ctypes
from ctypes.util import find_library
from functools import cmp_to_key
import json
import logging
import subprocess
import sys
import threading
import time


LOGGER = logging.getLogger(__name__)
PLATFORM = sys.platform
CACHE_SECONDS = 3.0
NATIVE_OUTPUT_LIMIT = 64 * 1024
_cache_lock = threading.Lock()
_cache_at: float | None = None
_cache_value: dict | None = None


def _macos_devices() -> list[dict]:
    """Enumera objetos nativos; el índice coincide con CAP_AVFOUNDATION."""
    objc_path = find_library("objc")
    if not objc_path:
        raise RuntimeError("No está disponible el runtime Objective-C.")
    objc = ctypes.CDLL(objc_path)
    avfoundation = ctypes.CDLL(
        "/System/Library/Frameworks/AVFoundation.framework/AVFoundation"
    )
    pointer = ctypes.c_void_p
    objc.objc_getClass.argtypes = [ctypes.c_char_p]
    objc.objc_getClass.restype = pointer
    objc.sel_registerName.argtypes = [ctypes.c_char_p]
    objc.sel_registerName.restype = pointer

    # No se modifica argtypes de objc_msgSend entre llamadas: cada firma es una
    # función independiente, también si distintos hilos consultan el módulo.
    def signature(result, *arguments):
        return ctypes.CFUNCTYPE(result, pointer, pointer, *arguments)(
            ("objc_msgSend", objc)
        )

    send_object = signature(pointer)
    send_object_arg = signature(pointer, pointer)
    send_count = signature(ctypes.c_ulong)
    send_index = signature(pointer, ctypes.c_ulong)
    send_text = signature(ctypes.c_char_p)
    send_compare = signature(ctypes.c_long, pointer)
    send_transport = signature(ctypes.c_int32)
    send_void = signature(None)

    def selector(name: str):
        return objc.sel_registerName(name.encode("ascii"))

    def text(obj) -> str:
        encoded = send_text(obj, selector("UTF8String"))
        return encoded.decode("utf-8") if encoded else ""

    pool_class = objc.objc_getClass(b"NSAutoreleasePool")
    capture_class = objc.objc_getClass(b"AVCaptureDevice")
    if not pool_class or not capture_class:
        raise RuntimeError("AVFoundation no expone los dispositivos de cámara.")
    pool = send_object(send_object(pool_class, selector("alloc")), selector("init"))
    try:
        video_type = pointer.in_dll(avfoundation, "AVMediaTypeVideo").value
        muxed_type = pointer.in_dll(avfoundation, "AVMediaTypeMuxed").value
        video = send_object_arg(capture_class, selector("devicesWithMediaType:"), video_type)
        muxed = send_object_arg(capture_class, selector("devicesWithMediaType:"), muxed_type)
        combined = send_object_arg(video, selector("arrayByAddingObjectsFromArray:"), muxed)
        devices = [
            send_index(combined, selector("objectAtIndex:"), index)
            for index in range(send_count(combined, selector("count")))
        ]

        def unique_id(device):
            return send_object(device, selector("uniqueID"))

        # Es la misma comparación nativa que OpenCV; no se supone que el orden
        # de system_profiler ni el orden de descubrimiento sean índices válidos.
        devices.sort(key=cmp_to_key(lambda left, right: send_compare(
            unique_id(left), selector("compare:"), unique_id(right)
        )))
        result = []
        for index, device in enumerate(devices):
            identifier = text(unique_id(device))
            name = text(send_object(device, selector("localizedName")))
            if not identifier or not name:
                raise RuntimeError("La cámara no proporciona nombre e identificador.")
            transport_number = send_transport(device, selector("transportType")) & 0xFFFFFFFF
            transport = transport_number.to_bytes(4, "big").decode("ascii", errors="replace").strip()
            label = name + (" (USB)" if transport == "usb" else "")
            result.append({"index": index, "label": label, "id": identifier,
                           "transport": transport})
        return result
    finally:
        send_void(pool, selector("drain"))


def _manual_options() -> dict:
    return {
        "enumerated": False,
        "options": [{"index": index, "label": f"Cámara {index}", "id": f"index:{index}"}
                    for index in range(10)],
        "default_index": 0,
        "message": "No se pudieron consultar los nombres. Selecciona el índice de cámara manualmente.",
    }


def _parse_native_devices(output: str) -> list[dict]:
    """Valida la respuesta del proceso auxiliar antes de usar sus índices."""
    if not isinstance(output, str) or len(output.encode("utf-8")) > NATIVE_OUTPUT_LIMIT:
        raise ValueError("La respuesta de cámaras supera el tamaño permitido.")
    entries = json.loads(output)
    if not isinstance(entries, list) or len(entries) > 64:
        raise ValueError("La respuesta de cámaras no es una lista válida.")
    result = []
    for expected_index, entry in enumerate(entries):
        if not isinstance(entry, dict):
            raise ValueError("La cámara no tiene una descripción válida.")
        index = entry.get("index")
        if type(index) is not int or index != expected_index:
            raise ValueError("Los índices de cámara no son consecutivos.")
        for key, maximum in (("id", 512), ("label", 256), ("transport", 32)):
            value = entry.get(key)
            if not isinstance(value, str) or len(value) > maximum or "\x00" in value:
                raise ValueError("La cámara no tiene una descripción válida.")
            if key != "transport" and not value:
                raise ValueError("Falta el nombre o identificador de cámara.")
        result.append({key: entry[key] for key in ("index", "label", "id", "transport")})
    return result


def _fresh_macos_devices() -> list[dict]:
    """Consulta AVFoundation en un proceso nuevo para evitar dispositivos antiguos.

    El servicio permanece abierto durante conexiones y cambios de puerto USB.
    Limpiar su caché Python no actualiza necesariamente la lista nativa de
    AVFoundation que ya cargó ese proceso. El auxiliar termina tras enumerar;
    no solicita permisos ni abre una sesión de captura.
    """
    completed = subprocess.run(
        [sys.executable, "-m", "simulation.webcam.devices", "--list-native"],
        capture_output=True, text=True, encoding="utf-8", timeout=8,
        check=True, shell=False,
    )
    return _parse_native_devices(completed.stdout)


def discover_cameras(force: bool = False) -> dict:
    """Devuelve nombres e índices actuales, con caché breve y sin abrir cámaras.

    force=True consulta en un proceso nuevo para actualizar conexiones USB. En plataformas
    sin este backend se mantienen índices manuales claramente identificados.
    Se devuelve una copia para que un consumidor no cambie la caché compartida.
    """
    global _cache_at, _cache_value
    with _cache_lock:
        now = time.monotonic()
        if not force and _cache_at is not None and now - _cache_at < CACHE_SECONDS:
            return copy.deepcopy(_cache_value)
        if PLATFORM == "darwin":
            try:
                options = _fresh_macos_devices()
                default = next((item["index"] for item in options if item.get("transport") == "usb"),
                               options[0]["index"] if options else 0)
                value = {"enumerated": True, "options": options, "default_index": default,
                         "message": "" if options else "No se detectaron cámaras conectadas."}
            except Exception as exc:
                # Enumerar es opcional: un fallo no debe impedir la captura por índice.
                LOGGER.warning("No se pudieron enumerar las cámaras: %s", type(exc).__name__)
                value = _manual_options()
        else:
            value = _manual_options()
        _cache_at = now
        _cache_value = value
        return copy.deepcopy(value)


def main() -> None:
    """Auxiliar interno: una consulta nativa, JSON por stdout y salida inmediata."""
    import argparse

    parser = argparse.ArgumentParser(description="Consulta nativa de cámaras, sin captura.")
    parser.add_argument("--list-native", action="store_true")
    args = parser.parse_args()
    if not args.list_native:
        parser.error("Utiliza --list-native para consultar las cámaras.")
    if PLATFORM != "darwin":
        parser.error("La consulta nativa requiere macOS.")
    print(json.dumps(_macos_devices(), ensure_ascii=False))


if __name__ == "__main__":
    main()
