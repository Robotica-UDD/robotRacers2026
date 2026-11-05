"""Actualización USB explícita del firmware; importar no abre dispositivos."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import time


ROOT = Path(__file__).resolve().parents[2]
SKETCH = ROOT / "firmware/RobotRacers_Prueba_WiFi"
BUILD = ROOT / "simulation/.runtime/firmware-wifi-encoder-build"
FQBN = "esp32:esp32:esp32s3:CDCOnBoot=cdc"


class UploadError(RuntimeError):
    pass


def find_arduino_cli() -> Path:
    found = shutil.which("arduino-cli")
    if found:
        return Path(found)
    for candidate in (
        Path("/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"),
        Path.home() / "Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli",
    ):
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate
    raise UploadError("No se encontró Arduino CLI. Instala Arduino IDE antes de actualizar.")


def usb_ports():
    try:
        from serial.tools import list_ports
    except ImportError:
        raise UploadError("Falta pyserial en el entorno del proyecto. Instala las dependencias de hardware.") from None
    return list(list_ports.comports())


def select_robot_port(ports) -> str:
    matching = [port for port in ports if getattr(port, "vid", None) == 0x303A
                and getattr(port, "pid", None) == 0x1001]
    if not matching:
        raise UploadError("No se detectó la ESP32-S3 por USB. Conéctala con un cable de datos y vuelve a abrir el actualizador.")
    if len(matching) != 1:
        raise UploadError("Hay varias ESP32-S3 conectadas. Deja conectada solamente la del robot y vuelve a intentarlo.")
    path = getattr(matching[0], "device", None)
    if not isinstance(path, str) or not path:
        raise UploadError("El puerto USB detectado no es válido.")
    return path


def verify_stopped(port: str, *, serial_factory=None, timeout_s=3., clock=time.monotonic):
    """Consulta estado sin mostrar respuestas USB, identificadores ni credenciales."""
    if serial_factory is None:
        try:
            import serial
        except ImportError:
            raise UploadError("Falta pyserial en el entorno del proyecto.") from None
        serial_factory = serial.Serial
    connection = None
    try:
        connection = serial_factory(port, 115200, timeout=.15, write_timeout=.5)
        connection.reset_input_buffer()
        started = clock()
        deadline = started + timeout_s
        next_query = 0.
        pending = bytearray()
        while clock() < deadline:
            now = clock()
            if now >= next_query:
                connection.write(b'{"command":"status"}\n')
                next_query = now + .4
            pending.extend(connection.readline(8193))
            if len(pending) > 8192:
                pending.clear()
                continue
            while b"\n" in pending:
                raw, _, rest = pending.partition(b"\n")
                pending = bytearray(rest)
                try:
                    status = json.loads(raw)
                except (ValueError, UnicodeError):
                    continue
                if not isinstance(status, dict) or status.get("device") != "RobotRacers" or type(status.get("version")) is not int or status["version"] != 1:
                    continue
                if type(status.get("moving")) is not bool or type(status.get("navigation_active", False)) is not bool:
                    continue
                if status["moving"] or status.get("navigation_active", False):
                    raise UploadError("Detén el recorrido antes de actualizar el robot.")
                # Observar también los informes siguientes, no aceptar el
                # primer estado que pudiera estar terminando de transmitirse.
                if clock() - started >= .3:
                    return {"moving": False, "navigation_active": False}
        raise UploadError("No se pudo confirmar que el robot esté detenido. Cierra el monitor serie y vuelve a intentarlo.")
    except UploadError:
        raise
    except Exception:
        raise UploadError("No se pudo consultar el robot por USB. Cierra el monitor serie y comprueba el cable.") from None
    finally:
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass


def update_robot(*, check_only=False, ports_provider=usb_ports, cli_finder=find_arduino_cli,
                 stopped_checker=verify_stopped, runner=subprocess.run):
    port = select_robot_port(ports_provider())
    cli = cli_finder()
    print(f"ESP32-S3 detectada: {port}", flush=True)
    if check_only:
        print("Inspección completada. No se abrió el puerto ni se compiló o cargó firmware.")
        return
    print("Comprobando que el robot esté detenido…", flush=True)
    stopped_checker(port)
    print("Compilando el firmware de control por encoders…", flush=True)
    BUILD.mkdir(parents=True, exist_ok=True)
    runner([str(cli), "compile", "--fqbn", FQBN, "--build-path", str(BUILD), str(SKETCH)],
           cwd=ROOT, check=True)
    # La compilación puede tardar: verificar otra vez antes de abrir el cargador.
    current = select_robot_port(ports_provider())
    if current != port:
        raise UploadError("El puerto USB cambió durante la compilación. Vuelve a abrir el actualizador.")
    stopped_checker(port)
    print("Cargando el firmware. Mantén conectado el cable USB…", flush=True)
    runner([str(cli), "upload", "--fqbn", FQBN, "--port", port,
            "--input-dir", str(BUILD), str(SKETCH)], cwd=ROOT, check=True)
    print("Actualización completada. La red Wi-Fi guardada se conserva; el robot no recibe órdenes de movimiento.")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Actualizar por USB el firmware del robot.")
    parser.add_argument("--check", action="store_true", help="Detectar la placa sin abrir USB, compilar ni cargar.")
    args = parser.parse_args(argv)
    try:
        update_robot(check_only=args.check)
        return 0
    except UploadError as exc:
        print(str(exc))
        return 1
    except subprocess.CalledProcessError:
        print("La compilación o la carga falló. Revisa el mensaje de Arduino que aparece arriba.")
        return 1
    except KeyboardInterrupt:
        print("\nActualización cancelada.")
        return 130
    except OSError:
        print("No se pudo ejecutar el actualizador. Comprueba Arduino IDE y los permisos de la carpeta.")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
