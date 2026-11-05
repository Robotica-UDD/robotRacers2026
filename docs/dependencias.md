# Dependencias reproducibles y uso sin conexión

La aplicación se instala y ejecuta localmente. `simulation/setup.sh` necesita
Internet para la primera instalación; `simulation/run.command` utiliza los
archivos ya instalados y no ejecuta gestores de paquetes ni descargas.

## Versiones

- Webots **R2025a**, instalador universal oficial de Cyberbotics.
- CPython **3.12**, entorno aislado `.venv` administrado con `uv`.
- Dependencias Python fijadas en **uv.lock**; instalación con `uv sync --frozen`.
- Node.js **24 LTS** para construir la interfaz. Versiones exactas en
  **simulation/ui/package-lock.json** y construcción con `npm ci`.
- Compilador C++17 de las herramientas de Apple para el controlador local.

La ejecución normal necesita Webots, el entorno Python y `simulation/ui/dist`.
Node y npm solo intervienen al instalar o reconstruir la interfaz. Las rutas de
la instalación se resuelven mediante `WEBOTS_HOME`; el valor predeterminado en
macOS es `/Applications/Webots.app/Contents`.

## Visor local

`simulation/tools/vendor_webots_web.py` descarga el visor oficial R2025a y sus
recursos transitivos en `simulation/.runtime/wwi`. El registro local
`wwi-manifest.json` contiene URL, tamaño y SHA-256. Las imágenes de respaldo y
logo que Webots referencia sin versión tienen sus hashes fijados en el script.
El lanzador conserva esta caché al abrir una nueva sesión.

Se aplican parches limitados y explícitos al visor: rutas locales, texturas del
proyecto, observación sin pausar la física, reconexión sin modal, ocultación de
ventanas técnicas y contador de fotogramas dibujados. El código de control no
se ejecuta dentro de ese visor.

La política de recursos del servicio permite el propio origen y el servidor
Webots local. No permite cargar imágenes, fuentes ni scripts de Internet. Los
archivos de este mundo usan materiales y geometría locales; las funciones
editoriales genéricas de Webots para descargar otros PROTO no forman parte de
la aplicación.

## Licencias

| Recurso | Licencia / fuente |
|---|---|
| Código de RobotRacers | MIT, `LICENSE` |
| Webots y visor W3D | Apache 2.0; [repositorio oficial](https://github.com/cyberbotics/webots/blob/R2025a/LICENSE) |
| Python | PSF; [licencia](https://docs.python.org/3/license.html) |
| FastAPI | MIT; [fuente](https://github.com/fastapi/fastapi) |
| NumPy | BSD; [fuente](https://github.com/numpy/numpy) |
| OpenCV | Apache 2.0; [fuente](https://github.com/opencv/opencv) |
| React, Vite y TypeScript | MIT; licencias en los paquetes instalados |
| Modelos y referencias | Procedencia y condiciones en `assets/manifest.json` y `docs/activos_robot.md` |

Los binarios de terceros y `node_modules` no se suben al repositorio. La caché
del visor conserva los avisos de los archivos originales. Las marcas y logos
siguen perteneciendo a sus titulares; la licencia MIT del proyecto no cambia
las condiciones de esos recursos.
