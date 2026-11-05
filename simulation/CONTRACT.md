# Contrato de integración

Proyecto: Webots R2025a + servicio Python + interfaz web. Unidades internas: metros, radianes, segundos. Base del robot: X hacia delante, Y izquierda, Z arriba. Mundo: Z arriba. Cuentas de encoder normalizadas: positivas al avanzar ambas ruedas; 59.94 cuentas por vuelta.

## Procesos y puertos

- Servicio FastAPI: localhost:8000; UDP recepción telemetría: 9201; envío consignas a driver: 9202. Configurables por entorno.
- Webots: visor W3D localhost:1234. Física 2 ms; driver C++ actualiza control a 10 ms; cámara cada 20 ms.
- Runtime compartido: variable ROBOTRACERS_RUNTIME_DIR; predeterminado simulation/.runtime. Nunca versionar.
- Driver → servicio, UDP JSON: {"type":"telemetry","seq":1,"sim_time":1.0,"stamp":monotonic_seconds,"left_count":0,"right_count":0,"left_velocity":0.0,"right_velocity":0.0,"left_pwm":0.0,"right_pwm":0.0,"battery_voltage":7.4,"reflectance":[0,0,0]}.
- Servicio → driver, UDP JSON: {"seq":1,"stamp":monotonic_seconds,"v":0.0,"w":0.0,"mode":"run"}. Modes: run, idle, pause, stop. Se conserva solo última consigna. Sin comandos durante 250 ms: freno local. No depender de las comunicaciones para medir encoder.
- Evaluación Supervisor → servicio por UDP: {"type":"evaluation","sim_time":1.0,"x":0,"y":0,"yaw":0,"contacts":0}. Solo estadísticas/evaluación/UI; NO usar para localización ni navegación. La evaluación debe estar separada del estimador.
- Comandos al supervisor: servicio escribe atómicamente .runtime/scene_command.json: {"seq":1,"action":"reset"|"configure","scale":1.0,"robot_mass_kg":0.18,"friction":0.8,"skid_friction":0.12,"fan_force_n":0,"start":{"x":...,"y":...,"yaw":...}}. Solo al estar detenido. Supervisor aplica y confirma por mensaje UDP {"type":"scene_ack","seq":1}.

## Cámara en memoria compartida

Archivo .runtime/camera.frame, mmap: cabecera64 bytes, formato little-endian `<8sIIIIQdd16x`: magic=b'RRFRAME1', width:uint32, height:uint32, channels:uint32=4, reserved:uint32=0, sequence:uint64 (offset24), sim_time:double, capture_monotonic:double. Imagen BGRA contiguous a continuación. Secuencia impar mientras se escribe; par al completar. Lector copia solo si las secuencias inicial/final coinciden y son pares. El lector debe remapear al cambiar resolución. El consumidor conserva únicamente el fotograma más nuevo. Cámara pinhole, robot marker40 mm, diccionario OpenCV DICT_APRILTAG_36h11, ID0. Esquinas de calibración de suelo IDs1..4.

Supervisor escribe .runtime/camera_calibration.json con {"width":1280,"height":960,"camera_matrix":[[...],[...],[...]],"dist_coeffs":[0,0,0,0,0],"camera_position":[...],"camera_target":[...],"marker_size_m":0.04,"marker_height_m":...,"marker_offset_m":[x,y],"arena_scale":1.0,"reference_tags":[{"id":1,"corners_world":[[x,y,z],...]},...]}. Calibración usa imágenes/markers del suelo; metadatos posición cámara solo sirven calibración inicial y pruebas; nunca son pose del robot.

## API aplicación

- GET /api/config → {robot,arena,defaults,webots:{url:"ws://127.0.0.1:1234",module:"/webots/wwi/WebotsView.js"}}.
- GET /api/state y WS /api/ws → mismo estado: {status:"idle"|"running"|"paused"|"stopped"|"completed"|"disconnected",message,sim_time,pose:{x,y,yaw,source,age_ms},robot:{speed,left_speed,right_speed,battery_voltage},mission:{goals:[{id,x,y}],active_goal,waypoints:[{x,y}],progress},settings:{speed,scale,mass_kg,friction,skid_friction,fan_force_n},vision:{visible,latency_ms},metrics:{...},connected,webots_url}.
- POST /api/mission/start, /pause, /stop, /reset. JSON body opcional. UI espera respuestaestado.
- POST /api/settings {speed?,scale?,mass_kg?,friction?,skid_friction?,fan_force_n?}. Velocidad en marcha con rampa; otros requieren estar detenido.
- PUT /api/goals {goals:[{id:"A",x,y},...]} proyección sobre caminos válidos; no cambiar en marcha.
- GET /api/camera.jpg para vista humana (10 Hz máximo; fuera del control).
- GET /api/health → diagnóstico.
- UI: elemento oficial webots-view; .connect(url,"w3d",true,false), .hideToolbar(); no dependencias externas/CDN en runtime. Root sirve archivos Webots desde instalación local /webots/.
- UI: editar objetivos arrastrando en mapa2D complementario; mainviewport W3D. Si W3D no disponible, informar desconexión, sin simular movimiento falso. Vistas orbital/cenital mediante API visor y cámara mediante camera.jpg.

## Plano del laberinto

Cuatro ramales H/dobleT. Centros x=-0.6,+0.6; verticales y=-0.55..+0.55, conectados en y=0. Corredores0.5m, paredes0.02m de espesor y0.1m de alto. Área2.4x1.8m. A=(-.6,.55), B=(.6,.55), C=(-.6,-.55), D=(.6,-.55), todo multiplicado por scale. InicioA mirando hacia sur. Nodos internosL=(-.6,0),R=(.6,0). Aristas A-L,C-L,L-R,R-B,R-D. API de geometría y planificador propiedad del servicio; mundo reproduce esas posiciones.

## Reparto de archivos

- Root: README, docscomparación, configuración, scripts/launcher, mundo Webots, cámara supervisor, driver Webots, bibliotecaC++, instalación, integración y pruebas físicas.
- Backend: simulation/service/**, simulation/tests/test_service*.py, pyproject.toml y configlectura. FastAPI+visión+estimador+planificador+navegación. No editar mundo/C++/UI.
- Frontend: simulation/ui/**. Vite+TypeScript+React, interfazespañolglass, npmtestsbuild. No editar backend/config/control.
- Assets: simulation/assets/**, simulation/tools/build_assets.py, docs/activos*.md. Modelo visual WRL/OBJ a escala metros, registro geométrico a partir RevQ.brd. Root enlaza mallas a articulaciones.

No modificar ni cargar firmware del robot real. La versión previa se conserva como referencia. No afirmar masa/inercia/fricción calibradas. Todos los cambios se hacen dentro de este checkout. Root hace commits; agentes no hacen gitcommit.
