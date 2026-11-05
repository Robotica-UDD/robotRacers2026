# Contrato de integración

Proyecto: Webots R2025a + servicio Python + interfaz web. Unidades internas: metros, radianes, segundos. Base del robot: X hacia delante, Y izquierda, Z arriba. Mundo: Z arriba. Cuentas de encoder normalizadas: positivas al avanzar ambas ruedas; 59.94 cuentas por vuelta.

## Procesos y puertos

- Servicio FastAPI: localhost:8000; UDP recepción telemetría: 9201; envío consignas a driver: 9202. Configurables por entorno.
- Webots: visor W3D localhost:1234. Física 2 ms; driver C++ actualiza control a 10 ms; cámara cada 20 ms.
- Reloj host: Python `time.monotonic()`. En macOS C++ utiliza `mach_absolute_time` con su `mach_timebase_info`, no `std::chrono::steady_clock` ni `CLOCK_MONOTONIC`, porque sus orígenes pueden diferir. Los valores `stamp`, `capture_monotonic` y `vision_stamp` usan ese mismo reloj. `sim_time` pertenece exclusivamente al reloj físico de Webots.
- Runtime compartido: variable ROBOTRACERS_RUNTIME_DIR; predeterminado simulation/.runtime. Nunca versionar.
- Driver → servicio, UDP JSON: {"type":"telemetry","seq":1,"sim_time":1.0,"stamp":monotonic_seconds,"left_count":0,"right_count":0,"left_velocity":0.0,"right_velocity":0.0,"left_pwm":0.0,"right_pwm":0.0,"battery_voltage":7.4,"reflectance":[0,0],"line_reflectance":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],"command_latency_ms":null}.
- Servicio → driver, UDP JSON: {"seq":1,"stamp":monotonic_seconds,"v":0.0,"w":0.0,"mode":"run","vision_stamp":monotonic_seconds}. Modes: run, idle, pause, stop. Se conserva solo última consigna. Sin comandos durante 250 ms: freno local. No depender de las comunicaciones para medir encoder. `vision_stamp` se omite cuando no hay medida visual; `command_latency_ms` mide desde la disponibilidad de esa imagen en el host hasta aplicar su consigna y no incluye una exposición física de cámara desconocida.
- Evaluación Supervisor → servicio por UDP: {"type":"evaluation","sim_time":1.0,"x":0,"y":0,"yaw":0,"contacts":0}. Solo estadísticas/evaluación/UI; NO usar para localización ni navegación. La evaluación debe estar separada del estimador. Las métricas interpolan estimación y evaluación al mismo `sim_time`, sin extrapolar; no comparan directamente una pose actual con una evaluación antigua. Una evaluación vieja, sin intervalo común o con desfase mayor de 100 ms produce errores `null`, no precisión cero.
- Comandos al supervisor: servicio escribe atómicamente .runtime/scene_command.json: {"seq":1,"action":"reset"|"configure","scale":1.0,"robot_mass_kg":0.18,"friction":0.8,"skid_friction":0.12,"fan_force_n":0,"start":{"x":...,"y":...,"yaw":...}}. Solo al estar detenido. Supervisor aplica y confirma por mensaje UDP {"type":"scene_ack","seq":1}.

## Cámara en memoria compartida

Archivo .runtime/camera.frame, mmap: cabecera64 bytes, formato little-endian `<8sIIIIQdd16x`: magic=b'RRFRAME1', width:uint32, height:uint32, channels:uint32=4, reserved:uint32=0, sequence:uint64 (offset24), sim_time:double, capture_monotonic:double. Imagen BGRA contiguous a continuación. Secuencia impar mientras se escribe; par al completar. Lector copia solo si las secuencias inicial/final coinciden y son pares. El productor reemplaza el archivo atómicamente al cambiar resolución, sin truncar una región que otro proceso tiene mapeada. El lector remapea al detectar el archivo nuevo. El consumidor conserva únicamente el fotograma más nuevo. Cámara pinhole, robot marker40 mm, diccionario OpenCV DICT_APRILTAG_36h11, ID0. Esquinas de calibración de suelo IDs1..4.

Supervisor escribe .runtime/camera_calibration.json con {"width":1920,"height":1440,"camera_matrix":[[...],[...],[...]],"dist_coeffs":[0,0,0,0,0],"camera_position":[...],"camera_target":[...],"marker_size_m":0.04,"marker_height_m":...,"marker_offset_m":[x,y],"arena_scale":1.0,"reference_tags":[{"id":1,"corners_world":[[x,y,z],...]},...]}. Calibración usa imágenes/markers del suelo; metadatos posición cámara solo sirven calibración inicial y pruebas; nunca son pose del robot.

El detector intenta la imagen nativa y, si faltan el robot o las referencias necesarias para calibrar, ampliaciones de 1,5 y 3 veces. Combina detecciones por identificador y devuelve todas las esquinas a las coordenadas originales antes de calcular la pose. Después de calibrar, la falta de una referencia fija por sí sola no exige ampliar cada imagen. El escalado facilita detectar marcadores pequeños; no añade resolución óptica ni sustituye una detección ausente por una pose conocida.

Durante una interrupción visual breve, la predicción utiliza únicamente los encoders. Se pausa la misión al superar 500 ms sin una medida visual aceptada; el umbral se configura con `ROBOTRACERS_VISION_GRACE_S` (0,10–1,00 s). El estimador rechaza imágenes que ya tengan más de 250 ms de antigüedad al procesarse. El margen de continuidad visual no modifica el frenado local de 250 ms por pérdida de comandos.

## API aplicación

- GET /api/config → {robot,arena,defaults,webots:{url:"ws://127.0.0.1:1234",module:"/webots/wwi/WebotsView.js"}}.
- GET /api/state y WS /api/ws → mismo estado: {status:"idle"|"running"|"paused"|"stopped"|"completed"|"disconnected",message,sim_time,pose:{x,y,yaw,source,age_ms},robot:{speed,left_speed,right_speed,battery_voltage,left_count,right_count,command_latency_ms},mission:{goals:[{id,x,y}],active_goal,waypoints:[{x,y}],progress,goal_measurements:[{id,x,y,error_m,sim_time}]},settings:{speed,scale,mass_kg,friction,skid_friction,fan_force_n},vision:{visible,latency_ms},metrics:{...},connected,webots_url}.
- POST /api/mission/start, /pause, /stop, /reset. JSON body opcional. UI espera respuestaestado.
- POST /api/settings {speed?,scale?,mass_kg?,friction?,skid_friction?,fan_force_n?}. Velocidad en marcha con rampa; otros requieren estar detenido.
- PUT /api/goals {goals:[{id:"A",x,y},...]} proyección sobre caminos válidos; no cambiar en marcha.
- GET /api/camera.jpg para vista humana (10 Hz máximo; fuera del control).
- GET /api/health → diagnóstico.
- Métricas: `position_error_m` y `heading_error_deg` corresponden a `evaluation_comparison_time`; `evaluation_aligned` indica si se pudieron alinear y `evaluation_alignment_lag_ms` expresa el retraso respecto al extremo más reciente de las dos trayectorias. `command_applied_latency_ms` conserva la medición enviada por el driver. `vision_detection_scale` identifica el último nivel de ampliación utilizado por el detector.
- Llegadas: se frena a 10 mm longitudinales, se espera al menos 1,1 s y quietud, y se registra el objetivo solo si el error de posición asentada no supera 18 mm. Cruzar el plano del punto no acredita una llegada. No se reanuda automáticamente tras perder visión o telemetría.
- UI: elemento oficial webots-view; .connect(url,"w3d",true,false), .hideToolbar(); no dependencias externas/CDN en runtime. El servicio publica los archivos Webots de la instalación local mediante /webots/.
- UI: editar objetivos arrastrando en mapa2D complementario; mainviewport W3D. Si W3D no disponible, informar desconexión, sin simular movimiento falso. Vistas orbital/cenital mediante API visor y cámara mediante camera.jpg.

## Plano del laberinto

Cuatro ramales H/dobleT. Centros x=-0.6,+0.6; verticales y=-0.55..+0.55, conectados en y=0. Corredores0.5m, paredes0.02m de espesor y0.1m de alto. Área2.4x1.8m. A=(-.6,.55), B=(.6,.55), C=(-.6,-.55), D=(.6,-.55), todo multiplicado por scale. InicioA mirando hacia sur. Nodos internosL=(-.6,0),R=(.6,0). Aristas A-L,C-L,L-R,R-B,R-D. API de geometría y planificador propiedad del servicio; mundo reproduce esas posiciones.

El firmware del robot real no se modifica ni se carga al ejecutar la simulación. Las masas, inercias y fricciones configuradas siguen siendo estimaciones hasta su medición física.

## Frecuencias y ensayos reproducibles

La física utiliza 2 ms; el controlador, el par retenido y los encoders se
actualizan cada 10 ms. El supervisor consulta contactos a 20 Hz y publica
evaluación a 100 Hz. La cámara nativa es 1920×1440, cada 20 ms de simulación.
Se aumentó desde 1280×960 porque los marcadores de 40 mm perdían detección al
girar en el entorno de escala 1,5. Frecuencia simulada y frecuencia por segundo
de pared son magnitudes distintas; el informe mide esta última y el factor de
tiempo real. El objetivo de la interfaz es una consigna, sujeta a aceleración,
frenado y dinámica del lazo; no es un limitador instantáneo de velocidad física.

`simulation/tools/validate_live.py` registra CSV/JSON durante diez minutos.
`validate_faults.py` comprueba oclusión renderizada, imagen atrasada, pausa y
parada. El ensayo opcional `--watchdog` requiere arrancar con
`ROBOTRACERS_TEST_TELEMETRY_PORT=9203`: recibe telemetría independiente mientras
suspende únicamente el servicio de esta sesión. Esa salida está deshabilitada
en el uso normal. La inyección `diagnostics.json` mueve el marcador mediante un
campo público del PROTO; no modifica ninguna pose del robot para navegar.

El visor publica una vez por segundo sus fotogramas realmente dibujados a
`/api/client-metrics`. Solo cuentan muestras con la vista 3D visible; ocultar una
pestaña o seleccionar Cámara no equivale a un resultado de rendimiento 3D.
