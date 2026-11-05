# ROS 2 y conexión futura al robot

La aplicación utiliza metros, radianes y segundos. La base del robot tiene X hacia delante, Y hacia la izquierda y Z hacia arriba. Los encoders normalizados son positivos al avanzar cada rueda.

Los adaptadores de `simulation/adapters/` son opcionales y no abren cámaras, puertos ni ROS 2 al importarlos. No se cargó firmware ni se envió ninguna orden al robot físico durante su desarrollo. La versión de Arduino existente se conserva.

## Arquitectura y propiedad del control

La navegación y la visión pertenecen al ordenador. El lazo de velocidad de cada rueda, la lectura del encoder, el límite de corriente y el frenado por pérdida de comunicación pertenecen al ESP32. Los mensajes del ordenador son consignas de velocidad; no sustituyen ese lazo local por una secuencia de pulsos enviada por la red.

La implementación común de motor y encoder está en `simulation/control/drive.hpp`. No depende de Webots ni de Arduino. El driver Webots ya la utiliza. Un futuro firmware puede incluir esa misma cabecera y conectar `Drive.observe`, `Drive.update` y las salidas PWM a los periféricos del ESP32, respetando las polaridades comprobadas. El modelo de corriente/par de la cabecera sirve a la simulación; en hardware, los límites eléctricos siguen correspondiendo al driver y a las protecciones reales.

Debe existir **un solo emisor de consignas**:

- En la aplicación local, el servicio Python manda y el driver Webots o el puente USB recibe.
- El adaptador ROS 2 publica datos sin controlar motores por defecto.
- Para que ROS 2 mande, usar `--enable-motion`, dirigir la salida del servicio a un puerto distinto y mantener la aplicación en pausa. No ejecutar dos navegadores mandando sobre el mismo puerto.
- El ESP32 debe frenar si pasan 250 ms sin una consigna válida. Esta protección se ejecuta localmente, aunque Python, ROS o el cable fallen.

## Cámara física con el mismo procesamiento

`real_camera.py` produce exactamente el archivo `camera.frame` del contrato, con imágenes BGRA y secuencia de escritura. Su lector conserva únicamente la imagen más reciente. Una resolución nueva reemplaza el archivo atómicamente; no trunca un archivo que otro proceso ya tenga mapeado.

Se necesita una calibración propia de la cámara real: resolución, matriz intrínseca, distorsión, cuatro referencias de suelo con sus esquinas medidas en el marco `map`, tamaño/altura/desplazamiento del marcador del robot. No reutilizar los intrínsecos del simulador en una cámara física.

Los marcadores son AprilTag 36h11: ID 0 sobre el robot e IDs 1–4 fijos en el suelo. La parte superior del marcador ID 0 apunta hacia delante del robot. El procesamiento estima la cámara a partir de los puntos de suelo y compensa la altura y el desplazamiento del marcador antes de obtener la posición de la base.

Ejemplo explícito, desde la raíz del repositorio:

```sh
.venv/bin/python -m simulation.adapters.real_camera \
  --device 0 \
  --calibration /ruta/a/calibracion-real.json \
  --runtime /ruta/a/runtime-hardware
```

El adaptador comprueba que la resolución recibida coincida con la calibración. `CAP_PROP_BUFFERSIZE=1` solicita reducir la cola, pero algunos controladores de cámara no respetan esa opción. La marca temporal representa la **recepción de la imagen en el host**, no el instante físico de exposición. La latencia mostrada a partir de ella no incluye una cola interna desconocida de la cámara; esa parte necesita medición propia.

El servicio se conecta al mismo directorio y utiliza el mismo filtro de encoders/visión. No se utiliza ninguna coordenada verdadera del simulador para sustituir la visión física.

## Puente USB para el ESP32

La dependencia `pyserial` solo se instala para este modo:

```sh
uv sync --extra hardware
```

El puente requiere un **futuro firmware que implemente el protocolo siguiente**. El programa Arduino REV3.11, que imprime diagnósticos de texto y ejecuta una secuencia con botones, no acepta todavía estas órdenes JSON. El puente no intenta modificarlo ni cargarlo.

Formato: un objeto JSON por línea, UTF-8, terminado en salto de línea. El ESP32 entrega muestras con cuentas acumuladas sin aplicar la normalización de signo:

```json
{"type":"encoder","left_count":-100,"right_count":100,"device_micros":12345000,"battery_voltage":7.4}
```

- `left_count` y `right_count`: enteros acumulados; conviene que el acumulador del firmware sea de 64 bits.
- `device_micros`: entero sin signo de 32 bits del reloj del dispositivo.
- El puente aplica los signos `[-1,+1]`, convierte 59,94 cuentas/vuelta y radio 11,5 mm a metros, y conserva por separado el tiempo del dispositivo.
- La marca `stamp` de telemetría se pone al recibir en el ordenador. No se compara directamente `micros()` del ESP32 con el reloj de Python.
- Se reconoce el desbordamiento de `device_micros`. Un retroceso pequeño se considera reinicio del dispositivo, conserva la continuidad de las cuentas publicadas y bloquea movimiento hasta recibir una orden nueva `idle`, `pause` o `stop` antes del siguiente `run`.

El puente envía:

```json
{"type":"wheel_target","seq":42,"source_seq":130,"stamp":1234.5,"mode":"run","ttl_ms":250,"left_mps":0.105,"right_mps":0.295}
```

Las velocidades ya están en el sentido físico normalizado. El firmware conserva las inversiones de PH/PWM necesarias para cada motor. `seq` identifica el mensaje del puente y `source_seq` la orden original. `stamp` es un reloj del host: el firmware utiliza su propio tiempo de recepción para el vencimiento de 250 ms. Tras vencer el watchdog debe permitir reiniciar la ventana de secuencia del emisor.

El transporte es estructural: cualquier implementación con `readline()`, `write(bytes)` y `close()` puede utilizarse. `SerialLineTransport` carga `pyserial` y abre el puerto solo cuando se llama a `open()`. Mantiene DTR y RTS desactivados para evitar un reinicio involuntario.

Ejemplo de recepción con movimiento deshabilitado:

```sh
.venv/bin/python -m simulation.adapters.hardware \
  --port /dev/cu.usbmodemPUERTO \
  --command-port 9212 --telemetry-port 9201
```

El valor predeterminado de UART es 921600 baudios, suficiente para mensajes frecuentes sin saturar una UART de 115200. Con USB CDC nativo, el valor de baudios es normalmente una configuración lógica y no el límite físico del enlace. Ajustarlo al transporte y al futuro firmware utilizado.

Para una prueba física intencional y preparada, el argumento `--enable-motion` habilita el envío después del rearme. Sin ese argumento, todas las órdenes de salida son de detención. El puente también envía detención si las órdenes del host dejan de llegar. Estas comprobaciones no eliminan el watchdog independiente del firmware.

El servicio debe ejecutarse con:

```sh
ROBOTRACERS_RUNTIME_DIR=/ruta/a/runtime-hardware \
ROBOTRACERS_COMMAND_PORT=9212 \
.venv/bin/python -m simulation.service
```

No ejecutar simultáneamente el driver Webots en ese mismo puerto. El directorio de hardware debe estar separado del usado por la simulación. La interfaz 3D de Webots sigue representando una simulación, no una reconstrucción visual del robot real.

## Adaptador ROS 2 opcional

`ros2_bridge.py` importa `rclpy` y los mensajes únicamente al arrancar explícitamente el nodo. El servicio y la interfaz local no requieren instalar ROS en macOS. Para ROS 2 Jazzy, utilizar un entorno ROS correctamente configurado, por ejemplo Ubuntu 24.04 con Python 3.12 y las dependencias Python del proyecto.

```sh
source /opt/ros/jazzy/setup.bash
python3 -m simulation.adapters.ros2_bridge \
  --runtime /ruta/al/runtime \
  --service http://127.0.0.1:8000
```

Publicaciones:

| Interfaz | Tipo | Significado |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Odometría relativa continua a partir de cuentas de encoder normalizadas. |
| `/tf` | Transformaciones | `map → odom → base_link`. |
| `/overhead/image_raw` | `sensor_msgs/Image` | Imagen BGRA de la cámara cenital. |
| `/overhead/camera_info` | `sensor_msgs/CameraInfo` | Intrínsecos/distorsión de la misma imagen y resolución. |
| `/clock` | `rosgraph_msgs/Clock` | Solo con `--use-sim-time`, para pruebas en simulación. |
| `/cmd_vel` | `geometry_msgs/Twist` | Entrada opcional: X lineal y Z angular. No provoca movimiento sin la habilitación explícita. |

`map` corresponde al laberinto y a las referencias visuales. `odom` es el marco relativo del encoder. Se integran las diferencias de cuentas publicadas por el servicio; para servicios anteriores sin esos campos, se integran las velocidades normalizadas como compatibilidad. `base_link` representa el centro entre ejes. Las correcciones visuales se expresan en `map → odom`, sin trasladarlas directamente a la odometría continua. La cámara utiliza `overhead_camera_optical_frame` y la convención óptica habitual: X derecha, Y abajo, Z hacia la escena. Sus extrínsecos reales deben obtenerse de la calibración; el adaptador no inventa una transformación fija.

La covarianza publicada de odometría es provisional y necesita calibración real. No equivale a una certificación de precisión. Para simulación, activar `--use-sim-time` en el puente y el parámetro `use_sim_time` de los nodos consumidores.

Para que ROS asuma el control, dirigir el servicio a un puerto sin receptor —por ejemplo `ROBOTRACERS_COMMAND_PORT=9222`— y arrancar el puente ROS con `--enable-motion --command-port 9212`. El puente USB escucha 9212. Ante una orden Twist vieja, pérdida del servicio o falta de visión reciente, se envía detención. Dejar la misión de la aplicación en pausa durante ese modo.

## Pruebas sin dispositivos

```sh
.venv/bin/python -m pytest simulation/tests/test_service_adapters.py
```

Las pruebas utilizan un transporte en memoria y una cámara simulada de prueba. Comprueban que construir/importar adaptadores no abre dispositivos, que el formato compartido es compatible, signos y unidades de encoders, desbordamiento/reinicio del reloj, rearme, vencimiento de órdenes, conversión diferencial y coherencia de transformaciones ROS. No se necesita GPIO, puerto serie, cámara física ni una instalación ROS 2.

## Métricas y pruebas de fallos

La pose estimada y la evaluación del simulador se comparan en un instante común del reloj simulado. Las muestras de ambas trayectorias se interpolan y los ángulos se interpolan respetando el salto entre +π y −π. La evaluación no entra al filtro ni al navegador. Si falta un intervalo temporal común o las muestras están obsoletas, el error se devuelve como no disponible.

En el host, imágenes y consignas utilizan el mismo `time.monotonic()` de Python. En macOS el driver C++ emplea `mach_absolute_time`; se comprobó que el origen de `std::chrono::steady_clock` era diferente en el equipo. No se corrige ese desfase aceptando mensajes viejos.

Las pruebas integradas de fallo inyectan cámara compartida y telemetría UDP sobre puertos temporales. Verifican pérdida de visión, pérdida de telemetría, parada solicitada y recuperación sin reinicio automático. No abren dispositivos físicos.

La navegación tolera por defecto hasta 500 ms sin una actualización visual aceptada, utilizando la predicción de los encoders durante ese intervalo. `ROBOTRACERS_VISION_GRACE_S` permite ajustar ese margen entre 0,10 y 1,00 s. Una imagen recibida con más de 250 ms de antigüedad se rechaza igualmente; aumentar el margen no convierte imágenes atrasadas en información actual. El watchdog local de comandos permanece en 250 ms.

```sh
.venv/bin/python -m pytest simulation/tests/test_service_metrics.py simulation/tests/test_service_faults.py
```
