# Detección de manos, curiosidad y regreso a Casa

Configuración confirmada por el usuario en el robot real: **240 mm/s de avance
máximo y 90°/s de giro máximo**. El panel conserva esa referencia escrita sin
sobrescribir los controles de velocidad.

## Uso

1. Con la cámara fija y calibrada, pulsa **Marcar casa** y selecciona el punto
   de refugio sobre la imagen. Guardarlo no mueve el robot.
2. Ajusta la distancia de reacción si hace falta. El valor inicial es **150 mm**
   desde la huella del robot hasta el recuadro de una mano.
3. Pulsa **Reaccionar a manos**. La vigilancia queda armada explícitamente;
   cuando está detenido solo envía consignas cero.
4. Una mano cercana confirmada en al menos dos imágenes y durante 150 ms
   interrumpe el recorrido y activa el regreso a Casa. Se usa la ruta calculada
   alrededor de los obstáculos dibujados y el control por encoders existente.
5. Si una mano permanece lejos durante **2 segundos**, el robot fija un
   **punto de interés** para acercarse. Se considera lejana a partir de
   **200 mm** desde la huella del robot; si aumentas la distancia de reacción,
   ese umbral será como mínimo la distancia de reacción más 50 mm.
   Una tolerancia de continuidad de 20 mm evita reiniciar el contador por ruido
   alrededor del umbral; nunca desactiva la prioridad de una mano cercana.
6. El punto de interés reserva **100 mm de separación mínima estimada entre
   la huella del robot y el recuadro de la mano**. El tamaño completo del robot
   y la tolerancia de llegada pueden dejar algo más de separación. Se comprueba
   que el punto sea accesible alrededor de los obstáculos dibujados. Aparece como **INTERÉS**
   en la imagen y queda fijo: mover o retirar la mano no cambia ni cancela
   ese viaje.
7. La mano que motivó el acercamiento puede permanecer quieta cerca del destino
   sin provocar un regreso inmediato. Si invade la huella del robot, se desplaza
   hacia él fuera de su posición de referencia o aparece otra mano cercana,
   el miedo vuelve a tener prioridad y puede interrumpir el acercamiento para
   regresar a Casa. Al llegar a Casa o al punto de interés, conserva el recorrido
   anterior para que puedas iniciarlo manualmente; no lo reanuda por su cuenta.

La confirmación de curiosidad acumula **2 segundos de observaciones visibles**.
Usa la palma si está disponible, acepta cambios de tamaño y conserva el progreso
ante una pérdida de hasta **1,5 segundos**. Ese intervalo sin detección pausa el
contador; no suma presencia inventada ni vuelve a cero inmediatamente.
No repite el acercamiento al mismo gesto:
la mano debe desaparecer durante un segundo antes de iniciar otra confirmación.
No se intenta atraer al robot mientras está regresando a Casa. Si no hay un
punto de acercamiento accesible, la vigilancia y la reacción de refugio siguen
activas. Un cálculo fallido no consume el gesto: vuelve a intentarlo tras
2 segundos mientras la misma mano siga visible y conserva el motivo en el panel.
El punto solo queda fijado después de aceptar el acercamiento.

Una pérdida breve del AprilTag no borra la confirmación si MediaPipe sigue
viendo la mano ya asociada. Se usa la geometría anterior solo para asociar su
palma en la cámara fija; iniciar el viaje siempre exige una posición actual
y válida del robot. La ausencia real de mano pausa el tiempo acumulado.

El panel distingue confirmación, espera por visión o enlace y reintentos. Muestra
la distancia estimada y el umbral de curiosidad: con la reacción de miedo inicial
de 150 mm, una mano a 120 mm corresponde a Casa; la curiosidad comienza desde
200 mm. El recuadro completo de la mano puede estar mucho más cerca que su palma.

**Cancelar regreso**, **Cancelar acercamiento**, **Detener** y **D48** tienen prioridad. Desactivar o borrar
Casa invalida también una orden pendiente durante el cálculo de ruta. Al
reiniciar el programa se recuperan Casa y la distancia, con la reacción
**desactivada** hasta volver a pulsar su botón.

El destino de curiosidad se alcanza con el **centro calibrado** a no más de
25 mm del punto elegido. Esto reserva la separación solicitada al detenerse;
la llegada habitual a los puntos del recorrido y a Casa conserva su criterio
de cobertura por la huella del robot.

La detección no exige que la mano esté a una altura determinada: usa su imagen
y no descarta por profundidad, altura ni tamaño aparente. Como en cualquier
modelo visual, necesita suficiente detalle y una parte reconocible de la mano.
La separación en centímetros se estima proyectando la imagen sobre el plano
del marcador. Una cámara monocular sin profundidad no permite garantizar que
sean 10 cm reales para cualquier altura; levantar la mano cambia esa proyección.
La mano inicia la reacción; los obstáculos
que delimitan el camino siguen siendo los dibujados en el mapa. Si deja de verse
el marcador, se conserva el destino y se aplican las reglas de seguimiento y
parada del navegador.

## Modelo y funcionamiento local

Se utiliza [MediaPipe Hand Landmarker de Google](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker?tab=Python),
con detección de palma y seguimiento de 21 puntos por mano en modo VIDEO.
Reemplaza al modelo YOLO anterior. La inferencia se ejecuta localmente en CPU,
sin ROS 2 ni envío de imágenes a servicios externos. Un trabajador independiente
procesa hasta 20 imágenes por segundo y conserva solo el fotograma pendiente
más reciente. AprilTag y el vídeo conservan su propio hilo.

Los umbrales de detección, presencia y seguimiento están en 0,35. El resultado
público marca `confidence=1` solo como aceptación de los umbrales internos;
no representa una probabilidad calibrada. La confianza de mano izquierda o
derecha se publica separada y no se usa para rechazar una mano reconocida.
El vídeo muestra el recuadro y los 21 puntos, sin usar la profundidad relativa
de esos puntos como una altura medida sobre el suelo.

Las cajas de manos se asocian a las esquinas del marcador del **mismo
fotograma** mediante secuencia y tiempo de captura. No se comparan manos de
una imagen antigua con una posición posterior del robot. Se rechazan detecciones
atrasadas, cambios de cámara y dimensiones incompatibles.

Antes de buscar maniobras de acercamiento, una cuadrícula de conectividad
descarta candidatos al otro lado de un perímetro cerrado. Esta comprobación
solo elimina regiones desconectadas; el punto elegido todavía necesita una
ruta válida con la huella orientada. Así no se agota el tiempo probando primero
puntos que están fuera del recinto del robot.

La curiosidad **no tiene distancia máxima entre la mano y el robot**. La mano
puede estar al otro lado de una pared y requerir un rodeo. Se eliminó la
preaprobación de ruta limitada a 180 ms en la elección del punto: ahora esa fase
elige un extremo libre y la navegación calcula y valida la trayectoria completa
antes de permitir movimiento. También se eliminó el máximo de 350 mm adicionales
para elegir una posición de observación; la búsqueda se adapta al campo visible.
Elegir un punto de interés no significa que exista paso para la huella: una C
con un cuello menor que el ancho real del robot puede seguir sin ruta válida.
El perímetro, la separación y las comprobaciones de las ruedas se conservan.
Se prefieren extremos con espacio para orientarse. Si el círculo de giro no
cabe, un extremo con la huella orientada libre sigue siendo candidato al
planificador completo; esa preferencia no prohíbe los pasillos estrechos.

Una detección no abre una sesión nueva en la ESP32: utiliza la sesión armada
por el botón de vigilancia o la del recorrido en curso. D48 invalida esa sesión
y la detección no puede volver a armarla automáticamente. Los latidos de la
vigilancia parada siempre contienen velocidades 0/0.

## Instalación reproducible

El paquete de modelo y su licencia se guardan en
`simulation/.runtime/hand-models`, excluidos de Git. El instalador fija las
direcciones oficiales versionadas y verifica SHA-256; la aplicación no descarga modelos
durante la captura:

```sh
.venv/bin/python -m simulation.tools.download_hand_model
.venv/bin/python -m simulation.tools.download_hand_model --check
```

El paquete Python queda fijado a `mediapipe==0.10.31` en `uv.lock`; se instala
con `uv sync --frozen`. El modelo oficial es Hand Landmarker full float16,
versión 1, descargado desde `storage.googleapis.com/mediapipe-models`.
La licencia Apache-2.0 y la ficha del modelo quedan referenciadas en el manifiesto
local. Los antiguos pesos YOLO pueden permanecer en la caché, pero no se cargan.

## Validación

El modelo real detectó ambas manos y sus 21 puntos en la imagen de demostración
oficial. La prueba local midió unos 27 ms de inferencia media en CPU en diez
fotogramas posteriores al arranque; no es la latencia completa de reacción.
Con separación solicitada de 100 mm, las pruebas de ruedas simuladas terminaron
a 125–176 mm según la orientación y la tolerancia de llegada.

Las pruebas sin hardware verifican la proximidad con el centro calibrado, la
confirmación temporal, la alternancia de detecciones válidas y pérdidas del
AprilTag, el reintento tras fallar sin retirar la mano, el destino fijo tras
desaparecer la mano, la separación final, el desvío alrededor de un obstáculo, la prioridad de Casa y la restauración
inactiva del recorrido anterior. También comprueban la prioridad de las paradas
incluso durante planificación o armado. La reacción completa en el suelo debe
probarse con el robot real; la separación depende de la calibración y de la altura
de la mano respecto al plano usado.

Revisión del 11 de noviembre de 2026: 1.065 pruebas aprobadas. El ensayo
`test_recorded_six_wall_feedback_completes_all_four_destinations_without_deadlock`
con máximas de 600 mm/s y 1.600°/s no completó los cuatro destinos en su plazo
de 35 s. El mismo fallo se reprodujo aislado en la versión anterior `eb246cf`,
con fuentes de navegación y control idénticas. Queda pendiente revisar ese
escenario de velocidad alta; este ajuste no cambia el controlador de movimiento.

El ajuste de selección sin distancia máxima superó 132 pruebas específicas de
curiosidad, interfaz, acercamiento y perímetros, incluidas manos a más de un
metro y el rechazo de movimientos si la trayectoria completa no es válida.
