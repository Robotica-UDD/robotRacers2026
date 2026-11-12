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
5. Si una mano permanece lejos durante **4 segundos**, el robot fija un
   **punto de interés** para acercarse. Se considera lejana a partir de
   **400 mm** desde la huella del robot; si aumentas la distancia de reacción,
   ese umbral será como mínimo la distancia de reacción más 200 mm.
6. El punto de interés deja al menos **250 mm entre la huella del robot y el
   recuadro de la mano**. Si la distancia de reacción es mayor de 150 mm, la
   separación será esa distancia más 100 mm. Se comprueba que el punto sea
   accesible alrededor de los obstáculos dibujados. Aparece como **INTERÉS**
   en la imagen y queda fijo: mover o retirar la mano no cambia ni cancela
   ese viaje.
7. Una mano cercana tiene prioridad y puede interrumpir el acercamiento para
   regresar a Casa. Al llegar a Casa o al punto de interés, conserva el recorrido
   anterior para que puedas iniciarlo manualmente; no lo reanuda por su cuenta.

La confirmación de curiosidad sigue la misma mano entre imágenes y tolera
ausencias de hasta medio segundo. No repite el acercamiento al mismo gesto:
la mano debe desaparecer durante un segundo antes de iniciar otra confirmación.
No se intenta atraer al robot mientras está regresando a Casa. Si no hay un
punto de acercamiento accesible, la vigilancia y la reacción de refugio siguen
activas.

**Cancelar regreso**, **Cancelar acercamiento**, **Detener** y **D48** tienen prioridad. Desactivar o borrar
Casa invalida también una orden pendiente durante el cálculo de ruta. Al
reiniciar el programa se recuperan Casa y la distancia, con la reacción
**desactivada** hasta volver a pulsar su botón.

El destino de curiosidad se alcanza con el **centro calibrado** a no más de
25 mm del punto elegido. Esto reserva la separación solicitada al detenerse;
la llegada habitual a los puntos del recorrido y a Casa conserva su criterio
de cobertura por la huella del robot.

La distancia es una estimación en el plano del marcador. La altura de la mano
y la perspectiva influyen en ella. La mano inicia la reacción; los obstáculos
que delimitan el camino siguen siendo los dibujados en el mapa. Si deja de verse
el marcador, se conserva el destino y se aplican las reglas de seguimiento y
parada del navegador.

## Modelo y funcionamiento local

Se utiliza [YOLOv4-tiny Cross-Hands del autor](https://github.com/cansik/yolo-hand-detection),
entrenado específicamente para detectar manos. Funciona con OpenCV DNN, sin
ROS 2, servicios externos ni envío de imágenes a Internet. La inferencia usa
imágenes de 416 × 416 y un trabajador independiente, limitado a diez análisis
por segundo y un único fotograma pendiente. AprilTag y el vídeo conservan su
propio hilo.

Las cajas de manos se asocian a las esquinas del marcador del **mismo
fotograma** mediante secuencia y tiempo de captura. No se comparan manos de
una imagen antigua con una posición posterior del robot. Se rechazan detecciones
atrasadas, cambios de cámara y dimensiones incompatibles.

Una detección no abre una sesión nueva en la ESP32: utiliza la sesión armada
por el botón de vigilancia o la del recorrido en curso. D48 invalida esa sesión
y la detección no puede volver a armarla automáticamente. Los latidos de la
vigilancia parada siempre contienen velocidades 0/0.

## Instalación reproducible

Los pesos, configuración y licencia se guardan en
`simulation/.runtime/hand-models`, excluidos de Git. El instalador fija las
direcciones del autor y verifica SHA-256; la aplicación no descarga modelos
durante la captura:

```sh
.venv/bin/python -m simulation.tools.download_hand_model
.venv/bin/python -m simulation.tools.download_hand_model --check
```

La procedencia está fijada al repositorio del autor en el commit
`42d038bbc5deed14b11b27a871ca4385b4d880a2` y a su publicación de modelos
`pretrained`. La licencia MIT del repositorio y las referencias a sus conjuntos
de entrenamiento se conservan en el manifiesto local del instalador.

## Validación

El modelo real detectó manos en la imagen de demostración del autor. La prueba
local inicial midió alrededor de 11 ms de inferencia media en CPU sobre ocho
fotogramas; no es una medición de la latencia completa de reacción.

Las pruebas sin hardware verifican la proximidad con el centro calibrado, la
confirmación temporal, el destino fijo tras desaparecer la mano, la separación
final, el desvío alrededor de un obstáculo, la prioridad de Casa y la restauración
inactiva del recorrido anterior. También comprueban la prioridad de las paradas
incluso durante planificación o armado. La reacción completa en el suelo debe
probarse con el robot real; la separación depende de la calibración y de la altura
de la mano respecto al plano usado.
