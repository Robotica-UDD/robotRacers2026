# Ir a un punto con la webcam

El panel de http://127.0.0.1:8010/ permite seleccionar un destino en la imagen
y enviar al robot una secuencia de giro, avance y parada. Python y OpenCV
obtienen la posición desde el AprilTag; las consignas de velocidad llegan a la
ESP32 por Wi-Fi y el firmware regula cada rueda con su encoder. Esta prueba
funciona sin Webots ni ROS 2.

El perfil actual de movimiento suave todavía no tiene una medición de precisión
física. Se han comprobado el enlace, la parada de las órdenes y el controlador
con un robot virtual. Las tolerancias indicadas son parámetros del programa,
no precisión medida del robot real.

## Preparar el montaje

1. Fija la webcam encima de una superficie horizontal, mirando hacia abajo.
   Mantén el área completa del recorrido dentro de la imagen, con buena luz y
   sin obstáculos, bordes de mesa, personas ni cables atravesando la trayectoria.
2. Coloca el AprilTag 36h11, identificador 0, en una base rígida y horizontal,
   paralela al suelo y por encima de los cables. El patrón negro debe medir
   **40 × 40 mm**, con **5 mm de margen blanco continuo por cada lado**. Imprime
   [el PDF](../simulation/webcam/assets/apriltag-0-40mm.pdf) al 100 %, sin ajustar
   a la página, y comprueba sus medidas. La flecha impresa apunta al frente real.
   Consulta la [referencia de montaje](prueba-webcam.md#colocar-el-marcador).
3. Conecta el robot a la misma red local que el Mac. La ESP32 necesita una red
   de 2,4 GHz; el Mac puede usar otra banda del mismo router si no hay aislamiento
   entre dispositivos. Configura inicialmente la red por USB siguiendo la
   [prueba de control Wi-Fi](prueba-control-wifi.md).
4. Enciende la alimentación de batería y comprueba que el robot esté listo.
   **Retira el cable USB del robot antes de moverlo**, para evitar que lo arrastre
   o lo desvíe. La webcam mantiene su cable USB al Mac. La batería debe mantener
   encendida tanto la ESP32 como la etapa de motores.
5. Ejecuta `./simulation/webcam/run.command`, selecciona la webcam y pulsa
   **Iniciar cámara**. Comprueba que el marcador se detecte de manera estable.

La turbina permanece apagada en esta prueba. El movimiento utiliza solamente
los dos motores de tracción.

## Calibrar el centro y el frente

La calibración de dos clics es obligatoria. El marcador puede estar desplazado
respecto al eje de las ruedas: su centro por sí solo no define el centro del robot.

1. Pulsa **Calibrar robot** en la tarjeta **Navegación**.
2. Haz el primer clic en el **centro entre las dos ruedas**, sobre la línea del
   eje de tracción. Si el montaje lo tapa, identifica su posición en la imagen.
3. Haz el segundo clic en un punto **hacia el frente real del robot en su posición
   actual**, distinto del centro y en el plano horizontal del marcador. Ese punto
   fija una dirección, no un destino. El programa admite una separación estimada
   de 10 a 250 mm.
4. Comprueba que la flecha y el punto del centro acompañen correctamente al robot
   al cambiar su posición. La tarjeta debe indicar **Centro y frente listos**.

El primer clic se guarda respecto al marcador, no como un punto fijo de la
imagen. Si el robot cambia de posición antes del segundo clic, el centro
seleccionado lo acompaña: elige el frente según la posición actual del robot.
Si se tapa el marcador, el panel espera una detección válida y conserva el paso
pendiente. El borrador también se conserva al recargar la misma pestaña, mediante
`sessionStorage`; contiene la selección geométrica y no credenciales del robot.

Cambiar realmente de cámara o de resolución invalida la calibración y el destino.
Si cambias físicamente la posición del marcador o del soporte, vuelve a calibrar
aunque uses la misma cámara.

En macOS, el identificador USB de la webcam puede cambiar al conectarla en otro
puerto. Se observó ese cambio con la Trust y llegó a invalidar una calibración
guardada aunque se trataba de la misma webcam. La captura conserva ahora una
identidad lógica para reconectar el mismo modelo cuando hay **una única cámara
con el mismo nombre exacto**. Si hay varias coincidencias o se elige otra cámara,
no las considera intercambiables. Una resolución diferente sigue requiriendo
calibrar; esta recuperación no sustituye la comprobación física del montaje.

El servicio conserva la calibración, el destino y la indicación de misión
pendiente en `simulation/.runtime/navigation-state.json`, excluido de Git.
También puede leer el archivo anterior `navigation-calibration.json`. Cargar
ese estado o abrir el panel no envía órdenes al robot.

## Elegir un destino y moverse

1. Con el robot detenido, haz clic sobre una zona despejada de la imagen. El
   punto y la línea de unión muestran el destino seleccionado.
2. **Seleccionar un punto nunca mueve el robot.** Revisa el sentido del frente,
   el espacio disponible para girar y la trayectoria completa antes de continuar.
3. Pulsa **Ir al punto** una sola vez. El robot gira para orientarse, avanza
   corrigiendo su dirección y envía una parada al acercarse al destino.
4. Usa **Detener recorrido**, **Detener** en la tarjeta Robot o el botón físico
   **D48** para cancelarlo. Esas paradas requieren un nuevo inicio explícito.

Si se pierde una detección o la conexión Wi-Fi, la misión **espera con los
motores a cero**, conservando el destino y la calibración. Al recuperarse una
imagen estable y el enlace, continúa automáticamente hacia el mismo punto.
No debes volver a seleccionarlo ni iniciar otra misión. Un cambio real de
cámara o resolución sí requiere repetir la calibración.

Si cierras o reinicias el **programa Python**, al abrirlo recupera el destino
como pendiente, pero exige pulsar **Ir al punto** para volver a mover el robot.
Es distinto de una reconexión de la webcam o del Wi-Fi durante una misión activa,
que se reanuda automáticamente.

Empieza con destinos cercanos. Se aceptan desplazamientos estimados de hasta
**0,60 m** desde el centro actual. Los puntos muy próximos al borde de la imagen
se rechazan: el margen mínimo es el mayor entre 20 píxeles y el 60 % del lado
medio del marcador detectado. Esto ayuda a conservar el seguimiento; también
debes comprobar que todo el robot permanezca dentro del área de prueba.

No puedes cambiar el destino ni recalibrar mientras un recorrido esté activo.
La prueba de **Avanzar 1 segundo** también queda bloqueada durante la navegación.

## Ajustar las velocidades

La tarjeta **Navegación** incluye dos sliders independientes:

- **Avance:** velocidad máxima entre 50 y 300 mm/s; inicialmente 100 mm/s.
- **Giro:** velocidad máxima entre 12 y 100°/s; inicialmente 32°/s.

Puedes cambiarlos antes de iniciar o durante el recorrido. El servicio aplica
las consignas gradualmente, conserva el destino y guarda los valores para la
próxima sesión en `simulation/.runtime/navigation-speeds.json`. Cambiar un
slider no inicia un recorrido. El robot reduce su velocidad al acercarse al
punto o terminar de alinearse, por lo que el número mostrado es una máxima,
no una velocidad constante garantizada durante todo el movimiento.

La interfaz agrupa los movimientos del slider y confirma el valor guardado con
el servicio. Si falla una actualización, muestra un aviso y recupera el último
valor confirmado. La versión anterior del firmware admite hasta 100 mm/s;
para superar ese límite se requiere la versión que anuncia
`max_velocity_mm_s: 300`. Cada inicio comprueba el límite real del robot.

## Límites del control y de la medición

| Parámetro del programa | Valor configurado |
|---|---|
| Frecuencia del control de movimiento | 20 Hz |
| Distancia máxima al seleccionar o iniciar | 600 mm |
| Paso de giro a avance | Error de orientación de hasta 8° |
| Volver a girar durante el avance | Error mayor de 25° |
| Velocidad máxima de avance | Slider de 50 a 300 mm/s; valor inicial de 100 mm/s y base mínima de marcha de 50 mm/s |
| Velocidad máxima de giro | Slider de 12 a 100°/s; valor inicial de 32°/s, con separación de ruedas estimada en 190 mm |
| Aproximación al destino | Reduce la consigna según distancia de frenado; usa 120 mm/s² para anticipar la parada |
| Fin del giro | Reduce según el ángulo pendiente; usa 70°/s² como referencia de frenado |
| Subida y bajada de consigna en Python | 120 y 180 mm/s², respectivamente |
| Cambio de sentido o de movimiento | Pasa por cero antes de invertir; frena ambas ruedas antes de cambiar entre giro y avance |
| Control local de velocidad | 100 Hz en la ESP32, con realimentación de ambos encoders |
| Inicio de la parada | Distancia estimada de hasta 25 mm |
| Asentamiento | 400 ms con consignas cero; acepta hasta 40 mm al terminar |
| Duración del intento | Sin cancelación automática por tiempo total; el tiempo se muestra como diagnóstico |
| Tiempo sin progreso observado | Tras 5 segundos activos, pausa 0,5 segundos, vuelve a estimar y reintenta con los mismos topes |
| Imagen requerida al iniciar | Antigüedad de hasta 200 ms |
| Imagen requerida para conducir | Observación actual visible, con antigüedad de hasta 150 ms |
| Fallo breve de detección | Hasta 100 ms desde la última captura válida, mantiene el último mando sin acelerar; exige misma cámara activa e imagen reciente |
| Recuperación del seguimiento | Al menos 3 fotogramas distintos y 150 ms de detección estable |

Las órdenes de navegación expresan **milímetros por segundo por rueda**, con el
signo indicando la dirección. No son valores PWM. Python coordina la orientación
y la llegada desde la cámara, aplica una rampa a la velocidad solicitada y envía
las consignas a la ESP32. El firmware incrementa el esfuerzo de arranque
hasta observar movimiento en los encoders y después regular la velocidad de
cada rueda. Una orden de 0/0 detiene también ese esfuerzo de arranque.

El mínimo de 50 mm/s se aplica a la velocidad base de avance antes de corregir
la orientación. La rueda interior puede recibir una velocidad inferior para
respetar la curva. No se fuerza un mínimo independiente de PWM ni se modifica
el cero de parada; las rampas también pasan por velocidades inferiores al mínimo
de marcha al arrancar y frenar.

El arranque comienza en PWM 35 y aumenta hasta un máximo de 160, de manera
independiente para cada rueda, hasta registrar dos cuentas en el sentido pedido.
Después pasa al controlador de velocidad, con transición gradual. Utiliza las
polaridades verificadas, rueda de 23 mm y 59,94 cuentas por vuelta. Si una rueda
no entrega progreso correcto durante cinco segundos con una orden activa, el
firmware cancela el movimiento y requiere un nuevo inicio; no sigue aumentando
la potencia indefinidamente. Una inversión espera a que la rueda frene.

Para cargar esta versión, conecta el robot por USB y ejecuta
`./simulation/webcam/upload_robot.command`. El actualizador comprueba que haya
una única ESP32, verifica que esté detenida y compila antes de cargar. Mantiene
las credenciales Wi-Fi y no ordena movimientos.

El inicio exige que el firmware informe `velocity_control: true` y
`encoder_ready: true`, tanto en el estado del panel como en una consulta fresca
al armar. **No hay alternativa automática de PWM abierto** si falta ese control.
La velocidad visual filtrada sigue disponible como diagnóstico; ya no reduce
el PWM ni sustituye la realimentación local de encoders.

Las velocidades solicitadas no garantizan una precisión física medida: la
conversión desde pulsos requiere los parámetros correctos de ruedas y encoders.
Ante falta de progreso visual, el servicio hace una pausa breve, recalcula el
movimiento desde la posición actual y vuelve a intentarlo. Puede seguir intentando
hasta llegar o hasta una parada manual, un fallo o un cambio de fuente. Sin
visión válida sigue esperando con consigna cero.

La geometría se calcula mediante una homografía del cuadrado conocido de 40 mm.
Su plano de referencia es **el plano del marcador**, que está por encima del
suelo. Un clic sobre el suelo no está a esa misma altura: la perspectiva y el
paralaje introducen error en la distancia y en el punto alcanzado. Mantener la
cámara cenital y el marcador horizontal reduce el problema, pero no equivale a
una calibración métrica del suelo. Las distancias mostradas son aproximadas.

Esta implementación **no evita obstáculos** ni planifica alrededor de ellos.
Los encoders regulan la velocidad de las ruedas; la cámara sigue determinando
la orientación hacia el destino y cuándo detenerse. El firmware anterior de
secuencias se conserva como referencia independiente. La precisión real depende del agarre,
la inercia, la iluminación, la altura de la cámara y el montaje del marcador.

## Comunicación y parada

El servicio Python abre una sesión de control mediante HTTP autenticado en
`POST /navigation/start`. Una confirmación correcta permite enviar consignas
asociadas a esa sesión al puerto UDP 4210, a 20 Hz, mediante `command: velocity`.
Los campos `left` y `right` son enteros en mm/s, entre −100 y 100; el perfil de
esta aplicación usa los intervalos menores de la tabla anterior. Cada paquete lleva una secuencia
creciente y renueva un plazo local de **300 ms** en el firmware. El armado inicial
concede hasta un segundo para recibir el primer mando.

Si dejan de llegar consignas válidas, la ESP32 pone los motores a cero, termina
la sesión y rechaza los paquetes atrasados de esa sesión. La parada explícita
se envía por UDP y también por HTTP. Este mecanismo no depende de que el
navegador siga abierto ni de que Python pueda enviar una orden posterior.

Si falla brevemente la detección, pero la misma cámara sigue activa y entregando
imágenes recientes, el servicio conserva el último mando mientras la última
captura válida tenga hasta 100 ms. No aumenta la consigna ni calcula nuevas
correcciones o una llegada usando la pose anterior. Esto evita frenar y volver
a arrancar por un fotograma sin marcador. La comprobación se realiza a 20 Hz;
los 100 ms son un umbral del programa, no un tiempo físico de parada garantizado.

Si la detección no vuelve dentro de ese margen, envía 0/0 y espera. La cámara
desconectada, una imagen atrasada, una fuente distinta o una parada manual no
usan ese margen. Tras tres fotogramas distintos y al menos 150 ms de seguimiento
estable, recalcula desde la posición observada, conservando el destino.

Si falla el enlace, caduca la sesión o se reinicia el robot, la misión espera
la conexión. Descarta la sesión local y envía una parada UDP de mejor esfuerzo,
sin bloquear el control esperando HTTP. Cuando el robot está listo y hay
seguimiento estable, solicita por HTTP **una sesión nueva** y continúa.
Ese armado se ejecuta en otro trabajador; una petición HTTP lenta no bloquea el
bucle de control. Una respuesta que indique una parada manual o un fallo cancela
la misión, en vez de volver a armarla.

**D48, Detener y un fallo de driver cancelan el recorrido.** En esos casos,
recuperar la imagen o el enlace no vuelve a mover el robot. El armado automático
comprueba también el estado fresco del robot antes de solicitar una sesión, para
no borrar una parada D48 ocurrida durante la desconexión. Si se cancela mientras
hay una confirmación HTTP pendiente, una confirmación tardía recibe una parada
y no inicia los motores. Un cambio real de cámara o resolución invalida la
calibración y cancela el destino; los valores temporales vacíos durante una
reconexión de la misma cámara no lo hacen.

El primer inicio manual sin confirmación no se reintenta automáticamente. La
recuperación automática corresponde a una misión que ya fue iniciada y confirmada.

El estado de navegación expone `left_target_mm_s` y `right_target_mm_s` para las
consignas enviadas. Los campos anteriores `last_left` y `last_right` se mantienen
por compatibilidad y ahora también representan mm/s, **no PWM**. Incluye además
orientación relativa al destino, velocidad y giro estimados visualmente, tiempo
transcurrido y secuencia de imagen. El estado del robot informa las velocidades
de encoder por rueda, los contadores y el PWM aplicado por el controlador local.
Los contadores `vision_pauses`, `brief_vision_gaps` y `mode_switches` permiten
revisar cuántas pausas de seguimiento, entradas al margen breve y cambios de
estado ocurrieron durante el intento. `vision_grace_active` indica cuándo se
está conservando el último mando. Estos datos ayudan a contrastar un video de
los tirones; por sí solos no identifican su causa física.
La latencia HTTP de la
tarjeta Robot mide una consulta de estado; no mide el recorrido completo desde
la captura de imagen hasta la aplicación de una consigna.

## Comprobaciones realizadas

- El firmware con encoders se compiló con Arduino-ESP32 **2.0.14**:
  **765001 bytes de programa y 50432 bytes de RAM**. Se cargó por USB y se
  verificaron `max_velocity_mm_s: 300`, `velocity_control: true`, `encoder_ready: true`, ambos PWM a cero
  y ninguna sesión de movimiento activa. La prueba de movimiento sigue pendiente.
- Con la versión anterior del robot físico se enviaron consignas UDP **0/0** para verificar
  la caducidad local de 300 ms y el rechazo de paquetes posteriores. Esta
  comprobación no hizo avanzar ni girar el robot.
- Se comprobó también el armado de una sesión nueva tras una caducidad usando
  consignas 0/0, y el bloqueo de reanudación después de una parada manual. Ese
  bloqueo permanece aunque cambie el motivo visible de parada; un nuevo inicio
  explícito puede liberarlo si el robot está listo.
- El modelo virtual integra ruedas diferenciales separadas 190 mm, proyecta el
  marcador en la cámara simulada y cierra el ciclo a través del controlador.
  Completó seis destinos delante, a ambos lados, detrás y en diagonales traseras,
  con error final de hasta 26 mm y parada. Es un modelo cinemático ideal; no
  reproduce la fricción ni la inercia medidas del robot real.
- Las pruebas automatizadas cubren geometría, sentido de los giros, correcciones,
  selección sin movimiento, límites de destino, pausa y recuperación de visión,
  cambios de fuente, D48, caducidad y recuperación de sesión, reinicio, confirmaciones
  tardías tras cancelar, persistencia sin movimiento, rampas, paso por cero,
  consignas métricas, rechazo de firmware sin control por encoder, transporte y solicitudes inválidas.
  Usan dobles de cámara y robot, sin abrir hardware ni enviar tráfico al robot.
- Once pruebas C++ del controlador real, compilado sin Arduino, cubren fricción
  estática, pulsos cuantizados, arranque, regulación, saturación, parada,
  inversión con inercia y el límite de cinco segundos sin respuesta.

**Queda pendiente validar este perfil de movimiento en el robot real y medir
su error final.** No se ha confirmado que el robot físico alcance las tolerancias
configuradas.

Para repetir las pruebas locales:

```sh
.venv/bin/python -m pytest -q \
  simulation/tests/test_webcam_navigation_geometry.py \
  simulation/tests/test_webcam_point_navigation.py \
  simulation/tests/test_webcam_robot_link.py
```
