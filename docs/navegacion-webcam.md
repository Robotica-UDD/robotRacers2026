# Recorrer puntos con la webcam

El panel de http://127.0.0.1:8010/ permite marcar varios destinos en la imagen
y recorrerlos en el orden elegido. Python y OpenCV
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

El servicio conserva la calibración, la lista de puntos, el siguiente punto pendiente y la indicación de misión
pendiente en `simulation/.runtime/navigation-state.json`, excluido de Git.
También puede leer el archivo anterior `navigation-calibration.json`. Cargar
ese estado o abrir el panel no envía órdenes al robot.

## Marcar un recorrido y moverse

1. Con el robot detenido, haz clic en los destinos sobre zonas despejadas de la
   imagen. Cada clic añade un punto numerado; la línea muestra el orden del recorrido.
2. **Marcar puntos nunca mueve el robot.** Revisa el sentido del frente,
   el espacio disponible para girar y la trayectoria completa antes de continuar.
3. Pulsa **Iniciar recorrido** una sola vez. El robot gira para orientarse, avanza
   corrigiendo su dirección, se detiene en cada punto y continúa con el siguiente.
4. Usa **Detener recorrido**, **Detener** en la tarjeta Robot o el botón físico
   **D48** para cancelarlo. Esas paradas requieren un nuevo inicio explícito.
5. **Borrar puntos** detiene el recorrido y vacía la lista. Conserva la calibración
   y las velocidades. Puedes marcar después una ruta nueva.

El panel indica el punto actual y el total. Tras una parada puedes continuar con
el punto pendiente pulsando **Iniciar recorrido**; después de completar toda la
lista, ese mismo botón permite repetirla desde el primer punto. Si añades puntos
después de completar una ruta, el siguiente inicio recorre solo los nuevos.

Un destino se considera alcanzado en cuanto queda **debajo del contorno del
robot**, sin exigir que coincida con el centro entre ruedas. El contorno nominal
abarca unos 83 mm delante, 53 mm detrás y hasta 116 mm a cada lado, según su forma
y la orientación actual. Cuando cubre el punto, frena y conserva esa llegada
aunque se desplace un poco al detenerse. Después calcula el siguiente tramo
desde la posición real. Los puntos intermedios necesarios para rodear obstáculos
mantienen su precisión y no usan esta zona de llegada ampliada.

Si la pérdida de detección supera el margen breve o se pierde la conexión Wi-Fi,
la misión **espera con los motores a cero**, conservando la lista, su progreso y la calibración. Al recuperarse una
imagen estable y el enlace, continúa automáticamente hacia el punto pendiente.
No debes volver a seleccionarlo ni iniciar otra misión. Un cambio real de
cámara o resolución sí requiere repetir la calibración.

Si cierras o reinicias el **programa Python**, al abrirlo recupera el destino
como pendiente, pero exige pulsar **Iniciar recorrido** para volver a mover el robot.
Es distinto de una reconexión de la webcam o del Wi-Fi durante una misión activa,
que se reanuda automáticamente.

Los destinos **no tienen un límite fijo de distancia desde el robot**: pueden
estar a más de 600 mm, dentro de la imagen de la cámara. La planificación y el
seguimiento siguen comprobando los obstáculos y la visibilidad del marcador.
Con el margen de seguridad en 0 mm no se añade un borde adicional al seleccionar
el punto. Comprueba que la trayectoria y el robot permanezcan dentro del área
de prueba.

No puedes añadir puntos ni recalibrar mientras un recorrido esté activo.
**Borrar puntos** sí está disponible durante el movimiento y lo detiene.
La prueba de **Avanzar 1 segundo** también queda bloqueada durante la navegación.

## Dibujar obstáculos y rodearlos

1. Deja el robot detenido y la cámara fija. Pulsa **Dibujar obstáculo**.
2. Marca las cuatro esquinas siguiendo el borde del objeto en orden, por ejemplo
   superior izquierda, superior derecha, inferior derecha e inferior izquierda.
   El cuarto clic cierra y guarda el polígono. No cruces sus lados. Puedes
   cancelar un dibujo incompleto o pulsar Escape.
3. Repite para los demás obstáculos. Los polígonos rojos representan los objetos;
   la zona amarilla representa el espacio reservado para rodearlos.
4. El **Margen de seguridad** admite de **0 a 200 mm**. Actualmente empieza en
   **0 mm**, sin separación adicional; puedes aumentar el valor para recuperarla.
5. Marca tus destinos y revisa el camino verde antes de pulsar
   **Iniciar recorrido**. El planificador introduce puntos intermedios para
   rodear los obstáculos y mantiene el orden de tus destinos numerados.
6. **Borrar obstáculos** detiene el recorrido y elimina solamente los obstáculos.
   Conserva tus destinos, la calibración y las velocidades. **Borrar puntos**
   conserva el mapa de obstáculos.

Con **0 mm**, la planificación usa el **contorno orientado del robot**: carrocería,
ruedas y portador del marcador, proyectados desde el centro entre ruedas según
la dirección real del frente. No utiliza el círculo de 116,1 mm como distancia
mínima a los obstáculos. Ese círculo cubría todas las orientaciones a la vez y
podía bloquear una postura que sí cabía. El contorno visible en verdeazul permite
comprobar qué forma está usando el programa.

El recorrido normal se realiza **siempre de frente**. Antes de cada tramo,
el robot gira hasta orientar su frente hacia el siguiente punto. La reversa
solo se admite como una salida inicial corta cuando está en contacto o no hay
espacio para girar; después se detiene, se orienta y continúa hacia delante.
No se elige la reversa para ahorrar un giro o hacer un rodeo completo.

Al rodear obstáculos, la prioridad es **pocos tramos largos, giros cercanos a
90° y espacio respecto de los bordes**, aunque el recorrido sea más largo.
Los ejes se estiman a partir de las paredes dibujadas; una pared inclinada en
la imagen no obliga a hacer el recorrido horizontal respecto de la cámara.
El plan compara rutas por carriles y penaliza los giros y la proximidad tanto
a los obstáculos como al borde visible. Prefiere el centro de los pasillos.
Es una preferencia de selección, no un margen nuevo que prohíba pasar.
Los tramos casi paralelos pueden mantenerse rectos sin añadir dos giros pequeños
para corregir unos milímetros. Si no cabe una maniobra ortogonal, conserva la
búsqueda con ángulos más flexibles y las salidas de contacto ya disponibles.

El plan contempla esas maniobras de liberación y giros en el sitio. Comprueba el barrido
de la huella durante cada movimiento, no solo sus extremos. Prefiere recorridos
con más espacio cuando hay alternativas, sin convertir esa preferencia en una
franja prohibida adicional. Si el robot está junto a un obstáculo, puede retroceder
antes de girar. Si la posición estimada ya aparece en contacto, busca una maniobra
que reduzca ese contacto hasta salir; no exige apartarlo manualmente para volver
a calcular. La salida se hace a velocidad reducida.

Durante esa salida se admiten hasta **0,25 mm de aumento transitorio** únicamente
en un contacto que el modelo ya detectaba al iniciar. Esto permite que la
envolvente de las ruedas comience un giro de liberación, sin confundir una
variación de décimas de milímetro con la ausencia de camino. El límite inicial
se conserva en todos los pasos y comprobaciones de mando del mismo plan;
no se acumula una tolerancia nueva en cada ciclo. Los extremos de las maniobras
de escape deben reducir el contacto hasta quedar libres, y no se permite
iniciar contacto con otro obstáculo. El recorrido normal conserva sus
comprobaciones sin esta tolerancia de escape.

El borde de la cámara tampoco lleva un radio reservado. Se comprueba la posición
real de las cuatro esquinas del marcador en cada postura prevista, sin los
84 mm que antes se exigían alrededor del centro para cualquier orientación.
El objetivo no impone una dirección final: el plan puede llegar con una orientación
que mantenga el marcador visible. Perder realmente la detección sigue pausando
el movimiento y conserva el recorrido para recalcular al recuperarla.

Un valor de separación positivo conserva el perfil circular anterior con el margen
elegido y su reserva de seguimiento. La configuración actual permanece en **0 mm**.
La huella y su procedencia están en `simulation/webcam/robot_footprint.py` y en
[la revisión geométrica](activos_revision_geometrica.md).

Los obstáculos y el camino se proyectan al plano del marcador mediante la
calibración existente. Las separaciones son **estimaciones del modelo**, no
distancias medidas sobre el suelo. El paralaje, la inclinación de la cámara y
el montaje del marcador afectan su exactitud. Comprueba el tamaño real del
robot con sus accesorios.
Marca la huella que ocupa cada objeto sobre el suelo y deja holgura adicional
para objetos altos o con piezas que sobresalgan.

Durante la marcha, el servicio vuelve a comprobar la maniobra restante con la
posición y la orientación observadas. Una proximidad lateral no fuerza velocidad
cero: se comprueba si la forma del robot cabe al avanzar, retroceder o girar.
La rampa y el control local por encoders siguen aplicándose a las velocidades
firmadas de ambas ruedas. Tras cada maniobra se confirma la parada antes de
cambiar de sentido; un giro no se cuenta como llegada a un destino.

Si cambia la posición o hay contacto inesperado, se detiene el mando de ese tramo
y se calcula una maniobra nueva desde la postura actual. También se vuelve a
intentar sin exigir un desplazamiento manual de 10 mm. Una pared que cierre
completamente el paso todavía puede impedir llegar: el programa no sustituye
una búsqueda fallida por una orden de atravesarla.

Al perder el marcador envía cero y espera una imagen estable; conserva el destino
y recalcula el camino desde la posición recuperada. La ventana de continuación
sin marcador descrita más abajo se aplica únicamente cuando no hay obstáculos.
Cambiar la cámara o su resolución invalida el mapa; mover físicamente la cámara
requiere recalibrar y dibujarlo de nuevo, aunque la resolución sea la misma.

El mapa es manual y estático: solo representa lo que dibujaste. No detecta ni
añade automáticamente objetos que aparezcan después. La implementación funciona
directamente en Python y OpenCV. La separación por tamaño del robot y expansión
de obstáculos sigue el enfoque de espacio de configuración descrito en
[Costmap 2D de Nav2](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/);
esta aplicación no requiere ROS 2.

## Ajustar las velocidades

La tarjeta **Navegación** incluye dos sliders independientes:

- **Avance:** consigna máxima desde 50 mm/s; inicialmente 100 mm/s.
- **Giro:** consigna máxima desde 12°/s; inicialmente 32°/s.

Los topes de aplicación de 300 mm/s y 100°/s se retiraron. Cada control tiene
**Ampliar escala**, que duplica el rango visible sin cambiar la consigna, y una
casilla numérica para introducir un valor mayor y confirmarlo con Enter o al
salir del campo. La escala se ajusta para mostrar el número confirmado. Llegar
al extremo del slider no duplica la velocidad por sí solo.

Puedes cambiarlos antes de iniciar o durante el recorrido. El servicio aplica
las consignas gradualmente, conserva el destino y guarda los valores para la
próxima sesión en `simulation/.runtime/navigation-speeds.json`. Cambiar un
slider no inicia un recorrido. El robot reduce su velocidad al acercarse al
punto o terminar de alinearse, por lo que el número mostrado es una máxima,
no una velocidad constante garantizada durante todo el movimiento.

La interfaz agrupa los movimientos del slider y confirma el valor guardado con
el servicio. Si falla una actualización, muestra un aviso y recupera el último
valor confirmado. Las versiones anteriores del firmware admiten 100 o
300 mm/s. La versión actual acepta el intervalo del formato firmado de 32 bits
y anuncia `max_velocity_mm_s: 2147483647`; ese número describe el mensaje, no la
velocidad física del robot. Cada inicio comprueba la capacidad de la versión
conectada, y los números que no caben en el formato se rechazan.

La potencia máxima sigue siendo PWM 160 de 255. Las rampas de consigna son ahora
600 mm/s² al acelerar y 900 mm/s² al desacelerar, tanto en Python como en el
controlador local. El frenado cerca del destino se anticipa con 450 mm/s² y
240°/s² al terminar un giro. Son valores de control, pendientes de medir sobre
el robot real. La velocidad alcanzable depende de la alimentación, carga y motores.
Pololu especifica 6500 rpm sin carga a 6 V para el modelo #5178; ese dato no es
una medición del robot sobre el suelo. [Especificaciones de Pololu](https://www.pololu.com/product/5178/specs).

## Corrección suave durante el avance

El avance utiliza un control proporcional y derivativo (PD) del rumbo, con
filtrado de la orientación y de su variación entre imágenes. Una zona muerta de
1,5° evita perseguir pequeñas oscilaciones de la medición. La corrección está
limitada a 8°/s y cambia gradualmente, hasta 25°/s². La rueda interior conserva
avance positivo y la corrección disminuye cerca del destino. No acumula una
corrección integral visual que pueda quedarse cargada tras una pérdida.

Solo procesa imágenes distintas. Reinicia el filtro al cambiar de punto, al
salir del avance o tras una pausa larga. Una lectura aislada que indique más de
25° de error ya no provoca un giro en el sitio: debe persistir durante 250 ms y
al menos tres imágenes diferentes. Los giros hacia el destino conservan el
perfil anterior. La regulación por encoder de cada rueda también se mantiene.

## Límites del control y de la medición

| Parámetro del programa | Valor configurado |
|---|---|
| Frecuencia del control de movimiento | 20 Hz |
| Distancia al seleccionar o iniciar | Sin máximo fijo; el destino debe estar dentro de la imagen |
| Paso de giro a avance | Hasta 8° sin obstáculos y hasta 3° con obstáculos |
| Volver a girar durante el avance | Más de 25° sin obstáculos o 15° con obstáculos durante 250 ms y al menos 3 imágenes distintas |
| Velocidad máxima de avance | Escala ampliable y entrada numérica desde 50 mm/s; valor inicial de 100 mm/s |
| Velocidad máxima de giro | Escala ampliable y entrada numérica desde 12°/s; valor inicial de 32°/s |
| Aproximación al destino | Reduce la consigna según distancia de frenado; usa 450 mm/s² para anticipar la parada |
| Fin del giro | Reduce según el ángulo pendiente; usa 240°/s² como referencia de frenado |
| Subida y bajada de consigna en Python y ESP32 | 600 y 900 mm/s², respectivamente |
| Subida y bajada de salida PWM local | 600 y 900 unidades/s; el aumento inicial hasta detectar movimiento conserva 300 unidades/s |
| Cambio de sentido o de movimiento | Pasa por cero antes de invertir; frena ambas ruedas antes de cambiar entre giro y avance |
| Control local de velocidad | 100 Hz en la ESP32, con realimentación de ambos encoders |
| Llegada a un destino del usuario | El punto queda dentro del contorno orientado del robot; no exige coincidencia con su centro |
| Asentamiento | 150 ms entre puntos y 400 ms al terminar, con consignas cero; conserva la llegada al frenar y recalcula el siguiente tramo desde la posición real |
| Duración del intento | Sin cancelación automática por tiempo total; el tiempo se muestra como diagnóstico |
| Tiempo sin progreso observado | Tras 5 segundos activos, pausa 0,5 segundos, vuelve a estimar y reintenta con los mismos topes |
| Imagen requerida al iniciar | Antigüedad de hasta 200 ms |
| Imagen requerida para conducir | Observación actual visible, con antigüedad de hasta 150 ms |
| Fallo breve de detección | En avance, hasta 400 ms, 60 mm o 15° estimados, lo primero que se agote; también reduce la ventana cerca del destino. Exige la misma cámara entregando imágenes nuevas y recientes |
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

La geometría usa el cuadrado conocido de 40 mm. Con un marcador pequeño
(hasta 48 píxeles de lado medio), el programa ajusta las cuatro esquinas mediante
un modelo afín. Ese modelo aproxima la cámara cenital y evita que
el ruido de las esquinas deforme todo el mapa. Con una perspectiva bien resuelta
conserva la homografía. Planificación, huella y control comparten el mismo modelo.
No se rechaza un campo por superar 10 m: la cuadrícula adapta su resolución para
mantener acotada la memoria, y cada maniobra sigue comprobándose contra los
obstáculos. Si falla una vista previa, vuelve a calcularla pasado un segundo;
un fotograma defectuoso no deja bloqueada permanentemente la ruta.

Su plano de referencia es **el plano del marcador**, que está por encima del
suelo. Un clic sobre el suelo no está a esa misma altura: la perspectiva y el
paralaje introducen error en la distancia y en el punto alcanzado. Mantener la
cámara cenital y el marcador horizontal reduce el problema, pero no equivale a
una calibración métrica del suelo. Las distancias mostradas son aproximadas.

La evitación utiliza únicamente los obstáculos dibujados: no detecta otros
objetos ni obstáculos en movimiento automáticamente.
Los encoders regulan la velocidad de las ruedas; la cámara sigue determinando
la orientación hacia el destino y cuándo detenerse. El firmware anterior de
secuencias se conserva como referencia independiente. La precisión real depende del agarre,
la inercia, la iluminación, la altura de la cámara y el montaje del marcador.

## Comunicación y parada

El servicio Python abre una sesión de control mediante HTTP autenticado en
`POST /navigation/start`. Una confirmación correcta permite enviar consignas
asociadas a esa sesión al puerto UDP 4210, a 20 Hz, mediante `command: velocity`.
Los campos `left` y `right` son enteros en mm/s, entre −2147483647 y 2147483647;
se rechaza −2147483648 para evitar desbordar el valor absoluto. El perfil de
esta aplicación usa los intervalos menores de la tabla anterior. Cada paquete lleva una secuencia
creciente y renueva un plazo local de **300 ms** en el firmware. El armado inicial
concede hasta un segundo para recibir el primer mando.

Si dejan de llegar consignas válidas, la ESP32 pone los motores a cero, termina
la sesión y rechaza los paquetes atrasados de esa sesión. La parada explícita
se envía por UDP y también por HTTP. Este mecanismo no depende de que el
navegador siga abierto ni de que Python pueda enviar una orden posterior.

Si falla brevemente la detección mientras avanza, pero la misma cámara sigue
entregando imágenes nuevas y recientes y no hay obstáculos dibujados, el servicio conserva exactamente el
último mando. La ventana máxima es de **400 ms** desde la última detección real,
con presupuestos de **60 mm de recorrido por rueda** y **15° de giro** estimados
según las consignas; se aplica lo que se agote antes. Cerca del destino reduce
la ventana para reservar el margen de llegada y la distancia de frenado estimada. Al girar en el sitio conserva
como máximo 100 ms, también reducido por los presupuestos de recorrido y giro.
Frenar, cambiar de movimiento o llegar no extiende el último mando.

Los presupuestos de recorrido y giro no son desplazamientos físicos medidos:
el host recibe estados de encoder aproximadamente una vez por segundo y no hace
odometría con ellos. El control de cada rueda sí permanece activo en la ESP32 a
100 Hz. No aumenta la consigna, confirma llegadas ni cambia de punto mientras
no ve el marcador. Las imágenes nuevas sin marcador no renuevan el presupuesto.
La interfaz muestra **Recuperando marcador** y retira la posición visual anterior.

Si la detección no vuelve dentro de ese margen, envía 0/0 y espera, conservando la ruta y el punto pendiente. La cámara
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

La API local añade un punto con `POST /api/navigation/targets`, usando las
coordenadas `x`, `y`, `frame_width`, `frame_height` y `camera_id` de la imagen.
`POST /api/navigation/clear-targets` detiene y borra la lista. El estado incluye
`waypoints_px`, `waypoint_index` (desde cero) y `waypoint_count`;
`target_px` representa el punto pendiente y queda vacío al completar la lista.
El antiguo `POST /api/navigation/target` conserva su significado de reemplazar
el destino por una ruta de un solo punto. El panel incluye `expected_route_revision` al añadir e iniciar para rechazar
peticiones anteriores a una edición o borrado de la ruta. Estas peticiones usan la cabecera
local `X-Requested-With: XMLHttpRequest`, igual que los controles anteriores.

El mapa se edita mediante `POST /api/navigation/obstacles`, con la lista completa
`polygons_px`, la identidad y resolución de cámara y `expected_route_revision`.
`POST /api/navigation/clear-obstacles` detiene y vacía solo el mapa;
`POST /api/navigation/obstacle-settings` recibe `safety_margin_mm` y la revisión
esperada. Cada edición invalida las órdenes anteriores y obliga a comprobar el
camino de nuevo. El mapa y su margen se guardan en
`simulation/.runtime/navigation-obstacles.json`, excluido de Git.

El estado incluye `obstacles_px`, `obstacle_settings`, `planned_path_px`,
`inflated_obstacles_px`, `planning_status` y `planning_message`.
`steering_target_px` señala el punto intermedio que sigue el robot;
`target_px` conserva el destino elegido por el usuario. Los puntos de desvío no
incrementan el contador de destinos completados. Un mapa guardado que no pueda
validarse bloquea el inicio hasta borrarlo o sustituirlo.
`collision_model` identifica el modelo de colisión y `robot_footprint_px` contiene
el contorno proyectado. `maneuver` informa el paso actual y su dirección de avance
o retroceso. `clearance_info.footprint_gap_mm` muestra la holgura entre el contorno
y el obstáculo; los valores negativos representan contacto estimado. Con margen
cero, las distancias requeridas de centro a obstáculo y centro a borde son cero:
la aceptación depende del contorno y del movimiento, no de esos radios.
Una distancia al obstáculo vacía significa que
no hay obstáculos dibujados, no una lectura anterior ni un valor infinito.

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
  **764889 bytes de programa y 50432 bytes de RAM**. Se cargó por USB y se
  verificaron `max_velocity_mm_s: 2147483647`, `velocity_control: true`, `encoder_ready: true`, ambos PWM a cero
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
- Un recorrido virtual de cuatro puntos con giros se completó en orden dentro
  de una misma sesión. Se verificaron la continuación desde un punto intermedio,
  el borrado durante el movimiento y el rechazo de solicitudes atrasadas después
  de borrar o detener. Las pruebas incluyen ruido de orientación,
  correcciones suaves, continuidad breve sin obstáculos, dibujo y persistencia de mapas,
  caminos bloqueados y desvíos por varios destinos.
- Las pruebas automatizadas cubren geometría, sentido de los giros, correcciones,
  selección sin movimiento, límites de destino, pausa y recuperación de visión,
  cambios de fuente, D48, caducidad y recuperación de sesión, reinicio, confirmaciones
  tardías tras cancelar, persistencia sin movimiento, rampas, paso por cero,
  consignas métricas, rechazo de firmware sin control por encoder, transporte y solicitudes inválidas.
  Usan dobles de cámara y robot, sin abrir hardware ni enviar tráfico al robot.
- El planificador tiene pruebas de cuadriláteros cóncavos y convexos, pasillos
  bloqueados, tangencias, perspectiva y límites de cálculo. Con cámara fija,
  rotar o trasladar el marcador conserva el lado elegido para el desvío.
- Una prueba cinemática con margen de 30 mm completa ida y vuelta entre dos
  destinos alrededor del mismo obstáculo, comprobando la separación del centro
  respecto al radio del modelo más el margen en toda la traza.
- Una regresión reproduce la disposición del 6 de noviembre: robot a unos
  145 mm del obstáculo, objetivo por encima y a la izquierda. La envolvente
  anterior bloquea el inicio; la corregida encuentra un rodeo por la izquierda
  y comprueba cada tramo. También se ensayan rotaciones, desplazamientos del
  marcador y posiciones que realmente colisionan. Son pruebas del modelo,
  no una medición física.
- Las pruebas de maniobras cubren el inicio a 100 mm de un obstáculo con una
  postura que cabe, la salida de un contacto inicial de 3,15 mm, la llegada a
  un objetivo a 80 mm del borde con el marcador visible y una pared que cierra
  todo el paso. Se verifican las velocidades firmadas, el barrido orientado y
  el arco diferencial de las correcciones, además de Stop, D48 y recuperación
  de visión sin perder los destinos.
- Se comprobó visualmente el dibujo de cuatro esquinas y el camino calculado
  en un navegador con captura sintética y movimiento deshabilitado.
- Catorce pruebas C++ del controlador real, compilado sin Arduino, cubren fricción
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
