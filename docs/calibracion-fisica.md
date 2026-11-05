# Calibración del robot real

La simulación reproduce la PCB RevQ y las dimensiones nominales registradas,
pero todavía utiliza masa, centro de gravedad, inercia, rozamientos y succión
estimados. Sus trayectorias no certifican la trayectoria del robot fabricado.

## Medidas que sustituyen estimaciones

1. Pesar el robot completo, con batería, cables, patín y regleta montados.
2. Medir diámetro efectivo de las ruedas con carga y separación entre los
   centros de sus bandas de rodadura. La referencia inicial es 23 mm y 190 mm.
3. Obtener el centro de gravedad mediante apoyos/balanzas y actualizar
   `body_center_of_mass_m`. La masa configurada incluye dos ruedas, cuya masa
   individual se descuenta para la masa del cuerpo.
4. Estimar o medir el tensor de inercia. `body_inertia_kg_m2` contiene los tres
   términos diagonales; el modelo inicial omite productos de inercia. El ajuste
   de masa de la interfaz escala esa inercia conservando la distribución.
5. Medir tiempo, distancia, cuentas de encoder y velocidad durante rectas y
   giros a varios PWM, y el desplazamiento tras la orden de frenado.
6. Ajustar rozamiento del neumático y patín contra esas medidas, sin compensar
   un diámetro o separación incorrectos con ganancias de control.
7. Medir la fuerza de succión de la turbina antes de activarla en el modelo.

Los parámetros se encuentran en `simulation/config/robot.json`. Los cambios de
geometría/configuración se aplican al volver a abrir el lanzador. Masa,
rozamiento y succión también se pueden ensayar desde la interfaz estando
parado. La distribución visual de batería, regleta, cables y patín sigue
identificada como provisional.

## Modelo de motor

Se utilizan los puntos nominales del [Pololu #5178](https://www.pololu.com/product/5178/specs):
6 V, 6500 rpm y 170 mA sin carga; 1,5 A y 0,09 kg·cm de bloqueo extrapolado.
El modelo obtiene resistencia, constante de fuerza contraelectromotriz y
constante de par de esos puntos. Incluye pérdida mecánica suave, saturación de
corriente/par y frenado a PWM cero; no simula conmutaciones individuales del
puente H, rizado de los reguladores, temperatura o descarga de la batería.

El control y el par se actualizan cada 10 ms, con retención del valor entre
actualizaciones. Webots integra contactos y movimiento cada 2 ms. El encoder
se cuantiza a 59,94 cuentas por vuelta de rueda; no se multiplica de nuevo por
cuatro. Las velocidades observadas y la odometría tienen esa cuantización.

## Medidas de cámara

La matriz intrínseca y la distorsión deben obtenerse de la cámara física.
Medir posición de referencias de suelo, altura del marcador y desplazamiento
respecto al centro entre ruedas. El marcador ID 0 apunta con su borde superior
hacia delante. El marcador removible y su soporte deben quedar visibles,
firmes y sin tapar ventilación; su ubicación simulada requiere comprobarse en
el robot real.

La cámara simulada tiene obturador ideal, distorsión nula y no incorpora la
latencia interna de un dispositivo real. La latencia del informe comienza al
estar disponible la imagen en el ordenador. Para medir extremo a extremo en
hardware habrá que incluir exposición, transferencia de cámara y transporte
hasta el ESP32.
