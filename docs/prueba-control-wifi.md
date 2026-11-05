# Prueba del robot desde el panel de cámara

El botón **Avanzar 1 segundo** envía por Wi-Fi una orden fija: motor izquierdo
100, motor derecho 100 y duración de 1000 ms. Los valores son PWM de 8 bits
(escala 0 a 255), no coordenadas ni velocidades medidas. La cámara muestra la
escena; esta prueba todavía no utiliza su posición para dirigir el robot.

## Enlace elegido

| Enlace | Uso en esta prueba |
|---|---|
| Wi-Fi con HTTP | Elegido: el servicio Python del panel envía una orden y recibe confirmación. |
| Bluetooth LE | Posible, pero requiere un servicio y cliente Bluetooth adicionales. |
| USB serie | Se utiliza para cargar el firmware y configurar el Wi-Fi inicialmente. |

El robot se conecta como estación a la red habitual. Necesita Wi-Fi de 2,4 GHz;
el Mac puede estar en otra banda del mismo router si ambos comparten la red y
no existe aislamiento de clientes. No se necesita ROS 2. Las interfaces usadas
están documentadas por Espressif: [Wi-Fi](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/wifi.html),
[Bluetooth del ESP32-S3](https://docs.espressif.com/projects/esp-idf/en/v5.2/esp32s3/api-guides/bluetooth.html)
y [WebServer en Arduino-ESP32 2.0.14](https://github.com/espressif/arduino-esp32/blob/2.0.14/libraries/WebServer/examples/HelloServer/HelloServer.ino).

## Usar el botón

1. Ejecuta `./simulation/webcam/run.command` y abre http://127.0.0.1:8010/.
2. En la tarjeta **Robot**, abre **Configurar Wi-Fi por USB**. Selecciona el
   puerto de la ESP32, escribe el nombre y la contraseña de la red y pulsa
   **Conectar robot**. Para reutilizar la red ya guardada, deja ambos campos
   vacíos. El monitor serie de Arduino debe estar cerrado mientras se enlaza.
3. Enciende el interruptor general y espera **Robot listo para la prueba**.
   La batería alimenta la etapa de motores; USB permite configurar la ESP32.
4. Pulsa una vez **Avanzar 1 segundo**. El botón queda bloqueado mientras la
   orden está pendiente o ejecutándose. **Detener** envía una parada; D48
   también desactiva los drivers desde el propio robot.

Una vez configurado, las órdenes viajan por Wi-Fi aunque el cable USB siga
conectado. El USB del robot se puede retirar si la alimentación de batería
mantiene encendida la ESP32. La webcam conserva su propia conexión USB al Mac.

La clave Wi-Fi se introduce localmente y se guarda en la memoria del ESP32 para
su conexión posterior. El servicio Python no la guarda ni la incluye en errores.
El enlace del Mac conserva únicamente IP, identidad del robot y una credencial
de control en `simulation/.runtime/robot-link.json`, excluido de Git y con
permisos 0600. Ese archivo no se comparte con otros equipos. Si cambia la IP,
vuelve a enlazar por USB. La contraseña se limpia del formulario al aceptar
la configuración.

## Duración y comportamiento

El ESP32 utiliza un [temporizador local](https://docs.espressif.com/projects/esp-idf/en/v4.4.6/esp32s3/api-reference/system/esp_timer.html)
para poner ambos PWM a cero a los 1000 ms nominales, con frenado activo. El
temporizador es independiente del procesamiento HTTP y del navegador. No se
usa una espera de un segundo en Python para ordenar después la parada. No es
una medición de precisión temporal con instrumental.

Cada pulsación tiene un identificador único. El firmware reconoce los ocho
últimos identificadores para evitar repetir una orden recibida de nuevo. El
servicio no reintenta automáticamente movimientos cuando falta confirmación.
El firmware arranca con ambos motores y la turbina apagados, y solo acepta en
esta prueba la combinación 100/100/1000. No incluye PID, navegación ni encoders.

La cifra de latencia del panel corresponde al recorrido de una consulta de
estado por HTTP; no mide la latencia completa desde una imagen hasta un motor.

## Firmware y reproducción

El programa está en
[RobotRacers_Prueba_WiFi.ino](../firmware/RobotRacers_Prueba_WiFi/RobotRacers_Prueba_WiFi.ino).
Se compiló y cargó con Arduino-ESP32 **2.0.14**, utilizando:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc \
  --build-path simulation/.runtime/firmware-wifi-build \
  firmware/RobotRacers_Prueba_WiFi
arduino-cli upload --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc \
  --port /dev/cu.usbmodem101 \
  --input-dir simulation/.runtime/firmware-wifi-build \
  firmware/RobotRacers_Prueba_WiFi
```

El puerto puede cambiar. En este Mac, si `arduino-cli` no está en PATH, se
encuentra en `/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli`.
El firmware previo de secuencias permanece intacto en
[la referencia REV3.11](../firmware/reference/RobotRacers_Recta_Encoders_REV3.11.ino).

Se conserva la inicialización de U11 a 6 V, con SDA41/SCL40 y dirección I2C
0x74, y las polaridades verificadas de la PCB:

| Señal | Izquierdo | Derecho |
|---|---|---|
| PWM, 20 kHz y 8 bits | GPIO47 | GPIO8 |
| Dirección de avance | GPIO5 alto | GPIO9 bajo |
| Activación del driver | GPIO6 | GPIO10 |
| Fallo del driver | GPIO7 | GPIO11 |

## Comprobación realizada

- Compilación real: 745009 bytes de programa y 47880 bytes de RAM.
- Carga por USB y respuesta JSON de la ESP32 confirmadas.
- Configuración de la red introducida por el usuario directamente en el panel;
  enlace Wi-Fi y disponibilidad de los drivers comprobados.
- El usuario pulsó el botón y confirmó que **ambos motores se movieron y se
  detuvieron**. El estado recibido registró el identificador de esa orden y
  `stop_reason: completed`.
- Las pruebas automatizadas del enlace usan dispositivos y red simulados:
  comprueban identidad, configuración, ausencia de reintentos, bloqueo de
  órdenes simultáneas, parada y protección de credenciales. No accionan motores.

Para repetir las pruebas del enlace:

```sh
.venv/bin/python -m pytest simulation/tests/test_webcam_robot_link.py -q
```
