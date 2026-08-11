# RobotRacers 2026

Robot velocista de seguimiento de línea, con dos ruedas y turbina de succión. Este repositorio contiene la PCB, el firmware, las piezas mecánicas y las pruebas del proyecto. (si no estan los archivos estan en producción xd)


## Componentes elegidos

| Bloque | Componente | Enlace |
| --- | --- | --- |
| Controlador | ESP32-S3-WROOM-1 | [JLCPCB](https://jlcpcb.com/partdetail/JLCPCBAssembly-ESP32_S3_WROOM1/C9900171795) · [datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf) |
| Sensor de línea | XLINE 16 V2, salida digital y analógica | [JSumo](https://www.jsumo.com/xline-16-line-sensor-board-digital-v2) |
| Motores de tracción | 2 × ProFast 12 V / 5900 rpm | [JSumo](https://www.jsumo.com/profast-12v-5900rpm-fast-gearmotor) |
| Drivers de ruedas | 2 × DRV8874PWPR, uno por motor | [JLCPCB](https://jlcpcb.com/partdetail/TexasInstruments-DRV8874PWPR/C1855818) · [TI](https://www.ti.com/product/DRV8874/part-details/DRV8874PWPR) |
| Control de ruedas | PH/EN: PWM + dirección por motor | [Pololu 4035 como módulo de pruebas](https://www.pololu.com/product/4035) |
| PCB | 4 capas, con plano GND continuo | — |
| Batería | LiPo 3S | Modelo pendiente |

## Decisiones principales

### Drivers de tracción

Comparamos TB6612FNG, L298N, DRV8421/DRV8848, MAX22203 y TB67H420FTG. Elegimos **2 × DRV8874PWPR** porque entregan margen de corriente, tienen baja resistencia del puente, medición de corriente y permiten repartir la disipación entre ambos lados.

Para las primeras pruebas usaremos módulos **Pololu 4035**. En la PCB final integraremos directamente los DRV8874 para reducir tamaño, peso y conexiones.

### Alimentación y PCB

La placa tendrá un plano GND común y separará físicamente los caminos de potencia de las señales del sensor. El XLINE trabaja a 5 V y sus salidas pueden llegar a 5 V, por lo que necesitaremos adaptación de nivel antes de conectarlo al ESP32-S3.

La distribución prevista es:

- `VBAT`: motores de tracción.
- `3V3_MAIN`: ESP32-S3 y lógica.
- `5V_SENSOR`: XLINE 16 V2.
- `FAN_V`: turbina.

### Turbina

La dirección elegida es un motor **brushed coreless 1020**, con impeller centrífugo de aproximadamente 32 mm y un MOSFET N-channel low-side controlado por PWM. El motor exacto, el MOSFET, el buck de `FAN_V` y el diseño del impeller todavía no están cerrados.

El impeller no se montará directamente sobre el eje. Primero hay que confirmar el diámetro real del eje para diseñar un hub y balancear el conjunto. El cutout inicial de la PCB, si el impeller atraviesa la placa, será de aproximadamente 33,5–34 mm.

Referencia mecánica: [ATLAS de Exotic Team MX](https://exoticteam.com.mx/atlas).

## Pendientes

- Confirmar la variante exacta del ESP32-S3-WROOM-1.
- Medir corriente de arranque y stall de los ProFast.
- Diseñar la adaptación de nivel del XLINE.
- Comprar y medir el motor 1020: tensión, rpm, corriente y eje.
- Elegir el buck y el MOSFET de la turbina.
- Diseñar y balancear el impeller.
- Probar la cámara de succión y el faldón.

## Registro

Las decisiones se van a actualizar acá a medida que pasemos de la selección en datasheet a las pruebas de banco y de pista. Una pieza aparece como elegida solo cuando ya tenemos claro qué componente es y cómo lo vamos a validar.
