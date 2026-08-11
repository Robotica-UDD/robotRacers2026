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

## Bitácora de decisiones

### Drivers de tracción

Los ProFast seleccionados son motores de 12 V y tienen una corriente de stall indicada de 1,2 A cada uno. Eso deja una demanda teórica de 2,4 A para los dos motores, pero no queríamos elegir un driver simplemente porque alcanza ese número. En aceleraciones, frenadas y cambios de dirección aparecen transitorios, así que también importan la caída de tensión, la temperatura y el margen de corriente.

Comparamos varias alternativas:

- **TB6612FNG:** es común y fácil de encontrar, pero queda demasiado cerca del límite de corriente y tiene más pérdidas.
- **L298N:** fue descartado por su baja eficiencia y la caída de tensión del puente.
- **DRV8421 / DRV8848:** tienen capacidad suficiente, pero su resistencia del puente es mayor para esta aplicación.
- **MAX22203:** es un driver dual compacto, aunque concentra la disipación de ambos motores en un solo encapsulado.
- **TB67H420FTG:** tiene un buen historial en Robotrace, pero está sobredimensionado para nuestros motores y complica más el diseño.

Elegimos **2 × DRV8874PWPR**, uno por motor. La combinación de baja resistencia del puente, margen de corriente, medición de corriente y protecciones nos pareció más importante que ahorrar un integrado. Además, repartir la disipación permite ubicar cada driver cerca de su motor y aislar mejor una eventual falla.

Para las primeras pruebas usaremos módulos **Pololu 4035**. En la PCB final integraremos directamente los DRV8874 para reducir tamaño, peso y conexiones.

El modo de control elegido para partir es **PH/EN**: una señal PWM regula la potencia y otra define la dirección. IN/IN queda como alternativa si más adelante necesitamos controlar explícitamente los estados coast y brake.

### Alimentación y PCB

La primera idea era separar completamente las tierras digital y analógica. La descartamos: preferimos un plano GND único, continuo y de baja impedancia, pero con los caminos de potencia separados físicamente de las señales sensibles.

La razón es que los motores y la turbina generan corrientes pulsantes. Si esas corrientes comparten el mismo recorrido que el retorno del sensor, pueden aparecer cambios de referencia y ruido en las mediciones. Por eso los drivers, los reguladores y sus capacitores deben quedar agrupados, mientras que el XLINE y sus señales deben mantenerse alejados de los nodos de switching.

La distribución prevista es:

- `VBAT`: motores de tracción.
- `3V3_MAIN`: ESP32-S3 y lógica.
- `5V_SENSOR`: XLINE 16 V2.
- `FAN_V`: turbina.

La PCB será de 4 capas, con una capa interna dedicada a GND. En el layout habrá que cuidar especialmente los capacitores del DRV8874, el PowerPAD, las vías térmicas y los loops de los buck. También dejaremos test points para las alimentaciones, GND y las señales de corriente.

El XLINE es un punto que todavía requiere atención: funciona a 5 V y sus salidas pueden llegar a 5 V, mientras que el ESP32-S3 trabaja con lógica de 3,3 V. Antes de cerrar el esquemático tendremos que definir la adaptación de nivel de la salida digital y el acondicionamiento de la salida analógica.

### Turbina

Partimos mirando motores 8520 porque aparecen mucho en robots con turbina. Después de comparar referencias y revisar el ATLAS de Exotic Team MX, nos inclinamos por un **brushed coreless 1020**: sigue siendo compacto, tiene más margen mecánico y permite usar una turbina centrífuga cercana a 32 mm.

Para el accionamiento no necesitamos otro puente H, porque la turbina solo girará en una dirección. La arquitectura elegida es un **MOSFET N-channel low-side** con PWM y diodo de recirculación. El AO3400A apareció como candidato para prototipar, pero el MOSFET final depende de la corriente real del motor.

La alimentación de la turbina tendrá un rail propio. Una LiPo 3S puede llegar a 12,6 V, por lo que no conectaremos directamente un motor pensado para aproximadamente 7,4 V. Primero necesitaremos un buck con margen para la corriente de arranque y luego el MOSFET para regular la potencia.

El impeller no se montará directamente sobre el eje. Primero hay que confirmar el diámetro real del eje para diseñar un hub y balancear el conjunto. El cutout inicial de la PCB, si el impeller atraviesa la placa, será de aproximadamente 33,5–34 mm.

La idea es que la turbina sea un módulo reemplazable: soporte, shroud, impeller y cámara de succión deberían poder modificarse sin rediseñar toda la PCB. El faldón también será importante, porque la fuerza de succión depende tanto de la presión generada como de las fugas contra la pista.

Para prototipos podemos imprimir el soporte y el shroud, pero no queremos asumir que PLA-CF sea suficiente para el impeller final a decenas de miles de rpm. Antes de subir la velocidad habrá que revisar concentricidad, balanceo y contención. También debemos mantener la turbina alejada de la IMU o aislarla mecánicamente para que la vibración no afecte el control.

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
