# RobotRacers 2026

Bitácora de diseño y construcción de nuestro robot velocista para 2026.

Este repositorio va a guardar todo lo que vayamos aprendiendo mientras pasamos de una idea a un robot que realmente pueda correr: esquemáticos, PCB, firmware, piezas mecánicas, pruebas y, sobre todo, las decisiones que hay detrás de cada componente.

La idea de esta bitácora es que alguien nuevo pueda llegar al proyecto y entender no solo **qué elegimos**, sino también **qué otras opciones miramos y por qué las dejamos fuera**. Muchas decisiones van a cambiar después de medir en el banco o probar en pista; por eso preferimos registrar el proceso en vez de fingir que todo estuvo claro desde el principio.

## En resumen

Estamos diseñando un robot diferencial de alta velocidad, con dos ruedas motrices, seguimiento de línea y una turbina de succión para aumentar el agarre en pista. El corazón del sistema será un ESP32-S3 que leerá los sensores, ejecutará el control y manejará los motores mediante PWM.

La arquitectura que tenemos al día de hoy es:

```text
LiPo 3S
  ├── motores ProFast de tracción
  │     └── 2 × DRV8874, uno por motor
  ├── buck de 3,3 V
  │     └── ESP32-S3 + lógica
  ├── buck de 5 V
  │     └── sensor XLINE 16 V2
  └── rail dedicado de la turbina
        └── motor 1020 + MOSFET N-channel + PWM
```

### Componentes elegidos hasta ahora

| Bloque | Elección actual | Estado |
| --- | --- | --- |
| Controlador | ESP32-S3-WROOM-1 | Elegido |
| Sensado de línea | XLINE 16 V2 | Elegido; falta resolver adaptación de niveles |
| Tracción | 2 × motor ProFast, 12 V, 1,2 A de stall | Elegido |
| Drivers de ruedas | 2 × Texas Instruments DRV8874PWPR, uno por motor | Elegido para la PCB final |
| Control de ruedas | Modo PH/EN: una señal PWM y una de dirección por motor | Elegido para partir |
| Turbina | Motor brushed coreless 1020, aproximadamente 7,4 V y 44.500 rpm | Candidato elegido; falta congelar modelo exacto |
| Impeller | Centrífugo, cercano a 32 mm | Dirección elegida; falta validar geometría |
| Driver de turbina | MOSFET N-channel low-side | Arquitectura elegida; MOSFET aún pendiente |
| PCB | 4 capas, con plano GND continuo | Decisión de diseño |
| Batería | LiPo 3S | Base de diseño actual |

## Bitácora de decisiones

### 1. La tracción: por qué terminamos mirando el DRV8874

Los ProFast seleccionados son motores de 12 V con una corriente de bloqueo de 1,2 A cada uno. Eso significa que los dos motores podrían exigir cerca de 2,4 A en una situación extrema, pero no queríamos elegir un driver solamente porque alcanza ese número en una tabla.

En un robot rápido importan mucho la resistencia del puente H, la caída de tensión, el calor, el frenado y la capacidad de medir lo que está pasando.

Barajamos varias opciones:

- **TB6612FNG:** es muy común y funcionaría en proyectos más tranquilos, pero queda demasiado cerca del límite de corriente y tiene más pérdida.
- **DRV8421 / DRV8848:** tienen capacidad, pero su resistencia del puente es bastante mayor para esta aplicación.
- **L298N:** quedó descartado por antiguo, ineficiente y con una caída de tensión demasiado alta.
- **MAX22203:** es una alternativa dual muy interesante y ahorra espacio, pero concentra más disipación en un solo encapsulado.
- **TB67H420FTG:** tiene un buen historial en Robotrace y fue usado en robots de alto nivel, aunque para nuestros motores está sobredimensionado y complica más el layout.
- **2 × DRV8874:** fue la opción que mejor equilibró eficiencia, margen y control independiente.

La decisión actual es usar **un DRV8874 por motor**. No necesitamos sus 6 A peak; lo interesante es que el motor queda lejos del límite, el puente tiene baja resistencia, cada lado puede medir y limitar corriente por separado y la disipación se reparte entre dos zonas de la placa.

Para una primera prueba de banco sí tiene sentido usar módulos **Pololu 4035**, que ya traen el DRV8874 y sus componentes externos. Para la PCB final preferimos integrar directamente los dos DRV8874: ocupa menos, pesa menos y elimina headers y conexiones innecesarias.

Referencias: [DRV8874 de Texas Instruments](https://www.ti.com/lit/ds/symlink/drv8874.pdf), [Pololu G2 High-Power Motor Driver 18v17](https://www.pololu.com/product/4035).

### 2. Cómo vamos a controlar las ruedas

El DRV8874 permite distintas formas de control. Las dos que más nos interesan son:

- **PH/EN:** una señal define la dirección y otra recibe el PWM.
- **IN/IN:** se usan dos entradas PWM para controlar con más detalle los estados de avance, retroceso, freno y coast.

Partiremos con **PH/EN** porque para dos ruedas diferenciales nos deja una interfaz clara y rápida:

```text
ESP32-S3                 DRV8874
GPIO PWM_L  -----------> EN_L
GPIO DIR_L  -----------> PH_L
GPIO PWM_R  -----------> EN_R
GPIO DIR_R  -----------> PH_R
```

La ventaja práctica es que cada motor necesita solo un canal PWM y una señal de dirección. Además, el comportamiento drive/brake debería darle más autoridad al control cuando el PID necesita bajar rápidamente la velocidad de una rueda.

### 3. La alimentación no se va a separar cortando la tierra

Al principio apareció la idea de separar alimentación digital y analógica como si fueran dos sistemas completamente independientes. En este robot eso no corresponde: el ESP32-S3-WROOM-1 se alimenta con 3,3 V y no tiene una alimentación analógica externa separada.

La solución será mantener un **plano GND único, sólido y continuo**, y separar físicamente los caminos de corriente. Los pulsos grandes de los motores y la turbina deben formar loops cortos, lejos de las rutas del sensor y del ADC.

De momento vemos al menos estos rails:

- `VBAT`: batería y potencia de los motores de tracción.
- `3V3_MAIN`: ESP32-S3 y lógica de 3,3 V.
- `5V_SENSOR`: alimentación del XLINE 16 V2.
- `FAN_V`: alimentación regulada de la turbina.
- `3V3_QUIET`: rama filtrada opcional para electrónica analógica sensible, si finalmente hace falta.

Un detalle importante que quedó registrado: el **XLINE 16 V2 trabaja a 5 V**, y sus salidas pueden llegar a 5 V. No vamos a conectarlas directamente al ESP32, cuya tolerancia de GPIO es menor. Habrá que incorporar adaptación de nivel y acondicionamiento para la salida analógica antes de congelar el circuito.

### 4. Decisión de la PCB

La dirección elegida es una PCB de **4 capas**:

```text
L1  Componentes, señales críticas y potencia local
L2  Plano GND continuo
L3  Distribución de VBAT, 5 V y 3,3 V
L4  Señales y pours auxiliares
```

La colocación también importa. Los drivers deberían quedar cerca de sus motores; el sensor y sus señales deben mantenerse alejados de los nodos de switching; y la antena del ESP32 debe quedar hacia un borde, respetando su keep-out.

Algunas reglas que queremos mantener desde el primer layout:

- capacitores de VM y charge pump del DRV8874 pegados al IC;
- PowerPAD y vías térmicas bien conectadas al plano GND;
- loops de switching de los bucks lo más pequeños posible;
- nodo SW de los reguladores pequeño y lejos de señales analógicas;
- test points para `VBAT`, `3V3`, `5V`, `FAN_V`, GND y mediciones de corriente;
- algunos resistores de 0 ohm para poder aislar ramas durante el debugging.

### 5. La turbina: de un 8520 genérico a un 1020 coreless

La referencia más útil que encontramos fue el ATLAS de Exotic Team MX, que usa un motor coreless 1020 con una turbina centrífuga de 32 mm y un faldón flexible. También hay otros robots de Robotrace que usan motores DC pequeños, 8520 y DRW002, así que no es una idea aislada.

Primero miramos el **8520** porque aparece mucho en proyectos de succión. Luego preferimos el **1020**: entrega más margen de torque, sigue siendo pequeño y coincide mejor con la referencia del ATLAS.

La especificación que estamos buscando es:

- motor brushed coreless 1020;
- aproximadamente 7,4 V;
- cerca de 44.500 rpm sin carga;
- eje liso, idealmente de 1,2 a 1,4 mm;
- sin engranaje de latón montado en el eje.

Todavía no congelamos el vendedor ni el modelo exacto. El nombre “1020” se usa para motores con distintos ejes, tensiones y corrientes, así que antes de diseñar el hub necesitamos una referencia concreta y medir el motor real.

### 6. Driver de la turbina: no necesitamos otro puente H

La turbina solo tiene que girar en una dirección. Por eso descartamos usar otro DRV8874 y preferimos un **MOSFET N-channel como interruptor low-side**, controlado con PWM desde el ESP32:

```text
FAN_V ─── motor 1020 ─── drain MOSFET
                           source ─── GND
ESP32 PWM ── resistencia ── gate
```

También necesitaremos un diodo de recirculación adecuado y una resistencia pull-down en el gate. El AO3400A apareció como buen candidato para prototipar porque está caracterizado a 2,5 V de gate, pero no está congelado: primero queremos conocer la corriente real de arranque y stall del motor elegido.

El PWM inicial que queremos probar está en el rango de **20–25 kHz**, con una rampa de arranque en vez de saltar de 0 a 100 %. La turbina tampoco tiene por qué estar siempre al máximo; la idea es que el firmware pueda ajustar la succión según la velocidad y la situación del robot.

### 7. La batería 3S obliga a tener un rail dedicado para el fan

Una LiPo 3S puede llegar a 12,6 V completamente cargada. Eso es demasiado para conectar directamente un motor 1020 pensado para aproximadamente 7,4 V.

Tampoco queremos “solucionarlo” simplemente limitando el duty-cycle: durante cada pulso el motor seguiría viendo el voltaje de la batería. La arquitectura preferida es:

```text
LiPo 3S → buck dedicado → FAN_V ≈ 7,4 V → MOSFET PWM → motor 1020
```

El buck de la turbina queda pendiente hasta conocer la corriente real del motor. Si el sistema de succión llega a acercarse a los 2,4 A de la referencia del ATLAS, un regulador de 3 A puede quedar demasiado justo por los transitorios de arranque.

### 8. Mecánica de la turbina y el agujero de la PCB

La fuerza de succión no la genera solamente la turbina: depende de la diferencia de presión y del área efectiva de la cámara. El faldón flexible es igual de importante, porque reduce las fugas de aire contra la pista.

La arquitectura mecánica que más nos convence es hacer un **módulo de succión reemplazable**, con soporte, shroud, impeller y cámara separados de la PCB. Así podemos probar impellers de 31, 32 o 33 mm sin tener que cambiar la placa completa.

Si el impeller de 32 mm atraviesa la PCB, el cutout inicial sería de aproximadamente **33,5–34 mm**. Es una medida de partida, no una dimensión final: la holgura aerodinámica fina debería definirla una pieza impresa o mecanizada, no el borde del FR4.

Para unir el motor al impeller no queremos imprimir un agujero diminuto directamente en el rotor. La idea es usar un **hub de precisión** —por ejemplo, de aluminio o latón— con un agujero calibrado para el eje, y luego fijar el impeller al hub. Un tornillo prisionero puede deformar un eje de 1 mm y agregar desbalance, por eso preferimos un ajuste adecuado con compuesto retenedor, siempre validando el balanceo.

El PLA-CF puede servir para prototipos del shroud y los soportes, pero no queremos asumir que sea seguro para el impeller final a decenas de miles de rpm. La prioridad será balance, concentricidad y contención física. El rotor debe probarse con protección y subiendo las rpm de manera progresiva.

También vamos a cuidar la IMU: una turbina desbalanceada puede meter vibración al giroscopio. El módulo debe quedar mecánicamente aislado y la IMU fuera del camino directo de esa vibración, aunque eso implique sacrificar un poco de masa o espacio.

## Pendientes antes de cerrar el diseño

- elegir y comprar la referencia exacta del motor 1020;
- medir tensión, corriente sin carga, corriente de arranque, stall y rpm reales;
- confirmar diámetro y longitud del eje;
- elegir el buck de `FAN_V` con margen para transitorios;
- cerrar el MOSFET y su disipación;
- definir el hub y el método de balanceo del impeller;
- probar el faldón y medir fugas de la cámara;
- diseñar la adaptación de nivel del XLINE hacia el ESP32;
- definir límites de corriente y lectura `IPROPI` de los DRV8874;
- hacer una primera prueba de banco con módulos Pololu 4035 antes de fabricar la PCB integrada.

## Referencias principales

- [Texas Instruments DRV8874](https://www.ti.com/lit/ds/symlink/drv8874.pdf)
- [Pololu 4035](https://www.pololu.com/product/4035)
- [Exotic Team MX — ATLAS](https://exoticteam.com.mx/atlas)
- XLINE 16 V2: referencia consultada en JSumo durante la selección del sensor
- [Espressif ESP32-S3](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/hw-reference/esp32s3/user-guide-devkitc-1.html)

> Esta bitácora representa el estado actual del diseño. Las decisiones eléctricas y mecánicas importantes se consideran cerradas solo después de probarlas, medirlas y documentar el resultado.
