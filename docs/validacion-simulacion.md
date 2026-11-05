# Validación de la simulación

Ensayo local del **5 de noviembre de 2026**, Mac Apple M5, 24 GB, macOS 26.6.2,
Webots R2025a y Python 3.12.14. La física y el control ensayados corresponden a
`bd27319`; los hashes de los archivos de referencia están en
`simulation/validation/entorno.json`.

## Recorridos y detenciones

Se registraron **600,75 segundos continuos**, 5.739 muestras y **seis misiones
A → B → C → D completas**, sin fallos ni contactos con paredes. Se ensayaron:

| Entorno | Escala | Consigna lineal | Misiones completas |
|---|---:|---:|---:|
| Normal | 1,0 | 0,30 m/s | 2 |
| Reducido | 0,8 | 0,15 m/s | 2 |
| Ampliado | 1,5 | 0,50 m/s | 1 |
| Consigna alta | 1,0 | 1,50 m/s | 1 |

La consigna alta no significa que se mantuvieran 1,50 m/s: los tramos son cortos
y el perfil reduce velocidad antes de cada punto. La dinámica también puede
producir transitorios alrededor de una consigna. Las llegadas asentadas quedaron
a un máximo de **10,21 mm** del destino estimado, dentro del margen de 18 mm.
Los giros de 90° y 180° forman parte de estas misiones. El robot espera quietud
antes de la acción siguiente; cruzar una coordenada en movimiento no acredita
una llegada.

## Mediciones

Los errores comparan estimación y posición de referencia al mismo tiempo
simulado. La posición verdadera no alimenta la navegación. El percentil 95
indica el valor que no supera el 95 % de las muestras.

| Medida | Mediana | Percentil 95 | Máximo |
|---|---:|---:|---:|
| Error de posición estimada | 0,38 mm | 0,84 mm | 2,64 mm |
| Error de orientación | 0,41° | 1,72° | 4,37° |
| Imagen disponible → orden aplicada | 23,40 ms | 43,64 ms | 95,67 ms |
| Procesamiento de visión | 3,32 ms | 3,89 ms | 36,30 ms |
| Cámara procesada por segundo real | 42,73 | 44,10 | 44,45 |
| Fotogramas dibujados del visor visible | 49,24 | 51,18 | 52,27 |

Se cumplieron los criterios de posición/orientación en percentil 95 y latencia
menor a 80 ms en percentil 95. No se afirma un límite absoluto de 3° u 80 ms:
los picos de la tabla lo superan.

**Rendimiento pendiente de optimizar:** no se alcanzaron 50 imágenes/s de cámara
ni 60 fotogramas/s del visor. El factor de tiempo real mediano fue **0,855**:
la simulación avanza aproximadamente 0,855 segundos por segundo real. Los pasos
siguen siendo 2 ms de física y 10 ms de control simulado. El reloj de la interfaz
se identifica como tiempo simulado.

El visor se midió sobre 2.325 muestras válidas del navegador integrado, visible,
con área de 319×853 píxeles. Safari se verificó visualmente en formato de
escritorio, pero suspendía los temporizadores al permanecer oculto; sus muestras
ocultas se excluyeron. Los FPS del navegador integrado no son una medición de
Safari a pantalla completa ni una promesa para otras resoluciones.

La cámara nativa final es **1920×1440**. Se aumentó desde la resolución inicial
para conservar detección del marcador de 40 mm al ampliar el laberinto.

## Fallos provocados

Siete condiciones comprobadas sobre Webots y sus imágenes reales renderizadas:

- Oclusión de 100 ms: continúa mediante encoders.
- Oclusión de 950 ms: pausa y frena.
- Recuperación visual: permanece en pausa hasta recibir una orden nueva.
- Imagen con marca temporal atrasada 900 ms: se rechaza y se pausa.
- Pausa de misión: detiene ruedas.
- Parada de misión: detiene ruedas.
- Servicio Python suspendido: el controlador local vence la orden a 250 ms;
  45 muestras independientes posteriores registran PWM cero. La inercia física
  puede mantener velocidad residual durante el frenado.

La oclusión desplaza únicamente el marcador bajo el suelo mediante un campo
público del PROTO. No se desplaza el robot para ejecutar rutas. La prueba del
watchdog suspende solo el servicio de la sesión del simulador; nunca un
programa ni una placa físicos del usuario.

## Instalación y comprobaciones

- **52 pruebas Python/C++**, incluida compilación de la biblioteca de motor,
  calibración desde píxeles, signos de encoder, límites, envejecimiento de datos,
  planificación, separación de evaluación y adaptadores.
- **8 pruebas de interfaz** y construcción de producción correctas.
- **28 comprobaciones geométricas** de escala, triángulos y formato correctas.
- **168 recursos del visor y seis texturas** servidos localmente con hashes
  verificados; sin fallos de integridad.
- Arranque nuevo comprobado con proxies HTTP/HTTPS externos dirigidos a un
  puerto local sin servicio y exclusión de localhost. Servicio, controlador y
  localización arrancaron. La red física del Mac no se apagó. La política del
  documento bloquea recursos de Internet y el mundo usa archivos locales.
- Orbital, Cenital y Cámara revisados visualmente; texturas PCB/AprilTags
  correctas. Interfaz ajustada también al panel estrecho del navegador.

## Repetición

Desde la raíz del repositorio, después de instalar dependencias:

```sh
.venv/bin/python -m pytest
cd simulation/ui
npm test
npm run build
```

En otra terminal, con la aplicación arrancada:

```sh
.venv/bin/python simulation/tools/validate_live.py --seconds 600
.venv/bin/python -m simulation.tools.validate_offline
```

Para repetir también el fallo del servicio, cerrar primero la aplicación y
arrancar expresamente el canal de telemetría de prueba:

```sh
ROBOTRACERS_TEST_TELEMETRY_PORT=9203 ./simulation/run.command
# En otra terminal, desde la misma raíz:
.venv/bin/python -m simulation.tools.validate_faults --watchdog
```

Estos comandos mueven únicamente el robot simulado. Mantener la vista 3D visible
para medir el renderizador. No ejecutar benchmark y pruebas de fallos a la vez.
Los resultados CSV/JSON se guardan en `simulation/validation/`.

## Fidelidad

La cámara simulada tiene distorsión y exposición ideales. La masa, distribución,
rozamientos y succión requieren medición real. La validación demuestra que la
aplicación funciona y que el control cierra el lazo en este modelo; no acredita
estos errores ni tiempos sobre el robot fabricado. Las limitaciones mecánicas
conocidas se registran en `activos_revision_geometrica.md` y la calibración
pendiente en `calibracion-fisica.md`.
