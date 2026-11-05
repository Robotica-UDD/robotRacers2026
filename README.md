# RobotRacers

Robot diferencial con PCB RevQ, dos motores Pololu #5178 de 6 V, ruedas de 23 × 40 mm, batería LiPo 2S y control ESP32-S3. Incluye simulación local con cámara cenital, localización AprilTag y corrección por encoders.

## Ejecutar

```sh
./simulation/setup.sh
./simulation/run.command
```

Abre http://127.0.0.1:8000. Iniciar ejecuta A → B → C → D en el laberinto de doble T. La velocidad puede cambiar en marcha; objetivos y tamaño se editan con el robot detenido. Hay vista orbital, cenital y cámara, y ajustes de masa, rozamiento y succión.

Webots R2025a realiza la física y genera las imágenes. Python estima la posición desde esas imágenes y los encoders. El control C++ aplica par a las ruedas; el visor web muestra esa simulación. La posición verdadera solo se utiliza para evaluar errores.

Para cerrar una sesión iniciada en Terminal, pulsa **Ctrl+C** en esa terminal.
Para mostrar también la escena 3D en la ventana de Webots, inicia una sesión con
`./simulation/run.command --visible-webots`. El modo habitual utiliza el visor
web y desactiva la vista local de Webots para ahorrar recursos; por eso esa
ventana puede indicar **No Rendering**, aunque la física y la cámara sigan
activas. El modo visible activa ambas vistas y puede consumir más recursos.

El lanzador cierra sus procesos de Webots y Python. Los registros están en
`simulation/.runtime/`. Si el puerto 8000 está ocupado, cierra la sesión anterior
antes de iniciar otra. Los requisitos de compilación y uso sin conexión se
explican en [Dependencias](docs/dependencias.md).

La validación completó seis misiones en diez minutos sin colisiones. Latencia
imagen → orden aplicada: percentil 95 de 43,64 ms. Cámara: mediana 42,73 imágenes/s;
visor visible: 49,24 fotogramas/s. Los objetivos de 50/60 siguen pendientes de
optimización. Consulta [resultados y límites](docs/validacion-simulacion.md).

El recuadro verde y la línea hacia el objetivo muestran el seguimiento del robot
en las vistas Orbital, Cenital y Cámara. Durante una misión la línea sigue el
objetivo activo; en espera sigue el destino seleccionado. [Cómo funciona el
seguimiento visual](docs/seguimiento-visual.md).

## Probar una webcam real

Para probar una webcam real sin simulador ni ROS 2, ejecuta
`./simulation/webcam/run.command`. Abre un panel independiente en
http://127.0.0.1:8010/ con detección del marcador y orientación del robot.
Consulta [montaje del marcador y prueba de cámara](docs/prueba-webcam.md).

La tarjeta **Robot** permite enlazar la ESP32 por Wi-Fi y ejecutar un único
avance con PWM 100/100 durante 1000 ms. La configuración inicial se envía por
USB desde el mismo panel. [Cómo configurar y probar el control Wi-Fi](docs/prueba-control-wifi.md).

## Referencia física

| Parámetro | Valor | Procedencia |
|---|---|---|
| PCB | RevQ, 130 × 70 × 1,6 mm | Archivos de fabricación |
| Tracción | 2 × Pololu #5178, 6 V, reducción 4,995:1 | Fabricante y montaje |
| Driver | 2 × DRV8874, PH/EN | PCB |
| Ruedas | 23 mm de diámetro, 40 mm de ancho | Ja-Bots |
| Encoder | 59,94 cuentas por vuelta de rueda | 12 × 4,995; ya incluye cuadratura |
| Separación entre centros | 190 mm | Inferida del ancho exterior; ajustable |
| Batería | LiPo 2S, 400 mAh | Usuario |
| Masa inicial | 180 g | Estimación editable |
| Contacto adicional | Patín deslizante | Usuario |
| Sensores laterales | Pololu #4401 de reflectancia | Selección del proyecto |

La dinámica es provisional hasta medir masa, centro de gravedad, fricción y respuesta de motores y turbina. La apariencia se reconstruye desde RevQ; geometrías dimensionales y estimadas se identifican en el manifiesto. La turbina comienza apagada.

## Documentación

- [Comparación de simuladores](docs/comparativa-simuladores.md)
- [Arquitectura y contratos](simulation/CONTRACT.md)
- [Modelos y procedencia](docs/activos_robot.md)
- [Revisión geométrica](docs/activos_revision_geometrica.md)
- [Calibración física](docs/calibracion-fisica.md)
- [ROS 2 y hardware](docs/ros2-y-hardware.md)
- [Validación y rendimiento](docs/validacion-simulacion.md)
- [PCB final](hardware/RevQ)
- [Firmware físico de referencia](firmware/reference)
- [README histórico](docs/historial/README-original.md)

El servicio Python separa cámara, navegación y transporte para incorporar una cámara real y conectar ROS 2 o la ESP32 mediante adaptadores. El simulador no programa el robot físico.

## Licencias

Código del proyecto: MIT. Webots y los modelos mantienen sus propias licencias. La licencia MIT del repositorio no reemplaza las condiciones de los fabricantes.

Los dos sensores laterales y la regleta provisional de 16 canales exponen
reflectancias normalizadas mediante `/api/state`. No participan en la navegación
por cámara de esta entrega. El multiplexor de la regleta no se añade a la PCB
principal.
