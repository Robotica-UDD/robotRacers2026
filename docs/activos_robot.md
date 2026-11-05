# Activos visuales del robot

El modelo de la simulación se reconstruye a partir de la **PCB RevQ**, no del
README histórico de selección de componentes ni del ensamblaje Fusion RevN.
RevN todavía incluye la IMU, los interruptores y los inductores que se retiraron.

## Geometría y escala

- La entrada principal es `simulation/assets/sources/pcb/RobotRacers_RevQ.brd`.
  Incluye posiciones, rotaciones, huellas y agujeros del diseño final. La lista
  comercial se conserva en `BOM_RevQ.csv`.
- PCB de **130 × 70 × 1,6 mm**, recorte central de **32 mm** y diez agujeros
  mecánicos. La superficie inferior está a 4 mm y la superior a 5,6 mm en el
  marco visual. Las perforaciones de conectores/interruptores de potencia más
  grandes también están representadas.
- Los **210 componentes montados** se colocan en Top. C5/C6 no se montan;
  J40/J41 son pads para cableado, no cuerpos de interruptor. U30, S5 y S6 no se
  incorporan. Los inductores L11/L20 corresponden al IHLP4040DZ de RevQ.
- Las envolventes electrónicas, terminales, conectores y componentes se
  reconstruyen de forma dimensional. No se presentan como una conversión
  exacta de todos los modelos STEP de los fabricantes.
- Las ruedas son objetos separados, de diámetro **23 mm**, anchura **40 mm** y
  eje local Y. Incluyen neumático, rin, taladro de 3 mm y tornillo de fijación.
  La rotación debe proceder de las articulaciones físicas del simulador.
- Los motores representan los **Pololu #5178**, con carcasa del encoder trasero,
  reductora, eje y cables JST SH de seis conductores. Las dimensiones nominales
  del motor proceden de Pololu; los soportes siguen el plano 1089 y la ubicación
  de los agujeros de RevQ.

Los ejes visuales están en `X=-0,01315 m`, `Y=±0,095 m`, `Z=0,0115 m` respecto
al centro de la PCB. El modelo físico sitúa el origen en el eje de tracción;
por ello el cuerpo se inserta con una traslación `0,01315 0 0`. Las ruedas se
insertan en sus articulaciones con el centro local en cero.

## Partes provisionales

La batería, su montaje, la turbina 1020, el impulsor, el patín y los recorridos
de cable se ilustran de manera dimensional y **no se consideran mediciones del
robot ensamblado**. La batería visual mide 36 × 20 × 13 mm y se coloca elevada
para dejar espacio sobre los componentes; no determina la masa de la física.

Los dos sensores Pololu #4401 se representan mirando al suelo. Son sensores de
reflectancia, no sensores de distancia de largo alcance. La regleta delantera
es una representación provisional de una regleta personalizada de 16 canales;
el modelo Pololu #4415 es solo la referencia eléctrica/mecánica de partida.
La placa real de la regleta y su multiplexor aún no se han identificado.

No se infieren masa total, centro de gravedad, tensor de inercia, fricción ni
fuerza de succión a partir de estos volúmenes visuales. Esos parámetros están
separados en la configuración física y requieren calibración.

La plataforma y la textura AprilTag se incorporan desde el mundo/robot de la
simulación; no están horneadas en esta malla, para que puedan retirarse.

## Archivos de integración

En `simulation/assets/robot/`:

- `body.inc.wbt` y `wheel.inc.wbt`: nodos `Group` de Webots con materiales PBR.
  Se insertan como texto dentro del PROTO del robot, sin cabecera adicional.
- `body.wrl` y `wheel.wrl`: la misma geometría con cabecera Webots R2025a.
- `body.obj/.mtl` y `wheel.obj/.mtl`: alternativa intercambiable para inspección
  y otros motores gráficos, conservando metros y colores.
- `pcb_top.png`: textura de fabricación derivada de la serigrafía y pads RevQ;
  se aplica máscara verde conforme a la configuración de fabricación guardada.
- `preview.png/.blend`: escena de inspección del activo, sin comportamiento ni
  colisiones del simulador. No es una captura que certifique la dinámica.

El material de la PCB en el include usa la URL
`../../assets/robot/pcb_top.png`, relativa a `simulation/webots/protos/`.
Si se inserta en otra ubicación, debe ajustarse esa URL. Los archivos OBJ usan
la textura relativa al propio directorio `robot/`.

`simulation/assets/manifest.json` registra límites geométricos, componentes,
procedencia, archivos, huellas SHA-256 y todas las aproximaciones. La malla
visual **no contiene `Solid`, `Physics` ni `boundingObject`**. Las colisiones y
la inercia pertenecen a la definición física independiente del robot.

## Regeneración

```sh
python3 simulation/tools/build_assets.py
```

El generador utiliza la biblioteca estándar de Python. Pillow se necesita solo
si se vuelve a hornear la textura de la PCB. Los activos ya generados no lo
necesitan en ejecución. Si faltan las entradas locales, puede indicarse la
carpeta original mediante `--source-root`.

Para repetir la inspección visual con Blender instalado:

```sh
"/Applications/Blender.app/Contents/MacOS/Blender" --background \
  --python simulation/assets/preview_scene.py
```

## Procedencia y licencias

- La reconstrucción paramétrica y los archivos originales de este proyecto se
  publican bajo la licencia MIT del repositorio.
- El STEP del ESP32 utilizado como referencia se conserva con la licencia de
  bibliotecas KiCad y su excepción de uso en diseños, en
  `sources/reference/LICENSE_Espressif_KiCad.md`. No se sustituye por MIT.
- El STEP Pololu 1089 no se redistribuye: no se encontró una licencia explícita
  para hacerlo. `Pololu1089_reference.json` conserva la fuente y las dimensiones
  empleadas para la reconstrucción original.
- Las dimensiones del motor proceden de
  [Pololu #5178](https://www.pololu.com/product/5178/specs), las del soporte de
  [Pololu 1089](https://www.pololu.com/product/1089), las ruedas de
  [Ja-Bots 23×40](https://ja-bots.com/producto/llantas-seguidor-de-linea-largas-23x40-negro/)
  y los sensores de [Pololu #4401](https://www.pololu.com/product/4401).

El modelo anterior `/Downloads/rueda.stl` no se reutilizó: su envolvente medida
es de aproximadamente 65 × 65 × 5 unidades, incompatible con las ruedas actuales.
