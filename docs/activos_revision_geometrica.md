# Revisión geométrica de la simulación

Esta revisión compara las mallas visuales de RevQ con las envolventes del
generador de Webots. El resultado es una comprobación de coherencia del modelo,
no una certificación de la placa fabricada ni un gemelo digital calibrado.
Las coordenadas siguientes usan el origen en el eje de las ruedas.

## Envolventes y separaciones

La envolvente principal abarca X=-52,85..83,15 mm, Y=±75 mm y Z=4..35,3 mm.
Contiene las reconstrucciones de PCB, motores, conectores, batería, cables y
regleta, además del portador blanco del marcador. Este último mide 52×52 mm;
el patrón codificado interior mide 40×40 mm. La envolvente llena conservadoramente
el recorte central y los espacios libres; no reproduce cada superficie.

| Relación nominal | Separación en el modelo |
| --- | ---: |
| Cara interior de rueda a borde lateral de PCB | 10 mm |
| Cara interior de rueda a carcasa del motor | 10 mm |
| Cara interior de rueda a soporte de motor | 9 mm |
| Fondo de batería a componente PCB más alto | 3,46 mm |
| Fondo de batería a parte superior del motor/encoder | 3,25 mm |
| Portador del marcador a carcasa de turbina, longitudinal | 2 mm |
| Portador del marcador a correa de batería, lateral | 2,5 mm |

Las holguras de batería se refieren al cuerpo elevado, no a sus cables. Los
cables se representan terminando cerca del XT30 y de sus conectores; sus
recorridos y longitudes no están medidos. Las cajas de motores y soportes no
se solapan con las cajas de componentes electrónicos. Tampoco hay solape de
las cajas del portador del marcador con las piezas visuales.

El patín provisional está delante, con contacto en X=65 mm y disco entre Z=0
y 2 mm, alineado con el apoyo físico de la simulación. Su lengüeta y fijación
son estimadas. El disco coincide con el cilindro físico de 9 mm de diámetro.
Ese cilindro y la envolvente principal no representan
exactamente la delgada unión del patín; las ruedas y el apoyo son los contactos
previstos con el suelo.

## Límites mecánicos pendientes de medir

### Huella para evitar obstáculos con la webcam

La navegación usa la huella convexa del ensamblaje alrededor del centro del
eje, en lugar de las esquinas de su caja rectangular. Se auditaron los vértices
de `body.obj` y `wheel.obj`, que coinciden con las inclusiones visuales de Webots
(96.420 y 14.520 vértices, respectivamente). El cuerpo se desplaza +13,15 mm en
X; las ruedas quedan en `(0, ±95, 11,5)` mm. La rotación del cilindro de colisión
no se aplica de nuevo a las mallas de las ruedas.

El portador blanco del marcador, de 52 × 52 mm, se genera aparte. La huella
resultante abarca X=−52,85..83,15 mm e Y=±115,9 mm; los tacos sobresalen 0,9 mm
por lado respecto al ancho nominal de 230 mm. Su radio máximo en una postura
es 115,983490 mm. Incluyendo cualquier posición de giro de las ruedas es
116,006415 mm, redondeado hacia arriba a **116,1 mm** para navegación.
`simulation/webcam/robot_footprint.py` conserva los vértices y deriva el radio.
Los extremos X e Y pertenecen a piezas distintas: combinar ambos en la esquina
de una caja había originado una reserva inexistente de 150 mm de radio.

La calibración de la webcam fija el centro entre ruedas y el frente real; no
se sustituye por la posición del marcador simulado. En la configuración física
comprobada el portador desplazado también queda dentro de ese círculo. Esta
comprobación corresponde al modelo reconstruido: accesorios, cables sueltos o
cambios de montaje todavía requieren comprobar el robot físico.

La separación entre ruedas de 190 mm se infiere de los 230 mm exteriores
indicados y los 40 mm de ancho nominal de cada rueda. Falta medirla entre los
centros reales. Con esa separación, el eje visual llega al plano interior del
rin, pero no se modela inserción axial dentro del rin. El desplazamiento del
cubo, la longitud de eje expuesta y el montaje real requieren mediciones antes
de emplear el modelo como comprobación de ensamblaje.

La plataforma del marcador es removible en el modelo, pero sus patas o fijación
no se han medido. La batería, el impulsor, la regleta personalizada, el patín,
la posición de los sensores y el cableado siguen siendo reconstrucciones
provisionales. Los orígenes de los sensores simulados no certifican la posición
exacta de las lentes del hardware.

## Tres candidatos entre cuerpos electrónicos paramétricos

| Piezas | Solape de las cajas en X×Y |
| --- | ---: |
| L11–C119 | 0,885×0,571 mm |
| L20–C210 | 0,161×0,745 mm |
| L20–C224 | 0,026×1,506 mm |

Estos valores provienen de cajas que incluyen terminales y soldadura dibujada.
Los modelos se reconstruyen con cuerpos nominales y no son los STEP exactos
de cada pieza. El solape de cajas no demuestra contacto material ni un problema
eléctrico en la placa real. Una intersección entre triángulos de esas mismas
aproximaciones tampoco certificaría la interferencia de las piezas físicas.
No se modificaron la PCB, las huellas ni las posiciones para ocultar estos
candidatos.

## Dinámica y repetición del informe

La masa total, la masa por rueda, el centro de masa, los momentos de inercia,
las fricciones y la fuerza de succión siguen siendo parámetros estimados. El
informe comprueba positividad y desigualdades triangulares de la inercia, pero
eso solo establece coherencia matemática; no sustituye una medición física.

Los límites numéricos, hashes de entrada y candidatos se conservan en
`simulation/assets/geometry_audit.json`. Para actualizarlo sin iniciar Webots
ni modificar una misión:

```sh
python simulation/tools/audit_assets_geometry.py
```

`simulation/assets/validation.json` comprueba por separado las 28 condiciones
de escala, fuentes, triángulos y compatibilidad de formato. Ninguno de los dos
informes acredita precisión dinámica del robot real.
