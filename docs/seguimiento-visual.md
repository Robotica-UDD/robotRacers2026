# Seguimiento visual del robot

El visor del panel muestra un recuadro verde alrededor del robot, una retícula
en su centro y una línea hacia el destino. La letra del destino identifica el
extremo de la línea. Durante el recorrido se utiliza el objetivo activo; en
espera se utiliza el destino seleccionado en «Tu recorrido». La línea representa
la dirección al objetivo, mientras el planificador sigue los pasillos del mapa.

Está disponible en **Orbital, Cenital y Cámara**. Cambia con el movimiento,
los objetivos y el punto de vista. El dibujo deja pasar el ratón para conservar
la órbita, el zoom y los controles. En la vista cenital se conserva el encuadre
del laberinto al cambiar el tamaño de la ventana.

## Datos que dibujan el recuadro

- En Cámara, el centro y la orientación proceden del AprilTag detectado en ese
  fotograma. La envolvente conocida del robot se proyecta con la calibración
  óptica de ese mismo fotograma. El JPEG y sus metadatos viajan juntos; se
  compensa el espacio vacío que deja `object-fit: contain`.
- En Orbital/Cenital se proyecta la posición estimada por cámara y encoders con
  la cámara actual del visor, incluyendo cambios de órbita y zoom. Solo se lee
  el reloj y el punto de vista de Webots, nunca el nodo de posición verdadera
  del robot. La alineación temporal por encoders queda limitada a 80 ms.
- El recuadro abarca las dimensiones conocidas de la PCB, ruedas y piezas
  sobresalientes (`visual_bounds_m`). Es una reconstrucción del volumen a partir
  del marcador; no es una red neuronal que segmente la silueta del robot.
- Verde indica localización visual reciente. En las vistas 3D, ámbar indica
  predicción breve con encoders. Al perder localización o conexión, se oculta la
  posición y su línea; el destino fijo puede seguir visible.

Las anotaciones son SVG sobre el visor. No se incorporan a las imágenes que
procesa OpenCV, al mundo físico ni a la cámara de navegación. No cambian el
control, la planificación o la trayectoria. La ventana nativa de Webots muestra
el mundo; esta presentación de seguimiento pertenece al panel web.

## Validación

Se comprobaron las tres vistas, el cambio de objetivo activo A→B durante una
prueba en movimiento y 72 fotogramas seguidos con detección. No hubo contactos
con paredes. El informe adicional está en
`simulation/validation/tracking-overlay.json`; los resultados anteriores de
diez minutos no se sustituyen por esta prueba corta. Las pruebas geométricas
cubren ejes FLU, encuadres verticales, márgenes de imagen, datos atrasados y
recorte de líneas sin inventar un objetivo en el borde de la pantalla.

## Restauración al actualizar

El lanzador permite `--resume-session ruta/al/estado.json`, con un estado público
guardado del servicio. Recupera ajustes, destinos, intención de pausa y posición
observada una sola vez al iniciar la física. El estimador vuelve a localizar por
cámara y no acepta esa posición guardada como una medición. Una misión guardada
en marcha se recupera en pausa; al reanudar se planifica desde la observación
actual. El robot físico no participa en esta restauración.
