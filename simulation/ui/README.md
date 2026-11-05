# RobotRacers Studio

Interfaz local de simulación. La escena usa el visor W3D oficial de Webots y el estado proviene del servicio; no se generan movimientos ni telemetría de demostración.

```sh
npm ci
npm run dev
npm test
npm run build
```

El desarrollo escucha en `127.0.0.1:5173` y reenvía `/api`, WebSocket y `/webots` al servicio en `127.0.0.1:8000`. En producción, FastAPI sirve `dist/`. Los recursos oficiales de Webots deben estar disponibles localmente en `/webots/wwi/`, incluidas sus dependencias transitivas; no se usan bibliotecas ni fuentes de un CDN durante la ejecución.

- La velocidad se puede cambiar en marcha. El tamaño del entorno, las condiciones físicas y los destinos requieren detener el recorrido.
- Los destinos se arrastran sobre el mapa de pasillos. Las flechas del teclado también los desplazan. El servicio valida la posición y calcula la trayectoria.
- Orbital y cenital modifican únicamente el punto de vista del visor; no alteran la cámara de navegación ni la física.
- La vista previa humana solicita imágenes a un máximo de 10 Hz y está fuera del lazo de control.
- Sin servicio o escena, se informa de la desconexión. No se sustituye la física por una animación.

Para diagnóstico del renderizador, `window.robotracersRendererStats()` devuelve fotogramas dibujados, frecuencia de dibujo y antigüedad del último fotograma. Lee el contador real del visor local (`robotRacersRenderStats`), actualizado después de `WrenRenderer.render`, no ciclos de `requestAnimationFrame`. Publica una muestra por segundo en `/api/client-metrics`; las muestras distinguen una vista visible de una pestaña oculta o la vista de cámara. No forma parte de la interfaz principal ni del control del robot.
