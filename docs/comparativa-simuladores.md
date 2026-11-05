# Elección del entorno — 5 de noviembre de 2026

Equipo comprobado: MacBook Pro Apple M5, 10 núcleos CPU/GPU, 24 GB y macOS 26.6.2. Unity 6000.5.8f1 y Blender 5.2.1 estaban instalados. Esta comparación evalúa compatibilidad e integración; el rendimiento medido se documenta por separado.

**Elegido: Webots R2025a + Python/OpenCV + interfaz web local.** Cámaras, motores, encoders y articulaciones integrados, física ODE y licencia Apache 2.0. El instalador oficial es universal Apple Silicon. La interfaz emplea el visor W3D con presentación y controles propios.

| Alternativa | Gratuidad / compatibilidad | Evaluación |
|---|---|---|
| Webots | Apache 2.0, Mac universal | Solución robótica integral, ROS 2 oficial |
| Gazebo Jetty | Apache 2.0, ARM por Homebrew; limitaciones de GUI Mac | Fuerte en Linux/ROS 2; más fricción local |
| MuJoCo | Apache 2.0, macOS/Python | Excelente física; más infraestructura de aplicación |
| Unity + PhysX | ARM/Metal, gratuidad condicionada a plan y uso | Gran presentación; mayor integración robótica |
| Unity + MuJoCo | Complemento oficial macOS; condiciones Unity | Híbrido sólido; fijar compatibilidad y único motor físico |
| Godot + Jolt | MIT, Apple Silicon | Producto abierto, mayor desarrollo de sensorística |
| CoppeliaSim Edu | Gratuito para usos educativos permitidos, ARM | Completo; restricciones educativas para otros usos |
| PyBullet | Abierto zlib | Útil en pruebas; comprobar instalación ARM y crear interfaz |
| Genesis | Abierto, soporte Apple Silicon actual | Investigación; requiere aplicación y puentes propios |
| Unreal | ARM, gratuidad condicionada | Mayor consumo y complejidad que esta escena requiere |
| Isaac Sim | Windows/Linux y GPU NVIDIA compatibles | No ejecutable localmente en este Mac |
| ManiSkill/SAPIEN | Mac CPU/renderizado con configuración Vulkan | Investigación; backend y configuración adicionales |
| CARLA | Conducción, plataformas Windows/Linux y GPU elevada | Especializado y desproporcionado |
| MORSE | Cadena Blender/Python antigua | No elegido para un desarrollo nuevo |

## Fuentes primarias

- Webots: https://github.com/cyberbotics/webots/releases/tag/R2025a
- Licencia: https://github.com/cyberbotics/webots/blob/master/LICENSE
- Visor: https://github.com/cyberbotics/webots/blob/master/docs/guide/web-streaming.md
- ROS 2: https://github.com/cyberbotics/webots_ros2
- Gazebo: https://gazebosim.org/docs/jetty/install_osx/
- MuJoCo: https://github.com/google-deepmind/mujoco
- Complemento Unity: https://mujoco.readthedocs.io/en/stable/unity.html
- Unity: https://unity.com/products
- Godot/Jolt: https://docs.godotengine.org/en/4.6/tutorials/physics/using_jolt_physics.html
- Licencia Godot: https://godotengine.org/license/
- CoppeliaSim: https://www.coppeliarobotics.com/
- PyBullet: https://pypi.org/project/pybullet/
- Genesis: https://genesis-world.readthedocs.io/en/v1.2.1/user_guide/overview/installation.html
- Unreal: https://dev.epicgames.com/documentation/en-us/unreal-engine/macos-development-requirements-for-unreal-engine
- Licencia Unreal: https://www.unrealengine.com/license
- Isaac Sim: https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html
- ManiSkill: https://github.com/mani-skill/ManiSkill/blob/main/docs/source/user_guide/getting_started/macos_install.md
- CARLA: https://github.com/carla-simulator/carla
- MORSE: https://www.openrobots.org/morse/doc/stable/user/installation.html

## Límites de fidelidad

La localización procesa píxeles y encoders cuantizados. La pose verdadera no alimenta navegación. Masa, fricción, pesos y succión requieren identificación real. Fidelidad visual no equivale a dinámica calibrada.

La comparación cubre alternativas generales y especializadas pertinentes, sin atribuir automáticamente funciones experimentales de una rama de desarrollo a versiones estables.
