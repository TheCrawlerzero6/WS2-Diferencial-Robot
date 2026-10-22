# Robot diferencial — grupo 14

Proyecto de ROS 2 Jazzy y Gazebo Harmonic. El robot dibuja el número **4** y deja el rastro visible en Gazebo.

## Compilar

Desde la raíz del repositorio:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Si faltan dependencias, instálalas con `rosdep` (debe estar inicializado):

```bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
```

## Iniciar Gazebo

En una terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch bajana_solis_ws2_description gazebo.launch.py
```

Gazebo inicia el robot en `(0, 0, 0)` y muestra su recorrido. El lanzamiento activa el renderizado por software por defecto; para desactivarlo, añade `renderizado_software:=false`. Para iniciar sin ventana, añade `gui:=false`.

## Ejecutar el recorrido

En otra terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ws2_trajectory trayectoria_4 --ros-args -p use_sim_time:=true
```

El 4 se traza desde la posición inicial. El robot termina en `(0, -2)` respecto a ese origen y se detiene ahí; no regresa al inicio. Gazebo conserva el rastro hasta reiniciar la simulación. Si se pierde la odometría durante más de 5 segundos, el nodo detiene el robot. Para repetir el recorrido, reinicia Gazebo y vuelve a ejecutar el comando.

## Colores del modelo

| Parte | Color |
|---|---|
| Chasis | Azul `#2563EB` |
| Ruedas | Grafito `#20242C` |
| Soportes y LiDAR | Plateado `#AAB4C0` |
| Pieza frontal | Amarillo `#FACC15` |

## Pruebas

Las pruebas automáticas verifican el controlador en una simulación numérica; no ejecutan Gazebo:

```bash
colcon test --packages-select ws2_trajectory
colcon test-result --test-result-base build/ws2_trajectory --verbose
```

La gráfica y los resultados de simulación están en `docs/`.
