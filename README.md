# Ebzer AI — Industrial Robotic Smart Picking Platform

Aplicación de escritorio GTK3 para una plataforma industrial de **smart picking** con cámara ToF Scepter/Vzense, panel de operador touchscreen para Jetson Orin NX, detección 3D y simulación de pick robótico.

```
Ebzer AI LIVE UI → Cámara 3D → Detección → Posición XYZ → Robot / PyBullet
```

---

## Hardware requerido

| Componente | Detalle |
|---|---|
| Cámara 3D | Vzense / Scepter (probado con NYX650, IP `192.168.1.101`) |
| Computador | NVIDIA Jetson Orin (aarch64, Ubuntu 20.04) |
| Conexión | Cable Ethernet directo PC ↔ cámara |

---

## Arquitectura del software

```
Ebzer AI/
  banapick.py              ← launcher oficial de la app de operador
  banapick/
    __main__.py            ← permite ejecutar: python3 -m banapick
    ui/
      live_panel.py        ← UI industrial LIVE: sidebar, cámara, flow, box, métricas
  docs/
    BANAPICK_LIVE_UI_SPEC.md  ← especificación del diseño industrial LIVE

examples/
  ball_viewer.py        ← herramienta legacy/técnica: cámara real RGB + Depth + nube 3D
  ball_sim.py           ← Simulación PyBullet (gripper pick)
  smart_pick.py         ← herramienta legacy/técnica: visor multi-modo (rgb/depth/cloud/pick)
  robot_server.py       ← Simulador TCP del robot
  scepter_realtime_open3d.py  ← Nube de puntos en tiempo real
  picker/
    camera.py           ← Wrapper ScepterCamera (context manager)
    detect.py           ← RANSAC + DBSCAN: detección de objetos
    robot.py            ← MockRobot / TcpRobot (protocolo JSON)
```

`banapick.py` es la app principal que debes abrir para ver el panel industrial Ebzer AI.  
`examples/ball_viewer.py` y `examples/smart_pick.py` quedan como herramientas técnicas para cámara real, calibración, pruebas de nube 3D y robot.

### Por qué `PYTHONNOUSERSITE=1`

El SDK de Scepter usa librerías `.so` que están en `ScepterSDK/BaseSDK/AArch64/Lib/`.  
PyBullet se instala en `~/.local` (user site).  
Si ambos se cargan en el mismo proceso, hay segfault por conflicto de `libstdc++`.

**Solución**: los scripts de cámara se lanzan con `PYTHONNOUSERSITE=1`, y PyBullet en un subproceso separado sin esa variable.

---

## Instalación paso a paso

### 1. Clonar el repo

```bash
git clone https://github.com/eostos/3d-smart-picking.git
cd 3d-smart-picking
```

### 2. Clonar el SDK de Scepter (GitHub público, sin registro)

```bash
# Dentro de la carpeta del proyecto:
git clone https://github.com/ScepterSW/ScepterSDK.git
```

La estructura resultante debe ser:

```
3d-smart-picking/
  ScepterSDK/
    BaseSDK/
      AArch64/Lib/          ← libScepter_api.so  (Jetson / aarch64)
      Ubuntu/Lib/           ← libScepter_api.so  (PC x86-64)
    MultilanguageSDK/
      Python/
        API/
          ScepterDS_api.py
          ScepterDS_enums.py
```

> **Jetson Orin / aarch64** → usa `AArch64/Lib/` (ya configurado en `picker/camera.py`).  
> **PC Ubuntu x86-64** → cambia la línea en `camera.py`:  
> `"AArch64"` → `"Ubuntu"`

Documentación oficial: https://wiki.vzense.com  
Código fuente SDK: https://github.com/ScepterSW

### 3. Instalar dependencias del sistema

```bash
sudo apt update
sudo apt install -y \
    python3 python3-pip \
    python3-gi python3-gi-cairo gir1.2-gtk-3.0 \
    python3-opencv python3-open3d \
    libglib2.0-dev
```

### 4. Instalar dependencias Python

```bash
pip3 install numpy
```

> `opencv-python` y `open3d` vienen del apt en Jetson para evitar conflictos con las librerías del SDK.

### 5. Instalar PyBullet (para simulación)

PyBullet no tiene wheel para aarch64, hay que compilarlo:

```bash
sudo apt install -y build-essential python3-dev cmake
pip3 install --user pybullet
```

La compilación tarda ~10 minutos en el Jetson Orin. Al terminar verifica:

```bash
python3 -c "import pybullet; print('PyBullet OK')"
```

### 6. Verificar conexión con la cámara

```bash
ping 192.168.1.101
```

Asegúrate de que tu interfaz Ethernet esté en el mismo segmento (ej: `192.168.1.50`).

### 7. Crear carpeta de capturas

```bash
mkdir -p captures
```

---

## Uso

### App principal Ebzer AI LIVE

```bash
cd /home/ebenezer/Documents/3d-camera
PYTHONNOUSERSITE=1 /usr/bin/python3 banapick.py 192.168.1.101
```

También puedes abrirlo como módulo:

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 -m banapick 192.168.1.101
```

Esta pantalla abre el panel industrial y conecta el LIVE tab con la cámara real. Si la cámara no está disponible, el panel muestra el error dentro del área de video.

| Zona | Contenido |
|---|---|
| Sidebar | Live, Cameras, Calibration, Detection, Robot, Routing, Log / DB; cada pestaña cambia de vista |
| Top bar | Logo Ebzer AI, estado RUNNING/STOPPED, E-STOP, Reconnect y hora actual |
| Columna izquierda | Dos visores simultáneos: RGB real y Depth/3D; badges STEM UP / STEM DOWN |
| Columna central | Modelo activo, parte objetivo y stepper operativo con done/active/alarm/idle |
| Columna derecha | Box fill 4×3, contador 8 / 12, progreso y alerta |
| Bottom bar | Throughput, grasp success rate y rejections today |

**Controles LIVE principales:**

| Acción | Cómo |
|---|---|
| Detener algoritmo y cámara | Botón `E-STOP` |
| Volver a conectar después de E-STOP/error | Botón `Reconnect` |
| Ver RGB y Depth juntos | Ambos visores están visibles en la columna izquierda |
| Abrir nube 3D grande | Botón `Open 3D Large`; reemplaza los visores RGB/Depth por un solo feed 3D |
| Volver a RGB + Depth | Botón `Back to RGB + Depth` |
| Rotar nube 3D | Arrastra con mouse sobre el feed 3D grande |
| Zoom nube 3D | Rueda del mouse sobre el feed 3D grande |
| Graduar cantidad de puntos | Slider `Points` bajo el feed, de 30k a 500k puntos |
| Medir un pixel | Click en RGB o Depth |

**Referencia de altura/XYZ:**

Al hacer click, Ebzer AI muestra `X/Y/Z` en milímetros usando el frame de la cámara ToF:

| Valor | Referencia |
|---|---|
| `X` | desplazamiento lateral respecto al centro óptico |
| `Y` | altura en el frame de cámara respecto al centro óptico |
| `Z` | distancia/profundidad desde el sensor ToF hacia el objeto |

El dashboard muestra el modelo activo (`banana-hand-pose-v1`) y la parte objetivo (`crown point + tip point + stem orientation`).

### Herramienta técnica RGB + Depth + 3D

`examples/ball_viewer.py` queda como visor de diagnóstico separado para probar cámara, depth, nube 3D, clicks XYZ y simulación PyBullet.

| Panel izquierdo | Panel derecho |
|---|---|
| Imagen RGB con overlay de detección | Depth colormap (rojo=cerca, azul=lejos) |
| Medidas reales: ancho y alto en mm | Vista 3D rotable (botón "Cambiar a 3D") |

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 examples/ball_viewer.py 192.168.1.101
```

**Controles:**

| Acción | Cómo |
|---|---|
| Alternar Depth ↔ Nube 3D | Botón "Cambiar a 3D / Depth" |
| Rotar nube 3D | Drag con mouse en panel derecho |
| Zoom en nube 3D | Rueda del mouse |
| Ver XYZ de un pixel | Click en panel Depth |
| Verificar alineación RGB↔Depth | Click en panel RGB |
| Simular pick en PyBullet | Botón "Simular Pick" (activo al detectar objeto) |
| Guardar captura PNG | Botón "Guardar" |

**Densidad de la nube 3D:**

En `examples/ball_viewer.py`, ajusta `CloudRenderer.MAX_PTS` para permitir más puntos y `CloudRenderer.POINT_RADIUS` para engrosar cada punto. El valor actual usa `300_000` puntos máximos y radio `1` (bloques 3x3 px).

Para presentación con cliente, usa `Open 3D Large` y sube `Points` a `500k`. Ese es el techo útil para un depth 800x600, porque la cámara solo puede aportar hasta ~480k puntos antes de filtrar inválidos. Si el depth es 640x480, el techo real ronda ~307k puntos.

### Simulación pick directa (sin GUI de cámara)

```bash
# Requiere captures/ball_latest.json generado por ball_viewer
python3 examples/ball_sim.py
```

### Visor multi-modo (RGB / Depth / Cloud / Pick)

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 examples/smart_pick.py \
    --ip 192.168.1.101 \
    --mode pick \
    --robot mock
```

### Simulador TCP del robot

```bash
python3 examples/robot_server.py --port 5005
```

---

## Detección de objetos

### Pelota naranja (HSV)

`ball_viewer.py` detecta objetos naranja por color HSV:

```python
lower = (2,  100, 60)   # H, S, V mínimo
upper = (30, 255, 255)  # H, S, V máximo
```

Si la iluminación de tu entorno es muy diferente, ajusta estos rangos.

### Objetos genéricos (RANSAC + DBSCAN)

`smart_pick.py --mode pick` usa segmentación de plano + clustering:

```python
# Plano (mesa): RANSAC
distance_threshold = 10   # mm
ransac_n           = 3
num_iterations     = 1000

# Clusters (objetos): DBSCAN
eps        = 15    # mm
min_points = 50
```

---

## Protocolo robot TCP

El robot (o `robot_server.py`) escucha en un puerto TCP y recibe JSON:

```json
{
  "pick":       [x, y, z],
  "approach":   [x, y, z],
  "confidence": 0.85,
  "pixel":      [u, v]
}
```

Responde `"OK\n"` o `"ERROR\n"`.

---

## Simulación PyBullet

`ball_sim.py` lee `captures/ball_latest.json` y ejecuta:

```
1. Approach  — gripper baja con dedos abiertos
2. Pick      — dedos se cierran alrededor del objeto
3. Lift      — gripper + objeto suben
4. Release   — dedos se abren, objeto cae
5. Retract   — gripper vuelve a posición home
```

Conversión de coordenadas cámara → mundo PyBullet:

```
cam_x → world_x   (horizontal)
cam_z → world_y   (profundidad = alejarse)
-cam_y → world_z  (vertical, arriba positivo)
```

---

## Solución de problemas

### `ModuleNotFoundError: No module named 'pybullet'`

PyBullet está en `~/.local` pero el proceso padre tiene `PYTHONNOUSERSITE=1`.  
El subproceso `ball_sim.py` se lanza sin esa variable — ya está corregido en `ball_viewer.py`.  
Para lanzarlo manualmente:

```bash
env -u PYTHONNOUSERSITE python3 examples/ball_sim.py
```

### `pybullet.error: Not connected to physics server`

No uses `p.createConstraint` entre cuerpos kinematic (masa=0) y dinámicos.  
Este proyecto mueve el objeto directamente con `resetBasePositionAndOrientation`.

### Pantalla negra en GTK

La causa más común es GC prematuro del buffer de pixbuf.  
Asegúrate de que `bgr_to_pixbuf` guarda `pb._keep = data`.

### `status=2 SC_OPENED` al abrir cámara

La cámara quedó abierta de una sesión anterior. El wrapper `camera.py`  
acepta status 1 y 2 en `find_device_by_ip`. Reinicia la cámara desconectando  
y reconectando el cable si persiste.

### `scGetSensorIntrinsicParameters` falla

Usa `ScSensorType.SC_TOF_SENSOR`, no el entero `0`:

```python
from API.ScepterDS_enums import ScSensorType
ret, intrinsics = cam.scGetSensorIntrinsicParameters(ScSensorType.SC_TOF_SENSOR)
```

---

## Licencia

MIT — libre para uso en proyectos de robótica e investigación.
