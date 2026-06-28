# 3D Smart Picking — Scepter ToF Camera + PyBullet

Sistema de **detección 3D de objetos y simulación de pick robótico** usando una cámara ToF Scepter/Vzense sobre Jetson Orin (aarch64).

```
Cámara 3D → Detección por color → Posición XYZ → Simulación PyBullet
```

![Flujo general](docs/flow.png)

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
examples/
  ball_viewer.py        ← Interfaz GTK3: RGB + Depth + nube 3D rotable
  ball_sim.py           ← Simulación PyBullet (gripper pick)
  smart_pick.py         ← Visor multi-modo (rgb/depth/cloud/pick)
  robot_server.py       ← Simulador TCP del robot
  scepter_realtime_open3d.py  ← Nube de puntos en tiempo real
  picker/
    camera.py           ← Wrapper ScepterCamera (context manager)
    detect.py           ← RANSAC + DBSCAN: detección de objetos
    robot.py            ← MockRobot / TcpRobot (protocolo JSON)
```

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

### Visor principal (RGB + Depth + detección de pelota)

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 examples/ball_viewer.py 192.168.1.101
```

**Interfaz GTK3:**

| Panel izquierdo | Panel derecho |
|---|---|
| Imagen RGB con overlay de detección | Depth colormap (rojo=cerca, azul=lejos) |
| Medidas reales: ancho y alto en mm | Vista 3D rotable (botón "Cambiar a 3D") |

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
