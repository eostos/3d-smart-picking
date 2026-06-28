# BanaPick LIVE UI Spec

Professional desktop application UI for an industrial robotic banana hand smart picking platform called **BanaPick**.

## Product Style

- Dark industrial/tech theme.
- Accent color: `#1D9E75` teal green.
- Status colors:
  - Teal: `#1D9E75`
  - Amber: `#BA7517`
  - Reject red: `#DC2626`
  - Idle gray: `#374151`
- Optimized for touchscreen operator panels on Jetson Orin NX.
- Inter font where available, with Cantarell fallback.
- 8px corner radius.
- Dense, clean layout with subtle cards and only necessary grid lines.
- No gradients on buttons.

## Navigation

Persistent left sidebar with icon + label tabs:

- Live
- Cameras
- Calibration
- Detection
- Robot
- Routing
- Log / DB

## LIVE Tab

### Top Bar

- BanaPick logo on the left.
- Status pill:
  - Green `RUNNING`
  - Red `STOPPED`
- Red emergency stop button.
- Reconnect button for recovering camera/algorithm after E-STOP or camera error.
- Current time on the right.

### Main Area

Three columns:

- Left: Pick Camera feed.
  - Simultaneous RGB and Depth/3D views.
  - RGB scene with bounding boxes.
  - Two keypoints per banana hand:
    - Crown point in teal.
    - Tip point in orange.
  - Confidence score badge per detection.
  - Top-right camera badges: `STEM UP` and `STEM DOWN`.
  - Compact Depth / 3D toggle below.
  - In 3D mode, the camera area switches to a single large point-cloud feed; RGB/depth thumbnails are hidden.
  - 3D mode supports mouse drag to rotate and mouse wheel to zoom.
  - Point budget is adjustable from the operator controls, up to 500k points for full-sensor presentations.
  - Pixel click readout reports X/Y/Z in millimeters.
  - Height reference: `Y` is camera-frame height relative to the optical center; `Z` is distance from the ToF sensor.
- Center: Cell operational flow.
  - Active model and target part summary.
  - Vertical stepper:
    - Detect hand
    - Verify stem
    - Grasp by stem
    - Verify grasp
    - Rotate + Inspect
    - Classify destination
  - Active step highlighted in teal.
  - Alarm state in amber.
  - Success in green.
  - Idle in gray.
  - Current step label, elapsed time, and Manual Pick button below.
- Right: Box fill status.
  - Top-down 4x3 packing box schematic.
  - Empty cells dark.
  - Filled cells use teal banana icon.
  - Fill count `8 / 12`.
  - Progress bar.
  - Amber Box Full / nearly full alert.
  - Change Box button.

### Bottom Bar

Three metric cards:

- Throughput: `12 hands/min` with sparkline chart.
- Grasp success rate: `94.2%` with circular gauge.
- Rejections today: `3` with red badge.

## Implementation

Current GTK3 implementation:

- `banapick.py`
- `banapick/ui/live_panel.py`
- Live camera stream uses the Scepter camera wrapper through the existing `examples/ball_viewer.py` camera utilities.
- Sidebar tabs switch between operational pages.

Run:

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 banapick.py 192.168.1.101
```
