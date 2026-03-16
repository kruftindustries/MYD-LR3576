# User Applications for MYD-LR3576

Development tools and demos for the MYIR MYD-LR3576 (RK3576) board.
All applications are built natively on the board (aarch64).

## Board Connection

```bash
# SSH (password: 1234)
sshpass -p 1234 ssh root@192.168.12.63

# Copy files to board
sshpass -p 1234 scp -r user_apps/ root@192.168.12.63:/root/
```

## Applications

### mixed_quad — Mixed-Input Quad Viewer

2x2 grid display combining V4L2 (NVP6324 AHD camera), UVC USB camera, and RTSP
network streams. DRM direct rendering with RGA hardware scaling and GPU
barrel/pincushion undistortion.

```bash
cd /root/user_apps/mixed_quad && make
systemctl stop lightdm    # DRM needs exclusive display access
./mixed_quad
```

**Features:**
- 4 simultaneous video sources (V4L2 raw, V4L2 H.264 decode, RTSP HEVC)
- RGA hardware format conversion and scaling (UYVY/YUYV/NV12 to BGRX)
- GPU barrel/pincushion lens undistortion (GLES2 shaders)
- Per-source lens calibration saved to `/root/lens_calib_src{0..3}.conf`
- V4L2 auto-reconnect on device disconnect
- Screenshot: **P** key or `kill -USR1 $(pgrep mixed_quad)` saves to `/tmp/mixed_quad_NNN.ppm`

**Keys:** Left/Right = select source, Enter = fullscreen calibration, Q = quit,
Arrow keys = adjust k1/k2, +/- = zoom, G = grid, S = save calibration

**Input configuration** (edit top of `mixed_quad.c`):
```c
} inputs[MAX_SRCS] = {
    { SRC_V4L2,        "/dev/video13", 1920, 1080, "AHD ch2"   },
    { SRC_V4L2,        "/dev/video33",  640,  480, "UVC 480p"   },
    { SRC_RTSP, "rtsp://192.168.12.142:554/video1", 960, 1088, "RTSP 1" },
    { SRC_RTSP, "rtsp://192.168.12.142:554/video2", 960, 1088, "RTSP 2" },
};
```

**UVC camera controls** (Arducam 1080P Ultra-lowlight, Microdia 0c45:0576):
```bash
# Auto exposure with dynamic framerate (best for low light)
v4l2-ctl -d /dev/video33 -c auto_exposure=3 -c exposure_dynamic_framerate=1

# Manual exposure (value in 0.1ms units, e.g. 400 = 40ms)
v4l2-ctl -d /dev/video33 -c auto_exposure=1 -c exposure_time_absolute=400

# Ultra low light mode (backlight_compensation: 0=normal, 1-2=low light)
v4l2-ctl -d /dev/video33 -c backlight_compensation=2

# Reduce noise: lower sharpness (0-6) and gain (0-100)
v4l2-ctl -d /dev/video33 -c sharpness=0 -c gain=0
```

**UVC formats:** video33 = YUYV 640x480 / MJPEG 1080p, video35 = H.264 1080p.
To use 1080p H.264, change src1 to `{ SRC_V4L2_DECODE, "/dev/video35", 1920, 1080, "UVC 1080p" }`.

**DRM screenshot tool** (`drm_screenshot.c`): Standalone utility to capture the
DRM display output from another process. `./drm_screenshot [output.ppm]`

---

### quad_undistort — V4L2 Quad Viewer

V4L2-only quad viewer with GPU barrel/pincushion undistortion. Displays 4 V4L2
sources in a 2x2 grid on DRM. Predecessor to mixed_quad.

```bash
cd /root/user_apps/quad_undistort && make
systemctl stop lightdm
./v4l2_gpu_undistort_quad
```

---

### nvp_reg — NVP6324 Register Tool

Read/write NVP6324 AHD decoder registers via V4L2 subdevice ioctls.

```bash
cd /root/user_apps/nvp_reg && make
./nvp_reg         # interactive register access
```

---

### motion_detection/v3 — RTSP Motion Detection + RKNN YOLOv5s

Multi-stream RTSP viewer with hardware HEVC decode (RKMPP), frame-difference
motion detection, RKNN NPU object detection (YOLOv5s), and SQLite event logging.

```bash
cd /root/user_apps/motion_detection/v3 && make

# Copy RKNN model to board
sshpass -p 1234 scp models/yolov5s-640-640.rknn root@192.168.12.63:/root/user_apps/models/

DISPLAY=:0 XAUTHORITY=/var/run/lightdm/root/:0 ./motion_detect_v3
```

**RKNN model:** `models/yolov5s-640-640.rknn` (YOLOv5s, 640x640 input, RK3576 optimized)

**Detection classes:** person, bicycle, car, motorcycle, bus, truck, bird, cat, dog, horse, sheep, cow, bear

**RTSP streams:** Default `rtsp://192.168.12.142:554/video1` and `video2` (HEVC 960x1088)

---

### motion_detection/v4 — Latest Motion Detection

Latest iteration of motion detection with improved grid display.

```bash
cd /root/user_apps/motion_detection/v4 && make
DISPLAY=:0 XAUTHORITY=/var/run/lightdm/root/:0 ./motion_detect_v4
```

---

### vehicle_monitor_v3 — Vehicle Monitor

Full vehicle monitoring application with multi-camera views, bird's-eye view
with homography transform, DVR recording, motion detection, lens calibration,
and sensor monitoring.

```bash
cd /root/user_apps/vehicle_monitor_v3 && make
DISPLAY=:0 XAUTHORITY=/var/run/lightdm/root/:0 ./vehicle_monitor
```

**Kill before restart:** `killall vehicle_monitor`

**Structure:**
- `main.c` + `app.h` — Application entry, page routing
- `core/` — Camera capture, clip recording, config, database, homography, motion, playback, undistortion
- `pages/` — UI pages (cameras, birdview, config, DVR, events, sensors, lens calibration)
- `ui/` — Drawing utilities

---

## Build Dependencies (on board)

Most dependencies are pre-installed in the Armbian image built with this repo's
setup script. If building on a fresh board:

```bash
# Core build tools
apt-get install build-essential pkg-config

# Video/graphics libraries
apt-get install librga-dev libdrm-dev libgbm-dev libegl-dev libgles2-mesa-dev

# FFmpeg (for RTSP/decode apps)
apt-get install libavformat-dev libavcodec-dev libavutil-dev libswscale-dev

# RKNN runtime (for motion detection)
# Already installed if using the Armbian image from this repo

# SQLite (for motion detection v3 database)
apt-get install libsqlite3-dev

# X11 (for vehicle_monitor and X11 apps)
apt-get install libx11-dev libcairo2-dev
```

## Deploy Workflow

```bash
# From host: copy source to board, build, and run
sshpass -p 1234 scp -r mixed_quad/ root@192.168.12.63:/root/user_apps/mixed_quad/
sshpass -p 1234 ssh root@192.168.12.63 'cd /root/user_apps/mixed_quad && make && ./mixed_quad'
```

## NVP6324 Camera Notes

- AHD camera on channel 2: `/dev/video13` (1920x1080 UYVY, 25fps)
- Device mapping: video11=ch0, video12=ch1, video13=ch2, video14=ch3
- Sysfs register interface: `/sys/devices/platform/.../reg` and `channel_fmt`
- Format configuration: `echo "ch dv_idx" > channel_fmt`
  - DV indices: 0=CVBS NTSC, 1=CVBS PAL, 2-5=AHD 720p, 6-7=AHD 1080p
