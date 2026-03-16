# MYIR MYD-LR3576 Armbian Board Support + Development Tools

Armbian build support for the MYIR MYD-LR3576 evaluation board (Rockchip RK3576 SoC).
Includes kernel patches (NVP6324 AHD camera driver, HDMI, interlace), device trees,
U-Boot config, GPU userspace (Mali G52 Bifrost), patched xserver/libdrm/gstreamer,
and user-space development applications.

## Quick Start

```bash
# 1. Clone this repo
git clone https://github.com/kruftindustries/MYD-LR3576.git
cd MYD-LR3576

# 2. Run setup (clones Armbian framework and applies all board support)
./setup-armbian-myd-lr3576.sh ~/armbian-build

# 3. Build a full desktop image
cd ~/armbian-build
./compile.sh BOARD=myd-lr3576 BRANCH=vendor RELEASE=noble BUILD_DESKTOP=yes \
  DESKTOP_ENVIRONMENT=xfce DESKTOP_ENVIRONMENT_CONFIG_NAME=config_base \
  DESKTOP_APPGROUPS_SELECTED="browsers editors internet multimedia" \
  KERNEL_CONFIGURE=no CLEAN_LEVEL="oldcache"
```

The output image will be in `~/armbian-build/output/images/`.

## Build Commands

### Full Desktop Image

Builds kernel, U-Boot, root filesystem, and desktop packages into a flashable image:

```bash
./compile.sh BOARD=myd-lr3576 BRANCH=vendor RELEASE=noble BUILD_DESKTOP=yes \
  DESKTOP_ENVIRONMENT=xfce DESKTOP_ENVIRONMENT_CONFIG_NAME=config_base \
  DESKTOP_APPGROUPS_SELECTED="browsers editors internet multimedia" \
  KERNEL_CONFIGURE=no CLEAN_LEVEL="oldcache"
```

### Kernel Only (Fast Rebuild)

Rebuilds only the kernel and generates a .deb package — much faster than a full
image build. Use this when iterating on kernel patches or device tree changes:

```bash
./compile.sh kernel BOARD=myd-lr3576 BRANCH=vendor \
  CLEAN_LEVEL="oldcache" ARTIFACT_IGNORE_CACHE='yes'
```

The output .deb is in `output/debs/`. To force a clean rebuild, delete the cached
kernel .deb and object files:

```bash
rm -f output/debs/linux-image-*.deb
# If needed, also clean the kernel build tree:
rm -rf cache/sources/linux-kernel-vendor-rk35xx/.tmp
```

### Deploy Kernel to Board

After a kernel-only build, install the new kernel .deb over SSH:

```bash
# Copy .deb to board
sshpass -p 1234 scp output/debs/linux-image-vendor-rk35xx_*.deb \
  root@192.168.12.63:/tmp/

# Install on board and reboot
sshpass -p 1234 ssh root@192.168.12.63 \
  'dpkg -i /tmp/linux-image-vendor-rk35xx_*.deb && reboot'
```

### Kernel Configuration

To open the kernel menuconfig UI:

```bash
./compile.sh kernel BOARD=myd-lr3576 BRANCH=vendor KERNEL_CONFIGURE=yes
```

### Build Notes

- `DESKTOP_ENVIRONMENT_CONFIG_NAME=config_base` is required (avoids dialog stdin error)
- `KERNEL_BTF=no` may be needed on build hosts with <8GB RAM
- `CLEAN_LEVEL="oldcache"` reuses cached downloads; won't recompile if .deb exists
- Build requires `/dev/loop*` devices — create with `sudo mknod /dev/loopN b 7 N` if missing
- First build downloads ~5GB of toolchains and sources; subsequent builds are faster

## Repository Structure

```
MYD-LR3576/
├── setup-armbian-myd-lr3576.sh     # Setup script — clones Armbian + applies board support
├── config/
│   ├── boards/myd-lr3576.csc       # Board config (Mali GPU, xserver, libdrm, gstreamer, Xorg)
│   ├── bootscripts/boot-rk3576.cmd # U-Boot boot script
│   └── sources/families/           # rk35xx.conf additions (boot script + Mali CSF disable)
├── patch/
│   ├── kernel/rk35xx-vendor-6.1/   # 12 kernel patches + 8 device tree files
│   └── u-boot/.../                 # U-Boot defconfig + device tree
├── cache/
│   ├── mali/                       # Mali G52 Bifrost userspace driver .deb
│   ├── sources/                    # Pre-patched xserver, libdrm, gstreamer tarballs
│   ├── libdrm-patches/             # 4 Rockchip libdrm patches
│   ├── xserver-patches/            # 81 Rockchip xserver patches
│   ├── gstreamer-debs/             # 13 pre-built GStreamer .debs
│   └── *.deb                       # Pre-built libdrm + xserver .debs
└── user_apps/                      # Development tools and demos
    ├── mixed_quad/                 # Mixed-input quad viewer (V4L2 + UVC + RTSP)
    ├── quad_undistort/             # V4L2 quad viewer with GPU undistortion
    ├── nvp_reg/                    # NVP6324 register read/write tool
    ├── motion_detection/v3/        # RTSP + RKNN YOLOv5s + SQLite
    ├── motion_detection/v4/        # Latest motion detection
    ├── vehicle_monitor_v3/         # Full vehicle monitor application
    └── models/                     # RKNN model files (YOLOv5s)
```

## Kernel Patches

| Patch | Description |
|-------|-------------|
| 001-hid-sony | HID Sony controller fix |
| 002-nvp6324-jaguar-drivers | NVP6324 AHD camera base driver (1.4MB) |
| 003-jaguar2-mipi-vc-assignment | MIPI virtual channel assignment |
| 004-rkcif-multistream-fix | Arbiter bounce gating |
| 005-rkcif-size-err-no-drop | Skip CTRL reset on CSI_SIZE_ERR |
| 006-jaguar2-arbiter-vloss-mask | Arbiter video loss masking |
| 008-jaguar2-hotplug-reinit | GPIO I2C fix + per-channel reinit on reconnect |
| 009-jaguar2-vfc-dvtimings-motion | VFC, DV timings, motion, cross-format probe |
| 010-jaguar2-mipi-interlaced-sd | CVBS NTSC/PAL MIPI interlace support |
| 011-rkcif-mipi-interlace-soft-auto | CIF bit-1 field detection for SOFT_AUTO interlace |
| 012-jaguar2-sysfs-reg-interface | Sysfs register R/W + channel_fmt config |
| 013-jaguar2-slow-poll-streaming | Slow poll during streaming |

## Board Details

- **SoC:** Rockchip RK3576 (4x Cortex-A72 + 4x Cortex-A53)
- **GPU:** Mali G52 MC3 (Bifrost) — uses non-CSF mode
- **NPU:** RKNN 2 cores, v0.9.8 driver, v2.3.2 runtime
- **Kernel:** Vendor 6.1.115
- **Display:** DRM via rockchip-drm (card1), RKNPU is card0
- **Camera:** NVP6324 4-channel AHD decoder via MIPI CSI
- **SSH:** `root@192.168.12.63` password `1234`

## User Applications

See [user_apps/README.md](user_apps/README.md) for build instructions, usage,
and detailed documentation for each application.
