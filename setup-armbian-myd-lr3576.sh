#!/bin/bash
#
# setup-armbian-myd-lr3576.sh
#
# Clones the Armbian build framework and applies all MYIR MYD-LR3576 (RK3576)
# board support: board config, kernel patches (NVP6324 camera driver, HDMI, etc.),
# device trees, U-Boot defconfig/DT, boot script, family config additions,
# and pre-patched userspace package sources (xserver, libdrm, gstreamer).
#
# Usage:
#   ./setup-armbian-myd-lr3576.sh [target_dir]
#
# If target_dir is omitted, defaults to ./build in the current directory.
#
# After setup, build with:
#   cd <target_dir>
#   ./compile.sh BOARD=myd-lr3576 BRANCH=vendor RELEASE=noble BUILD_DESKTOP=yes \
#     DESKTOP_ENVIRONMENT=xfce DESKTOP_ENVIRONMENT_CONFIG_NAME=config_base \
#     DESKTOP_APPGROUPS_SELECTED="browsers editors internet multimedia" \
#     KERNEL_CONFIGURE=no CLEAN_LEVEL="oldcache"
#
set -euo pipefail

# ── Configuration ──
ARMBIAN_REPO="https://github.com/armbian/build.git"
ARMBIAN_TAG="v26.02.0-trunk"  # adjust if using a different release
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${1:-${SCRIPT_DIR}/build}"

# Source tree with our customizations (same directory as this script)
EXISTING_BUILD="${SCRIPT_DIR}"

# ── Helpers ──
info()  { echo -e "\033[1;32m>>>\033[0m $*"; }
warn()  { echo -e "\033[1;33mWARN:\033[0m $*"; }
error() { echo -e "\033[1;31mERROR:\033[0m $*"; exit 1; }

copy_if_exists() {
    local src="$1" dst="$2"
    if [[ -e "$src" ]]; then
        mkdir -p "$(dirname "$dst")"
        cp -a "$src" "$dst"
    else
        warn "Missing: $src"
    fi
}

# ── Verify required files exist alongside this script ──
if [[ ! -f "$EXISTING_BUILD/config/boards/myd-lr3576.csc" ]]; then
    error "Board config not found: $EXISTING_BUILD/config/boards/myd-lr3576.csc
       Run this script from the armbian-myd-lr3576-setup directory."
fi

# ── Step 1: Clone Armbian ──
if [[ -d "$BUILD_DIR/.git" ]]; then
    info "Armbian repo already exists at $BUILD_DIR, pulling latest..."
    cd "$BUILD_DIR"
    git fetch origin
else
    info "Cloning Armbian build framework..."
    git clone --depth=1 "$ARMBIAN_REPO" "$BUILD_DIR"
    cd "$BUILD_DIR"
fi

# Try to checkout the matching tag (non-fatal if tag doesn't exist in shallow clone)
git checkout "$ARMBIAN_TAG" 2>/dev/null || {
    warn "Tag $ARMBIAN_TAG not found, using current HEAD ($(git log --oneline -1))"
}

info "Armbian build directory: $BUILD_DIR"

# ── Step 2: Board configuration ──
info "Installing board config: myd-lr3576.csc"
copy_if_exists \
    "$EXISTING_BUILD/config/boards/myd-lr3576.csc" \
    "$BUILD_DIR/config/boards/myd-lr3576.csc"

# ── Step 3: Boot script ──
info "Installing RK3576 boot script"
copy_if_exists \
    "$EXISTING_BUILD/config/bootscripts/boot-rk3576.cmd" \
    "$BUILD_DIR/config/bootscripts/boot-rk3576.cmd"

# ── Step 4: Family config (rk35xx.conf) ──
# The family config needs two additions: RK3576 boot script selection and Mali CSF disable.
# We patch rather than overwrite so upstream changes are preserved.
FAMILY_CONF="$BUILD_DIR/config/sources/families/rk35xx.conf"
info "Patching family config: rk35xx.conf"

# Add RK3576 boot script selection if not already present
if ! grep -q 'boot-rk3576.cmd' "$FAMILY_CONF" 2>/dev/null; then
    # Insert after OVERLAY_PREFIX line
    sed -i "/^OVERLAY_PREFIX=/a\\
\\
if [[ \"\$BOOT_SOC\" == \"rk3576\" ]]; then\\
\\tBOOTSCRIPT='boot-rk3576.cmd:boot.cmd'\\
fi" "$FAMILY_CONF"
    info "  Added RK3576 boot script selection"
else
    info "  RK3576 boot script selection already present"
fi

# Add Mali CSF disable function if not already present
if ! grep -q 'custom_kernel_config__rk3576_mali_no_csf' "$FAMILY_CONF" 2>/dev/null; then
    cat >> "$FAMILY_CONF" << 'EOF'

# Disable CSF support for RK3576 Mali G52 Bifrost GPU
# CSF (Command Stream Frontend) is only for Valhall GPUs (RK3588), not Bifrost (RK3576)
# The CSF mode requires a much larger register space that RK3576's Mali G52 doesn't support
function custom_kernel_config__rk3576_mali_no_csf() {
	if [[ "$BOOT_SOC" == "rk3576" ]]; then
		display_alert "Disabling Mali CSF support" "RK3576 uses Bifrost GPU, not Valhall" "info"
		opts_n+=("MALI_CSF_SUPPORT")
	fi
}
EOF
    info "  Added Mali CSF disable for RK3576"
else
    info "  Mali CSF disable already present"
fi

# ── Step 5: Kernel patches ──
KERNEL_PATCH_SRC="$EXISTING_BUILD/patch/kernel/rk35xx-vendor-6.1"
KERNEL_PATCH_DST="$BUILD_DIR/patch/kernel/rk35xx-vendor-6.1"
mkdir -p "$KERNEL_PATCH_DST"

info "Installing kernel patches..."
for patch in "$KERNEL_PATCH_SRC"/*.patch; do
    [[ -f "$patch" ]] || continue
    name="$(basename "$patch")"
    cp "$patch" "$KERNEL_PATCH_DST/$name"
    info "  $name"
done

# ── Step 6: Kernel device tree files ──
info "Installing kernel device tree files..."
mkdir -p "$KERNEL_PATCH_DST/dt"
for dt in "$KERNEL_PATCH_SRC/dt"/*; do
    [[ -f "$dt" ]] || continue
    name="$(basename "$dt")"
    cp "$dt" "$KERNEL_PATCH_DST/dt/$name"
    info "  dt/$name"
done

# ── Step 7: U-Boot defconfig and device tree ──
UBOOT_PATCH_SRC="$EXISTING_BUILD/patch/u-boot/legacy/u-boot-radxa-rk35xx"
UBOOT_PATCH_DST="$BUILD_DIR/patch/u-boot/legacy/u-boot-radxa-rk35xx"

info "Installing U-Boot defconfig and DT..."
copy_if_exists \
    "$UBOOT_PATCH_SRC/defconfig/myd-lr3576-rk3576_defconfig" \
    "$UBOOT_PATCH_DST/defconfig/myd-lr3576-rk3576_defconfig"
copy_if_exists \
    "$UBOOT_PATCH_SRC/dt/rk3576-myd-lr3576.dts" \
    "$UBOOT_PATCH_DST/dt/rk3576-myd-lr3576.dts"

# ── Step 8: Userspace package cache (pre-patched sources + built debs) ──
CACHE_SRC="$EXISTING_BUILD/cache/rk3576-packages"
CACHE_DST="$BUILD_DIR/cache/rk3576-packages"

if [[ -d "$CACHE_SRC" ]]; then
    info "Copying userspace package cache (xserver, libdrm, gstreamer)..."
    mkdir -p "$CACHE_DST"

    # Source tarballs (needed for chroot builds if debs aren't cached)
    if [[ -d "$CACHE_SRC/sources" ]]; then
        cp -a "$CACHE_SRC/sources" "$CACHE_DST/"
        info "  sources/ (xserver tarball, libdrm orig+debian, gstreamer patched tarballs)"
    fi

    # libdrm patches
    if [[ -d "$CACHE_SRC/libdrm-patches" ]]; then
        cp -a "$CACHE_SRC/libdrm-patches" "$CACHE_DST/"
        info "  libdrm-patches/ (4 Rockchip patches)"
    fi

    # Pre-built .debs (avoids slow QEMU chroot rebuilds)
    deb_count=0
    for deb in "$CACHE_SRC"/*.deb; do
        [[ -f "$deb" ]] || continue
        cp "$deb" "$CACHE_DST/"
        deb_count=$((deb_count + 1))
    done
    if [[ $deb_count -gt 0 ]]; then
        info "  $deb_count cached .debs (libdrm, xserver)"
    fi

    # GStreamer cached debs
    if [[ -d "$CACHE_SRC/gstreamer-debs" ]]; then
        cp -a "$CACHE_SRC/gstreamer-debs" "$CACHE_DST/"
        gst_count=$(ls "$CACHE_SRC/gstreamer-debs"/*.deb 2>/dev/null | wc -l)
        info "  gstreamer-debs/ ($gst_count cached .debs)"
    fi
else
    warn "No userspace package cache found at $CACHE_SRC"
    warn "xserver, libdrm, and gstreamer will be built from source during image build (slow)"
fi

# ── Step 9: Mali GPU driver cache ──
MALI_SRC="$EXISTING_BUILD/cache/mali"
MALI_DST="$BUILD_DIR/cache/mali"
if [[ -d "$MALI_SRC" ]]; then
    info "Copying Mali GPU driver cache..."
    mkdir -p "$MALI_DST"
    cp -a "$MALI_SRC"/* "$MALI_DST/" 2>/dev/null || true
else
    info "No Mali driver cache (will be downloaded during build)"
fi

# ── Summary ──
echo ""
info "=== Setup complete ==="
echo ""
echo "  Build directory: $BUILD_DIR"
echo ""
echo "  Files installed:"
echo "    config/boards/myd-lr3576.csc                          (board config)"
echo "    config/bootscripts/boot-rk3576.cmd                    (boot script)"
echo "    config/sources/families/rk35xx.conf                   (patched)"
echo "    patch/kernel/rk35xx-vendor-6.1/*.patch                ($(ls "$KERNEL_PATCH_DST"/*.patch 2>/dev/null | wc -l) kernel patches)"
echo "    patch/kernel/rk35xx-vendor-6.1/dt/*                   ($(ls "$KERNEL_PATCH_DST/dt"/* 2>/dev/null | wc -l) device tree files)"
echo "    patch/u-boot/.../defconfig/myd-lr3576-rk3576_defconfig"
echo "    patch/u-boot/.../dt/rk3576-myd-lr3576.dts"
if [[ -d "$CACHE_DST" ]]; then
    echo "    cache/rk3576-packages/                                ($(du -sh "$CACHE_DST" 2>/dev/null | cut -f1) userspace cache)"
fi
echo ""
echo "  Kernel patches:"
for p in "$KERNEL_PATCH_DST"/*.patch; do
    [[ -f "$p" ]] && echo "    $(basename "$p")"
done
echo ""
echo "  Build commands:"
echo ""
echo "    # Kernel only (fast):"
echo "    ./compile.sh kernel BOARD=myd-lr3576 BRANCH=vendor CLEAN_LEVEL=\"oldcache\" ARTIFACT_IGNORE_CACHE='yes'"
echo ""
echo "    # Full desktop image:"
echo "    ./compile.sh BOARD=myd-lr3576 BRANCH=vendor RELEASE=noble BUILD_DESKTOP=yes \\"
echo "      DESKTOP_ENVIRONMENT=xfce DESKTOP_ENVIRONMENT_CONFIG_NAME=config_base \\"
echo "      DESKTOP_APPGROUPS_SELECTED=\"browsers editors internet multimedia\" \\"
echo "      KERNEL_CONFIGURE=no CLEAN_LEVEL=\"oldcache\""
echo ""
