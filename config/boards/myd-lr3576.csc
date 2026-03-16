# Rockchip RK3576 SoC octa core 4-16GB RAM
# MYIR MYD-LR3576 Evaluation Board
BOARD_NAME="MYIR MYD-LR3576"
BOARD_VENDOR="myir"
BOARDFAMILY="rk35xx"
BOOTCONFIG="myd-lr3576-rk3576_defconfig"
BOOT_SOC="rk3576"
KERNEL_TARGET="vendor"
FULL_DESKTOP="yes"
BOOT_LOGO="desktop"
BOOT_FDT_FILE="rockchip/rk3576-myd-lr3576.dtb"
BOOT_SCENARIO="spl-blobs"
IMAGE_PARTITION_TABLE="gpt"
BOARD_MAINTAINER=""

# Serial console - UART0 via fiq-debugger
SERIALCON="ttyFIQ0"

# Hardware features
BOOT_SUPPORT_SPI="no"
DDR_BLOB="rk35/rk3576_ddr_lp4_2112MHz_lp5_2736MHz_v1.09.bin"
BL31_BLOB="rk35/rk3576_bl31_v1.20.elf"
BL32_BLOB="rk35/rk3576_bl32_v1.06.bin"

# Install real Firefox from Mozilla APT repo (Ubuntu Noble only ships snap stubs)
function post_family_tweaks__myd_lr3576_browsers() {
	[[ "${BUILD_DESKTOP}" != "yes" ]] && return 0

	display_alert "$BOARD" "Installing Firefox from Mozilla APT repo" "info"

	# Add Mozilla GPG key
	mkdir -p "${SDCARD}/etc/apt/keyrings"
	chroot_sdcard "wget -q https://packages.mozilla.org/apt/repo-signing-key.gpg -O /etc/apt/keyrings/packages.mozilla.org.asc" || {
		display_alert "Failed to download Mozilla signing key" "" "wrn"
		return 0
	}

	# Add Mozilla APT repo
	cat > "${SDCARD}/etc/apt/sources.list.d/mozilla.list" <<- 'MOZEOF'
	deb [signed-by=/etc/apt/keyrings/packages.mozilla.org.asc] https://packages.mozilla.org/apt mozilla main
	MOZEOF

	# Pin Mozilla packages over Ubuntu snap stubs
	cat > "${SDCARD}/etc/apt/preferences.d/mozilla" <<- 'PINEOF'
	Package: firefox*
	Pin: origin packages.mozilla.org
	Pin-Priority: 1001
	PINEOF

	# Remove snap stub and install real Firefox
	chroot_sdcard apt-get update
	chroot_sdcard apt-get remove -y firefox 2>/dev/null || true
	chroot_sdcard apt-get remove -y chromium-browser 2>/dev/null || true
	chroot_sdcard_apt_get_install firefox || {
		display_alert "Failed to install Firefox from Mozilla repo" "" "wrn"
		return 0
	}

	# Install GLES2 GPU benchmark (uses Mali GPU via EGL, not GLX)
	chroot_sdcard_apt_get_install glmark2-es2-x11 || true

	# Fix ping capability for non-root users
	chroot_sdcard setcap cap_net_raw+p /usr/bin/ping || true

	display_alert "Firefox and GPU tools installed" "" "info"
	return 0
}

function post_family_tweaks__myd_lr3576_naming_audios() {
	display_alert "$BOARD" "Renaming MYD-LR3576 audio" "info"

	mkdir -p $SDCARD/etc/udev/rules.d/
	echo 'SUBSYSTEM=="sound", ENV{ID_PATH}=="platform-hdmi-sound", ENV{SOUND_DESCRIPTION}="HDMI Audio"' > $SDCARD/etc/udev/rules.d/90-naming-audios.rules
	return 0
}

# Configure serial console for fiq-debugger (UART0 at 115200)
function post_family_tweaks__myd_lr3576_serial_console() {
	display_alert "$BOARD" "Configuring serial console for fiq-debugger" "info"

	# Set serial console to ttyFIQ0 at 115200 baud for fiq-debugger
	cat >> "${SDCARD}/boot/armbianEnv.txt" <<- EOF
	serialconsole=ttyFIQ0,115200
	earlycon=on
	extraargs=earlycon=uart8250,mmio32,0x2ad40000
	EOF
	return 0
}

# Install Mali G52 Bifrost GPU userspace driver from Armbian libmali-rockchip mirror
function post_family_tweaks__myd_lr3576_vendor_libs() {
	local mali_deb_name="libmali-bifrost-g52-g24p0-x11-gbm_1.9-1_arm64.deb"
	local mali_deb_url="https://github.com/armbian/libmali-rockchip/releases/download/v1.9-1-bb0fba6/${mali_deb_name}"
	local mali_cache_dir="${SRC}/cache/mali"

	display_alert "$BOARD" "Installing Mali G52 Bifrost GPU userspace driver" "info"

	# Download to host cache if not already present
	mkdir -p "${mali_cache_dir}"
	if [[ ! -f "${mali_cache_dir}/${mali_deb_name}" ]]; then
		run_host_command_logged wget -O "${mali_cache_dir}/${mali_deb_name}" "${mali_deb_url}" || {
			display_alert "Failed to download Mali driver" "${mali_deb_url}" "wrn"
			rm -f "${mali_cache_dir}/${mali_deb_name}"
			return 0
		}
	fi

	# Copy to chroot and install
	run_host_command_logged cp -v "${mali_cache_dir}/${mali_deb_name}" "${SDCARD}/tmp/"
	chroot_sdcard dpkg -i "/tmp/${mali_deb_name}" || {
		display_alert "dpkg install failed, attempting dependency fix" "${mali_deb_name}" "wrn"
		chroot_sdcard_apt_get_install -f || true
	}
	rm -f "${SDCARD}/tmp/${mali_deb_name}"
	chroot_sdcard ldconfig

	display_alert "Mali G52 driver installed" "${mali_deb_name}" "info"
	return 0
}

# Enable deb-src repos temporarily for build-dep resolution
# Call _restore_sources after build-dep is done
_enable_deb_src_repos() {
	local sources_file="${SDCARD}/etc/apt/sources.list.d/ubuntu.sources"
	if [[ -f "$sources_file" ]] && ! grep -q 'deb-src' "$sources_file"; then
		display_alert "Enabling deb-src repos" "for build-dep resolution" "info"
		sed -i 's/^Types: deb$/Types: deb deb-src/' "$sources_file"
		chroot_sdcard apt-get update
	fi
}

_restore_deb_src_repos() {
	local sources_file="${SDCARD}/etc/apt/sources.list.d/ubuntu.sources"
	if [[ -f "$sources_file" ]]; then
		sed -i 's/^Types: deb deb-src$/Types: deb/' "$sources_file"
	fi
}

# Build and install patched xserver-xorg-core with all 81 Rockchip patches
# Patches: Glamor BGR format fix for Mali GLES, GPU sync (glFenceSync),
#   pixmap state tracking (gl_synced), FBO clear, cursor fix, DRI2/DRI3,
#   modesetting improvements, and many more from MYIR RK3576 SDK.
# Pre-patched source tarball: cache/rk3576-packages/sources/glamor-fix-xorg-server-full.tar.gz
function post_family_tweaks__myd_lr3576_build_xserver() {
	[[ "${BUILD_DESKTOP}" != "yes" ]] && return 0

	local cache_dir="${SRC}/cache/rk3576-packages"
	local xserver_deb=""

	# Check for cached .deb first (avoids slow QEMU rebuild)
	xserver_deb=$(ls "${cache_dir}"/xserver-xorg-core_*.deb 2>/dev/null | head -1)

	if [[ -n "$xserver_deb" ]]; then
		display_alert "$BOARD" "Using cached patched xserver: $(basename "$xserver_deb")" "info"
	else
		display_alert "$BOARD" "Building patched xserver-xorg-core in chroot (this will take a while)" "info"

		local src_tarball="${cache_dir}/sources/glamor-fix-xorg-server-full.tar.gz"
		if [[ ! -f "$src_tarball" ]]; then
			display_alert "Missing pre-patched xserver source" "$src_tarball" "err"
			display_alert "Create it by applying all 81 Rockchip patches to xorg-server 21.1.12" "" "info"
			return 1
		fi

		local build_dir="/tmp/xserver-build"

		# Enable deb-src repos for build-dep
		_enable_deb_src_repos

		# Install build tools and dependencies
		chroot_sdcard_apt_get_install devscripts dpkg-dev || {
			display_alert "Failed to install build tools" "" "wrn"
			return 1
		}
		chroot_sdcard apt-get build-dep -y xorg-server || {
			display_alert "Failed to install xorg-server build deps" "" "wrn"
			return 1
		}

		# Extract pre-patched source (all 81 Rockchip patches + debian/ already included)
		mkdir -p "${SDCARD}${build_dir}"
		tar xf "$src_tarball" -C "${SDCARD}${build_dir}/"

		# Build with dpkg-buildpackage (nocheck skips tests)
		chroot_sdcard "cd ${build_dir}/xorg-server-21.1.12 && DEB_BUILD_OPTIONS='nocheck nostrip parallel=\$(nproc)' dpkg-buildpackage -us -uc -b" || {
			display_alert "xserver-xorg-core build FAILED" "check build output above" "err"
			rm -rf "${SDCARD}${build_dir}"
			return 1
		}

		# Cache built .debs on host for future builds
		mkdir -p "${cache_dir}"
		cp "${SDCARD}${build_dir}"/xserver-xorg-core_*.deb "${cache_dir}/" 2>/dev/null
		cp "${SDCARD}${build_dir}"/xserver-xorg-dev_*.deb "${cache_dir}/" 2>/dev/null
		xserver_deb=$(ls "${cache_dir}"/xserver-xorg-core_*.deb 2>/dev/null | head -1)

		# Clean up build directory in image
		rm -rf "${SDCARD}${build_dir}"

		# Restore binary-only repos (done with all source builds)
		_restore_deb_src_repos

		if [[ -z "$xserver_deb" ]]; then
			display_alert "xserver-xorg-core .deb not found after build" "" "err"
			return 1
		fi

		display_alert "xserver build complete" "$(basename "$xserver_deb")" "info"
	fi

	# Install the patched .deb
	display_alert "$BOARD" "Installing patched xserver-xorg-core" "info"
	run_host_command_logged cp -v "$xserver_deb" "${SDCARD}/tmp/"
	chroot_sdcard dpkg -i "/tmp/$(basename "$xserver_deb")" || {
		display_alert "dpkg install failed, attempting dependency fix" "" "wrn"
		chroot_sdcard_apt_get_install -f || true
	}
	rm -f "${SDCARD}/tmp/$(basename "$xserver_deb")"

	# Hold to prevent apt from overwriting with stock version
	chroot_sdcard apt-mark hold xserver-xorg-core

	display_alert "Patched xserver-xorg-core installed and held" "" "info"
	return 0
}

# Build and install patched libdrm with Rockchip defaults
# Patches: default rockchip DRM device, auth bypass, modetest speed improvement
function post_family_tweaks__myd_lr3576_build_libdrm() {
	local cache_dir="${SRC}/cache/rk3576-packages"
	local libdrm_deb=""

	# Check for cached .deb
	libdrm_deb=$(ls "${cache_dir}"/libdrm2_*.deb 2>/dev/null | head -1)

	if [[ -n "$libdrm_deb" ]]; then
		display_alert "$BOARD" "Using cached patched libdrm: $(basename "$libdrm_deb")" "info"
	else
		display_alert "$BOARD" "Building patched libdrm in chroot" "info"

		local orig_tarball="${cache_dir}/sources/libdrm_2.4.120.orig.tar.xz"
		local deb_tarball="${cache_dir}/sources/libdrm_2.4.120-2build1.debian.tar.xz"
		local patch_dir="${cache_dir}/libdrm-patches"

		if [[ ! -f "$orig_tarball" ]] || [[ ! -f "$deb_tarball" ]]; then
			display_alert "Missing libdrm source tarballs" "${cache_dir}/sources/" "err"
			return 1
		fi

		local build_dir="/tmp/libdrm-build"

		# Enable deb-src if not already (may already be enabled from xserver build)
		_enable_deb_src_repos

		# Install build dependencies
		chroot_sdcard_apt_get_install devscripts dpkg-dev || true
		chroot_sdcard apt-get build-dep -y libdrm || true

		# Extract upstream source + Ubuntu debian packaging
		mkdir -p "${SDCARD}${build_dir}"
		tar xf "$orig_tarball" -C "${SDCARD}${build_dir}/"
		tar xf "$deb_tarball" -C "${SDCARD}${build_dir}/libdrm-2.4.120/"

		# Apply 4 Rockchip patches
		if [[ -d "$patch_dir" ]]; then
			for p in "${patch_dir}"/0*.patch; do
				[[ -f "$p" ]] || continue
				display_alert "Applying libdrm patch" "$(basename "$p")" "info"
				patch -p1 --fuzz=3 -d "${SDCARD}${build_dir}/libdrm-2.4.120" < "$p" || {
					display_alert "libdrm patch failed" "$(basename "$p")" "wrn"
				}
			done
		fi

		# Update changelog
		local timestamp
		timestamp=$(date -R)
		local orig_changelog="${SDCARD}${build_dir}/libdrm-2.4.120/debian/changelog"
		if [[ -f "$orig_changelog" ]]; then
			local tmpfile
			tmpfile=$(mktemp)
			cat > "$tmpfile" << CHLOG
libdrm (2.4.120-2build1+rk3576.1) noble; urgency=medium

  * Apply Rockchip RK3576 patches from MYIR SDK:
    - Default to rockchip DRM device
    - Bypass DRM auth APIs for BSP kernel
    - Speed up modetest info dumping

 -- Armbian Builder <builder@armbian.com>  ${timestamp}

CHLOG
			cat "$orig_changelog" >> "$tmpfile"
			mv "$tmpfile" "$orig_changelog"
		fi

		# Build
		chroot_sdcard "cd ${build_dir}/libdrm-2.4.120 && DEB_BUILD_OPTIONS='nocheck parallel=\$(nproc)' dpkg-buildpackage -us -uc -b" || {
			display_alert "libdrm build FAILED" "" "err"
			rm -rf "${SDCARD}${build_dir}"
			return 1
		}

		# Cache built .debs on host
		cp "${SDCARD}${build_dir}"/libdrm*.deb "${cache_dir}/" 2>/dev/null
		libdrm_deb=$(ls "${cache_dir}"/libdrm2_*.deb 2>/dev/null | head -1)
		rm -rf "${SDCARD}${build_dir}"

		if [[ -z "$libdrm_deb" ]]; then
			display_alert "libdrm .deb not found after build" "" "err"
			return 1
		fi

		display_alert "libdrm build complete" "$(basename "$libdrm_deb")" "info"
	fi

	# Install patched libdrm .debs (libdrm2, libdrm-common, etc.)
	display_alert "$BOARD" "Installing patched libdrm" "info"
	for deb in "${cache_dir}"/libdrm*.deb; do
		[[ -f "$deb" ]] || continue
		# Skip debug symbol packages
		[[ "$(basename "$deb")" == *-dbgsym* ]] && continue
		cp "$deb" "${SDCARD}/tmp/"
		chroot_sdcard dpkg -i "/tmp/$(basename "$deb")" 2>/dev/null || true
	done
	chroot_sdcard_apt_get_install -f || true
	rm -f "${SDCARD}"/tmp/libdrm*.deb

	# Hold libdrm2 to prevent apt from overwriting
	chroot_sdcard apt-mark hold libdrm2

	display_alert "Patched libdrm installed and held" "" "info"
	return 0
}

# Build and install patched GStreamer 1.24.2 with Rockchip RK3576 patches
# 4 sub-packages: core (4 patches), plugins-base (18), plugins-good (13), plugins-bad (36)
# Patches: kmssink improvements, v4l2 enhancements, RGA 2D accel, hardware codec support
# Pre-patched source tarballs: cache/rk3576-packages/sources/gstreamer/
function post_family_tweaks__myd_lr3576_build_gstreamer() {
	local cache_dir="${SRC}/cache/rk3576-packages"
	local gst_src_dir="${cache_dir}/sources/gstreamer"
	local gst_cache_dir="${cache_dir}/gstreamer-debs"

	# Check for cached .debs first
	if [[ -d "$gst_cache_dir" ]] && ls "${gst_cache_dir}"/libgstreamer*.deb &>/dev/null; then
		display_alert "$BOARD" "Using cached patched GStreamer packages" "info"
	else
		display_alert "$BOARD" "Building patched GStreamer in chroot (this will take a while)" "info"

		# Verify source tarballs exist
		local missing=0
		for f in gstreamer1-1.24.2-rk3576-patched.tar.xz \
				 gst-plugins-base1.0-1.24.2-rk3576-patched.tar.xz \
				 gst-plugins-good1.0-1.24.2-rk3576-patched.tar.xz \
				 gst-plugins-bad1.0-1.24.2-rk3576-patched.tar.xz; do
			if [[ ! -f "${gst_src_dir}/${f}" ]]; then
				display_alert "Missing GStreamer source" "${f}" "err"
				missing=1
			fi
		done
		[[ $missing -eq 1 ]] && return 1

		# Enable deb-src repos for build-dep
		_enable_deb_src_repos

		# Install build dependencies for all 4 sub-packages
		chroot_sdcard_apt_get_install devscripts dpkg-dev meson ninja-build || true
		chroot_sdcard apt-get build-dep -y gstreamer1.0 gst-plugins-base1.0 gst-plugins-good1.0 gst-plugins-bad1.0 || {
			display_alert "Failed to install GStreamer build deps" "" "wrn"
			return 1
		}

		mkdir -p "${gst_cache_dir}"
		local build_dir="/tmp/gst-build"
		mkdir -p "${SDCARD}${build_dir}"

		# Build order: core -> base -> good -> bad
		local -A gst_pkgs=(
			[1-gstreamer1.0]="gstreamer1-1.24.2-rk3576-patched.tar.xz|gstreamer1.0_1.24.2-1ubuntu0.1.debian.tar.xz|gstreamer-1.24.2|1.24.2-1ubuntu0.1"
			[2-gst-plugins-base1.0]="gst-plugins-base1.0-1.24.2-rk3576-patched.tar.xz|gst-plugins-base1.0_1.24.2-1ubuntu0.3.debian.tar.xz|gst-plugins-base-1.24.2|1.24.2-1ubuntu0.3"
			[3-gst-plugins-good1.0]="gst-plugins-good1.0-1.24.2-rk3576-patched.tar.xz|gst-plugins-good1.0_1.24.2-1ubuntu1.2.debian.tar.xz|gst-plugins-good-1.24.2|1.24.2-1ubuntu1.2"
			[4-gst-plugins-bad1.0]="gst-plugins-bad1.0-1.24.2-rk3576-patched.tar.xz|gst-plugins-bad1.0_1.24.2-1ubuntu4.debian.tar.xz|gst-plugins-bad-1.24.2|1.24.2-1ubuntu4"
		)

		local failed=0
		for key in $(echo "${!gst_pkgs[@]}" | tr ' ' '\n' | sort); do
			local pkg_name="${key#*-}"
			IFS='|' read -r src_tar deb_tar src_dirname orig_version <<< "${gst_pkgs[$key]}"

			display_alert "Building GStreamer" "${pkg_name}" "info"

			# Extract pre-patched source
			tar xf "${gst_src_dir}/${src_tar}" -C "${SDCARD}${build_dir}/"

			# Extract debian packaging
			if [[ -f "${gst_src_dir}/${deb_tar}" ]]; then
				tar xf "${gst_src_dir}/${deb_tar}" -C "${SDCARD}${build_dir}/${src_dirname}/"
			fi

			# Update changelog
			local changelog="${SDCARD}${build_dir}/${src_dirname}/debian/changelog"
			if [[ -f "$changelog" ]]; then
				local tmpfile
				tmpfile=$(mktemp)
				cat > "$tmpfile" << CHLOG
${pkg_name} (${orig_version}+rk3576.1) noble; urgency=medium

  * Apply Rockchip RK3576 patches from MYIR SDK

 -- Armbian Builder <builder@armbian.com>  $(date -R)

CHLOG
				cat "$changelog" >> "$tmpfile"
				mv "$tmpfile" "$changelog"
			fi

			# Build
			chroot_sdcard "cd ${build_dir}/${src_dirname} && DEB_BUILD_OPTIONS='nocheck nostrip parallel=\$(nproc)' dpkg-buildpackage -us -uc -b" || {
				display_alert "GStreamer build FAILED" "${pkg_name}" "err"
				failed=1
				continue
			}

			# Install immediately (later packages depend on earlier ones)
			for deb in "${SDCARD}${build_dir}"/*.deb; do
				[[ -f "$deb" ]] || continue
				[[ "$(basename "$deb")" == *-dbgsym* ]] && continue
				cp "$deb" "${gst_cache_dir}/"
				chroot_sdcard dpkg -i "/tmp/gst-build/$(basename "$deb")" 2>/dev/null || true
			done
			chroot_sdcard_apt_get_install -f || true

			# Clean up this sub-package build artifacts (keep source dirs for deps)
			rm -f "${SDCARD}${build_dir}"/*.deb "${SDCARD}${build_dir}"/*.changes \
				  "${SDCARD}${build_dir}"/*.buildinfo "${SDCARD}${build_dir}"/*.ddeb 2>/dev/null

			display_alert "GStreamer build complete" "${pkg_name}" "info"
		done

		# Clean up build directory
		rm -rf "${SDCARD}${build_dir}"

		# NOTE: deb-src repos left enabled for build_libdrm/build_xserver (restored in build_xserver)

		if [[ $failed -eq 1 ]]; then
			display_alert "Some GStreamer packages failed to build" "check output above" "wrn"
		fi
	fi

	# Install cached .debs (skip if just built - already installed above)
	if [[ -d "$gst_cache_dir" ]]; then
		display_alert "$BOARD" "Installing patched GStreamer packages" "info"
		for deb in "${gst_cache_dir}"/*.deb; do
			[[ -f "$deb" ]] || continue
			[[ "$(basename "$deb")" == *-dbgsym* ]] && continue
			cp "$deb" "${SDCARD}/tmp/"
			chroot_sdcard dpkg -i "/tmp/$(basename "$deb")" 2>/dev/null || true
		done
		chroot_sdcard_apt_get_install -f || true
		rm -f "${SDCARD}"/tmp/gst*.deb "${SDCARD}"/tmp/libgst*.deb

		# Hold key packages to prevent apt overwriting
		chroot_sdcard apt-mark hold libgstreamer1.0-0 gstreamer1.0-plugins-base \
			gstreamer1.0-plugins-good gstreamer1.0-plugins-bad 2>/dev/null || true

		display_alert "Patched GStreamer installed and held" "71 Rockchip patches" "info"
	fi

	return 0
}

# Configure Xorg for Rockchip VOP2 with Mali G52 Glamor acceleration
# Configuration matches MYIR MYD-LR3576 SDK (Glamor, DRI2, FlipFB, UseGammaLUT)
# card0 = RKNPU (not display), card1 = rockchip-drm (VOP2)
function post_family_tweaks__myd_lr3576_xorg_config() {
	[[ "${BUILD_DESKTOP}" != "yes" ]] && return 0

	display_alert "$BOARD" "Configuring Xorg for Rockchip GPU acceleration" "info"

	mkdir -p "${SDCARD}/etc/X11/xorg.conf.d"

	# Xorg configuration from MYIR SDK with Glamor acceleration
	# NOTE: FlipFB removed - causes compositor deadlock with Muffin/Clutter on Mali Bifrost
	cat > "${SDCARD}/etc/X11/xorg.conf.d/01-armbian-defaults.conf" <<- 'XORGEOF'
	Section "OutputClass"
	    Identifier  "RockchipDRM"
	    MatchDriver "rockchip"
	    Option      "PrimaryGPU"     "yes"
	EndSection

	Section "Device"
	    Identifier  "Rockchip Graphics"
	    Driver      "modesetting"
	    Option      "AccelMethod"    "glamor"
	    Option      "DRI"            "2"
	    Option      "NoEDID"         "true"
	    Option      "UseGammaLUT"    "true"
	EndSection

	Section "Screen"
	    Identifier  "Default Screen"
	    Device      "Rockchip Graphics"
	    Monitor     "Default Monitor"
	EndSection

	Section "Monitor"
	    Identifier  "Default Monitor"
	    Option      "Rotate"         "normal"
	EndSection
	XORGEOF

	# Mali Bifrost X11 environment (DRI2 preferred over DRI3 for stability)
	mkdir -p "${SDCARD}/etc/profile.d"
	cat > "${SDCARD}/etc/profile.d/rockchip-gpu.sh" <<- 'ENVEOF'
	export MALI_X11_PREFER_DRI3=0
	ENVEOF

	# Disable XFCE compositor (xfwm4) - Mali Bifrost causes deadlock in compositing WMs
	# MYIR SDK uses Openbox (non-compositing) for the same reason
	if [[ "${DESKTOP_ENVIRONMENT}" == "xfce" ]]; then
		local xfce_defaults="${SDCARD}/etc/xdg/xfce4/xfconf/xfce-perchannel-xml"
		mkdir -p "${xfce_defaults}"
		cat > "${xfce_defaults}/xfwm4.xml" <<- 'XFCEEOF'
		<?xml version="1.0" encoding="UTF-8"?>
		<channel name="xfwm4" version="1.0">
		  <property name="general" type="empty">
		    <property name="use_compositing" type="bool" value="false"/>
		  </property>
		</channel>
		XFCEEOF
	fi

	# Enable display manager - patched xserver with Glamor BGR fix is installed
	chroot_sdcard systemctl enable lightdm 2>/dev/null || {
		# lightdm may lack [Install] section, use display-manager.service symlink
		mkdir -p "${SDCARD}/etc/systemd/system"
		ln -sf /usr/lib/systemd/system/lightdm.service \
			"${SDCARD}/etc/systemd/system/display-manager.service" 2>/dev/null || true
	}

	display_alert "Xorg configured with Glamor + DRI2" "lightdm enabled" "info"
	return 0
}

# Copy BL32 (OP-TEE) to tee.bin and enable OP-TEE in FIT before U-Boot compilation
function pre_config_uboot_target__myd_lr3576_optee_setup() {
	if [[ -n "${BL32_BLOB}" && -f "${RKBIN_DIR}/${BL32_BLOB}" ]]; then
		display_alert "Enabling OP-TEE support" "Copying ${BL32_BLOB} and enabling gen_bl32_node" "info"

		# Copy BL32 blob to tee.bin
		run_host_command_logged cp -v "${RKBIN_DIR}/${BL32_BLOB}" tee.bin

		# Uncomment gen_bl32_node in make_fit_atf.sh to include OP-TEE in FIT image
		if grep -q "^#gen_bl32_node" arch/arm/mach-rockchip/make_fit_atf.sh; then
			sed -i 's/^#gen_bl32_node/gen_bl32_node/' arch/arm/mach-rockchip/make_fit_atf.sh
			display_alert "Enabled gen_bl32_node" "in make_fit_atf.sh" "info"
		fi
	fi
}

