/*
 * nvp_reg - NVP6324 register read/write/detect/motion tool
 *
 * Uses existing ioctls on /dev/jaguar2 to access NVP6324 registers
 * via GPIO I2C from userspace. No kernel changes required.
 *
 * Ioctls used:
 *   0xB2 (BANK_DUMP_GET)     - Read all 256 regs in a bank
 *   0xA7 (COAX_TEST_DATA_SET) - Write single register
 *   0xA8 (COAX_TEST_DATA_READ)- Read single register (fast)
 *   0x70-0x76 (MOTION_*)     - Motion detection control
 *
 * Usage:
 *   nvp_reg read <bank> <reg>           Read single register
 *   nvp_reg write <bank> <reg> <val>    Write single register
 *   nvp_reg dump <bank>                 Dump full bank (256 regs)
 *   nvp_reg status                      Show all channel status
 *   nvp_reg detect                      Show format detection state
 *   nvp_reg vfc [channel]               Enable VFC and read detected format
 *   nvp_reg autodetect [channel]        Full auto-detection sequence
 *   nvp_reg motion status               Show motion detection state
 *   nvp_reg motion on <ch>              Enable motion detection
 *   nvp_reg motion off <ch>             Disable motion detection
 *   nvp_reg motion tsen <ch> <0-255>    Set temporal sensitivity
 *   nvp_reg motion psen <ch> <0-7>      Set pixel sensitivity
 *   nvp_reg motion mask <ch> [val]      Set/get pixel mask (24 bytes)
 *   nvp_reg motion poll                 Poll motion events (Ctrl-C)
 *   nvp_reg monitor                     Continuously poll status
 *
 * Build (on board):
 *   gcc -O2 -o nvp_reg nvp_reg.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <time.h>

/* ---- Ioctl command defines (from jaguar2_ioctl.h) ---- */
#define IOC_VDEC_MOTION_SET           0x70
#define IOC_VDEC_MOTION_PIXEL_SET     0x71
#define IOC_VDEC_MOTION_PIXEL_GET     0x72
#define IOC_VDEC_MOTION_TSEN_SET      0x73
#define IOC_VDEC_MOTION_PSEN_SET      0x74
#define IOC_VDEC_MOTION_ALL_PIXEL_SET 0x75
#define IOC_VDEC_MOTION_DETECTION_GET 0x76
#define IOC_VDEC_COAX_TEST_DATA_SET   0xA7
#define IOC_VDEC_COAX_TEST_DATA_READ  0xA8
#define IOC_VDEC_COAX_BANK_DUMP_GET   0xB2

/* ---- Struct definitions (from driver headers) ---- */

/* For bank dump (read 256 regs) */
typedef struct {
	unsigned char ch;
	unsigned char vd_dev;
	unsigned char bank;
	unsigned char rx_pelco_data[256];
} NC_VD_COAX_BANK_DUMP_STR;

/* For single register read/write */
typedef struct {
	unsigned char ch;
	unsigned char chip_num;
	unsigned char bank;
	unsigned char data_addr;
	unsigned char param;
	/* Remaining fields from NC_VD_COAX_TEST_STR - must match kernel struct size */
	unsigned char rx_src;
	unsigned char rx_slice_lev;
	unsigned char tx_baud;
	unsigned char tx_pel_baud;
	unsigned char tx_line_pos0;
	unsigned char tx_line_pos1;
	unsigned char tx_pel_line_pos0;
	unsigned char tx_pel_line_pos1;
	unsigned char tx_line_count;
	unsigned char tx_line_count_max;
	unsigned char tx_mode;
	unsigned char tx_sync_pos0;
	unsigned char tx_sync_pos1;
	unsigned char tx_even;
	unsigned char tx_zero_length;
} NC_VD_COAX_TEST_STR;

/* For motion detection */
typedef struct {
	unsigned char ch;
	unsigned char devnum;
	unsigned char set_val;
	unsigned char fmtdef;
} motion_mode;

#define NVP_DEV "/dev/jaguar2"
#define FUNC_ON  1
#define FUNC_OFF 0

static volatile int g_running = 1;

static void sigint_handler(int sig)
{
	(void)sig;
	g_running = 0;
}

/* ---- Low-level register access ---- */

static int bank_read(int fd, unsigned char bank, unsigned char data[256])
{
	NC_VD_COAX_BANK_DUMP_STR dump;

	memset(&dump, 0, sizeof(dump));
	dump.vd_dev = 0;
	dump.bank = bank;

	if (ioctl(fd, IOC_VDEC_COAX_BANK_DUMP_GET, &dump) < 0) {
		perror("ioctl BANK_DUMP_GET");
		return -1;
	}

	memcpy(data, dump.rx_pelco_data, 256);
	return 0;
}

static int reg_read(int fd, unsigned char bank, unsigned char reg)
{
	unsigned char data[256];

	if (bank_read(fd, bank, data) < 0)
		return -1;

	return data[reg];
}

/* Fast single-register read via COAX_TEST_DATA_READ */
static int reg_read_fast(int fd, unsigned char bank, unsigned char reg)
{
	NC_VD_COAX_TEST_STR test;

	memset(&test, 0, sizeof(test));
	test.chip_num = 0;
	test.bank = bank;
	test.data_addr = reg;

	if (ioctl(fd, IOC_VDEC_COAX_TEST_DATA_READ, &test) < 0) {
		perror("ioctl COAX_TEST_DATA_READ");
		return -1;
	}

	return test.param;
}

/* Single register write via COAX_TEST_DATA_SET */
static int reg_write(int fd, unsigned char bank, unsigned char reg,
		     unsigned char val)
{
	NC_VD_COAX_TEST_STR test;

	memset(&test, 0, sizeof(test));
	test.chip_num = 0;
	test.bank = bank;
	test.data_addr = reg;
	test.param = val;

	if (ioctl(fd, IOC_VDEC_COAX_TEST_DATA_SET, &test) < 0) {
		perror("ioctl COAX_TEST_DATA_SET");
		return -1;
	}

	return 0;
}

/* ---- Name lookup tables ---- */

static const char *ahd_mode_name(unsigned char val)
{
	switch (val) {
	case 0x00: return "SD/off";
	case 0x02: return "1080p30";
	case 0x03: return "1080p25";
	case 0x04: return "720p60";
	case 0x05: return "720p50";
	case 0x0C: return "720p30";
	case 0x0D: return "720p25";
	default:   return "unknown";
	}
}

static const char *sd_mode_name(unsigned char val)
{
	switch (val) {
	case 0x00: return "off";
	case 0x0E: return "SD_NTSC";
	case 0x0F: return "SD_PAL";
	default:   return "unknown";
	}
}

static const char *vfc_name(unsigned char val)
{
	switch (val) {
	case 0xFF: return "no video";
	case 0x03: return "AHD 1080p25";
	case 0x04: return "AHD 1080p30";
	case 0x0D: return "AHD 720p25";
	case 0x0E: return "AHD 720p30";
	case 0x05: return "AHD 720p50";
	case 0x06: return "AHD 720p60";
	case 0x01: return "AHD 720p25 EX_B";
	case 0x02: return "AHD 720p30 EX_B";
	case 0x21: return "CVBS PAL";
	case 0x22: return "CVBS NTSC";
	case 0x00: return "SD/default";
	default:   return "unknown";
	}
}

/* Map VFC raw value to resolution string for autodetect output */
static const char *vfc_resolution(unsigned char val)
{
	switch (val) {
	case 0x03: return "1920x1080p25";
	case 0x04: return "1920x1080p30";
	case 0x0D: case 0x01: return "1280x720p25";
	case 0x0E: case 0x02: return "1280x720p30";
	case 0x05: return "1280x720p50";
	case 0x06: return "1280x720p60";
	case 0x21: return "960x576i50 (PAL)";
	case 0x22: return "960x480i60 (NTSC)";
	default:   return "unknown";
	}
}

/* Map VFC to DV timings index (for v4l2-ctl --set-dv-timings) */
static int vfc_to_dv_index(unsigned char val)
{
	switch (val) {
	case 0x22: return 0;  /* CVBS NTSC */
	case 0x21: return 1;  /* CVBS PAL */
	case 0x0D: case 0x01: return 2;  /* AHD 720p25 */
	case 0x0E: case 0x02: return 3;  /* AHD 720p30 */
	case 0x05: return 4;  /* AHD 720p50 */
	case 0x06: return 5;  /* AHD 720p60 */
	case 0x03: return 6;  /* AHD 1080p25 */
	case 0x04: return 7;  /* AHD 1080p30 */
	default:   return -1;
	}
}

/*
 * Lock-based detection: when VFC returns 0x00 (doesn't work on this chip
 * variant), use the AHD_MODE and SD_MODE registers that the NVP6324
 * auto-configures based on the input signal.
 *
 * Returns DV timings index (>=0) or -1 if not determinable.
 * Fills name/resolution strings if pointers are non-NULL.
 */
static int lock_based_detect(unsigned char ahd_mode, unsigned char sd_mode,
			     const char **name, const char **resolution)
{
	/* AHD modes (from driver ahd_mode field) */
	if (ahd_mode != 0x00) {
		if (name) *name = ahd_mode_name(ahd_mode);
		switch (ahd_mode) {
		case 0x03:
			if (resolution) *resolution = "1920x1080p25";
			return 6;
		case 0x02:
			if (resolution) *resolution = "1920x1080p30";
			return 7;
		case 0x0D:
			if (resolution) *resolution = "1280x720p25";
			return 2;
		case 0x0C:
			if (resolution) *resolution = "1280x720p30";
			return 3;
		case 0x05:
			if (resolution) *resolution = "1280x720p50";
			return 4;
		case 0x04:
			if (resolution) *resolution = "1280x720p60";
			return 5;
		default:
			if (resolution) *resolution = "unknown";
			return -1;
		}
	}

	/* SD modes (CVBS) */
	if (sd_mode == 0x0F) {
		if (name) *name = "CVBS PAL";
		if (resolution) *resolution = "960x576i50 (PAL)";
		return 1;
	}
	if (sd_mode == 0x0E) {
		if (name) *name = "CVBS NTSC";
		if (resolution) *resolution = "960x480i60 (NTSC)";
		return 0;
	}

	if (name) *name = "unknown";
	if (resolution) *resolution = "unknown";
	return -1;
}

static const char *motion_pic_name(unsigned char val)
{
	switch (val) {
	case 0: return "none";
	case 1: return "EVEN_FLD(luma-32)";
	case 2: return "EVEN_FLD(luma-48)";
	case 3: return "ALL_FLD(luma-48)";
	default: return "unknown";
	}
}

static const char *psen_name(unsigned char val)
{
	switch (val & 0x07) {
	case 0: return "bypass";
	case 1: return "1/2";
	case 2: return "1/4";
	case 3: return "1/8";
	case 4: return "1/16";
	case 5: return "1/32";
	default: return "1/64";
	}
}

/* ---- VFC (Video Format Check) ---- */

/*
 * Enable VFC auto-detection on a channel.
 * Reverses vd_vi_manual_set_seq1() which disables VFC during init.
 *
 * Bank 0x13 reg 0x30: set bits (1<<ch) | (1<<(ch+4))
 * Bank 0x13 reg 0x31: set bits (1<<ch) | (1<<(ch+4))
 * Bank 0x13 reg 0x32: set bit (1<<ch)
 * Bank 0x01 reg 0x7C: set bit (1<<ch) — clock auto-select
 */
static int vfc_enable(int fd, int ch)
{
	unsigned char val;
	int r;

	/* Bank 0x13 reg 0x30: VFC_INIT_EN */
	r = reg_read_fast(fd, 0x13, 0x30);
	if (r < 0) return -1;
	val = (unsigned char)r;
	val |= (1 << ch) | (1 << (ch + 4));
	if (reg_write(fd, 0x13, 0x30, val) < 0) return -1;

	/* Bank 0x13 reg 0x31: AHD_DET_EN */
	r = reg_read_fast(fd, 0x13, 0x31);
	if (r < 0) return -1;
	val = (unsigned char)r;
	val |= (1 << ch) | (1 << (ch + 4));
	if (reg_write(fd, 0x13, 0x31, val) < 0) return -1;

	/* Bank 0x13 reg 0x32: CVI_DET_EN */
	r = reg_read_fast(fd, 0x13, 0x32);
	if (r < 0) return -1;
	val = (unsigned char)r;
	val |= (1 << ch);
	if (reg_write(fd, 0x13, 0x32, val) < 0) return -1;

	/* Bank 0x01 reg 0x7C: clock auto-select */
	r = reg_read_fast(fd, 0x01, 0x7C);
	if (r < 0) return -1;
	val = (unsigned char)r;
	val |= (1 << ch);
	if (reg_write(fd, 0x01, 0x7C, val) < 0) return -1;

	return 0;
}

/* Read VFC result from Bank 0x05+ch reg 0xF0 */
static int vfc_read(int fd, int ch)
{
	return reg_read_fast(fd, 0x05 + ch, 0xF0);
}

/* ---- Command implementations ---- */

static void cmd_read(int fd, int argc, char **argv)
{
	unsigned long bank, reg;
	int val;

	if (argc < 2) {
		fprintf(stderr, "Usage: nvp_reg read <bank> <reg>\n");
		return;
	}

	bank = strtoul(argv[0], NULL, 0);
	reg = strtoul(argv[1], NULL, 0);

	if (bank > 0xFF || reg > 0xFF) {
		fprintf(stderr, "Bank and reg must be 0x00-0xFF\n");
		return;
	}

	val = reg_read_fast(fd, bank, reg);
	if (val >= 0)
		printf("Bank 0x%02X Reg 0x%02X = 0x%02X (%d)\n",
		       (unsigned)bank, (unsigned)reg, val, val);
}

static void cmd_write(int fd, int argc, char **argv)
{
	unsigned long bank, reg, val;

	if (argc < 3) {
		fprintf(stderr, "Usage: nvp_reg write <bank> <reg> <value>\n");
		return;
	}

	bank = strtoul(argv[0], NULL, 0);
	reg = strtoul(argv[1], NULL, 0);
	val = strtoul(argv[2], NULL, 0);

	if (bank > 0xFF || reg > 0xFF || val > 0xFF) {
		fprintf(stderr, "Bank, reg, and value must be 0x00-0xFF\n");
		return;
	}

	if (reg_write(fd, bank, reg, val) == 0) {
		/* Read back to verify */
		int readback = reg_read_fast(fd, bank, reg);
		if (readback >= 0) {
			printf("Bank 0x%02X Reg 0x%02X: wrote 0x%02X, readback 0x%02X%s\n",
			       (unsigned)bank, (unsigned)reg, (unsigned)val,
			       readback,
			       (readback == (int)val) ? " (OK)" : " (MISMATCH - read-only?)");
		}
	}
}

static void cmd_dump(int fd, int argc, char **argv)
{
	unsigned long bank;
	unsigned char data[256];
	int i;

	if (argc < 1) {
		fprintf(stderr, "Usage: nvp_reg dump <bank>\n");
		return;
	}

	bank = strtoul(argv[0], NULL, 0);
	if (bank > 0xFF) {
		fprintf(stderr, "Bank must be 0x00-0xFF\n");
		return;
	}

	if (bank_read(fd, bank, data) < 0)
		return;

	printf("Bank 0x%02X:\n", (unsigned)bank);
	printf("     ");
	for (i = 0; i < 16; i++)
		printf("  %X", i);
	printf("\n");

	for (i = 0; i < 256; i++) {
		if ((i % 16) == 0)
			printf("  %02X:", i);
		printf(" %02X", data[i]);
		if ((i % 16) == 15)
			printf("\n");
	}
}

static void cmd_status(int fd)
{
	unsigned char b0[256], b5[256], b6[256], b7[256], b8[256];
	int ch;

	if (bank_read(fd, 0x00, b0) < 0) return;
	if (bank_read(fd, 0x05, b5) < 0) return;
	if (bank_read(fd, 0x06, b6) < 0) return;
	if (bank_read(fd, 0x07, b7) < 0) return;
	if (bank_read(fd, 0x08, b8) < 0) return;

	printf("=== NVP6324 Channel Status ===\n\n");

	/* Global status */
	printf("NOVID (0xA0):    0x%02X  [", b0[0xA0]);
	for (ch = 0; ch < 4; ch++)
		printf(" ch%d=%s", ch, (b0[0xA0] & (1 << ch)) ? "NOVID" : "video");
	printf(" ]\n");

	printf("NOVID_B (0xB0):  0x%02X  [", b0[0xB0]);
	for (ch = 0; ch < 4; ch++)
		printf(" ch%d=%s", ch, (b0[0xB0] & (1 << ch)) ? "NOVID" : "video");
	printf(" ]\n");

	printf("MOTION (0xB1):   0x%02X  [", b0[0xB1]);
	for (ch = 0; ch < 4; ch++)
		printf(" ch%d=%s", ch, (b0[0xB1] & (1 << ch)) ? "MOT" : "---");
	printf(" ]\n\n");

	printf("DevID: 0x%02X  RevID: 0x%02X\n\n", b0[0xF4], b0[0xF5]);

	/* Per-channel */
	printf("%-4s %-8s %-8s %-6s %-6s %-6s %-6s %-8s %-8s\n",
	       "Ch", "AHD_MD", "SD_MD", "AGC", "CMP", "HLOCK", "BW", "VidFmt", "NOVID_B");

	for (ch = 0; ch < 4; ch++) {
		unsigned char ahd = b0[0x08 + ch];
		unsigned char sd = b0[0x04 + ch];
		unsigned char agc = b0[0xD0 + ch];
		unsigned char cmp = b0[0xD4 + ch];
		unsigned char hlock = b0[0xD8 + ch];
		unsigned char bw = b0[0xDC + ch];
		unsigned char vfmt = b0[0x10 + ch];
		unsigned char novid_b = b0[0x18 + ch];

		printf("%-4d 0x%02X=%-4s 0x%02X=%-4s 0x%02X   0x%02X   0x%02X   0x%02X   0x%02X     0x%02X\n",
		       ch,
		       ahd, ahd_mode_name(ahd),
		       sd, sd_mode_name(sd),
		       agc, cmp, hlock, bw, vfmt, novid_b);
	}

	/* VFC per channel (Bank 0x05+ch, reg 0xF0) */
	printf("\n%-4s %-12s %-12s %-12s %-12s\n",
	       "Ch", "VFC (0xF0)", "SyncRS", "CmlMode", "AgcOp");

	unsigned char *bch[4] = { b5, b6, b7, b8 };
	for (ch = 0; ch < 4; ch++) {
		unsigned char vfc = bch[ch][0xF0];
		unsigned char sync_rs = bch[ch][0x47];
		unsigned char cml = bch[ch][0x01];
		unsigned char agc_op = bch[ch][0x05];

		printf("%-4d 0x%02X=%-6s 0x%02X       0x%02X       0x%02X\n",
		       ch, vfc, vfc_name(vfc), sync_rs, cml, agc_op);
	}

	/* Clock regs */
	unsigned char b1[256];
	if (bank_read(fd, 0x01, b1) < 0) return;

	printf("\n%-4s %-8s %-8s %-8s %-8s\n",
	       "Ch", "ClkADC", "ClkPre", "ClkPost", "ClkSel");

	for (ch = 0; ch < 4; ch++) {
		printf("%-4d 0x%02X     0x%02X     0x%02X     0x%02X\n",
		       ch, b1[0x84 + ch], b1[0x88 + ch],
		       b1[0x8C + ch], b1[0x7C + ch]);
	}
}

static void cmd_detect(int fd)
{
	unsigned char b0[256], b13[256];
	unsigned char bch[4][256];
	int ch;

	if (bank_read(fd, 0x00, b0) < 0) return;
	if (bank_read(fd, 0x13, b13) < 0) return;

	for (ch = 0; ch < 4; ch++)
		if (bank_read(fd, 0x05 + ch, bch[ch]) < 0) return;

	printf("=== NVP6324 Format Detection ===\n\n");

	printf("NOVID (0xA0): 0x%02X\n", b0[0xA0]);
	printf("NOVID_B (0xB0): 0x%02X\n\n", b0[0xB0]);

	/* VFC control regs (Bank 0x13) */
	printf("VFC Control (Bank 0x13):\n");
	printf("  0x30 (VFC_INIT_EN):   0x%02X\n", b13[0x30]);
	printf("  0x31 (AHD_DET_EN):    0x%02X\n", b13[0x31]);
	printf("  0x32 (CVI_DET_EN):    0x%02X\n", b13[0x32]);
	printf("  0x70 (VFC_STATUS_0):  0x%02X\n", b13[0x70]);
	printf("  0x71 (VFC_STATUS_1):  0x%02X\n", b13[0x71]);
	printf("  0x72 (VFC_STATUS_2):  0x%02X\n", b13[0x72]);
	printf("  0x73 (VFC_STATUS_3):  0x%02X\n\n", b13[0x73]);

	printf("%-4s %-10s %-10s %-10s %-10s %-8s %-8s %-8s\n",
	       "Ch", "VFC(0xF0)", "AHD_MD", "SD_MD", "VidFmt",
	       "AGC", "HLOCK", "BW");

	for (ch = 0; ch < 4; ch++) {
		unsigned char vfc = bch[ch][0xF0];
		unsigned char ahd = b0[0x08 + ch];
		unsigned char sd = b0[0x04 + ch];
		unsigned char vfmt = b0[0x10 + ch];
		unsigned char agc = b0[0xD0 + ch];
		unsigned char hlock = b0[0xD8 + ch];
		unsigned char bw = b0[0xDC + ch];

		printf("%-4d 0x%02X=%-5s 0x%02X=%-5s 0x%02X=%-5s 0x%02X     0x%02X   0x%02X   0x%02X\n",
		       ch,
		       vfc, vfc_name(vfc),
		       ahd, ahd_mode_name(ahd),
		       sd, sd_mode_name(sd),
		       vfmt, agc, hlock, bw);
	}

	/* Sync status from Bank 0 */
	printf("\n%-4s %-10s %-10s %-10s %-10s\n",
	       "Ch", "Sync(D0)", "HPer(D8)", "CmpLk(D4)", "A4_status");

	for (ch = 0; ch < 4; ch++) {
		printf("%-4d 0x%02X       0x%02X       0x%02X       0x%02X\n",
		       ch, b0[0xD0 + ch], b0[0xD8 + ch],
		       b0[0xD4 + ch], b0[0xA4 + ch]);
	}
}

static void cmd_vfc(int fd, int argc, char **argv)
{
	int ch_start = 0, ch_end = 3;
	int ch, vfc;

	if (argc >= 1) {
		ch_start = ch_end = atoi(argv[0]);
		if (ch_start < 0 || ch_start > 3) {
			fprintf(stderr, "Channel must be 0-3\n");
			return;
		}
	}

	printf("=== VFC Auto-Detect ===\n\n");

	for (ch = ch_start; ch <= ch_end; ch++) {
		printf("ch%d: Enabling VFC... ", ch);
		fflush(stdout);

		if (vfc_enable(fd, ch) < 0) {
			printf("FAILED\n");
			continue;
		}

		/* Wait for VFC to settle — 150ms for reliable detection */
		usleep(150000);

		vfc = vfc_read(fd, ch);
		if (vfc < 0) {
			printf("read FAILED\n");
			continue;
		}

		printf("raw=0x%02X -> %s", vfc, vfc_name(vfc));
		if (vfc != 0xFF && vfc != 0x00) {
			int idx = vfc_to_dv_index(vfc);
			printf(" (%s)", vfc_resolution(vfc));
			if (idx >= 0)
				printf("  [DV timings index=%d]", idx);
		}
		printf("\n");
	}

	/* Also show VFC control state */
	printf("\nVFC Control after enable:\n");
	int r30 = reg_read_fast(fd, 0x13, 0x30);
	int r31 = reg_read_fast(fd, 0x13, 0x31);
	int r32 = reg_read_fast(fd, 0x13, 0x32);
	if (r30 >= 0 && r31 >= 0 && r32 >= 0)
		printf("  VFC_INIT_EN=0x%02X  AHD_DET_EN=0x%02X  CVI_DET_EN=0x%02X\n",
		       r30, r31, r32);
}

static void cmd_autodetect(int fd, int argc, char **argv)
{
	int ch_start = 0, ch_end = 3;
	int ch, pass;

	if (argc >= 1) {
		ch_start = ch_end = atoi(argv[0]);
		if (ch_start < 0 || ch_start > 3) {
			fprintf(stderr, "Channel must be 0-3\n");
			return;
		}
	}

	printf("=== NVP6324 Auto-Detection ===\n\n");

	/* Step 1: Read current NOVID status */
	unsigned char b0[256];
	if (bank_read(fd, 0x00, b0) < 0) return;

	printf("Step 1: Video presence (NOVID)\n");
	for (ch = ch_start; ch <= ch_end; ch++) {
		int novid = (b0[0xA0] >> ch) & 1;
		int novid_b = (b0[0xB0] >> ch) & 1;
		printf("  ch%d: %s (NOVID=0x%02X, NOVID_B=0x%02X)\n",
		       ch, novid ? "NO VIDEO" : "VIDEO PRESENT",
		       b0[0xA0], b0[0xB0]);
		(void)novid_b;
	}

	/* Step 2: Enable VFC on all target channels */
	printf("\nStep 2: Enabling VFC auto-detect...\n");
	for (ch = ch_start; ch <= ch_end; ch++) {
		if (vfc_enable(fd, ch) < 0)
			printf("  ch%d: VFC enable FAILED\n", ch);
	}

	/* Step 3: Multiple reads with increasing delays for stability */
	printf("\nStep 3: Reading VFC (3 passes, 100ms apart)...\n");
	unsigned char vfc_vals[4][3];
	memset(vfc_vals, 0xFF, sizeof(vfc_vals));

	for (pass = 0; pass < 3; pass++) {
		usleep(100000); /* 100ms between passes */
		for (ch = ch_start; ch <= ch_end; ch++) {
			int v = vfc_read(fd, ch);
			vfc_vals[ch][pass] = (v >= 0) ? v : 0xFF;
		}
	}

	for (ch = ch_start; ch <= ch_end; ch++) {
		printf("  ch%d: pass0=0x%02X pass1=0x%02X pass2=0x%02X",
		       ch, vfc_vals[ch][0], vfc_vals[ch][1], vfc_vals[ch][2]);
		/* Check stability */
		if (vfc_vals[ch][0] == vfc_vals[ch][1] &&
		    vfc_vals[ch][1] == vfc_vals[ch][2])
			printf(" (STABLE)\n");
		else
			printf(" (UNSTABLE)\n");
	}

	/* Step 4: Read lock status for corroboration */
	printf("\nStep 4: Lock status...\n");
	if (bank_read(fd, 0x00, b0) < 0) return;

	printf("  %-4s %-6s %-6s %-6s %-6s %-10s %-10s\n",
	       "Ch", "AGC", "CMP", "HLOCK", "BW", "AHD_MODE", "SD_MODE");

	for (ch = ch_start; ch <= ch_end; ch++) {
		printf("  %-4d %-6s %-6s %-6s %-6s 0x%02X=%-5s 0x%02X=%-5s\n",
		       ch,
		       (b0[0xD0 + ch] & 1) ? "LOCK" : "----",
		       (b0[0xD4 + ch] & 1) ? "LOCK" : "----",
		       (b0[0xD8 + ch] & 1) ? "LOCK" : "----",
		       (b0[0xDC + ch] & 1) ? "B&W" : "color",
		       b0[0x08 + ch], ahd_mode_name(b0[0x08 + ch]),
		       b0[0x04 + ch], sd_mode_name(b0[0x04 + ch]));
	}

	/* Step 5: Final verdict
	 *
	 * Detection priority:
	 * 1. VFC non-zero/non-0xFF + stable → use VFC mapping
	 * 2. VFC=0x00 + locks → use AHD_MODE/SD_MODE (lock-based)
	 * 3. NOVID + no locks → NO SIGNAL
	 * 4. Otherwise → UNKNOWN
	 *
	 * VFC reg 0xF0 returns 0x00 on NVP6324 for both AHD and CVBS.
	 * Lock-based detection uses the chip's own AHD_MODE/SD_MODE
	 * registers which are auto-configured by the signal decoder.
	 */
	printf("\nStep 5: Detection result\n");
	for (ch = ch_start; ch <= ch_end; ch++) {
		unsigned char final_vfc = vfc_vals[ch][2]; /* Use last reading */
		int novid = (b0[0xA0] >> ch) & 1;
		int agc_lock = b0[0xD0 + ch] & 1;
		int hlock = b0[0xD8 + ch] & 1;
		int cmp_lock = b0[0xD4 + ch] & 1;
		unsigned char ahd_mode = b0[0x08 + ch];
		unsigned char sd_mode = b0[0x04 + ch];
		int stable = (vfc_vals[ch][0] == vfc_vals[ch][1] &&
			      vfc_vals[ch][1] == vfc_vals[ch][2]);

		printf("  ch%d: ", ch);

		if (novid && !agc_lock && !hlock) {
			printf("NO SIGNAL\n");
		} else if (final_vfc != 0xFF && final_vfc != 0x00 && stable) {
			/* VFC gave a real format code */
			int idx = vfc_to_dv_index(final_vfc);
			printf("DETECTED (VFC): %s  %s",
			       vfc_name(final_vfc), vfc_resolution(final_vfc));
			if (agc_lock && hlock)
				printf("  [LOCKED]");
			else
				printf("  [locks: AGC=%d HLOCK=%d]", agc_lock, hlock);
			if (idx >= 0)
				printf("\n         -> v4l2-ctl --set-dv-timings index=%d", idx);
			printf("\n");
		} else if (agc_lock && hlock) {
			/* VFC=0x00 but locks acquired — use AHD_MODE/SD_MODE */
			const char *det_name, *det_res;
			int idx = lock_based_detect(ahd_mode, sd_mode,
						    &det_name, &det_res);
			printf("DETECTED (locks): %s  %s  [AGC+HLOCK+CMP=%d]",
			       det_name, det_res, cmp_lock);
			if (idx >= 0)
				printf("\n         -> v4l2-ctl --set-dv-timings index=%d", idx);
			printf("\n");
		} else if (!stable) {
			printf("UNSTABLE (VFC fluctuating: 0x%02X/0x%02X/0x%02X)\n",
			       vfc_vals[ch][0], vfc_vals[ch][1], vfc_vals[ch][2]);
		} else if (agc_lock && !hlock) {
			/* AGC locked but no HLOCK — signal present but not decoded */
			printf("WEAK SIGNAL (AGC lock, no HLOCK, AHD_MODE=0x%02X)\n",
			       ahd_mode);
		} else {
			printf("UNKNOWN (VFC=0x%02X, NOVID=%d, AGC=%d, HLOCK=%d)\n",
			       final_vfc, novid, agc_lock, hlock);
		}
	}
}

/* ---- Motion detection commands ---- */

static void cmd_motion_status(int fd)
{
	unsigned char b0[256], b4[256];
	int ch;

	if (bank_read(fd, 0x00, b0) < 0) return;
	if (bank_read(fd, 0x04, b4) < 0) return;

	printf("=== NVP6324 Motion Detection Status ===\n\n");

	/* Global motion status from Bank 0 */
	printf("Motion status (Bank0 0xB1): 0x%02X  [", b0[0xB1]);
	for (ch = 0; ch < 4; ch++)
		printf(" ch%d=%s", ch, (b0[0xB1] & (1 << ch)) ? "MOTION" : "------");
	printf(" ]\n");

	/* Also check real-time A4-A7 */
	printf("Per-ch status (Bank0 0xA4): [");
	for (ch = 0; ch < 4; ch++)
		printf(" ch%d=%s", ch, (b0[0xA4 + ch] & 1) ? "MOT" : "---");
	printf(" ]\n\n");

	/* Per-channel config from Bank 4 */
	printf("%-4s %-10s %-16s %-10s %-10s\n",
	       "Ch", "Enabled", "MOTION_PIC", "TSEN", "PSEN");

	for (ch = 0; ch < 4; ch++) {
		int base = 0x07 * ch;
		unsigned char onoff = b4[0x00 + base];
		unsigned char pic = (b4[0x00 + base] >> 2) & 0x03;
		unsigned char tsen = b4[0x01 + base];
		unsigned char psen = b4[0x02 + base];

		printf("%-4d %-10s %-16s 0x%02X=%-4d %-10s\n",
		       ch,
		       (onoff & 1) ? "OFF (0x0D)" : "ON (0x0C)",
		       motion_pic_name(pic),
		       tsen, tsen,
		       psen_name(psen));
	}

	/* Pixel mask summary */
	printf("\nPixel mask (24 bytes per channel, Bank 4):\n");
	for (ch = 0; ch < 4; ch++) {
		int base = 0x40 + (0x18 * ch);
		int i, enabled = 0, total = 192;

		for (i = 0; i < 24; i++) {
			unsigned char byte = b4[base + i];
			/* Count set bits */
			while (byte) {
				enabled += byte & 1;
				byte >>= 1;
			}
		}

		printf("  ch%d: %d/%d pixels enabled (0x%02X-0x%02X)\n",
		       ch, enabled, total, base, base + 23);
	}
}

static void cmd_motion_on(int fd, int ch)
{
	motion_mode m;

	memset(&m, 0, sizeof(m));
	m.ch = ch;
	m.devnum = 0;
	m.set_val = FUNC_ON;
	m.fmtdef = 0; /* Standard AHD/CVBS modes */

	if (ioctl(fd, IOC_VDEC_MOTION_SET, &m) < 0) {
		perror("ioctl MOTION_SET (on)");
		return;
	}

	/* Also enable all pixels (otherwise motion won't detect anything) */
	memset(&m, 0, sizeof(m));
	m.ch = ch;
	m.devnum = 0;
	m.set_val = 0xFF; /* All pixels enabled */
	m.fmtdef = 0;

	if (ioctl(fd, IOC_VDEC_MOTION_ALL_PIXEL_SET, &m) < 0) {
		perror("ioctl MOTION_ALL_PIXEL_SET");
		return;
	}

	printf("ch%d: Motion detection ENABLED (all pixels)\n", ch);
}

static void cmd_motion_off(int fd, int ch)
{
	motion_mode m;

	memset(&m, 0, sizeof(m));
	m.ch = ch;
	m.devnum = 0;
	m.set_val = FUNC_OFF;
	m.fmtdef = 0;

	if (ioctl(fd, IOC_VDEC_MOTION_SET, &m) < 0) {
		perror("ioctl MOTION_SET (off)");
		return;
	}

	printf("ch%d: Motion detection DISABLED\n", ch);
}

static void cmd_motion_tsen(int fd, int ch, int val)
{
	motion_mode m;

	memset(&m, 0, sizeof(m));
	m.ch = ch;
	m.devnum = 0;
	m.set_val = val;
	m.fmtdef = 0;

	if (ioctl(fd, IOC_VDEC_MOTION_TSEN_SET, &m) < 0) {
		perror("ioctl MOTION_TSEN_SET");
		return;
	}

	printf("ch%d: TSEN (temporal sensitivity) = %d (0x%02X)\n", ch, val, val);
	printf("  (0=most sensitive, 255=least sensitive)\n");
}

static void cmd_motion_psen(int fd, int ch, int val)
{
	motion_mode m;

	memset(&m, 0, sizeof(m));
	m.ch = ch;
	m.devnum = 0;
	m.set_val = val;
	m.fmtdef = 0;

	if (ioctl(fd, IOC_VDEC_MOTION_PSEN_SET, &m) < 0) {
		perror("ioctl MOTION_PSEN_SET");
		return;
	}

	printf("ch%d: PSEN (pixel sensitivity) = %d = %s\n", ch, val, psen_name(val));
}

static void cmd_motion_mask(int fd, int ch, int argc, char **argv)
{
	unsigned char b4[256];
	int base = 0x40 + (0x18 * ch);
	int i;

	if (argc >= 1) {
		/* Set mask: write value to all 24 bytes */
		unsigned long val = strtoul(argv[0], NULL, 0);
		if (val > 0xFF) {
			fprintf(stderr, "Mask byte value must be 0x00-0xFF\n");
			return;
		}

		motion_mode m;
		memset(&m, 0, sizeof(m));
		m.ch = ch;
		m.devnum = 0;
		m.set_val = val;
		m.fmtdef = 0;

		if (ioctl(fd, IOC_VDEC_MOTION_ALL_PIXEL_SET, &m) < 0) {
			perror("ioctl MOTION_ALL_PIXEL_SET");
			return;
		}

		printf("ch%d: Set all 24 pixel mask bytes to 0x%02X\n",
		       ch, (unsigned)val);
		printf("  0x00=all disabled, 0xFF=all enabled (%d of 192 pixels)\n",
		       (val == 0xFF) ? 192 : 0);
		return;
	}

	/* Get mask: read all 24 bytes */
	if (bank_read(fd, 0x04, b4) < 0) return;

	printf("ch%d Pixel mask (Bank 0x04, regs 0x%02X-0x%02X):\n",
	       ch, base, base + 23);

	/* Display as bit grid: 24 bytes x 8 bits = 192 pixels */
	int enabled = 0;
	for (i = 0; i < 24; i++) {
		unsigned char byte = b4[base + i];
		int b;

		printf("  [%02d] 0x%02X = ", i, byte);
		for (b = 7; b >= 0; b--) {
			int bit = (byte >> b) & 1;
			printf("%c", bit ? '#' : '.');
			enabled += bit;
		}
		if (i < 23)
			printf("  (pixels %d-%d)\n", i * 8, i * 8 + 7);
		else
			printf("  (pixels %d-%d)\n", i * 8, i * 8 + 7);
	}

	printf("\n  %d of 192 pixels enabled\n", enabled);
}

static void cmd_motion_poll(int fd)
{
	motion_mode m;
	int ch;

	signal(SIGINT, sigint_handler);
	printf("Polling motion detection (Ctrl-C to stop)...\n");
	printf("%-10s", "Time");
	for (ch = 0; ch < 4; ch++)
		printf(" ch%d    ", ch);
	printf("  B1_raw\n");

	while (g_running) {
		struct timespec ts;
		clock_gettime(CLOCK_MONOTONIC, &ts);

		/* Read motion status from Bank 0 reg 0xB1 (with hold) */
		int b1 = reg_read_fast(fd, 0x00, 0xB1);
		/* Also read per-channel real-time from 0xA4-0xA7 */
		int a4[4];
		for (ch = 0; ch < 4; ch++) {
			memset(&m, 0, sizeof(m));
			m.ch = ch;
			m.devnum = 0;
			if (ioctl(fd, IOC_VDEC_MOTION_DETECTION_GET, &m) < 0)
				a4[ch] = -1;
			else
				a4[ch] = m.set_val;
		}

		printf("%3ld.%03ld  ", ts.tv_sec % 1000, ts.tv_nsec / 1000000);
		for (ch = 0; ch < 4; ch++) {
			if (a4[ch] > 0)
				printf(" MOTION ");
			else if (a4[ch] == 0)
				printf(" ------ ");
			else
				printf(" ERROR  ");
		}
		if (b1 >= 0)
			printf("  0x%02X", b1);
		printf("\n");
		fflush(stdout);

		usleep(100000); /* 100ms */
	}
	printf("\nStopped.\n");
}

static void cmd_motion(int fd, int argc, char **argv)
{
	if (argc < 1) {
		fprintf(stderr, "Usage:\n");
		fprintf(stderr, "  nvp_reg motion status             Show motion config & status\n");
		fprintf(stderr, "  nvp_reg motion on <ch>             Enable motion on channel\n");
		fprintf(stderr, "  nvp_reg motion off <ch>            Disable motion on channel\n");
		fprintf(stderr, "  nvp_reg motion tsen <ch> <0-255>   Set temporal sensitivity\n");
		fprintf(stderr, "  nvp_reg motion psen <ch> <0-7>     Set pixel sensitivity\n");
		fprintf(stderr, "  nvp_reg motion mask <ch> [val]     Get/set pixel mask\n");
		fprintf(stderr, "  nvp_reg motion poll                Poll motion events\n");
		return;
	}

	if (strcmp(argv[0], "status") == 0) {
		cmd_motion_status(fd);
	} else if (strcmp(argv[0], "on") == 0) {
		if (argc < 2) {
			fprintf(stderr, "Usage: nvp_reg motion on <channel 0-3>\n");
			return;
		}
		int ch = atoi(argv[1]);
		if (ch < 0 || ch > 3) {
			fprintf(stderr, "Channel must be 0-3\n");
			return;
		}
		cmd_motion_on(fd, ch);
	} else if (strcmp(argv[0], "off") == 0) {
		if (argc < 2) {
			fprintf(stderr, "Usage: nvp_reg motion off <channel 0-3>\n");
			return;
		}
		int ch = atoi(argv[1]);
		if (ch < 0 || ch > 3) {
			fprintf(stderr, "Channel must be 0-3\n");
			return;
		}
		cmd_motion_off(fd, ch);
	} else if (strcmp(argv[0], "tsen") == 0) {
		if (argc < 3) {
			fprintf(stderr, "Usage: nvp_reg motion tsen <channel 0-3> <0-255>\n");
			return;
		}
		int ch = atoi(argv[1]);
		int val = atoi(argv[2]);
		if (ch < 0 || ch > 3) {
			fprintf(stderr, "Channel must be 0-3\n");
			return;
		}
		if (val < 0 || val > 255) {
			fprintf(stderr, "TSEN must be 0-255\n");
			return;
		}
		cmd_motion_tsen(fd, ch, val);
	} else if (strcmp(argv[0], "psen") == 0) {
		if (argc < 3) {
			fprintf(stderr, "Usage: nvp_reg motion psen <channel 0-3> <0-7>\n");
			return;
		}
		int ch = atoi(argv[1]);
		int val = atoi(argv[2]);
		if (ch < 0 || ch > 3) {
			fprintf(stderr, "Channel must be 0-3\n");
			return;
		}
		if (val < 0 || val > 7) {
			fprintf(stderr, "PSEN must be 0-7\n");
			return;
		}
		cmd_motion_psen(fd, ch, val);
	} else if (strcmp(argv[0], "mask") == 0) {
		if (argc < 2) {
			fprintf(stderr, "Usage: nvp_reg motion mask <channel 0-3> [byte_value]\n");
			return;
		}
		int ch = atoi(argv[1]);
		if (ch < 0 || ch > 3) {
			fprintf(stderr, "Channel must be 0-3\n");
			return;
		}
		cmd_motion_mask(fd, ch, argc - 2, argv + 2);
	} else if (strcmp(argv[0], "poll") == 0) {
		cmd_motion_poll(fd);
	} else {
		fprintf(stderr, "Unknown motion command: %s\n", argv[0]);
	}
}

static void cmd_monitor(int fd)
{
	unsigned char b0[256];
	int ch;

	signal(SIGINT, sigint_handler);
	printf("Monitoring NVP6324 status (Ctrl-C to stop)...\n");
	printf("%-8s %-6s %-6s ", "Time", "NOVID", "MOT");
	for (ch = 0; ch < 4; ch++)
		printf("ch%d_AGC ch%d_HL ch%d_BW ", ch, ch, ch);
	printf("\n");

	while (g_running) {
		if (bank_read(fd, 0x00, b0) < 0)
			break;

		printf("         0x%02X   0x%02X   ",
		       b0[0xA0], b0[0xB1]);

		for (ch = 0; ch < 4; ch++) {
			printf("0x%02X    0x%02X   0x%02X   ",
			       b0[0xD0 + ch], b0[0xD8 + ch], b0[0xDC + ch]);
		}
		printf("\r");
		fflush(stdout);

		usleep(200000); /* 200ms */
	}
	printf("\n");
}

static void usage(const char *prog)
{
	printf("NVP6324 Register Tool v2\n\n");
	printf("Register access:\n");
	printf("  %s read <bank> <reg>              Read single register\n", prog);
	printf("  %s write <bank> <reg> <value>     Write single register\n", prog);
	printf("  %s dump <bank>                    Dump full bank (256 regs)\n", prog);
	printf("\nStatus & detection:\n");
	printf("  %s status                         Show all channel status\n", prog);
	printf("  %s detect                         Show format detection state\n", prog);
	printf("  %s vfc [channel]                  Enable VFC and read format\n", prog);
	printf("  %s autodetect [channel]           Full auto-detection sequence\n", prog);
	printf("  %s monitor                        Continuously poll status\n", prog);
	printf("\nMotion detection:\n");
	printf("  %s motion status                  Show motion config & state\n", prog);
	printf("  %s motion on <ch>                 Enable motion on channel\n", prog);
	printf("  %s motion off <ch>                Disable motion on channel\n", prog);
	printf("  %s motion tsen <ch> <0-255>       Set temporal sensitivity\n", prog);
	printf("  %s motion psen <ch> <0-7>         Set pixel sensitivity\n", prog);
	printf("  %s motion mask <ch> [val]         Get/set pixel mask (24 bytes)\n", prog);
	printf("  %s motion poll                    Poll motion events (Ctrl-C)\n", prog);
	printf("\nExamples:\n");
	printf("  %s write 0x13 0x30 0xFF           Enable VFC on all channels\n", prog);
	printf("  %s autodetect 2                   Auto-detect format on ch2\n", prog);
	printf("  %s motion on 2 && %s motion poll  Enable + poll ch2 motion\n", prog, prog);
	printf("  %s vfc                            Quick VFC read all channels\n", prog);
}

int main(int argc, char **argv)
{
	int fd;

	if (argc < 2) {
		usage(argv[0]);
		return 1;
	}

	fd = open(NVP_DEV, O_RDWR);
	if (fd < 0) {
		perror("open " NVP_DEV);
		fprintf(stderr, "Make sure /dev/jaguar2 exists and you have root access\n");
		return 1;
	}

	if (strcmp(argv[1], "read") == 0) {
		cmd_read(fd, argc - 2, argv + 2);
	} else if (strcmp(argv[1], "write") == 0) {
		cmd_write(fd, argc - 2, argv + 2);
	} else if (strcmp(argv[1], "dump") == 0) {
		cmd_dump(fd, argc - 2, argv + 2);
	} else if (strcmp(argv[1], "status") == 0) {
		cmd_status(fd);
	} else if (strcmp(argv[1], "detect") == 0) {
		cmd_detect(fd);
	} else if (strcmp(argv[1], "vfc") == 0) {
		cmd_vfc(fd, argc - 2, argv + 2);
	} else if (strcmp(argv[1], "autodetect") == 0) {
		cmd_autodetect(fd, argc - 2, argv + 2);
	} else if (strcmp(argv[1], "motion") == 0) {
		cmd_motion(fd, argc - 2, argv + 2);
	} else if (strcmp(argv[1], "monitor") == 0) {
		cmd_monitor(fd);
	} else {
		fprintf(stderr, "Unknown command: %s\n", argv[1]);
		usage(argv[0]);
		close(fd);
		return 1;
	}

	close(fd);
	return 0;
}
