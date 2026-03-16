/* drm_screenshot.c — capture active DRM CRTC to PPM file */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

int main(int argc, char **argv)
{
    const char *dev = "/dev/dri/card1";
    const char *out = "/tmp/drm_screenshot.ppm";
    if (argc > 1) out = argv[1];

    int fd = open(dev, O_RDWR);
    if (fd < 0) { perror("open drm"); return 1; }

    /* Need master or at least read access to FBs */
    drmSetMaster(fd);  /* may fail if another process is master, that's ok */

    drmModeResPtr res = drmModeGetResources(fd);
    if (!res) { perror("GetResources"); close(fd); return 1; }

    /* Find first active CRTC */
    drmModeCrtcPtr crtc = NULL;
    for (int i = 0; i < res->count_crtcs; i++) {
        crtc = drmModeGetCrtc(fd, res->crtcs[i]);
        if (crtc && crtc->buffer_id) break;
        if (crtc) { drmModeFreeCrtc(crtc); crtc = NULL; }
    }
    if (!crtc) { fprintf(stderr, "No active CRTC\n"); close(fd); return 1; }

    printf("CRTC %u: fb=%u %ux%u\n", crtc->crtc_id, crtc->buffer_id,
           crtc->width, crtc->height);

    /* Get framebuffer info */
    drmModeFBPtr fb = drmModeGetFB(fd, crtc->buffer_id);
    if (!fb) { perror("GetFB"); close(fd); return 1; }

    printf("FB: %ux%u pitch=%u bpp=%u handle=%u\n",
           fb->width, fb->height, fb->pitch, fb->bpp, fb->handle);

    if (!fb->handle) {
        fprintf(stderr, "FB handle is 0 — need DRM master or CAP_SYS_ADMIN\n");
        close(fd);
        return 1;
    }

    /* Map the dumb buffer */
    struct drm_mode_map_dumb map_req = { .handle = fb->handle };
    if (drmIoctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &map_req)) {
        perror("MAP_DUMB");
        close(fd);
        return 1;
    }

    size_t size = (size_t)fb->pitch * fb->height;
    void *map = mmap(NULL, size, PROT_READ, MAP_SHARED, fd, map_req.offset);
    if (map == MAP_FAILED) { perror("mmap"); close(fd); return 1; }

    /* Write PPM (BGRX8888 -> RGB) */
    FILE *fp = fopen(out, "wb");
    if (!fp) { perror("fopen"); close(fd); return 1; }
    fprintf(fp, "P6\n%u %u\n255\n", fb->width, fb->height);

    uint8_t *row = map;
    for (uint32_t y = 0; y < fb->height; y++) {
        for (uint32_t x = 0; x < fb->width; x++) {
            uint8_t *px = row + x * 4;
            /* BGRX8888: B=px[0] G=px[1] R=px[2] */
            uint8_t rgb[3] = { px[2], px[1], px[0] };
            fwrite(rgb, 1, 3, fp);
        }
        row += fb->pitch;
    }
    fclose(fp);
    printf("Saved %s (%ux%u)\n", out, fb->width, fb->height);

    munmap(map, size);
    drmModeFreeFB(fb);
    drmModeFreeCrtc(crtc);
    drmModeFreeResources(res);
    close(fd);
    return 0;
}
