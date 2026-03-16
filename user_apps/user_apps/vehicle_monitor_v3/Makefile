CC      = gcc
PKG     = PKG_CONFIG_PATH=/usr/lib/pkgconfig pkg-config
CFLAGS  = -Wall -Wextra -O2 -march=armv8-a -Icore -Iui -I. \
          $(shell $(PKG) --cflags libavformat libavcodec libswscale libavutil gtk+-3.0)
LDFLAGS = $(shell $(PKG) --libs libavformat libavcodec libswscale libavutil gtk+-3.0) \
          -lsqlite3 -lpthread -lm

TARGET  = vehicle_monitor

SRCS    = main.c \
          core/camera.c core/motion.c core/database.c core/config.c \
          core/homography.c core/playback.c core/clip.c \
          core/undistort.c \
          pages/page_cameras.c pages/page_camera_config.c \
          pages/page_motion_config.c pages/page_dvr.c \
          pages/page_birdview.c pages/page_birdview_config.c \
          pages/page_sensors.c pages/page_sensor_config.c \
          pages/page_events.c pages/page_dvr_config.c \
          pages/page_lens_config.c \
          pages/page_config.c \
          ui/util.c

OBJS    = $(SRCS:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(TARGET) $(OBJS)

.PHONY: all clean
