CC = gcc

# Prefer ffmpeg-rockchip pkg-config over system ffmpeg
PKG = PKG_CONFIG_PATH=/usr/lib/pkgconfig pkg-config

CFLAGS = -Wall -Wextra -O2 \
	$(shell $(PKG) --cflags libavformat libavcodec libswscale libavutil x11 rknn)
LDFLAGS = \
	$(shell $(PKG) --libs libavformat libavcodec libswscale libavutil x11 rknn) \
	-lsqlite3 -lpthread -lm

TARGET = motion_detect_v3
SRCS = motion_detect_v3.c yolo_postprocess.c detection_db.c

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(TARGET)

.PHONY: all clean
