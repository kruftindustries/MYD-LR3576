CC = gcc

# Prefer ffmpeg-rockchip pkg-config over system ffmpeg
PKG = PKG_CONFIG_PATH=/usr/lib/pkgconfig pkg-config

CFLAGS = -Wall -Wextra -O2 \
	$(shell $(PKG) --cflags libavformat libavcodec libswscale libavutil gtk+-3.0)
LDFLAGS = \
	$(shell $(PKG) --libs libavformat libavcodec libswscale libavutil gtk+-3.0) \
	-lpthread -lm

TARGET = motion_detect_v4
SRCS = motion_detect_v4.c

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(TARGET)

.PHONY: all clean
