CC      ?= gcc
CFLAGS  ?= -O2 -Wall -Wextra -Wno-unused-parameter
TARGET  = mixed_quad

PKG_CFLAGS := $(shell pkg-config --cflags librga libdrm gbm egl glesv2 libavformat libavcodec libavutil libswscale 2>/dev/null)
PKG_LIBS   := $(shell pkg-config --libs   librga libdrm gbm egl glesv2 libavformat libavcodec libavutil libswscale 2>/dev/null)

LIBS = $(PKG_LIBS) -lm -lpthread

$(TARGET): mixed_quad.c
	$(CC) $(CFLAGS) $(PKG_CFLAGS) -o $@ $< $(LIBS)

clean:
	rm -f $(TARGET)

.PHONY: clean
