CC ?= gcc
CFLAGS ?= -O2 -Wall
PKGS = librga libdrm gbm egl glesv2
PKG_CFLAGS := $(shell pkg-config --cflags $(PKGS))
PKG_LIBS := $(shell pkg-config --libs $(PKGS))
LIBS = -lm -lpthread
TARGET = v4l2_gpu_undistort_quad

all: $(TARGET)

$(TARGET): v4l2_gpu_undistort_quad.c
	$(CC) $(CFLAGS) $(PKG_CFLAGS) -o $@ $< $(PKG_LIBS) $(LIBS)

clean:
	rm -f $(TARGET)

install: $(TARGET)
	install -m 755 $(TARGET) /usr/local/bin/

.PHONY: all clean install
