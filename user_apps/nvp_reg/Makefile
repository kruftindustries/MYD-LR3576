CC ?= gcc
CFLAGS ?= -O2 -Wall
TARGET = nvp_reg

all: $(TARGET)

$(TARGET): nvp_reg.c
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f $(TARGET)

install: $(TARGET)
	install -m 755 $(TARGET) /usr/local/bin/

.PHONY: all clean install
