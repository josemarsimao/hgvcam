# Copyright (C) 2026 Josemar Simao <josemarsimao@gmail.com>
# Co-authored with Claude Opus 5.5 (Anthropic) <noreply@anthropic.com>

obj-m += hgvcam.o

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

.PHONY: module clean player

module:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

player: hgvcam_player.c hgvcam.h
	$(CC) -O2 -Wall -Wextra -Werror -o $@ $< \
		$$(pkg-config --cflags --libs libavformat libavcodec libavutil libswscale)

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -f player
