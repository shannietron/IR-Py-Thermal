CC ?= clang
CFLAGS ?= -O2 -Wall -Wextra -Wno-unused-parameter -Wno-deprecated-declarations
FRAMEWORKS = -framework IOKit -framework CoreFoundation

all: libuvcctl.dylib uvcctl_cli

libuvcctl.dylib: uvcctl.c uvcctl.h
	$(CC) $(CFLAGS) -shared -o $@ uvcctl.c $(FRAMEWORKS)

uvcctl_cli: uvcctl_cli.c uvcctl.c uvcctl.h
	$(CC) $(CFLAGS) -o $@ uvcctl_cli.c uvcctl.c $(FRAMEWORKS)

clean:
	rm -f libuvcctl.dylib uvcctl_cli *.o
.PHONY: all clean
