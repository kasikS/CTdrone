CC      = gcc
PROJECT = quadcontrol

SRCS = main.c
SRCS += config.c
SRCS += serial.c
SRCS += link.c
SRCS += ../../src/link_common.c
SRCS += joystick.c
SRCS += utils.c

OBJS = $(SRCS:.c=.o)

CFLAGS = -ggdb -O0 -std=c99 -DDEBUG -I. -I../../src
WFLAGS = -Wall -Wextra -Warray-bounds -Wno-unused-parameter -Wno-unused-function
LFLAGS =

.PHONY: all
all: $(PROJECT)

$(PROJECT): $(OBJS)
	$(CC) $(LFLAGS) $^ -o $@

%.o: %.c
	$(CC) -c -o $@ $(INCLUDE) $(DEFS) $(CFLAGS) $^

clean:
	rm *.o
