SRCS = $(wildcard *.c)
OBJS = $(SRCS:.c = .o)
CC = gcc
INCLUDES = -l./include
LIBS = -L./lib -lsqlite3
CCFLAGS = -g -Wall -O0

main:$(OBJS)
	$(CC) $^ -o $@ $(INCLUDES) $(LIBS)

%.o:%.c
	$(CC) -c $< $(CCFLAGS)

.PHONY:clean

clean:
	rm *.o
