SOURCES = emu2149.c
FLAGS = -Wall -ansi -g -c

all: emu2149.o

emu2149.o: emu2149.c emu2149.h
	$(CC) $(FLAGS) -o emu2149.o $(SOURCES)

.PHONY: clean
clean:
	rm -f *.o

