SRC= $(wildcard src/*.c)
OBJ = $(SRC:.c=.o)
LDFLAGS = -lbcm2835
CC = gcc

read_sensors: $(OBJ)
	$(CC) -o $@ $^ $(LDFLAGS)

.PHONY: clean
clean:
	rm -f $(OBJ) read_sensors
