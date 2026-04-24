CC = gcc
TARGET = renderer

# Optimization flags
CFLAGS = -O3 -march=native -ffast-math -funroll-loops -flto -pthread
# Tell the compiler where the headers are
CFLAGS += -I./files/include -I./files/src

LDFLAGS = -pthread -flto -lm

# VPATH tells Make where to search for source files (.c files)
VPATH = files/src

# Just the filenames, VPATH will find them in files/src
SRC = main.c scene.c renderer.c geometry.c math.c
OBJ = $(SRC:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(OBJ) -o $(TARGET) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o $(TARGET)
