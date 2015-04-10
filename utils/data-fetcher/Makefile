CC=g++
CFLAGS=-g -Wall



all: client server

client: client.cpp
	$(CC) -o $@ client.cpp $(CFLAGS)


server: server.cpp
	$(CC) -o $@ server.cpp $(CFLAGS)


.PHONEY: clean

clean:
	rm -f *.o client server