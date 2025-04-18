CXX ?= g++

all:
	$(CXX) -o tinyraytracer tinyraytracer_sdl2.cpp -lSDL2

.PHONY: clean
clean:
	rm -f tinyraytracer