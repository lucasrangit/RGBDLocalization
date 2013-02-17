#OCV_LIBS += -lcv -lcvaux -lhighgui -lcxcore
OCV_LIBS = `pkg-config --cflags --libs opencv`
includes = $(wildcard *.h)

all: rgbdlocalization

rgbdlocalization: rgbdlocalization.c libfreenect_cv.c helpers.c test.c ${includes}
	gcc -Wall -g -o $@ $^ -lm -lfreenect_sync $(OCV_LIBS)

clean:
	rm -f rgbdlocalization *.o