#OCV_LIBS += -lcv -lcvaux -lhighgui -lcxcore
OCV_LIBS = `pkg-config --cflags --libs opencv`

all: rgbdlocalization

rgbdlocalization: rgbdlocalization.c libfreenect_cv.c helpers.c test.c
	gcc -Wall -g -o $@ $^ -lm -lfreenect_sync $(OCV_LIBS)

clean:
	rm -f rgbdlocalization *.o