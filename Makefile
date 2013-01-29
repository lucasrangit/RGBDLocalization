#OCV_LIBS += -lcv -lcvaux -lhighgui -lcxcore
OCV_LIBS = `pkg-config --cflags --libs opencv`

all: test

test: cvdemo.c libfreenect_cv.c helpers.c
	gcc -Wall -g -o $@ $^ -lfreenect_sync $(OCV_LIBS)

clean:
	rm -f test *.o