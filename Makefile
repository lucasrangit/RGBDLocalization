all: test

test: cvdemo.c libfreenect_cv.c
	gcc -Wall -g -o $@ $^ -lfreenect_sync `pkg-config --cflags --libs opencv` 

clean:
	rm -f test *.o