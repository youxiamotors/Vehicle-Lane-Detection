basicfile = basic.o\
			  util.o\
			  equation.o\
			  img_transform.o\
			  laneline.o\
			  laneflow.o\
			  lane.o\
			  lane_estimate.o\
			  lane_track.o\

mainobjects = main.o ${basicfile}

util_testobjects = util_test.o ${basicfile}
equation_testobjects = equation_test.o ${basicfile}
			  
STD=-g -Wall
mode="bend"
#mode="stright"

ifeq ("stright", ${mode})
DEFINE_MACROS=-D ROI_Y_RATIO=0.68 \
			  -D IPM_STARTX_RATIO=0.43 \
			  -D ROI_HEIGHT_RATIO=0.32 \
			  -D LANE_WIDTH=78 \
			  -D LANELINE_WIDTH_MAX=6 \
			  -D LANELINE_WIDTH_MIN=3 \
			  -D LANELINE_HEIGHT_MIN=12\
			  -D CANNY_THRESHOLD=40
endif
ifeq ("bend", ${mode})
DEFINE_MACROS=-D ROI_Y_RATIO=0.698 \
			  -D IPM_STARTX_RATIO=0.43 \
			  -D ROI_HEIGHT_RATIO=0.302 \
			  -D LANE_WIDTH=32 \
			  -D LANELINE_WIDTH_MAX=6 \
			  -D LANELINE_WIDTH_MIN=3 \
			  -D LANELINE_HEIGHT_MIN=12\
			  -D CANNY_THRESHOLD=40
endif

prefix=/usr/local
OCVFLAGS=-I${prefix}/include/opencv2 -I${prefix}/include/opencv -I${prefix}/include
OCVLIBS=-L${prefix}/lib -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_flann -lopencv_objdetect -lopencv_calib3d

LIBRARY=
CFLAGS=$(STD) $(DEFINE_MACROS)
CC:=g++
#CC:=$(shell sh -c 'type $(CC) >/dev/null 2>/dev/null && echo $(CC) || echo gcc') $(CFLAGS)
all:main

main:$(mainobjects)
	$(CC) -o main $(mainobjects) $(LIBRARY) $(OCVLIBS) $(OCVFLAGS)

util_test:$(util_testobjects)
	$(CC) -o util_test $(util_testobjects) $(LIBRARY) $(OCVLIBS) $(OCVFLAGS)

equation_test:$(equation_testobjects)
	$(CC) -o equation_test $(equation_testobjects) $(LIBRARY) $(OCVLIBS) $(OCVFLAGS)

%.o: %.cc 
	$(CC) -c $(CFLAGS) $<  -o $*.o

clean:
	rm -f ./*.o ./*.a
	rm ./main ./util_test ./equation_test
