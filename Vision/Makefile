CC = gcc
CXX = g++
CFLAGS = -O3 -Wall 	-lrt
CFLAGS += -I/usr/local/include/opencv4 \
	-I/usr/local/include \
	-L/usr/local/lib -lopencv_dnn \
	-lopencv_ml \
	-lopencv_objdetect \
	-lopencv_shape \
	-lopencv_stitching \
	-lopencv_superres \
	-lopencv_videostab \
	-lopencv_calib3d \
	-lopencv_features2d \
	-lopencv_highgui \
	-lopencv_videoio \
	-lopencv_imgcodecs \
	-lopencv_video \
	-lopencv_photo \
	-lopencv_imgproc \
	-lopencv_flann \
	-lopencv_core\
	-lwiringPi
CFLAGS += -lpthread
all:
	@$(CXX) -o main main.cpp wiringSerial.c $(CFLAGS)
clean:
	rm -rf $(EXE1) $(EXE2) $(EXE3) $(EXE4) $(EXE5) $(EXE6) $(EXE7) $(EXE8) $(EXE9) $(EXE10) $(EXE11) $(EXE12) *.o
