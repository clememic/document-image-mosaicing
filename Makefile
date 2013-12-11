CC			= g++
CFLAGS		= -Wall -Wextra
INCLUDES	= -I/usr/include/opencv
LIBS		= -lopencv_core -lopencv_highgui -lopencv_features2d -lopencv_stitching
SRC			= src
TARGET		= mosaicing

all: $(TARGET)

$(TARGET):
	$(CC) $(CFLAGS) $(wildcard $(SRC)/*.cpp) -o $(TARGET) $(INCLUDES) $(LIBS)
