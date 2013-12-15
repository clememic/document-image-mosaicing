TARGET = mosaicing

INCLUDEPATH += /usr/include/opencv
LIBS        += -lopencv_core -lopencv_calib3d -lopencv_highgui -lopencv_features2d -lopencv_flann -lopencv_nonfree -lopencv_stitching

SOURCES += src/main.cpp
