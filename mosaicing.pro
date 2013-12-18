TARGET = mosaicing

LIBS += -lopencv_core -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgproc -lopencv_nonfree -lopencv_stitching

OBJECTS_DIR = obj

SOURCES += src/main.cpp
