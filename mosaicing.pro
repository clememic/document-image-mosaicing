TARGET = mosaicing

QT += core gui widgets
LIBS += -lopencv_core -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgproc -lopencv_nonfree -lopencv_stitching

OBJECTS_DIR = obj
MOC_DIR = moc

HEADERS += src/Registration.hpp src/Compositing.hpp src/Mosaicing.hpp
SOURCES += src/Registration.cpp src/Compositing.cpp src/Mosaicing.cpp src/main.cpp
