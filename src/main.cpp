#include <iostream>
#include <string>
#include <vector>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include <QApplication>

#include "Registration.hpp"
#include "Compositing.hpp"
#include "Mosaicing.hpp"

using namespace std;
using namespace cv;
using namespace cv::detail;

void readme();

int main(int argc, char** argv) {
	QApplication application(argc, argv);
	Mosaicing mosaicing;
	mosaicing.show();
	return application.exec();
}

void readme() {
	cerr << "Document image mosaicing. Usage: ./mosaicing img1 img2 [img3 img4]" << endl;
}
