#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/stitching/stitcher.hpp"

using namespace std;
using namespace cv;

void readme();

int main(int argc, char** argv) {

	if (argc < 3) {
		readme();
		return -1;
	}

	// Read input images
	vector<Mat> imgs; 
	for (int i = 1; i < argc; i++) {
		Mat img = imread(argv[i]);
		if (img.empty()) {
			cerr << "Can't read image '" << argv[i] << "'" << endl;
			return -1;
		}
		imgs.push_back(img);
	}

	// Default stitching
	Mat pano;
	Stitcher stitcher = Stitcher::createDefault();
	Stitcher::Status status = stitcher.stitch(imgs, pano);
	if (status != Stitcher::OK) {
		cerr << "Cannot stitch images, error code = " << status << endl;
		return -1;
	}
	imshow("Document Image Mosaicing", pano);
	waitKey(0);

	return 0;

}

void readme() {
	cerr << "Document image mosaicing. Usage: ./main img1 img2 [...imgN]" << endl;
}