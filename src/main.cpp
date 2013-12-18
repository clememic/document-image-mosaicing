
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/stitching/stitcher.hpp>

using namespace std;
using namespace cv;
using namespace cv::detail;

void readme();

int main(int argc, char** argv) {

	if (argc < 3) {
		readme();
		return -1;
	}

	/* Read input images */
	vector<Mat> imgs;
	for (int i = 1; i < argc; i++) {
		Mat img = imread(argv[i]);
		if (img.empty()) {
			cerr << "Cannot read input image " << argv[i] << endl;
			return -1;
		}
		imgs.push_back(img);
	}
	int num_imgs = imgs.size();

	/* Find features in images */
	SurfFeaturesFinder surf;
	vector<ImageFeatures> features(num_imgs);
	for (int i = 0; i < num_imgs; i++) {
		surf(imgs[i], features[i]);
		features[i].img_idx = i;
	}

	/* Match features */
	BestOf2NearestMatcher matcher;
	vector<MatchesInfo> pairwise_matches;
	matcher(features, pairwise_matches);
	matcher.collectGarbage();

	/* Estimate homographies between images using feature matches */
	HomographyBasedEstimator estimator;
	vector<CameraParams> cameras;
	estimator(features, pairwise_matches, cameras);

	/* Bundle adjustment for camera parameters refinement */
	// TODO

	/* Wave correction */
	// TODO

	/* Warp images */
	// TODO

	/* Compensate exposure errors */
	// TODO

	/* Find seam masks */
	// TODO

	/* Blend warped images */
	// TODO

	return 0;

}

void readme() {
	cerr << "Document image mosaicing. Usage: ./mosaicing img1 img2 [...imgN]" << endl;
}
