#include <iostream>
#include <string>
#include <vector>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/stitching/stitcher.hpp>

#include "Registration.hpp"
#include "Compositing.hpp"

using namespace std;
using namespace cv;
using namespace cv::detail;

void readme();

int main(int argc, char** argv) {

	if (argc < 3 || argc > 5) {
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
	vector<ImageFeatures> features = Registration::getSurfFeatures(imgs);

	/* Match features */
	vector<MatchesInfo> pairwise_matches = Registration::getMatches(features);

	/* Estimate homographies between images using feature matches */
	vector<CameraParams> cameras = Registration::estimateHomographies(features, pairwise_matches);

	/* Bundle adjustment for camera parameters refinement */
	Registration::bundleAdjusterRay(features, pairwise_matches, cameras);

	/* Warp images */
	vector<Mat> warped_imgs(num_imgs);
	vector<Mat> warped_masks(num_imgs);
	vector<Point> warped_corners(num_imgs);
	vector<Size> warped_sizes(num_imgs);
	Compositing::warpImages(imgs, cameras, warped_imgs, warped_corners, warped_masks, warped_sizes);

	/* Find seam masks */
	Compositing::graphCutSeamEstimation(warped_imgs, warped_corners, warped_masks);

	/* Compensate exposure errors */
	Compositing::gainBlocksExposureCompensation(warped_corners, warped_imgs, warped_masks);

	/* Blend warped images */
	Mat result = Compositing:: blendImagesMultiBand(warped_imgs, warped_corners, warped_masks, warped_sizes);

	imwrite("result.jpg", result);

	return 0;

}

void readme() {
	cerr << "Document image mosaicing. Usage: ./mosaicing img1 img2 [img3 img4]" << endl;
}
