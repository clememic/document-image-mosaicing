
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
	// Rotation matrices must be converted to floating-point numbers
	for (unsigned int i = 0; i < cameras.size(); i++) {
		cameras[i].R.convertTo(cameras[i].R, CV_32F);
	}

	/* Bundle adjustment for camera parameters refinement */
	BundleAdjusterRay bundleAdjuster;
	bundleAdjuster(features, pairwise_matches, cameras);

	/* Wave correction */
	// TODO

	/* Warp images */
	// Compute median focal length between cameras for warping scale
	// TODO Use mean focal instead?
	float warping_scale;
	vector<double> focals;
	for (unsigned int i = 0; i < cameras.size(); i++) {
		focals.push_back(cameras[i].focal);
	}
	int num_focals = focals.size();
	sort(focals.begin(), focals.end());
	if (num_focals % 2 == 1) {
		warping_scale = focals[num_focals/2];
	}
	else {
		warping_scale = (focals[num_focals/2 - 1] + focals[num_focals/2]) * 0.5;
	}
	// Prepare image masks
	vector<Mat> masks(num_imgs);
	for (int i = 0; i < num_imgs; i++) {
		masks[i].create(imgs[i].size(), CV_8U);
		masks[i].setTo(Scalar::all(255));
	}
	// Perform warping on images and their masks
	Ptr<WarperCreator> warper_creator = new cv::PlaneWarper();
	Ptr<RotationWarper> warper = warper_creator->create(warping_scale);
	vector<Mat> warped_imgs(num_imgs);
	vector<Mat> warped_masks(num_imgs);
	vector<Point> warped_corners(num_imgs);
	vector<Size> warped_imgs_sizes(num_imgs);
	for (int i = 0; i < num_imgs; i++) {
		Mat K;
		cameras[i].K().convertTo(K, CV_32F);
		// TODO Try different pixel interpolation/extrapolation methods?
		warped_corners[i] = warper->warp(imgs[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, warped_imgs[i]);
		warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, warped_masks[i]);
		warped_imgs_sizes[i] = warped_imgs[i].size();
	}

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
