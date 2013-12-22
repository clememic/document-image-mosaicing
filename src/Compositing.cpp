#include "Compositing.hpp"

using namespace std;
using namespace cv;
using namespace cv::detail;

void Compositing::warpImages(const vector<Mat>& imgs, const vector<CameraParams>& cameras,
		vector<Mat>& warped_imgs, vector<Point>& warped_corners, vector<Mat>& warped_masks) {

	// Compute warping scale (median focal length between cameras)
	// TODO Why not use mean focal instead?
	float warping_scale;
	vector<double> focals;
	for (unsigned int i = 0; i < cameras.size(); i++) {
		focals.push_back(cameras[i].focal);
	}
	int num_focals = focals.size();
	sort(focals.begin(), focals.end());
	if (num_focals % 2 == 1) {
		warping_scale = ((float) focals[num_focals/2]);
	}
	else {
		warping_scale = ((float) (focals[num_focals/2 - 1] + focals[num_focals/2]) * 0.5f);
	}

	// Warp images onto a plane surface
	Ptr<RotationWarper> warper = (new cv::PlaneWarper())->create(warping_scale);
	for (unsigned int i = 0; i < imgs.size(); i++) {
		// Warp image
		Mat K;
		cameras[i].K().convertTo(K, CV_32F);
		warped_corners[i] = warper->warp(imgs[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, warped_imgs[i]);
		// Warp image mask
		Mat mask(imgs[i].size(), CV_8U);
		mask.setTo(Scalar::all(255));
		warper->warp(mask, K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, warped_masks[i]);
	}

}

void Compositing::voronoiSeamEstimation(const vector<Mat>& imgs, const vector<Point>& corners,
		vector<Mat>& masks) {
	Ptr<SeamFinder> seam_finder = new VoronoiSeamFinder();
	estimateSeams(imgs, corners, masks, seam_finder);
}

void Compositing::dynamicProgrammingSeamEstimation(const vector<Mat>& imgs, const vector<Point>& corners,
		vector<Mat>& masks) {
	Ptr<SeamFinder> seam_finder = new DpSeamFinder();
	estimateSeams(imgs, corners, masks, seam_finder);
}

void Compositing::graphCutSeamEstimation(const vector<Mat>& imgs, const vector<Point>& corners,
		vector<Mat>& masks, int cost_type) {
	Ptr<SeamFinder> seam_finder = new GraphCutSeamFinder(cost_type);
	vector<Mat>imgs_f(imgs.size());
	for (unsigned int i = 0; i < imgs.size(); i++) {
		imgs[i].convertTo(imgs_f[i], CV_32F);
	}
	estimateSeams(imgs_f, corners, masks, seam_finder);
}

void Compositing::estimateSeams(const vector<Mat>& imgs, const vector<Point>& corners,
		vector<Mat>& masks, SeamFinder* seam_finder) {
	seam_finder->find(imgs, corners, masks);
}
