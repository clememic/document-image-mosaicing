#include "Compositing.hpp"

using namespace std;
using namespace cv;
using namespace cv::detail;

void Compositing::warpImages(const vector<Mat>& imgs, const vector<CameraParams>& cameras,
		vector<Mat>& warped_imgs, vector<Point>& warped_corners, vector<Mat>& warped_masks,
		vector<Size>& warped_sizes) {

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
		warped_sizes[i] = warped_imgs[i].size();
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

void Compositing::gainExposureCompensation(vector<Point>& corners, vector<Mat>& imgs, const vector<Mat>& masks) {
	Ptr<ExposureCompensator> exposure_compensator = new GainCompensator();
	compensateExposureErrors(corners, imgs, masks, exposure_compensator);
}

void Compositing::gainBlocksExposureCompensation(vector<Point>& corners, vector<Mat>& imgs, const vector<Mat>& masks,
		int bl_width, int bl_height) {
	Ptr<ExposureCompensator> exposure_compensator = new BlocksGainCompensator(bl_width, bl_height);
	compensateExposureErrors(corners, imgs, masks, exposure_compensator);
}

void Compositing::compensateExposureErrors(vector<Point>& corners, vector<Mat>& imgs, const vector<Mat>& masks,
		ExposureCompensator* exposure_compensator) {
	exposure_compensator->feed(corners, imgs, masks);
	for (unsigned int i = 0; i < imgs.size(); i++) {
		exposure_compensator->apply(i, corners[i], imgs[i], masks[i]);
	}
}

Mat Compositing::blendImagesMultiBand(const vector<Mat>& imgs, const vector<Point>& corners, const vector<Mat>& masks,
		const vector<Size>& sizes, float blend_strength) {
	float blend_width = getBlendWidth(sizes, corners, blend_strength);
	int num_bands = (int) ceil(log(blend_width) / log(2.)) - 1.;
	Ptr<Blender> blender = new detail::MultiBandBlender(false, num_bands);
	return blendImages(imgs, corners, masks, sizes, blender);
}

Mat Compositing::blendImagesFeather(const vector<Mat>& imgs, const vector<Point>& corners, const vector<Mat>& masks,
		const vector<Size>& sizes, float blend_strength) {
	float blend_width = getBlendWidth(sizes, corners, blend_strength);
	Ptr<Blender> blender = new detail::FeatherBlender(1.f / blend_width);
	vector<Mat> imgs_s(imgs.size());
	for (unsigned int i = 0; i < imgs.size(); i++) {
		imgs[i].convertTo(imgs_s[i], CV_16S);
	}
	return blendImages(imgs_s, corners, masks, sizes, blender);
}

Mat Compositing::blendImages(const vector<Mat>& imgs, const vector<Point>& corners, const vector<Mat>& masks,
		const vector<Size>& sizes, Blender* blender) {
	blender->prepare(corners, sizes);
	Mat mask, dilated_mask, seam_mask;
	for (unsigned int i = 0; i < imgs.size(); i++) {
		dilate(masks[i], dilated_mask, Mat());
		resize(dilated_mask, seam_mask, masks[i].size());
		mask = seam_mask & masks[i];
		blender->feed(imgs[i], mask, corners[i]);
	}
	Mat result, result_mask;
	blender->blend(result, result_mask);
	return result;
}

float Compositing::getBlendWidth(const vector<Size>& sizes, const vector<Point>& corners, float blend_strength) {
	Size dest_size = resultRoi(corners, sizes).size();
	return sqrt((float) dest_size.area()) * blend_strength / 100.f;
}
