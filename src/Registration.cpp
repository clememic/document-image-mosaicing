#include "Registration.hpp"

using namespace std;
using namespace cv;
using namespace detail;

vector<ImageFeatures> Registration::getSurfFeatures(const vector<Mat>& imgs, double hess_thresh,
		int num_octaves, int num_layers, int num_octaves_descr, int num_layers_descr) {
	Ptr<FeaturesFinder> surf = new SurfFeaturesFinder(hess_thresh, num_octaves, num_layers,
		num_octaves_descr, num_layers_descr);
	vector<ImageFeatures> features(imgs.size());
	for (unsigned int i = 0; i < imgs.size(); i++) {
		(*surf)(imgs[i], features[i]);
		features[i].img_idx = i;
	}
	surf->collectGarbage();
	return features;
}

vector<ImageFeatures> Registration::getSiftFeatures(const vector<Mat>& imgs, int nfeatures,
		int nOctaveLayers, double contrastThreshold, double edgeThreshold, double sigma) {
	Ptr<SIFT> sift = new SIFT(nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);
	vector<ImageFeatures> features(imgs.size());
	for (unsigned int i = 0; i < imgs.size(); i++) {
		sift->detect(imgs[i], features[i].keypoints);
		sift->compute(imgs[i], features[i].keypoints, features[i].descriptors);
		features[i].img_idx = i;
	}
	return features;
}

vector<MatchesInfo> Registration::getMatches(const vector<ImageFeatures> &features, float match_conf,
		int num_matches_thresh1, int num_matches_thresh2) {
	Ptr<FeaturesMatcher> matcher = new BestOf2NearestMatcher(false, match_conf, num_matches_thresh1,
		 num_matches_thresh2);
	vector<MatchesInfo> pairwise_matches;
	(*matcher)(features, pairwise_matches);
	matcher->collectGarbage();
	return pairwise_matches;
}

vector<CameraParams> Registration::estimateHomographies(const vector<ImageFeatures>& features,
		const vector<MatchesInfo>& pairwise_matches) {
	Ptr<Estimator> estimator = new HomographyBasedEstimator();
	vector<CameraParams> cameras;
	(*estimator)(features, pairwise_matches, cameras);
	return cameras;
}

void Registration::bundleAdjusterReproj(const vector<ImageFeatures>& features, const vector<MatchesInfo>& pairwise_matches,
		vector<CameraParams>& cameras, double conf_thresh) {
	Ptr<BundleAdjusterBase> bundle_adjuster = new BundleAdjusterReproj();
	Registration::bundleAdjuster(features, pairwise_matches, cameras, bundle_adjuster, conf_thresh);
}

void Registration::bundleAdjusterRay(const vector<ImageFeatures>& features, const vector<MatchesInfo>& pairwise_matches,
		vector<CameraParams>& cameras, double conf_thresh) {
	Ptr<BundleAdjusterBase> bundle_adjuster = new BundleAdjusterRay();
	Registration::bundleAdjuster(features, pairwise_matches, cameras, bundle_adjuster, conf_thresh);
}

void Registration::bundleAdjuster(const vector<ImageFeatures>& features, const vector<MatchesInfo>& pairwise_matches,
		vector<CameraParams>& cameras, BundleAdjusterBase* bundle_adjuster, double conf_thresh) {
	// Rotation matrices must be converted to floating-point numbers
	for (unsigned int i = 0; i < cameras.size(); i++) {
		cameras[i].R.convertTo(cameras[i].R, CV_32F);
	}
	bundle_adjuster->setConfThresh(conf_thresh);
	(*bundle_adjuster)(features, pairwise_matches, cameras);
}
