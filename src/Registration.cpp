#include "Registration.hpp"

using namespace std;
using namespace cv;
using namespace detail;

vector<ImageFeatures> Registration::getSurfFeatures(const vector<Mat>& imgs, double hess_thresh,
		int num_octaves, int num_layers, int num_octaves_descr, int num_layers_descr) {
	Ptr<FeaturesFinder> surf = new SurfFeaturesFinder(hess_thresh, num_octaves, num_layers,
		num_octaves_descr, num_layers_descr);
	return getFeatures(imgs, surf);
}

vector<ImageFeatures> Registration::getOrbFeatures(const vector<Mat>& imgs, Size grid_size,
		int nfeatures, float scaleFactor, int nlevels) {
	Ptr<FeaturesFinder> orb = new OrbFeaturesFinder(grid_size, nfeatures, scaleFactor, nlevels);
	return getFeatures(imgs, orb);
}

vector<ImageFeatures> Registration::getFeatures(const vector<Mat>& imgs, FeaturesFinder* features_finder) {
	vector<ImageFeatures> features(imgs.size());
	for (unsigned int i = 0; i < imgs.size(); i++) {
		(*features_finder)(imgs[i], features[i]);
		features[i].img_idx = i;
	}
	features_finder->collectGarbage();
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
