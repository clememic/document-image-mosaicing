#ifndef REGISTRATION_HPP_
#define REGISTRATION_HPP_

#include <vector>
#include <opencv2/stitching/stitcher.hpp>

class Registration {

public:

	static std::vector<cv::detail::ImageFeatures> getSurfFeatures(const std::vector<cv::Mat>& imgs,
		double hess_thresh=300, int num_octaves=3, int num_layers=4, int num_octaves_descr=3, int num_layers_descr=4);

	static std::vector<cv::detail::ImageFeatures> getOrbFeatures(const std::vector<cv::Mat>& imgs,
		cv::Size grid_size=cv::Size(3,1), int nfeatures=1500, float scaleFactor=1.3f, int nlevels=5);

	static std::vector<cv::detail::MatchesInfo> getMatches(const std::vector<cv::detail::ImageFeatures> &features,
		float match_conf=0.3f, int num_matches_thresh1=6, int num_matches_thresh2=6);

	static std::vector<cv::detail::CameraParams> estimateHomographies(const std::vector<cv::detail::ImageFeatures>& features,
		const std::vector<cv::detail::MatchesInfo>& pairwise_matches);

	static void bundleAdjusterReproj(const std::vector<cv::detail::ImageFeatures>& features,
		const std::vector<cv::detail::MatchesInfo>& pairwise_matches, std::vector<cv::detail::CameraParams>& cameras,
		double conf_thresh=1.);

	static void bundleAdjusterRay(const std::vector<cv::detail::ImageFeatures>& features,
		const std::vector<cv::detail::MatchesInfo>& pairwise_matches, std::vector<cv::detail::CameraParams>& cameras,
		double conf_thresh=1.);

private:

	static std::vector<cv::detail::ImageFeatures> getFeatures(const std::vector<cv::Mat>& imgs,
		cv::detail::FeaturesFinder* features_finder);

	static void bundleAdjuster(const std::vector<cv::detail::ImageFeatures>& features,
		const std::vector<cv::detail::MatchesInfo>& pairwise_matches, std::vector<cv::detail::CameraParams>& cameras,
		cv::detail::BundleAdjusterBase* bundleAdjuster, double conf_thresh=1.);

};

#endif /* REGISTRATION_HPP_ */
