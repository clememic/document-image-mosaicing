#ifndef REGISTRATION_HPP_
#define REGISTRATION_HPP_

#include <vector>
#include <opencv2/stitching/stitcher.hpp>
#include <opencv2/nonfree/features2d.hpp>

class Registration {

public:

	static std::vector<cv::detail::ImageFeatures> getSurfFeatures(const std::vector<cv::Mat>& imgs,
		double hess_thresh=300, int num_octaves=3, int num_layers=4, int num_octaves_descr=3, int num_layers_descr=4);

	static std::vector<cv::detail::ImageFeatures> getSiftFeatures(const std::vector<cv::Mat>& imgs,
		int nfeatures=0, int nOctaveLayers=3, double contrastThreshold=0.04, double edgeThreshold=10, double sigma=1.6);

	static std::vector<cv::detail::MatchesInfo> getMatches(const std::vector<cv::detail::ImageFeatures> &features,
		float match_conf=0.65f, int num_matches_thresh1=6, int num_matches_thresh2=6);

	static std::vector<cv::detail::CameraParams> estimateHomographies(const std::vector<cv::detail::ImageFeatures>& features,
		const std::vector<cv::detail::MatchesInfo>& pairwise_matches);

	static void bundleAdjusterReproj(const std::vector<cv::detail::ImageFeatures>& features,
		const std::vector<cv::detail::MatchesInfo>& pairwise_matches, std::vector<cv::detail::CameraParams>& cameras,
		double conf_thresh=1.);

	static void bundleAdjusterRay(const std::vector<cv::detail::ImageFeatures>& features,
		const std::vector<cv::detail::MatchesInfo>& pairwise_matches, std::vector<cv::detail::CameraParams>& cameras,
		double conf_thresh=1.);

private:

	static void bundleAdjuster(const std::vector<cv::detail::ImageFeatures>& features,
		const std::vector<cv::detail::MatchesInfo>& pairwise_matches, std::vector<cv::detail::CameraParams>& cameras,
		cv::detail::BundleAdjusterBase* bundleAdjuster, double conf_thresh=1.);

};

#endif /* REGISTRATION_HPP_ */
