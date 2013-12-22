#ifndef COMPOSITING_HPP_
#define COMPOSITING_HPP_

#include <vector>
#include <opencv2/stitching/stitcher.hpp>

class Compositing {

public:

	static void warpImages(const std::vector<cv::Mat>& imgs, const std::vector<cv::detail::CameraParams>& cameras,
		std::vector<cv::Mat>& warped_imgs, std::vector<cv::Point>& warped_corners, std::vector<cv::Mat>& warped_masks,
		std::vector<cv::Size>& warped_sizes);

	static void voronoiSeamEstimation(const std::vector<cv::Mat>& imgs, const std::vector<cv::Point>& corners,
		std::vector<cv::Mat>& masks);

	static void dynamicProgrammingSeamEstimation(const std::vector<cv::Mat>& imgs, const std::vector<cv::Point>& corners,
		std::vector<cv::Mat>& masks);

	static void graphCutSeamEstimation(const std::vector<cv::Mat>& imgs, const std::vector<cv::Point>& corners,
		std::vector<cv::Mat>& masks, int cost_type=cv::detail::GraphCutSeamFinder::COST_COLOR);

	static void gainExposureCompensation(std::vector<cv::Point>& corners, std::vector<cv::Mat>& imgs,
		const std::vector<cv::Mat>& masks);

	static void gainBlocksExposureCompensation(std::vector<cv::Point>& corners, std::vector<cv::Mat>& imgs,
		const std::vector<cv::Mat>& masks, int bl_width=32, int bl_height=32);

	static cv::Mat blendImagesMultiBand(const std::vector<cv::Mat>& imgs, const std::vector<cv::Point>& corners,
		const std::vector<cv::Mat>& masks, const std::vector<cv::Size>& sizes, float blend_strength=5.f);

private:

	static void estimateSeams(const std::vector<cv::Mat>& imgs, const std::vector<cv::Point>& corners,
		std::vector<cv::Mat>& masks, cv::detail::SeamFinder* seam_finder);

	static void compensateExposureErrors(std::vector<cv::Point>& corners, std::vector<cv::Mat>& imgs,
		const std::vector<cv::Mat>& masks, cv::detail::ExposureCompensator* exposure_compensator);

	static float getBlendWidth(const std::vector<cv::Size>& sizes, const std::vector<cv::Point>& corners,
		float blend_strength);

	static cv::Mat blendImages(const std::vector<cv::Mat>& imgs, const std::vector<cv::Point>& corners,
		const std::vector<cv::Mat>& masks, const std::vector<cv::Size>& sizes, cv::detail::Blender* blender);

};

#endif /* COMPOSITING_HPP_ */
