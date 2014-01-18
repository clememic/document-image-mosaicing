#ifndef MOSAICING_HPP_
#define MOSAICING_HPP_

#include <iostream>
#include <string>

#include <QWidget>
#include <QButtonGroup>
#include <QCheckBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>

#include <opencv2/opencv.hpp>
#include <opencv2/stitching/stitcher.hpp>

#include "Registration.hpp"
#include "Compositing.hpp"

class Mosaicing : public QWidget {
Q_OBJECT

public:

	Mosaicing();
	~Mosaicing();

private:

	int num_imgs;
	std::vector<std::string> img_names;
	std::vector<cv::Mat> imgs;
	std::vector<cv::detail::ImageFeatures> features;
	std::vector<cv::detail::MatchesInfo> pairwise_matches;
	std::vector<cv::detail::CameraParams> cameras;
	std::vector<cv::Mat> warped_imgs, warped_masks;
	std::vector<cv::Point> warped_corners;
	std::vector<cv::Size> warped_sizes;
	cv::Mat result;

	QButtonGroup* rawButtons;
	QButtonGroup* featuresButtons;
	QButtonGroup* matchesCheckBoxes;
	QPushButton* matchesButton;
	QButtonGroup* warpedButtons;
	QButtonGroup* maskButtons;
	QPushButton* resultButton;

public slots:

	void showRaw(int i);
	void showFeatures(int i);
	void showMatches();
	void showWarped(int i);
	void showMask(int i);
	void showResult();

};


#endif /* MOSAICING_HPP_ */
