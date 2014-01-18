#include "Mosaicing.hpp"

using namespace std;
using namespace cv;
using namespace cv::detail;

Mosaicing::Mosaicing() : QWidget() {

	/* Read input images */
	QStringList names = QFileDialog::getOpenFileNames(this, "Select input images", "./samples/2-translation-0.3MP",
		"Images (*.png *.gif *.jpg *.jpeg)");
	num_imgs = names.size();
	for (int i = 0; i < num_imgs; i++) {
		Mat img = imread(names.at(i).toStdString());
		if (img.empty()) {
			cerr << "Cannot read input image " << names.at(i).toStdString() << endl;
			this->close();
		}
		imgs.push_back(img);
		img_names.push_back(QFileInfo(QFile(names.at(i))).fileName().toStdString());
	}

	/* Find features in images */
	features = Registration::getSurfFeatures(imgs);

	/* Match features */
	pairwise_matches = Registration::getMatches(features);

	/* Estimate homographies between images using feature matches */
	cameras = Registration::estimateHomographies(features, pairwise_matches);

	/* Bundle adjustment for camera parameters refinement */
	Registration::bundleAdjusterRay(features, pairwise_matches, cameras);

	/* Warp images */
	warped_imgs.resize(num_imgs);
	warped_masks.resize(num_imgs);
	warped_corners.resize(num_imgs);
	warped_sizes.resize(num_imgs);
	Compositing::warpImages(imgs, cameras, warped_imgs, warped_corners, warped_masks, warped_sizes);

	/* Find seam masks */
	Compositing::graphCutSeamEstimation(warped_imgs, warped_corners, warped_masks);

//	/* Compensate exposure errors */
	Compositing::gainBlocksExposureCompensation(warped_corners, warped_imgs, warped_masks);

	/* Blend warped images */
	result = Compositing::blendImagesFeather(warped_imgs, warped_corners, warped_masks, warped_sizes);
	if (imwrite("result.jpg", result)) {
		result = imread("result.jpg");
	}

	/* Layout */
	rawButtons = new QButtonGroup();
	featuresButtons = new QButtonGroup();
	matchesCheckBoxes = new QButtonGroup();
	matchesCheckBoxes->setExclusive(false);
	warpedButtons = new QButtonGroup();
	maskButtons = new QButtonGroup();
	QGridLayout* layout = new QGridLayout();
	for (unsigned int i = 0; i < imgs.size(); i++) {
		/* Create widgets */
		QPushButton* rawButton = new QPushButton("Raw");
		QPushButton* featuresButton = new QPushButton("Features");
		QCheckBox* matchesCheckBox = new QCheckBox();
		QPushButton* warpedButton = new QPushButton("Warped");
		QPushButton* maskButton = new QPushButton("Mask");
		/* Position widgets */
		rawButtons->addButton(rawButton, i);
		featuresButtons->addButton(featuresButton, i);
		matchesCheckBoxes->addButton(matchesCheckBox, i);
		warpedButtons->addButton(warpedButton, i);
		maskButtons->addButton(maskButton, i);
		layout->addWidget(matchesCheckBoxes->button(i), i, 0);
		layout->addWidget(new QLabel(img_names[i].c_str()), i, 1);
		layout->addWidget(rawButtons->button(i), i, 2);
		layout->addWidget(featuresButtons->button(i), i, 3);
		layout->addWidget(warpedButtons->button(i), i, 4);
		layout->addWidget(maskButtons->button(i), i, 5);

		/* Connect signals to slots */
		QObject::connect(rawButtons, SIGNAL(buttonClicked(int)), this, SLOT(showRaw(int)));
		QObject::connect(featuresButtons, SIGNAL(buttonClicked(int)), this, SLOT(showFeatures(int)));
		QObject::connect(warpedButtons, SIGNAL(buttonClicked(int)), this, SLOT(showWarped(int)));
		QObject::connect(maskButtons, SIGNAL(buttonClicked(int)), this, SLOT(showMask(int)));

	}
	matchesButton = new QPushButton("Matches");
	QObject::connect(matchesButton, SIGNAL(clicked()), this, SLOT(showMatches()));
	layout->addWidget(matchesButton, imgs.size(), 0, 1, 2);
	resultButton = new QPushButton("Result");
	QObject::connect(resultButton, SIGNAL(clicked()), this, SLOT(showResult()));
	layout->addWidget(resultButton, imgs.size(), 2, 1, 4);
	this->setLayout(layout);
}

Mosaicing::~Mosaicing() {
	destroyAllWindows();
}

void Mosaicing::showRaw(int i) {
	string windowName = "Image " + img_names[i];
	namedWindow(windowName, CV_WINDOW_NORMAL);
	imshow(windowName, this->imgs[i]);
}

void Mosaicing::showFeatures(int i) {
	string windowName = "Features " + img_names[i];
	namedWindow(windowName, CV_WINDOW_NORMAL);
	Mat features_img;
	drawKeypoints(imgs[i], features[i].keypoints, features_img, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	imshow(windowName, features_img);
}

void Mosaicing::showMatches() {
	// TODO DIRTY
	int src = 0, dst = 0;
	vector<int> checked;
	for (int i = 0; i < matchesCheckBoxes->buttons().size(); i++) {
		if (matchesCheckBoxes->button(i)->isChecked()) {
			checked.push_back(i);
		}
	}
	if (checked.size() > 1) {
		src = checked[0];
		dst = checked[1];
	}
	MatchesInfo matches;
	for (unsigned int i = 0; i < pairwise_matches.size(); i++) {
		if (pairwise_matches[i].src_img_idx == src && pairwise_matches[i].dst_img_idx == dst) {
			matches = pairwise_matches[i];
		}
	}
	vector<char> inliers_mask(matches.inliers_mask.size());
	for (unsigned int i = 0; i < matches.inliers_mask.size(); i++) {
		inliers_mask[i] = matches.inliers_mask[i];
	}
	Mat matches_img;
	drawMatches(imgs[src], features[src].keypoints, imgs[dst], features[dst].keypoints, matches.matches, matches_img,
			Scalar::all(-1), Scalar::all(-1), inliers_mask, DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	string windowName = "Matches " + img_names[src] + " " + img_names[dst];
	namedWindow(windowName, CV_WINDOW_NORMAL);
	imshow(windowName, matches_img);
}

void Mosaicing::showWarped(int i) {
	string windowName = "Warped image " + img_names[i];
	namedWindow(windowName, CV_WINDOW_NORMAL);
	imshow(windowName, this->warped_imgs[i]);
}

void Mosaicing::showMask(int i) {
	string windowName = "Mask for image " + img_names[i];
	namedWindow(windowName, CV_WINDOW_NORMAL);
	imshow(windowName, this->warped_masks[i]);
}

void Mosaicing::showResult() {
	string windowName = "Result";
	namedWindow(windowName, CV_WINDOW_NORMAL);
	cout << result.type() << endl;
	imshow(windowName, result);
}
