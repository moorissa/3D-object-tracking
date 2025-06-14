
#include <iostream>
#include <numeric>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include "matching2D.hpp"
using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, 
                      cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, 
                      std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = cv::NORM_HAMMING;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F) {
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    {
        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(descSource, descRef, knn_matches, 2);

        const float ratio_thresh = 0.8f;
        for (auto& knn_match : knn_matches) {
            if (knn_match.size() >= 2 && knn_match[0].distance < ratio_thresh * knn_match[1].distance) {
                matches.push_back(knn_match[0]);
            }
        }
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    
    if (descriptorType.compare("BRISK") == 0)
    {
        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("ORB") == 0)
    {
        extractor = cv::ORB::create();
    }
    else if (descriptorType.compare("AKAZE") == 0)
    {
        extractor = cv::AKAZE::create();
    }
    else if (descriptorType.compare("SIFT") == 0)
    {
        extractor = cv::SIFT::create();
    }
    else
    {
        // Default to BRISK if unknown
        extractor = cv::BRISK::create();
        cout << "Unknown descriptor type: " << descriptorType << ", defaulting to BRISK" << endl;
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
// Add these functions to your matching2D_Student.cpp file

void detKeypointsShiTomasi(std::vector<cv::KeyPoint>& keypoints, cv::Mat& img, bool bVis)
{
    // Shi-Tomasi corner detection parameters
    int maxCorners = 1000;
    double qualityLevel = 0.01;
    double minDistance = 10;
    int blockSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;

    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, 
                           cv::Mat(), blockSize, useHarrisDetector, k);

    // Convert corners to KeyPoint format
    for (auto& corner : corners) {
        cv::KeyPoint kp(corner, 1.0f);
        keypoints.push_back(kp);
    }

    if (bVis) {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        cv::imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsHarris(std::vector<cv::KeyPoint>& keypoints, cv::Mat& img, bool bVis)
{
    // Harris corner detection parameters
    int maxCorners = 1000;
    double qualityLevel = 0.01;
    double minDistance = 10;
    int blockSize = 3;
    bool useHarrisDetector = true;
    double k = 0.04;

    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, 
                           cv::Mat(), blockSize, useHarrisDetector, k);

    // Convert corners to KeyPoint format
    for (auto& corner : corners) {
        cv::KeyPoint kp(corner, 1.0f);
        keypoints.push_back(kp);
    }

    if (bVis) {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "Harris Corner Detector Results";
        cv::namedWindow(windowName, 6);
        cv::imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsModern(std::vector<cv::KeyPoint>& keypoints, cv::Mat& img, std::string detectorType, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> detector;

    if (detectorType.compare("FAST") == 0) {
        detector = cv::FastFeatureDetector::create();
    }
    else if (detectorType.compare("BRISK") == 0) {
        detector = cv::BRISK::create();
    }
    else if (detectorType.compare("ORB") == 0) {
        detector = cv::ORB::create();
    }
    else if (detectorType.compare("AKAZE") == 0) {
        detector = cv::AKAZE::create();
    }
    else if (detectorType.compare("SIFT") == 0) {
        detector = cv::SIFT::create();
    }
    else {
        throw std::invalid_argument("Unknown detector type: " + detectorType);
    }

    detector->detect(img, keypoints);

    if (bVis) {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = detectorType + " Detector Results";
        cv::namedWindow(windowName, 6);
        cv::imshow(windowName, visImage);
        cv::waitKey(0);
    }
}