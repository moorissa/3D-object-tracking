/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include <boost/circular_buffer.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"
#include "lidarData.hpp"
#include "camFusion.hpp"

using namespace std;

int main(int argc, const char *argv[])
{
    /* INIT VARIABLES AND DATA STRUCTURES */
    string dataPath = "../";
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_02/data/000000";
    string imgFileType = ".png";
    int imgStartIndex = 0;
    int imgEndIndex = 18;
    int imgStepWidth = 1;
    int imgFillWidth = 4;

    string yoloBasePath = dataPath + "dat/yolo/";
    string yoloClassesFile = yoloBasePath + "coco.names";
    string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    string yoloModelWeights = yoloBasePath + "yolov3.weights";

    string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
    string lidarFileType = ".bin";

    // Calibration matrices (keeping your existing values)
    cv::Mat P_rect_00(3,4,cv::DataType<double>::type);
    cv::Mat R_rect_00(4,4,cv::DataType<double>::type);
    cv::Mat RT(4,4,cv::DataType<double>::type);

    RT.at<double>(0,0) = 7.533745e-03; RT.at<double>(0,1) = -9.999714e-01; RT.at<double>(0,2) = -6.166020e-04; RT.at<double>(0,3) = -4.069766e-03;
    RT.at<double>(1,0) = 1.480249e-02; RT.at<double>(1,1) = 7.280733e-04; RT.at<double>(1,2) = -9.998902e-01; RT.at<double>(1,3) = -7.631618e-02;
    RT.at<double>(2,0) = 9.998621e-01; RT.at<double>(2,1) = 7.523790e-03; RT.at<double>(2,2) = 1.480755e-02; RT.at<double>(2,3) = -2.717806e-01;
    RT.at<double>(3,0) = 0.0; RT.at<double>(3,1) = 0.0; RT.at<double>(3,2) = 0.0; RT.at<double>(3,3) = 1.0;

    R_rect_00.at<double>(0,0) = 9.999239e-01; R_rect_00.at<double>(0,1) = 9.837760e-03; R_rect_00.at<double>(0,2) = -7.445048e-03; R_rect_00.at<double>(0,3) = 0.0;
    R_rect_00.at<double>(1,0) = -9.869795e-03; R_rect_00.at<double>(1,1) = 9.999421e-01; R_rect_00.at<double>(1,2) = -4.278459e-03; R_rect_00.at<double>(1,3) = 0.0;
    R_rect_00.at<double>(2,0) = 7.402527e-03; R_rect_00.at<double>(2,1) = 4.351614e-03; R_rect_00.at<double>(2,2) = 9.999631e-01; R_rect_00.at<double>(2,3) = 0.0;
    R_rect_00.at<double>(3,0) = 0; R_rect_00.at<double>(3,1) = 0; R_rect_00.at<double>(3,2) = 0; R_rect_00.at<double>(3,3) = 1;

    P_rect_00.at<double>(0,0) = 7.215377e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 6.095593e+02; P_rect_00.at<double>(0,3) = 0.000000e+00;
    P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 7.215377e+02; P_rect_00.at<double>(1,2) = 1.728540e+02; P_rect_00.at<double>(1,3) = 0.000000e+00;
    P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;

    double sensorFrameRate = 10.0 / imgStepWidth;
    int dataBufferSize = 2;

    // Define detector/descriptor combinations to test
    vector<pair<string, string>> combinations = {
        {"SHITOMASI", "BRISK"}, {"SHITOMASI", "BRIEF"}, {"SHITOMASI", "ORB"}, {"SHITOMASI", "FREAK"}, {"SHITOMASI", "SIFT"},
        {"HARRIS", "BRISK"}, {"HARRIS", "BRIEF"}, {"HARRIS", "ORB"}, {"HARRIS", "FREAK"}, {"HARRIS", "SIFT"},
        {"FAST", "BRISK"}, {"FAST", "BRIEF"}, {"FAST", "ORB"}, {"FAST", "FREAK"}, {"FAST", "SIFT"},
        {"BRISK", "BRISK"}, {"BRISK", "BRIEF"}, {"BRISK", "ORB"}, {"BRISK", "FREAK"}, {"BRISK", "SIFT"},
        {"ORB", "BRISK"}, {"ORB", "BRIEF"}, {"ORB", "ORB"}, {"ORB", "FREAK"}, {"ORB", "SIFT"},
        {"AKAZE", "BRISK"}, {"AKAZE", "BRIEF"}, {"AKAZE", "ORB"}, {"AKAZE", "FREAK"}, {"AKAZE", "AKAZE"}, {"AKAZE", "SIFT"},
        {"SIFT", "BRISK"}, {"SIFT", "BRIEF"}, {"SIFT", "FREAK"}, {"SIFT", "SIFT"}
    };

    // Open CSV file for writing
    ofstream csvFile("ttc_analysis_results.csv");
    csvFile << "Detector,Descriptor,Frame,TTC_Lidar,TTC_Camera,TTC_Difference,Keypoints,Matches,Valid_TTC\n";

    bool bVis = false; // Turn off visualization for batch processing

    // Loop through all detector/descriptor combinations
    for (const auto& combo : combinations) {
        string detectorType = combo.first;
        string descriptorType = combo.second;
        
        cout << "\nTesting " << detectorType << " + " << descriptorType << endl;

        // Reset data buffer for each combination
        boost::circular_buffer<DataFrame> dataBuffer(dataBufferSize);

        // Process all images for this combination
        for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex += imgStepWidth) {
            
            /* LOAD IMAGE INTO BUFFER */
            ostringstream imgNumber;
            imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
            string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;
            cv::Mat img = cv::imread(imgFullFilename);

            DataFrame frame;
            frame.cameraImg = img;
            dataBuffer.push_back(frame);

            /* DETECT & CLASSIFY OBJECTS */
            float confThreshold = 0.2;
            float nmsThreshold = 0.4;
            detectObjects((dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->boundingBoxes, 
                         confThreshold, nmsThreshold, yoloBasePath, yoloClassesFile, 
                         yoloModelConfiguration, yoloModelWeights, false);

            /* CROP LIDAR POINTS */
            string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
            std::vector<LidarPoint> lidarPoints;
            loadLidarFromFile(lidarPoints, lidarFullFilename);

            float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1;
            cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);
            (dataBuffer.end() - 1)->lidarPoints = lidarPoints;

            /* CLUSTER LIDAR POINT CLOUD */
            float shrinkFactor = 0.10;
            clusterLidarWithROI((dataBuffer.end()-1)->boundingBoxes, (dataBuffer.end() - 1)->lidarPoints, 
                               shrinkFactor, P_rect_00, R_rect_00, RT);

            /* DETECT IMAGE KEYPOINTS */
            cv::Mat imgGray;
            cv::cvtColor((dataBuffer.end()-1)->cameraImg, imgGray, cv::COLOR_BGR2GRAY);
            vector<cv::KeyPoint> keypoints;

            try {
                if (detectorType.compare("SHITOMASI") == 0) {
                    detKeypointsShiTomasi(keypoints, imgGray, false);
                } else if (detectorType.compare("HARRIS") == 0) {
                    detKeypointsHarris(keypoints, imgGray, false);
                } else {
                    detKeypointsModern(keypoints, imgGray, detectorType, false);
                }
            } catch (const exception& e) {
                cout << "Error in keypoint detection: " << e.what() << endl;
                continue;
            }

            (dataBuffer.end() - 1)->keypoints = keypoints;

            /* EXTRACT KEYPOINT DESCRIPTORS */
            cv::Mat descriptors;
            try {
                descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, 
                             descriptors, descriptorType);
            } catch (const exception& e) {
                cout << "Error in descriptor extraction: " << e.what() << endl;
                continue;
            }

            (dataBuffer.end() - 1)->descriptors = descriptors;

            if (dataBuffer.size() > 1) {
                /* MATCH KEYPOINT DESCRIPTORS */
                vector<cv::DMatch> matches;
                string matcherType = "MAT_BF";
                string descType = (descriptorType.compare("SIFT") == 0) ? "DES_HOG" : "DES_BINARY";
                string selectorType = "SEL_KNN";

                try {
                    matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                   (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                   matches, descType, matcherType, selectorType);
                } catch (const exception& e) {
                    cout << "Error in descriptor matching: " << e.what() << endl;
                    continue;
                }

                (dataBuffer.end() - 1)->kptMatches = matches;

                /* TRACK 3D OBJECT BOUNDING BOXES */
                map<int, int> bbBestMatches;
                try {
                    matchBoundingBoxes(matches, bbBestMatches, *(dataBuffer.end()-2), *(dataBuffer.end()-1));
                    (dataBuffer.end()-1)->bbMatches = bbBestMatches;
                } catch (const exception& e) {
                    cout << "Error in bounding box matching: " << e.what() << endl;
                    continue;
                }

                /* COMPUTE TTC ON OBJECT IN FRONT */
                for (auto it1 = (dataBuffer.end() - 1)->bbMatches.begin(); it1 != (dataBuffer.end() - 1)->bbMatches.end(); ++it1) {
                    BoundingBox *prevBB = nullptr, *currBB = nullptr;
                    
                    // Find current bounding box
                    for (auto it2 = (dataBuffer.end() - 1)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 1)->boundingBoxes.end(); ++it2) {
                        if (it1->second == it2->boxID) {
                            currBB = &(*it2);
                            break;
                        }
                    }

                    // Find previous bounding box
                    for (auto it2 = (dataBuffer.end() - 2)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 2)->boundingBoxes.end(); ++it2) {
                        if (it1->first == it2->boxID) {
                            prevBB = &(*it2);
                            break;
                        }
                    }

                    // Validate bounding boxes before proceeding
                    if (currBB == nullptr || prevBB == nullptr) {
                        cout << "Warning: Could not find matching bounding boxes" << endl;
                        continue;
                    }

                    if( currBB->lidarPoints.size()>0 && prevBB->lidarPoints.size()>0 ) {
                        double ttcLidar = -1, ttcCamera = -1;
                        bool validTTC = true;

                        try {
                            // Validate pointers before use
                            if (currBB == nullptr || prevBB == nullptr) {
                                validTTC = false;
                                continue;
                            }

                            computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints, sensorFrameRate, ttcLidar);
                            
                            // Check if we have valid keypoints and matches before camera TTC
                            if ((dataBuffer.end() - 2)->keypoints.size() > 0 && 
                                (dataBuffer.end() - 1)->keypoints.size() > 0 && 
                                matches.size() > 10) {  // Need minimum matches for stable TTC
                                
                                clusterKptMatchesWithROI(*currBB, (dataBuffer.end() - 2)->keypoints, 
                                                       (dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->kptMatches);
                                
                                // Only compute camera TTC if we have enough matches after clustering
                                if (currBB->kptMatches.size() > 5) {
                                    computeTTCCamera((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, 
                                                   currBB->kptMatches, sensorFrameRate, ttcCamera);
                                } else {
                                    ttcCamera = -1;  // Not enough matches
                                }
                            } else {
                                ttcCamera = -1;  // Not enough initial matches
                            }

                            // Validate TTC results
                            if (std::isnan(ttcLidar) || std::isinf(ttcLidar)) ttcLidar = -1;
                            if (std::isnan(ttcCamera) || std::isinf(ttcCamera)) ttcCamera = -1;
                            
                        } catch (const exception& e) {
                            cout << "Error in TTC computation: " << e.what() << endl;
                            validTTC = false;
                            ttcLidar = -1;
                            ttcCamera = -1;
                        } catch (...) {
                            cout << "Unknown error in TTC computation" << endl;
                            validTTC = false;
                            ttcLidar = -1;
                            ttcCamera = -1;
                        }

                        // Write to CSV - handle invalid values
                        double ttcDiff = (ttcLidar > 0 && ttcCamera > 0) ? abs(ttcLidar - ttcCamera) : -1;
                        
                        csvFile << detectorType << ","
                               << descriptorType << ","
                               << imgIndex << ","
                               << (ttcLidar > 0 ? to_string(ttcLidar) : "invalid") << ","
                               << (ttcCamera > 0 ? to_string(ttcCamera) : "invalid") << ","
                               << (ttcDiff > 0 ? to_string(ttcDiff) : "invalid") << ","
                               << keypoints.size() << ","
                               << matches.size() << ","
                               << (validTTC ? "1" : "0") << "\n";

                        cout << "Frame " << imgIndex << ": TTC Lidar=" 
                             << (ttcLidar > 0 ? to_string(ttcLidar) : "invalid")
                             << ", TTC Camera=" << (ttcCamera > 0 ? to_string(ttcCamera) : "invalid")
                             << ", Keypoints=" << keypoints.size() 
                             << ", Matches=" << matches.size() << endl;
                    }
                }
            }
        }
    }

    csvFile.close();
    cout << "\nAnalysis complete! Results saved to ttc_analysis_results.csv" << endl;
    return 0;
}
