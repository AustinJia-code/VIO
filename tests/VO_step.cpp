/**
 * @file VO_step.cpp
 * @brief Interactive debugger for EVERY step of the visual odometry pipeline
 * 
 * Press SPACE to advance through steps
 * Press 'q' to quit
 * Press 's' to save current visualization
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <map>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>

// ============================================================================
// CONFIGURATION
// ============================================================================
const std::string ROOT = "../data/euroc_datasets/machine_hall/MH_01_easy/mav0/";
const std::pair<std::string, std::string> STEREO_0_PATH =
    {"cam0/data/1403636579763555584.png", "cam1/data/1403636579763555584.png"};
const std::pair<std::string, std::string> STEREO_1_PATH =
    {"cam0/data/1403636579813555456.png", "cam1/data/1403636579813555456.png"};

// Calibration files
const std::string CALIB_STEREO = "../data/calib/euroc_calib.yaml";
const std::string CALIB_CAM0 = ROOT + "cam0/sensor.yaml";
const std::string CALIB_CAM1 = ROOT + "cam1/sensor.yaml";

// ============================================================================
// HELPER STRUCTURES
// ============================================================================
struct CameraCalibration {
    cv::Mat K1, K2;           // Intrinsic matrices
    cv::Mat D1, D2;           // Distortion coefficients
    cv::Mat R1, R2;           // Rectification transforms (identity for rectified)
    cv::Mat P1, P2;           // Projection matrices
    cv::Mat Q;                // Disparity-to-depth mapping
    cv::Mat T_BS_cam0;        // Camera 0 extrinsics (body to sensor)
    cv::Mat T_BS_cam1;        // Camera 1 extrinsics
    cv::Size image_size;
    double baseline;          // Stereo baseline in meters
    
    bool load(const std::string& stereo_path, 
              const std::string& cam0_path, 
              const std::string& cam1_path) {
        
        // Load stereo calibration (K1, D1, Q)
        std::cout << "Loading stereo calibration: " << stereo_path << std::endl;
        cv::FileStorage fs_stereo(stereo_path, cv::FileStorage::READ);
        if (!fs_stereo.isOpened()) {
            std::cerr << "Failed to open: " << stereo_path << std::endl;
            return false;
        }
        
        fs_stereo["K1"] >> K1;
        fs_stereo["D1"] >> D1;
        fs_stereo["Q"] >> Q;
        fs_stereo.release();
        
        // K2 is same as K1 for EuRoC (same camera model)
        K2 = K1.clone();
        D2 = D1.clone();
        
        // Load cam0 sensor config (handle EuRoC format)
        std::cout << "Loading cam0 config: " << cam0_path << std::endl;
        if (!loadEuRoCSensor(cam0_path, T_BS_cam0, image_size)) {
            std::cerr << "Failed to load cam0 config" << std::endl;
            return false;
        }
        
        // Load cam1 sensor config
        std::cout << "Loading cam1 config: " << cam1_path << std::endl;
        cv::Size dummy_size;
        if (!loadEuRoCSensor(cam1_path, T_BS_cam1, dummy_size)) {
            std::cerr << "Failed to load cam1 config" << std::endl;
            return false;
        }
        
        // Calculate baseline from Q matrix
        // Q(3,2) = -1/Tx where Tx is baseline
        baseline = -1.0 / Q.at<double>(3, 2);
        
        // For pre-rectified images (EuRoC), R1 and R2 are identity
        R1 = cv::Mat::eye(3, 3, CV_64F);
        R2 = cv::Mat::eye(3, 3, CV_64F);
        
        // Projection matrices for rectified stereo
        // P1 = [K 0], P2 = [K [Tx; 0; 0]]
        P1 = cv::Mat::zeros(3, 4, CV_64F);
        K1.copyTo(P1(cv::Rect(0, 0, 3, 3)));
        
        P2 = cv::Mat::zeros(3, 4, CV_64F);
        K1.copyTo(P2(cv::Rect(0, 0, 3, 3)));
        P2.at<double>(0, 3) = K1.at<double>(0, 0) * baseline;  // fx * baseline
        
        return true;
    }
    
    // Helper to load EuRoC sensor.yaml files (which have comments that break FileStorage)
    bool loadEuRoCSensor(const std::string& path, cv::Mat& T_BS, cv::Size& size) {
        std::ifstream file(path);
        if (!file.is_open()) {
            std::cerr << "Could not open: " << path << std::endl;
            return false;
        }
        
        std::vector<double> T_BS_data;
        std::vector<int> resolution;
        std::string line;
        bool in_T_BS_data = false;
        
        while (std::getline(file, line)) {
            // Skip empty lines
            if (line.empty()) continue;
            
            // Remove leading/trailing whitespace
            size_t first = line.find_first_not_of(" \t\r\n");
            if (first == std::string::npos) continue;
            size_t last = line.find_last_not_of(" \t\r\n");
            line = line.substr(first, last - first + 1);
            
            // Skip comments
            if (line[0] == '#') continue;
            
            // Check for T_BS data section
            if (line.find("data:") != std::string::npos && !in_T_BS_data) {
                in_T_BS_data = true;
                // Try to parse data on same line
                size_t bracket_start = line.find('[');
                if (bracket_start != std::string::npos) {
                    line = line.substr(bracket_start);
                } else {
                    continue; // Data is on next lines
                }
            }
            
            // Parse T_BS data (can span multiple lines)
            if (in_T_BS_data) {
                // Remove brackets and parse numbers
                std::string cleaned = line;
                // Remove [ and ]
                size_t pos;
                while ((pos = cleaned.find('[')) != std::string::npos) {
                    cleaned.erase(pos, 1);
                }
                while ((pos = cleaned.find(']')) != std::string::npos) {
                    cleaned.erase(pos, 1);
                }
                
                // Parse comma-separated values
                std::istringstream iss(cleaned);
                std::string token;
                while (std::getline(iss, token, ',')) {
                    // Remove whitespace
                    size_t t_first = token.find_first_not_of(" \t\r\n");
                    if (t_first == std::string::npos) continue;
                    size_t t_last = token.find_last_not_of(" \t\r\n");
                    token = token.substr(t_first, t_last - t_first + 1);
                    
                    if (!token.empty()) {
                        try {
                            T_BS_data.push_back(std::stod(token));
                        } catch (...) {
                            // Skip invalid tokens
                        }
                    }
                }
                
                // Check if we found closing bracket
                if (line.find(']') != std::string::npos) {
                    in_T_BS_data = false;
                }
            }
            
            // Parse resolution (single line)
            if (line.find("resolution:") != std::string::npos) {
                size_t bracket_start = line.find('[');
                size_t bracket_end = line.find(']');
                if (bracket_start != std::string::npos && bracket_end != std::string::npos) {
                    std::string res_str = line.substr(bracket_start + 1, bracket_end - bracket_start - 1);
                    std::istringstream iss(res_str);
                    std::string token;
                    while (std::getline(iss, token, ',')) {
                        // Remove whitespace
                        size_t t_first = token.find_first_not_of(" \t\r\n");
                        if (t_first == std::string::npos) continue;
                        size_t t_last = token.find_last_not_of(" \t\r\n");
                        token = token.substr(t_first, t_last - t_first + 1);
                        
                        if (!token.empty()) {
                            try {
                                resolution.push_back(std::stoi(token));
                            } catch (...) {
                                // Skip invalid tokens
                            }
                        }
                    }
                }
            }
        }
        
        file.close();
        
        // Validate and convert T_BS
        if (T_BS_data.size() != 16) {
            std::cerr << "Invalid T_BS data size: " << T_BS_data.size() << std::endl;
            std::cerr << "Parsed values: ";
            for (auto v : T_BS_data) std::cerr << v << " ";
            std::cerr << std::endl;
            return false;
        }
        
        T_BS = cv::Mat(4, 4, CV_64F);
        for (int i = 0; i < 16; i++) {
            T_BS.at<double>(i / 4, i % 4) = T_BS_data[i];
        }
        
        // Set resolution
        if (resolution.size() == 2) {
            size = cv::Size(resolution[0], resolution[1]);
        }
        
        return true;
    }
    
    void print() const {
        std::cout << "\n=== CAMERA CALIBRATION ===" << std::endl;
        std::cout << "Image size: " << image_size << std::endl;
        std::cout << "Baseline: " << baseline << " m" << std::endl;
        
        std::cout << "\nK1 (Left intrinsics):\n" << K1 << std::endl;
        std::cout << "\nD1 (Left distortion):\n" << D1 << std::endl;
        
        std::cout << "\nP1 (Left projection):\n" << P1 << std::endl;
        std::cout << "\nP2 (Right projection):\n" << P2 << std::endl;
        
        std::cout << "\nQ (Disparity-to-depth):\n" << Q << std::endl;
        
        std::cout << "\nT_BS_cam0 (Cam0 extrinsics):\n" << T_BS_cam0 << std::endl;
        std::cout << "\nT_BS_cam1 (Cam1 extrinsics):\n" << T_BS_cam1 << std::endl;
        
        std::cout << "\nFocal length: " << K1.at<double>(0, 0) << " px" << std::endl;
        std::cout << "Principal point: (" 
                  << K1.at<double>(0, 2) << ", " 
                  << K1.at<double>(1, 2) << ")" << std::endl;
    }
    
    // Visualize calibration info
    void visualize() const {
        cv::Mat viz = cv::Mat::zeros(800, 1000, CV_8UC3);
        
        int y = 30;
        int line_h = 25;
        auto addText = [&](const std::string& text, cv::Scalar color = cv::Scalar(0, 255, 0)) {
            cv::putText(viz, text, cv::Point(20, y), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
            y += line_h;
        };
        
        addText("=== CAMERA CALIBRATION LOADED ===", cv::Scalar(0, 255, 255));
        y += 10;
        addText("Image Size: " + std::to_string(image_size.width) + " x " + 
                std::to_string(image_size.height));
        addText("Baseline: " + std::to_string(baseline) + " m");
        addText("Focal Length: " + std::to_string(K1.at<double>(0, 0)) + " px");
        y += 10;
        
        addText("Intrinsic Matrix K1:");
        char buf[200];
        for (int i = 0; i < 3; i++) {
            snprintf(buf, sizeof(buf), "  [%.2f, %.2f, %.2f]",
                    K1.at<double>(i, 0), K1.at<double>(i, 1), K1.at<double>(i, 2));
            addText(buf, cv::Scalar(200, 200, 200));
        }
        y += 10;
        
        addText("Distortion Coefficients:");
        snprintf(buf, sizeof(buf), "  [%.6f, %.6f, %.6f, %.6f]",
                D1.at<double>(0), D1.at<double>(1), D1.at<double>(2), D1.at<double>(3));
        addText(buf, cv::Scalar(200, 200, 200));
        y += 10;
        
        addText("Ready to process images!", cv::Scalar(0, 255, 0));
        y += 20;
        addText("Press SPACE to start pipeline...", cv::Scalar(255, 255, 0));
        
        cv::imshow("0. Calibration Info", viz);
    }
};

struct FeatureMatches {
    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;
    std::vector<cv::DMatch> matches;
    cv::Mat descriptors1;
    cv::Mat descriptors2;
};

// ============================================================================
// VISUALIZATION HELPERS
// ============================================================================
void waitForKey(const std::string& step_name) {
    std::cout << "\n>>> STEP: " << step_name << std::endl;
    std::cout << "Press SPACE to continue, 'q' to quit, 's' to save..." << std::endl;
    
    while (true) {
        int key = cv::waitKey(0);
        if (key == ' ') break;
        if (key == 'q' || key == 27) exit(0);
        if (key == 's') {
            std::cout << "Save functionality - implement as needed" << std::endl;
        }
    }
}

void showImage(const std::string& name, const cv::Mat& img, bool wait = true) {
    cv::namedWindow(name, cv::WINDOW_NORMAL);
    cv::resizeWindow(name, 960, 720);
    cv::imshow(name, img);
    if (wait) waitForKey(name);
}

void printStats(const std::string& label, const cv::Mat& img) {
    double min_val, max_val;
    cv::minMaxLoc(img, &min_val, &max_val);
    cv::Scalar mean, stddev;
    cv::meanStdDev(img, mean, stddev);
    
    std::cout << label << " stats:" << std::endl;
    std::cout << "  Size: " << img.size() << ", Type: " << img.type() << std::endl;
    std::cout << "  Range: [" << min_val << ", " << max_val << "]" << std::endl;
    std::cout << "  Mean: " << mean[0] << ", Stddev: " << stddev[0] << std::endl;
}

// ============================================================================
// PIPELINE STEPS
// ============================================================================

/**
 * STEP 1: Load raw images
 */
std::pair<cv::Mat, cv::Mat> loadRawImages(const std::string& left_path, 
                                           const std::string& right_path) {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "STEP 1: LOADING RAW IMAGES" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    
    cv::Mat left = cv::imread(left_path, cv::IMREAD_GRAYSCALE);
    cv::Mat right = cv::imread(right_path, cv::IMREAD_GRAYSCALE);
    
    if (left.empty() || right.empty()) {
        throw std::runtime_error("Failed to load images!");
    }
    
    std::cout << "Left path:  " << left_path << std::endl;
    std::cout << "Right path: " << right_path << std::endl;
    
    printStats("Left raw", left);
    printStats("Right raw", right);
    
    // Show raw images
    showImage("1a. Raw Left Image", left, false);
    showImage("1b. Raw Right Image", right, false);
    
    // Side by side comparison
    cv::Mat combined;
    cv::hconcat(left, right, combined);
    showImage("1c. Raw Stereo Pair (Left | Right)", combined, true);
    
    return {left, right};
}

/**
 * STEP 2: Undistort images (EuRoC images are already rectified)
 */
std::pair<cv::Mat, cv::Mat> rectifyImages(const cv::Mat& left_raw, 
                                           const cv::Mat& right_raw,
                                           const CameraCalibration& calib) {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "STEP 2: UNDISTORTION" << std::endl;
    std::cout << "(Note: EuRoC images are pre-rectified, only undistorting)" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    
    cv::Mat left_undist, right_undist;
    
    // For EuRoC, images are already rectified, we just need to undistort
    std::cout << "Undistorting images..." << std::endl;
    cv::undistort(left_raw, left_undist, calib.K1, calib.D1, calib.K1);
    cv::undistort(right_raw, right_undist, calib.K2, calib.D2, calib.K2);
    
    printStats("Left undistorted", left_undist);
    printStats("Right undistorted", right_undist);
    
    // Visualize with epipolar lines (should be horizontal since pre-rectified)
    cv::Mat left_color, right_color, combined;
    cv::cvtColor(left_undist, left_color, cv::COLOR_GRAY2BGR);
    cv::cvtColor(right_undist, right_color, cv::COLOR_GRAY2BGR);
    
    // Draw horizontal lines to verify rectification
    for (int y = 0; y < left_color.rows; y += 40) {
        cv::line(left_color, cv::Point(0, y), cv::Point(left_color.cols, y), 
                 cv::Scalar(0, 255, 0), 1);
        cv::line(right_color, cv::Point(0, y), cv::Point(right_color.cols, y), 
                 cv::Scalar(0, 255, 0), 1);
    }
    
    cv::hconcat(left_color, right_color, combined);
    showImage("2a. Undistorted with Epipolar Lines (pre-rectified)", combined, false);
    
    // Show difference
    cv::Mat diff_left, diff_right;
    cv::absdiff(left_raw, left_undist, diff_left);
    cv::absdiff(right_raw, right_undist, diff_right);
    
    // Amplify difference for visualization
    diff_left *= 5;
    diff_right *= 5;
    
    cv::Mat diff_combined;
    cv::hconcat(diff_left, diff_right, diff_combined);
    showImage("2b. Undistortion Difference (amplified 5x)", diff_combined, true);
    
    return {left_undist, right_undist};
}

/**
 * STEP 3: Compute disparity and depth
 */
std::pair<cv::Mat, cv::Mat> computeDisparityAndDepth(const cv::Mat& left_rect, 
                                                       const cv::Mat& right_rect,
                                                       const cv::Mat& Q,
                                                       double baseline,
                                                       double focal_length) {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "STEP 3: DISPARITY & DEPTH COMPUTATION" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    
    // Create stereo matcher
    int minDisparity = 0;
    int numDisparities = 16 * 6;  // Must be divisible by 16
    int blockSize = 11;
    
    cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(
        minDisparity,
        numDisparities,
        blockSize,
        8 * 3 * blockSize * blockSize,   // P1
        32 * 3 * blockSize * blockSize,  // P2
        1,                                // disp12MaxDiff
        63,                               // preFilterCap
        10,                               // uniquenessRatio
        100,                              // speckleWindowSize
        32,                               // speckleRange
        cv::StereoSGBM::MODE_SGBM_3WAY
    );
    
    std::cout << "SGBM Parameters:" << std::endl;
    std::cout << "  minDisparity: " << minDisparity << std::endl;
    std::cout << "  numDisparities: " << numDisparities << std::endl;
    std::cout << "  blockSize: " << blockSize << std::endl;
    
    cv::Mat disparity;
    std::cout << "Computing disparity (this may take a moment)..." << std::endl;
    stereo->compute(left_rect, right_rect, disparity);
    
    // Convert to float
    cv::Mat disparity_float;
    disparity.convertTo(disparity_float, CV_32F, 1.0/16.0);
    
    printStats("Disparity (raw)", disparity_float);
    
    // Visualize disparity
    cv::Mat disparity_vis;
    cv::normalize(disparity_float, disparity_vis, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::Mat disparity_color;
    cv::applyColorMap(disparity_vis, disparity_color, cv::COLORMAP_JET);
    
    showImage("3a. Disparity Map (color)", disparity_color, false);
    showImage("3b. Disparity Map (grayscale)", disparity_vis, false);
    
    // ===== COMPUTE DEPTH MAP MANUALLY =====
    std::cout << "\nComputing depth map from disparity..." << std::endl;
    std::cout << "Using formula: depth = (focal_length * baseline) / disparity" << std::endl;
    std::cout << "  focal_length = " << focal_length << " px" << std::endl;
    std::cout << "  baseline = " << baseline << " m" << std::endl;
    
    cv::Mat depth = cv::Mat::zeros(disparity_float.size(), CV_32F);
    
    double fb = focal_length * baseline;  // f * b
    std::cout << "  f * b = " << fb << std::endl;
    
    int valid_count = 0;
    for (int y = 0; y < disparity_float.rows; y++) {
        for (int x = 0; x < disparity_float.cols; x++) {
            float d = disparity_float.at<float>(y, x);
            if (d > 0.5) {  // Valid disparity threshold
                float z = fb / d;
                if (z > 0 && z < 100) {  // Realistic depth range
                    depth.at<float>(y, x) = z;
                    valid_count++;
                }
            }
        }
    }
    
    std::cout << "Valid depth pixels: " << valid_count << " / " 
              << (depth.rows * depth.cols) << std::endl;
    
    printStats("Depth map (meters)", depth);
    
    // Calculate valid depth statistics
    cv::Mat mask = (depth > 0);
    double min_depth, max_depth;
    cv::minMaxLoc(depth, &min_depth, &max_depth, nullptr, nullptr, mask);
    
    if (valid_count > 0) {
        std::cout << "Valid depth range: [" << min_depth << ", " << max_depth << "] meters" << std::endl;
    }
    
    // Visualize depth
    cv::Mat depth_vis;
    cv::normalize(depth, depth_vis, 0, 255, cv::NORM_MINMAX, CV_8U, mask);
    cv::Mat depth_color;
    cv::applyColorMap(depth_vis, depth_color, cv::COLORMAP_TURBO);
    depth_color.setTo(cv::Scalar(0, 0, 0), ~mask);  // Black for invalid
    
    showImage("3c. Depth Map (meters, color)", depth_color, true);
    
    return {disparity_float, depth};
}

/**
 * STEP 4: Extract features
 */
FeatureMatches extractAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                                        bool temporal = false) {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "STEP 4: FEATURE EXTRACTION & MATCHING" << std::endl;
    if (temporal) std::cout << "(TEMPORAL: Frame t and Frame t+1)" << std::endl;
    else std::cout << "(STEREO: Left and Right)" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    
    FeatureMatches result;
    
    // Detect features using ORB
    cv::Ptr<cv::ORB> detector = cv::ORB::create(2000);
    
    std::cout << "Detecting keypoints..." << std::endl;
    detector->detectAndCompute(img1, cv::noArray(), result.keypoints1, result.descriptors1);
    detector->detectAndCompute(img2, cv::noArray(), result.keypoints2, result.descriptors2);
    
    std::cout << "Keypoints in image 1: " << result.keypoints1.size() << std::endl;
    std::cout << "Keypoints in image 2: " << result.keypoints2.size() << std::endl;
    
    // Visualize keypoints
    cv::Mat img1_kp, img2_kp;
    cv::drawKeypoints(img1, result.keypoints1, img1_kp, cv::Scalar(0, 255, 0));
    cv::drawKeypoints(img2, result.keypoints2, img2_kp, cv::Scalar(0, 255, 0));
    
    cv::Mat kp_combined;
    cv::hconcat(img1_kp, img2_kp, kp_combined);
    showImage("4a. Detected Keypoints", kp_combined, true);
    
    // Match features
    std::cout << "Matching features with BFMatcher..." << std::endl;
    cv::BFMatcher matcher(cv::NORM_HAMMING, true);  // crossCheck = true
    matcher.match(result.descriptors1, result.descriptors2, result.matches);
    
    std::cout << "Raw matches: " << result.matches.size() << std::endl;
    
    // Filter matches by distance
    std::vector<cv::DMatch> good_matches;
    float min_dist = 1000.0f, max_dist = 0.0f;
    
    for (const auto& m : result.matches) {
        if (m.distance < min_dist) min_dist = m.distance;
        if (m.distance > max_dist) max_dist = m.distance;
    }
    
    std::cout << "Distance range: [" << min_dist << ", " << max_dist << "]" << std::endl;
    
    float threshold = std::min(std::max(2.5f * min_dist, 20.0f), 60.0f);
    for (const auto& m : result.matches) {
        if (m.distance < threshold) {
            good_matches.push_back(m);
        }
    }
    
    std::cout << "Good matches (distance < " << threshold << "): " 
              << good_matches.size() << std::endl;
    
    // Visualize all matches
    cv::Mat img_matches_all;
    cv::drawMatches(img1, result.keypoints1, img2, result.keypoints2,
                    result.matches, img_matches_all);
    showImage("4b. All Matches", img_matches_all, false);
    
    // Visualize good matches
    cv::Mat img_matches_good;
    cv::drawMatches(img1, result.keypoints1, img2, result.keypoints2,
                    good_matches, img_matches_good);
    showImage("4c. Filtered Matches", img_matches_good, true);
    
    result.matches = good_matches;
    return result;
}

/**
 * STEP 5: Triangulate 3D points
 */
std::vector<cv::Point3f> triangulatePoints(const FeatureMatches& matches,
                                            const cv::Mat& P1, 
                                            const cv::Mat& P2) {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "STEP 5: 3D POINT TRIANGULATION" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    
    if (matches.matches.empty()) {
        std::cout << "No matches to triangulate!" << std::endl;
        return {};
    }
    
    // Extract matched point pairs
    std::vector<cv::Point2f> pts1, pts2;
    for (const auto& m : matches.matches) {
        pts1.push_back(matches.keypoints1[m.queryIdx].pt);
        pts2.push_back(matches.keypoints2[m.trainIdx].pt);
    }
    
    std::cout << "Triangulating " << pts1.size() << " point pairs..." << std::endl;
    
    // Triangulate
    cv::Mat points4D;
    cv::triangulatePoints(P1, P2, pts1, pts2, points4D);
    
    // Convert to 3D points
    std::vector<cv::Point3f> points3D;
    points3D.reserve(points4D.cols);
    
    int valid_count = 0;
    for (int i = 0; i < points4D.cols; i++) {
        float w = points4D.at<float>(3, i);
        if (std::abs(w) > 1e-6) {
            cv::Point3f pt(
                points4D.at<float>(0, i) / w,
                points4D.at<float>(1, i) / w,
                points4D.at<float>(2, i) / w
            );
            
            // Filter out unrealistic points
            if (pt.z > 0 && pt.z < 100) {
                points3D.push_back(pt);
                valid_count++;
            }
        }
    }
    
    std::cout << "Valid 3D points: " << valid_count << " / " << points4D.cols << std::endl;
    
    // Calculate statistics
    if (!points3D.empty()) {
        float min_z = 1e10, max_z = -1e10, avg_z = 0;
        for (const auto& pt : points3D) {
            if (pt.z < min_z) min_z = pt.z;
            if (pt.z > max_z) max_z = pt.z;
            avg_z += pt.z;
        }
        avg_z /= points3D.size();
        
        std::cout << "Depth (Z) statistics:" << std::endl;
        std::cout << "  Min: " << min_z << " m" << std::endl;
        std::cout << "  Max: " << max_z << " m" << std::endl;
        std::cout << "  Avg: " << avg_z << " m" << std::endl;
    }
    
    waitForKey("3D Point Triangulation Complete");
    
    return points3D;
}

/**
 * STEP 6: Estimate camera motion (Essential matrix & pose recovery)
 */
void estimateCameraMotion(const FeatureMatches& matches_t0_t1,
                          const cv::Mat& K) {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "STEP 6a: CAMERA MOTION ESTIMATION (Essential Matrix)" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    
    if (matches_t0_t1.matches.size() < 8) {
        std::cout << "Not enough matches for motion estimation!" << std::endl;
        return;
    }
    
    // Extract matched points
    std::vector<cv::Point2f> pts1, pts2;
    for (const auto& m : matches_t0_t1.matches) {
        pts1.push_back(matches_t0_t1.keypoints1[m.queryIdx].pt);
        pts2.push_back(matches_t0_t1.keypoints2[m.trainIdx].pt);
    }
    
    std::cout << "Computing Essential matrix from " << pts1.size() << " matches..." << std::endl;
    
    // Find essential matrix
    cv::Mat mask;
    cv::Mat E = cv::findEssentialMat(pts1, pts2, K, cv::RANSAC, 0.999, 1.0, mask);
    
    int inliers = cv::countNonZero(mask);
    std::cout << "RANSAC inliers: " << inliers << " / " << pts1.size() 
              << " (" << (100.0 * inliers / pts1.size()) << "%)" << std::endl;
    
    std::cout << "\nEssential Matrix E:" << std::endl << E << std::endl;
    
    // Recover pose
    cv::Mat R, t;
    int pose_inliers = cv::recoverPose(E, pts1, pts2, K, R, t, mask);
    
    std::cout << "\nRecovered pose inliers: " << pose_inliers << std::endl;
    std::cout << "\nRotation Matrix R:" << std::endl << R << std::endl;
    std::cout << "\nTranslation vector t (UNIT DIRECTION ONLY):" << std::endl << t << std::endl;
    
    // Convert rotation to axis-angle
    cv::Mat rvec;
    cv::Rodrigues(R, rvec);
    double angle = cv::norm(rvec) * 180.0 / M_PI;
    
    std::cout << "\nMotion summary (NO SCALE):" << std::endl;
    std::cout << "  Rotation angle: " << angle << " degrees" << std::endl;
    std::cout << "  Translation magnitude: 1 (UNIT VECTOR - NO SCALE)" << std::endl;
    std::cout << "  Translation direction: [" 
              << t.at<double>(0) << ", " 
              << t.at<double>(1) << ", " 
              << t.at<double>(2) << "]" << std::endl;
    
    std::cout << "\n*** NOTE: Essential matrix does NOT give scale! ***" << std::endl;
    std::cout << "    We need stereo depth to recover absolute motion." << std::endl;
    
    waitForKey("Camera Motion Estimation (Essential Matrix)");
}

/**
 * STEP 6b: Absolute motion estimation using stereo depth (3D-2D PnP)
 */
void estimateAbsoluteMotion(const FeatureMatches& stereo_matches_t0,
                            const FeatureMatches& temporal_matches,
                            const cv::Mat& depth_t0,
                            const cv::Mat& K,
                            const cv::Mat& P1,
                            const cv::Mat& P2) {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "STEP 6b: ABSOLUTE MOTION ESTIMATION (3D-2D PnP with Stereo Depth)" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    
    // Step 1: Use depth map to get 3D positions for keypoints at t=0
    std::cout << "Extracting 3D positions from depth map at t=0..." << std::endl;
    
    // Camera intrinsics
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);
    
    std::vector<cv::Point3f> points_3d;  // 3D positions at t=0
    std::vector<cv::Point2f> points_2d;  // 2D positions at t=1
    
    int depth_valid_count = 0;
    int matched_count = 0;
    
    // For each temporal match
    for (const auto& m : temporal_matches.matches) {
        // Get keypoint position at t=0
        cv::Point2f pt_t0 = temporal_matches.keypoints1[m.queryIdx].pt;
        cv::Point2f pt_t1 = temporal_matches.keypoints2[m.trainIdx].pt;
        
        // Get depth at this pixel (nearest neighbor)
        int x = static_cast<int>(pt_t0.x + 0.5);
        int y = static_cast<int>(pt_t0.y + 0.5);
        
        // Check bounds
        if (x >= 0 && x < depth_t0.cols && y >= 0 && y < depth_t0.rows) {
            float depth = depth_t0.at<float>(y, x);
            
            if (depth > 0.5 && depth < 50.0) {  // Valid depth
                depth_valid_count++;
                
                // Back-project to 3D using pinhole model
                float Z = depth;
                float X = (x - cx) * Z / fx;
                float Y = (y - cy) * Z / fy;
                
                points_3d.push_back(cv::Point3f(X, Y, Z));
                points_2d.push_back(pt_t1);
                matched_count++;
            }
        }
    }
    
    std::cout << "Temporal matches: " << temporal_matches.matches.size() << std::endl;
    std::cout << "Matches with valid depth: " << depth_valid_count << std::endl;
    std::cout << "3D-2D correspondences: " << matched_count << std::endl;
    
    if (points_3d.size() < 6) {
        std::cout << "\nERROR: Not enough 3D-2D correspondences for PnP!" << std::endl;
        std::cout << "Need at least 6, got " << points_3d.size() << std::endl;
        std::cout << "\nPossible issues:" << std::endl;
        std::cout << "  - Depth map has no valid values (check stereo matching)" << std::endl;
        std::cout << "  - Temporal matches don't overlap with valid depth regions" << std::endl;
        waitForKey("Absolute Motion Estimation (insufficient data)");
        return;
    }
    
    // Step 2: Solve PnP to get camera pose at t=1 relative to t=0
    std::cout << "\nSolving PnP (3D-2D) for absolute camera motion..." << std::endl;
    
    cv::Mat rvec, tvec;
    cv::Mat inliers;
    
    bool success = cv::solvePnPRansac(
        points_3d,
        points_2d,
        K,
        cv::Mat(),  // No distortion (already undistorted)
        rvec,
        tvec,
        false,      // useExtrinsicGuess
        100,        // iterationsCount
        8.0,        // reprojectionError
        0.99,       // confidence
        inliers     // inliers
    );
    
    if (!success) {
        std::cout << "PnP failed!" << std::endl;
        waitForKey("Absolute Motion Estimation (PnP failed)");
        return;
    }
    
    int num_inliers = inliers.rows;
    std::cout << "PnP inliers: " << num_inliers << " / " << points_3d.size() 
              << " (" << (100.0 * num_inliers / points_3d.size()) << "%)" << std::endl;
    
    // Convert rotation vector to matrix
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    
    // Calculate rotation angle
    double angle = cv::norm(rvec) * 180.0 / M_PI;
    
    // Calculate translation magnitude (in meters!)
    double tx = tvec.at<double>(0);
    double ty = tvec.at<double>(1);
    double tz = tvec.at<double>(2);
    double t_magnitude = std::sqrt(tx*tx + ty*ty + tz*tz);
    
    std::cout << "\n=== ABSOLUTE MOTION (WITH SCALE) ===" << std::endl;
    std::cout << "\nRotation:" << std::endl;
    std::cout << "  Angle: " << angle << " degrees" << std::endl;
    std::cout << "  Axis (unit): [" 
              << rvec.at<double>(0)/cv::norm(rvec) << ", "
              << rvec.at<double>(1)/cv::norm(rvec) << ", "
              << rvec.at<double>(2)/cv::norm(rvec) << "]" << std::endl;
    
    std::cout << "\nTranslation (REAL SCALE in meters):" << std::endl;
    std::cout << "  Magnitude: " << t_magnitude << " meters" << std::endl;
    std::cout << "  Vector: [" << tx << ", " << ty << ", " << tz << "] m" << std::endl;
    std::cout << "  Direction (unit): [" 
              << tx/t_magnitude << ", " 
              << ty/t_magnitude << ", " 
              << tz/t_magnitude << "]" << std::endl;
    
    std::cout << "\nCamera Coordinate Frame:" << std::endl;
    std::cout << "  X-axis: Right" << std::endl;
    std::cout << "  Y-axis: Down" << std::endl;
    std::cout << "  Z-axis: Forward (optical axis)" << std::endl;
    
    std::cout << "\nInterpretation:" << std::endl;
    if (std::abs(tx) > std::abs(ty) && std::abs(tx) > std::abs(tz)) {
        std::cout << "  Primary motion: " << (tx > 0 ? "RIGHT" : "LEFT") << std::endl;
    } else if (std::abs(ty) > std::abs(tz)) {
        std::cout << "  Primary motion: " << (ty > 0 ? "DOWN" : "UP") << std::endl;
    } else {
        std::cout << "  Primary motion: " << (tz > 0 ? "FORWARD" : "BACKWARD") << std::endl;
    }
    
    // Create visualization
    cv::Mat viz = cv::Mat::zeros(600, 900, CV_8UC3);
    int y_pos = 30;
    
    auto addText = [&](const std::string& text, cv::Scalar color = cv::Scalar(0, 255, 0)) {
        cv::putText(viz, text, cv::Point(20, y_pos), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
        y_pos += 30;
    };
    
    addText("=== ABSOLUTE CAMERA MOTION ===", cv::Scalar(0, 255, 255));
    y_pos += 10;
    
    char buf[200];
    snprintf(buf, sizeof(buf), "Rotation: %.2f degrees", angle);
    addText(buf);
    
    snprintf(buf, sizeof(buf), "Translation: %.3f meters", t_magnitude);
    addText(buf, cv::Scalar(0, 255, 255));
    
    y_pos += 10;
    addText("Translation Vector (meters):");
    snprintf(buf, sizeof(buf), "  X (right):   %+.4f m", tx);
    addText(buf, cv::Scalar(200, 200, 200));
    snprintf(buf, sizeof(buf), "  Y (down):    %+.4f m", ty);
    addText(buf, cv::Scalar(200, 200, 200));
    snprintf(buf, sizeof(buf), "  Z (forward): %+.4f m", tz);
    addText(buf, cv::Scalar(200, 200, 200));
    
    y_pos += 10;
    snprintf(buf, sizeof(buf), "3D-2D Correspondences: %d", (int)points_3d.size());
    addText(buf);
    snprintf(buf, sizeof(buf), "PnP Inliers: %d (%.1f%%)", num_inliers, 
             100.0 * num_inliers / points_3d.size());
    addText(buf);
    
    showImage("6b. Absolute Motion with Scale", viz, true);
}

/**
 * STEP 7: Visualize full pipeline summary
 */
void visualizeSummary(const cv::Mat& left0, const cv::Mat& right0,
                      const cv::Mat& left1, const cv::Mat& right1,
                      const cv::Mat& disparity, const cv::Mat& depth,
                      const FeatureMatches& stereo_matches,
                      const FeatureMatches& temporal_matches) {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "STEP 7: PIPELINE SUMMARY" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    
    // Calculate depth statistics
    cv::Mat depth_mask = (depth > 0);
    double min_depth, max_depth, avg_depth;
    cv::minMaxLoc(depth, &min_depth, &max_depth, nullptr, nullptr, depth_mask);
    cv::Scalar depth_mean = cv::mean(depth, depth_mask);
    avg_depth = depth_mean[0];
    int valid_depth_pixels = cv::countNonZero(depth_mask);
    
    // Create summary visualization
    cv::Mat summary = cv::Mat::zeros(800, 1280, CV_8UC3);
    
    // Add text summary
    int y = 30;
    int line_height = 25;
    
    auto addText = [&](const std::string& text) {
        cv::putText(summary, text, cv::Point(20, y), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        y += line_height;
    };
    
    addText("=== VISUAL ODOMETRY PIPELINE SUMMARY ===");
    y += 10;
    addText("Image Size: " + std::to_string(left0.cols) + "x" + std::to_string(left0.rows));
    addText("");
    addText("Depth Map:");
    addText("  - Valid pixels: " + std::to_string(valid_depth_pixels) + " / " + 
            std::to_string(depth.rows * depth.cols));
    addText("  - Depth range: " + std::to_string(min_depth) + " - " + 
            std::to_string(max_depth) + " m");
    addText("  - Average depth: " + std::to_string(avg_depth) + " m");
    addText("");
    addText("Stereo Matching:");
    addText("  - Keypoints (L/R): " + std::to_string(stereo_matches.keypoints1.size()) + 
            " / " + std::to_string(stereo_matches.keypoints2.size()));
    addText("  - Matches: " + std::to_string(stereo_matches.matches.size()));
    addText("");
    addText("Temporal Matching:");
    addText("  - Keypoints (t0/t1): " + std::to_string(temporal_matches.keypoints1.size()) + 
            " / " + std::to_string(temporal_matches.keypoints2.size()));
    addText("  - Matches: " + std::to_string(temporal_matches.matches.size()));
    addText("");
    addText("Pipeline completed successfully!");
    addText("");
    addText("Press SPACE to finish");
    
    showImage("7. Pipeline Summary", summary, true);
    
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "ALL PIPELINE STEPS COMPLETE!" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
}

// ============================================================================
// MAIN
// ============================================================================
int main(int argc, char** argv) {
    try {
        std::cout << "\n";
        std::cout << "████████████████████████████████████████████████████████████████\n";
        std::cout << "█                                                              █\n";
        std::cout << "█     VISUAL ODOMETRY PIPELINE DEBUGGER                       █\n";
        std::cout << "█     Step-by-step visualization and debugging                █\n";
        std::cout << "█                                                              █\n";
        std::cout << "████████████████████████████████████████████████████████████████\n";
        std::cout << "\n";
        
        // Load calibration
        std::cout << "Loading camera calibration..." << std::endl;
        CameraCalibration calib;
        if (!calib.load(CALIB_STEREO, CALIB_CAM0, CALIB_CAM1)) {
            std::cerr << "ERROR: Could not load calibration files!" << std::endl;
            std::cerr << "Expected paths:" << std::endl;
            std::cerr << "  Stereo: " << CALIB_STEREO << std::endl;
            std::cerr << "  Cam0:   " << CALIB_CAM0 << std::endl;
            std::cerr << "  Cam1:   " << CALIB_CAM1 << std::endl;
            return -1;
        }
        calib.print();
        calib.visualize();
        waitForKey("Calibration Loaded");
        
        // ==================================================================
        // STEREO FRAME 0 (t = 0)
        // ==================================================================
        auto [left0_raw, right0_raw] = loadRawImages(
            ROOT + STEREO_0_PATH.first, 
            ROOT + STEREO_0_PATH.second
        );
        
        auto [left0_rect, right0_rect] = rectifyImages(left0_raw, right0_raw, calib);
        
        auto [disparity0, depth0] = computeDisparityAndDepth(left0_rect, right0_rect, 
                                                               calib.Q, calib.baseline, 
                                                               calib.K1.at<double>(0, 0));
        
        FeatureMatches stereo_matches = extractAndMatchFeatures(left0_rect, right0_rect, false);
        
        std::vector<cv::Point3f> points3D = triangulatePoints(
            stereo_matches, calib.P1, calib.P2
        );
        
        // ==================================================================
        // STEREO FRAME 1 (t = 1)
        // ==================================================================
        std::cout << "\n\n";
        std::cout << "████████████████████████████████████████████████████████████████\n";
        std::cout << "█  MOVING TO NEXT FRAME (temporal)                            █\n";
        std::cout << "████████████████████████████████████████████████████████████████\n";
        
        auto [left1_raw, right1_raw] = loadRawImages(
            ROOT + STEREO_1_PATH.first, 
            ROOT + STEREO_1_PATH.second
        );
        
        auto [left1_rect, right1_rect] = rectifyImages(left1_raw, right1_raw, calib);
        
        // ==================================================================
        // TEMPORAL MATCHING (frame 0 to frame 1)
        // ==================================================================
        FeatureMatches temporal_matches = extractAndMatchFeatures(
            left0_rect, left1_rect, true
        );
        
        estimateCameraMotion(temporal_matches, calib.K1);
        
        estimateAbsoluteMotion(stereo_matches, temporal_matches, depth0,
                              calib.K1, calib.P1, calib.P2);
        
        // ==================================================================
        // FINAL SUMMARY
        // ==================================================================
        visualizeSummary(left0_rect, right0_rect, left1_rect, right1_rect,
                        disparity0, depth0, stereo_matches, temporal_matches);
        
        std::cout << "\n✓ Debugging session complete!\n" << std::endl;
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "\n ERROR: " << e.what() << std::endl;
        return -1;
    }
}
