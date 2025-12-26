#include "CalibrationManager.h"
#include <iostream>
#include <fstream>   // 用于文件输入流 (ifstream)
#include <sstream>   // 用于字符串流 (stringstream)
#include <algorithm> 
#include <vector>
#include <QDebug>
#include "string"

// 修改构造函数实现
CalibrationManager::CalibrationManager(Camera& camA, Camera& camB, Chessboard& chessboardA, Chessboard& chessboardB)
    : cameraA_(camA), cameraB_(camB), chessboardA_(chessboardA), chessboardB_(chessboardB) {
    T_CamA_Board_ = cv::Mat::eye(4, 4, CV_64F);
}

bool CalibrationManager::loadCalibrationResults() {
    bool success = true;
    if (!cameraA_.loadParams(ConfigPaths::CAMERA_A_PARAMS)) {
        success = false;
        std::cerr << "Failed to load Camera A parameters." << std::endl;
    }
    if (!cameraB_.loadParams(ConfigPaths::CAMERA_B_PARAMS)) {
        success = false;
        std::cerr << "Failed to load Camera B parameters." << std::endl;
    }

    cv::FileStorage fs(ConfigPaths::HAND_EYE_TRANSFORM, cv::FileStorage::READ);
    if (fs.isOpened()) {
        fs["T_CamA_Board"] >> T_CamA_Board_;
        fs.release();
        std::cout << "Hand-eye transform loaded from " << ConfigPaths::HAND_EYE_TRANSFORM << std::endl;
    }
    else {
        std::cerr << "Warning: Could not open hand-eye transform file " << ConfigPaths::HAND_EYE_TRANSFORM << ". Using default identity matrix." << std::endl;
        // 如果文件不存在，可以认为加载失败，但为了不中断，可以设置success为false但不返回
        success = false;
    }
    return success;
}

bool CalibrationManager::saveCalibrationResults() {
    bool success = true;
    if (!cameraA_.saveParams(ConfigPaths::CAMERA_A_PARAMS)) {
        success = false;
        std::cerr << "Failed to save Camera A parameters." << std::endl;
    }
    if (!cameraB_.saveParams(ConfigPaths::CAMERA_B_PARAMS)) {
        success = false;
        std::cerr << "Failed to save Camera B parameters." << std::endl;
    }

    cv::FileStorage fs(ConfigPaths::HAND_EYE_TRANSFORM, cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "T_CamA_Board" << T_CamA_Board_;
        fs.release();
        std::cout << "Hand-eye transform saved to " << ConfigPaths::HAND_EYE_TRANSFORM << std::endl;
    }
    else {
        std::cerr << "Error: Could not create hand-eye transform file " << ConfigPaths::HAND_EYE_TRANSFORM << std::endl;
        success = false;
    }
    return success;
}

bool CalibrationManager::calibrateCameraA(const std::vector<std::string>& imagePaths) {
    std::cout << "Starting Camera A internal parameter calibration using its dedicated chessboard..." << std::endl;
    // 使用 chessboardA_ 进行相机A的标定
    bool success = camCalibrator_.calibrate(imagePaths, chessboardA_, cameraA_);
    if (success) {
        std::cout << "Camera A internal parameter calibration complete." << std::endl;
    }
    else {
        std::cerr << "Camera A internal parameter calibration failed." << std::endl;
    }
    return success;
}

bool CalibrationManager::calibrateCameraB(const std::vector<std::string>& imagePaths) {
    std::cout << "Starting Camera B internal parameter calibration using its dedicated chessboard..." << std::endl;
    // 使用 chessboardB_ 进行相机B的标定
    bool success = camCalibrator_.calibrate(imagePaths, chessboardB_, cameraB_); // <--- 这里修改为 chessboardB_
    if (success) {
        std::cout << "Camera B internal parameter calibration complete." << std::endl;
    }
    else {
        std::cerr << "Camera B internal parameter calibration failed." << std::endl;
    }
    return success;
}




// 辅助函数定义
// 将姿态向量（rvec, tvec）转换为4x4齐次矩阵
// poseVec: 包含 rvec 和 tvec 的行向量或两个单独的Mat
// 如果 poseVec 是 rvec 和 tvec 的拼接，需要知道如何解析
// 假设 poseVec 是 [rvec_x, rvec_y, rvec_z, tvec_x, tvec_y, tvec_z] 这样的单行Mat
cv::Mat attitudeVectorToMatrix(const cv::Mat& poseVec, bool /*is_camera_pose*/, const std::string& /*debug_name*/) {
    if (poseVec.empty() || poseVec.cols != 6) { // 假设是 rvec (3) + tvec (3)
        std::cerr << "Error: attitudeVectorToMatrix input poseVec has invalid size." << std::endl;
        return cv::Mat();
    }

    cv::Mat rvec = poseVec.colRange(0, 3).t(); // 提取 rvec, 转置为列向量
    cv::Mat tvec = poseVec.colRange(3, 6).t(); // 提取 tvec, 转置为列向量

    cv::Mat R;
    cv::Rodrigues(rvec, R); // rvec 到旋转矩阵

    cv::Mat H = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(H(cv::Rect(0, 0, 3, 3)));
    tvec.copyTo(H(cv::Rect(3, 0, 1, 3)));

    return H;
}

// 将4x4齐次矩阵分解为旋转矩阵和位移向量
void HomogeneousMtr2RT(const cv::Mat& H, cv::Mat& R, cv::Mat& t) {
    H(cv::Rect(0, 0, 3, 3)).copyTo(R);
    H(cv::Rect(3, 0, 1, 3)).copyTo(t);
}

// 将旋转矩阵和位移向量组合为4x4齐次矩阵
cv::Mat R_T2HomogeneousMatrix(const cv::Mat& R, const cv::Mat& t) {
    cv::Mat H = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(H(cv::Rect(0, 0, 3, 3)));
    t.copyTo(H(cv::Rect(3, 0, 1, 3)));
    return H;
}


// 核心手眼标定函数
//bool CalibrationManager::handEyeCalibration(const std::vector<std::string>& imagePaths_CamA,
//    const std::vector<std::string>& imagePaths_CamB) {
//    std::cout << "\n--- Starting Hand-Eye Calibration (AX=ZB) ---" << std::endl;
//
//    // 检查相机内参是否已加载
//    if (cameraA_.getCameraMatrix().empty() || cameraA_.getDistCoeffs().empty() ||
//        cameraB_.getCameraMatrix().empty() || cameraB_.getDistCoeffs().empty()) {
//        std::cerr << "Error: Camera internal parameters must be loaded before hand-eye calibration." << std::endl;
//        return false;
//    }
//
//    if (imagePaths_CamA.empty() || imagePaths_CamB.empty()) {
//        std::cerr << "Error: Hand-eye calibration requires images, input paths cannot be empty." << std::endl;
//        return false;
//    }
//    if (imagePaths_CamA.size() != imagePaths_CamB.size()) {
//        std::cerr << "Error: Number of images for Camera A and Camera B must match." << std::endl;
//        return false;
//    }
//
//    std::cout << "\n--- Calculating all poses using camera intrinsics... ---" << std::endl;
//    std::vector<cv::Mat> collected_poses_A_World, collected_poses_B_Plate; // 存储 (rvec, tvec) 向量
//
//    // 获取相机A和相机B的内参和畸变系数
//    cv::Mat camMatrixA = cameraA_.getCameraMatrix();
//    cv::Mat distCoeffsA = cameraA_.getDistCoeffs();
//    cv::Mat camMatrixB = cameraB_.getCameraMatrix();
//    cv::Mat distCoeffsB = cameraB_.getDistCoeffs();
//
//    // 确保 PoseEstimator 能够计算 pose (rvec, tvec)
//    // 假设 PoseEstimator 有一个方法 `estimatePose(image, cameraMatrix, distCoeffs, chessboard, rvec, tvec)`
//    // 或者你可以直接在这里实现 PnP 逻辑
//    for (size_t i = 0; i < imagePaths_CamA.size(); ++i) {
//        cv::Mat rvecA, tvecA;
//        // 注意：这里 `estimatePose` 应该返回的是 `T_Camera_World`，即世界坐标系到相机坐标系的变换
//        bool foundA = poseEstimator_.estimatePose(imagePaths_CamA[i], cameraA_, chessboardA_, rvecA, tvecA);
//
//        cv::Mat rvecB, tvecB;
//        // 注意：这里 `estimatePose` 应该返回的是 `T_Camera_Plate`，即固定板坐标系到相机B坐标系的变换
//        bool foundB = poseEstimator_.estimatePose(imagePaths_CamB[i], cameraB_, chessboardB_, rvecB, tvecB);
//
//        if (foundA && foundB) {
//            // 将 rvec 和 tvec 拼接成一个行向量，例如 [rvec_x, rvec_y, rvec_z, tvec_x, tvec_y, tvec_z]
//            cv::Mat poseVecA;
//            vconcat(rvecA.reshape(1, 1), tvecA.reshape(1, 1), poseVecA);
//            collected_poses_A_World.push_back(poseVecA);
//
//            cv::Mat poseVecB;
//            vconcat(rvecB.reshape(1, 1), tvecB.reshape(1, 1), poseVecB);
//            collected_poses_B_Plate.push_back(poseVecB);
//        }
//        else {
//            std::cerr << "Warning: Could not find chessboard in one or both images for pair " << i << ". Skipping." << std::endl;
//        }
//    }
//
//    if (collected_poses_A_World.size() < 3) {
//        std::cerr << "Error: Not enough valid pose pairs found for hand-eye calibration. At least 3 pairs are required." << std::endl;
//        return false;
//    }
//
//    // 将 vector<Mat> 转换为一个大的 Mat (行是每个姿态)
//    cv::Mat poses_A_sees_World_Mat, poses_B_sees_Plate_Mat;
//    cv::vconcat(collected_poses_A_World, poses_A_sees_World_Mat);
//    cv::vconcat(collected_poses_B_Plate, poses_B_sees_Plate_Mat);
//
//    std::cout << "\n--- Executing Hand-Eye Calibration (AX=ZB)... ---" << std::endl;
//    std::vector<cv::Mat> R_A_moves, t_A_moves; // 相机A的运动 (T_A1_A2)
//    std::vector<cv::Mat> R_P_moves, t_P_moves; // 标定板P的运动 (T_P1_P2)
//
//    for (int i = 0; i < poses_A_sees_World_Mat.rows - 1; ++i) {
//        // 1. 获取四组绝对位姿的齐次矩阵
//       // T_A_W: 从世界W到相机A的变换 (camA_World_poses[i])
//       // T_B_P: 从板P到相机B的变换 (camB_Plate_poses[i])
//
//       // T_A_W1 和 T_A_W2 是从世界坐标系到相机A坐标系的变换
//       // 假设 `attitudeVectorToMatrix` 返回 `T_World_Camera` (`T_W_A`):
//        cv::Mat T_W_A1 = attitudeVectorToMatrix(poses_A_sees_World_Mat.row(i), false, "CamA_W1");
//        cv::Mat T_W_A2 = attitudeVectorToMatrix(poses_A_sees_World_Mat.row(i + 1), false, "CamA_W2");
//        cv::Mat A_move = T_W_A1.inv() * T_W_A2; // T_A1_A2 (从相机A的第一个姿态到第二个姿态的相对变换)
//
//        // B_move: 标定板P的运动，相对于它自己的前一帧 (T_P1_P2)
//        // 这是 "target to camera" 的运动，即被相机B观测的板P的运动
//        // 假设 `attitudeVectorToMatrix` 返回 `T_Camera_Board` (`T_B_P`):
//        cv::Mat T_B_P1 = attitudeVectorToMatrix(poses_B_sees_Plate_Mat.row(i), true, "CamB_P1");
//        cv::Mat T_B_P2 = attitudeVectorToMatrix(poses_B_sees_Plate_Mat.row(i + 1), true, "CamB_P2");
//        cv::Mat B_move = T_B_P1.inv() * T_B_P2; // T_P1_P2 (从板P的第一个姿态到第二个姿态的相对变换)
//
//        // 3. 分解并存储
//        cv::Mat R_A, t_A, R_B, t_B;
//        HomogeneousMtr2RT(A_move, R_A, t_A);
//        HomogeneousMtr2RT(B_move, R_B, t_B);
//
//        R_A_moves.push_back(R_A);
//        t_A_moves.push_back(t_A);
//
//        R_P_moves.push_back(R_B); // 注意这里是 B_move 的 R
//        t_P_moves.push_back(t_B); // 注意这里是 B_move 的 t
//    }
//    cv::Mat R_X, t_X; // X 是 T_A_P (从相机A到板P的变换)
//    cv::calibrateHandEye(R_A_moves, t_A_moves, R_P_moves, t_P_moves, R_X, t_X, cv::CALIB_HAND_EYE_ANDREFF);
//
//    T_CamA_Board_ = R_T2HomogeneousMatrix(R_X, t_X); // 将结果存储到成员变量中
//
//    std::cout << "\n\n=================================================" << std::endl;
//    std::cout << "            *** Hand-Eye Calibration Completed ***" << std::endl;
//    std::cout << "=================================================" << std::endl;
//    std::cout << "\n[Final Result] Hand-Eye Calibration Matrix (T_A_P):\n" << T_CamA_Board_ << std::endl;
//    std::cout << "=================================================" << std::endl;
//
//
//    return true;
//}


bool CalibrationManager::saveMatToTxt(const cv::Mat& matrix, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file for writing: " << filename << std::endl;
        return false;
    }

    file << std::fixed << std::setprecision(15);

    for (int r = 0; r < matrix.rows; ++r) {
        for (int c = 0; c < matrix.cols; ++c) {
            if (matrix.type() == CV_64F) {
                file << matrix.at<double>(r, c);
            }
            else if (matrix.type() == CV_32F) {
                file << matrix.at<float>(r, c);
            }

            if (c < matrix.cols - 1) {
                file << " ";
            }
        }
        file << std::endl;
    }

    file.close();
    std::cout << "Successfully saved matrix to " << filename << std::endl;
    return true;
}

cv::Mat CalibrationManager::loadMatFromTxt(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file for reading: " << filename << std::endl;
        return cv::Mat();
    }

    std::vector<double> values;
    std::string line;
    int rows = 0;

    while (std::getline(file, line)) {
        rows++;
        std::stringstream ss(line);
        double value;
        while (ss >> value) {
            values.push_back(value);
        }
    }
    file.close();

    if (values.empty()) {
        std::cerr << "Error: No data found in file: " << filename << std::endl;
        return cv::Mat();
    }

    int cols = values.size() / rows;
    cv::Mat loaded_matrix(rows, cols, CV_64F);
    memcpy(loaded_matrix.data, values.data(), values.size() * sizeof(double));

    std::cout << "Successfully loaded matrix from " << filename << std::endl;

    return loaded_matrix;
}


std::vector<cv::Point3f> createObjectPoints(const cv::Size& boardSize, float squareSize) {
    std::vector<cv::Point3f> objectPoints;
    for (int i = 0; i < boardSize.height; ++i) {
        for (int j = 0; j < boardSize.width; ++j) {
            objectPoints.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
        }
    }
    return objectPoints;
}


//cv::Mat calculatePoseFromImage(const std::string& imagePath, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
//    const cv::Size& boardSize, float squareSize)
//{
//    // 使用 cv::imread 读取图像
//    cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
//    if (image.empty()) {
//        std::cerr << "错误: 无法加载图像: " << imagePath << std::endl;
//        return cv::Mat(); // 返回空的 cv::Mat
//    }
//
//
//    std::vector<cv::Point2f> corners;
//    // 使用 cv::findChessboardCorners 寻找棋盘格角点
//    bool found = cv::findChessboardCorners(image, boardSize, corners,
//        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK + cv::CALIB_CB_NORMALIZE_IMAGE);
//    
//    //cv::Mat image_with_corners = image_undistort.clone(); // 克隆一份图像，避免修改原始去畸变图像
//    //cv::drawChessboardCorners(image_with_corners, boardSize, cv::Mat(corners), found);
//
//    //const std::string window_name = "Detected Chessboard Corners"; // 定义窗口名称
//    //cv::namedWindow(window_name, cv::WINDOW_NORMAL);
//    //cv::resizeWindow(window_name, 1280, 960);
//
//    //cv::imshow(window_name, image_with_corners);
//    //cv::waitKey(0); // 等待500毫秒，以便观察，然后自动关闭窗口
//
//    if (found) {
//        cv::Mat gray;
//        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY); // 转换为灰度图
//
//        // 使用 cv::cornerSubPix 提高角点精度
//        cv::cornerSubPix(gray, corners, cv::Size(21, 21), cv::Size(-1, -1),
//            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 50, 0.001));
//
//        std::vector<cv::Point3f> objectPoints = createObjectPoints(boardSize, squareSize);
//        cv::Mat rvec, tvec;
//
//        //bool success = cv::solvePnPRansac(objectPoints, corners, cameraMatrix, distCoeffs, rvec, tvec,
//        //    false, 100, 8.0, 0.99, cv::noArray(), cv::SOLVEPNP_ITERATIVE);
//        // 可以调整迭代次数、重投影误差阈值、置信度等参数
//
//        bool success = cv::solvePnPRansac(objectPoints, corners, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_EPNP);
//
//        cv::Mat r_mat;
//		cv::Rodrigues(rvec, r_mat); // 将旋转向量转换为旋转矩阵
//
//        if (!success) {
//            std::cerr << "警告: solvePnPRansac 未能成功计算位姿: " << imagePath << std::endl;
//            return cv::Mat();
//        }
//
//        cv::Mat pose(1, 6, CV_64F); // 创建一个1x6的 Mat，用于存储平移和旋转向量
//        // 将 tvec 和 rvec 的数据拷贝到 pose 中
//        tvec.reshape(1, 1).copyTo(pose(cv::Rect(0, 0, 3, 1)));
//        rvec.reshape(1, 1).copyTo(pose(cv::Rect(3, 0, 3, 1)));
//
//        //std::cout << "成功计算位姿: " << imagePath << std::endl;
//        return pose;
//    }
//    else {
//        std::cerr << "警告: 未在图像中找到棋盘格: " << imagePath << std::endl;
//        return cv::Mat(); // 返回空的 cv::Mat
//    }
//}


//bool CalibrationManager::handEyeCalibration(const std::vector<std::string>& imageFiles_A,
//    const std::vector<std::string>& imageFiles_B) {
//    std::cout << "\n--- Starting Hand-Eye Calibration (AX=ZB) ---" << std::endl;
//
//    // 检查相机内参是否已加载
//    if (cameraA_.getCameraMatrix().empty() || cameraA_.getDistCoeffs().empty() ||
//        cameraB_.getCameraMatrix().empty() || cameraB_.getDistCoeffs().empty()) {
//        std::cerr << "Error: Camera internal parameters must be loaded before hand-eye calibration." << std::endl;
//        return false;
//    }
//
//    if (imageFiles_A.empty() || imageFiles_B.empty()) {
//        std::cerr << "Error: Hand-eye calibration requires images, input paths cannot be empty." << std::endl;
//        return false;
//    }
//    if (imageFiles_A.size() != imageFiles_B.size()) {
//        std::cerr << "Error: Number of images for Camera A and Camera B must match." << std::endl;
//        return false;
//    }
//
//    std::cout << "\n--- Calculating all poses using camera intrinsics... ---" << std::endl;
//    std::vector<cv::Mat> collected_poses_A_World, collected_poses_B_Plate; // 存储 (rvec, tvec) 向量
//
//    // 获取相机A和相机B的内参和畸变系数
//    cv::Mat cameraMatrix_A = cameraA_.getCameraMatrix();
//    cv::Mat distCoeffs_A = cameraA_.getDistCoeffs();
//    cv::Mat cameraMatrix_B = cameraB_.getCameraMatrix();
//    cv::Mat distCoeffs_B = cameraB_.getDistCoeffs();
//
//
//    std::cout << "\n--- 正在使用相机内参计算所有位姿... ---" << std::endl;
//
//    // collected_poses_A 和 collected_poses_B 存储的是OpenCV的Mat类型
//    std::vector<cv::Mat> collected_poses_A, collected_poses_B;
//
//    const cv::Size chessboardSize_World(11, 8);
//    const float squareSize_World = 15.0f;
//    const cv::Size chessboardSize_Plate(11, 8);
//    const float squareSize_Plate = 6.0f;
//
//    for (size_t i = 0; i < imageFiles_A.size(); ++i) {
//        cv::Mat poseA = calculatePoseFromImage(imageFiles_A[i], cameraMatrix_A, distCoeffs_A, chessboardSize_World, squareSize_World);
//        cv::Mat poseB = calculatePoseFromImage(imageFiles_B[i], cameraMatrix_B, distCoeffs_B, chessboardSize_Plate, squareSize_Plate);
//
//        if (!poseA.empty() && !poseB.empty()) {
//            collected_poses_A.push_back(poseA);
//            collected_poses_B.push_back(poseB);
//        }
//    }
//
//    if (collected_poses_A.size() < 3) {
//        std::cerr << "错误: 没有找到足够数量的有效位姿对用于手眼标定。至少需要3对。" << std::endl;
//        return -1;
//    }
//
//    cv::Mat poses_A_sees_World, poses_B_sees_Plate;
//    cv::vconcat(collected_poses_A, poses_A_sees_World);
//    cv::vconcat(collected_poses_B, poses_B_sees_Plate);
//
//    std::cout << "\n--- 正在执行手眼标定 (AX=ZB)... ---" << std::endl;
//
//    std::vector<cv::Mat> R_A_moves, t_A_moves;
//    std::vector<cv::Mat> R_P_moves, t_P_moves;
//
//    // 在 for 循环内部
//    for (int i = 0; i < poses_A_sees_World.rows - 1; ++i)
//    {
//        // 1. 获取四组绝对位姿的齐次矩阵
//        // 注意：T_A_W 代表从世界W到相机A的变换
//        // 假设 attitudeVectorToMatrix 返回 cv::Mat
//        cv::Mat T_A_W1 = attitudeVectorToMatrix(poses_B_sees_Plate.row(i), false, "");
//        cv::Mat T_A_W2 = attitudeVectorToMatrix(poses_B_sees_Plate.row(i + 1), false, "");
//
//        cv::Mat T_B_P1 = attitudeVectorToMatrix(poses_A_sees_World.row(i), false, "");
//        cv::Mat T_B_P2 = attitudeVectorToMatrix(poses_A_sees_World.row(i + 1), false, "");
//
//        // 2. 计算相对运动
//        // A_move (gripper2base): 相机A的运动，相对于它自己的前一帧
//        // 公式: T_A1_A2 = T_A1_W * T_W_A2 = T_A_W1 * T_A_W2.inv()
//        cv::Mat A_move = T_A_W2.inv() * T_A_W1; // .inv() 是 cv::Mat 的成员函数
//
//        // B_move (target2cam): 标定板P的运动，相对于它自己的前一帧
//        // 公式: T_P1_P2 = T_P1_B * T_B_P2 = T_B_P1.inv() * T_B_P2
//        cv::Mat B_move = T_B_P2 * T_B_P1.inv();
//
//        // 3. 分解并存储
//        cv::Mat R_A, t_A, R_B, t_B;
//        HomogeneousMtr2RT(A_move, R_A, t_A); // 假设 HomogeneousMtr2RT 处理 cv::Mat
//        HomogeneousMtr2RT(B_move, R_B, t_B);
//
//        R_A_moves.push_back(R_A);
//        t_A_moves.push_back(t_A);
//
//        // 注意：这里用 B_move 的 R 和 t
//        R_P_moves.push_back(R_B);
//        t_P_moves.push_back(t_B);
//    }
//
//    cv::Mat R_A_P, t_A_P;
//    //calibrateHandEye(R_A_moves, t_A_moves, R_P_moves, t_P_moves, R_A_P, t_A_P, cv::CALIB_HAND_EYE_DANIILIDIS);
//    //cv::Mat T_A_P = R_T2HomogeneousMatrix(R_A_P, t_A_P);
//    /* 手眼标定的方法
//    CALIB_HAND_EYE_TSAI         = 0, //!< A New Technique for Fully Autonomous and Efficient 3D Robotics Hand/Eye Calibration @cite Tsai89
//    CALIB_HAND_EYE_PARK         = 1, //!< Robot Sensor Calibration: Solving AX = XB on the Euclidean Group @cite Park94
//    CALIB_HAND_EYE_HORAUD       = 2, //!< Hand-eye Calibration @cite Horaud95
//    CALIB_HAND_EYE_ANDREFF      = 3, //!< On-line Hand-Eye Calibration @cite Andreff99
//    CALIB_HAND_EYE_DANIILIDIS   = 4  //!< Hand-Eye Calibration Using Dual Quaternions @cite Daniilidis98
//    */
//
//
//
//    //std::cout << "\n\n=================================================" << std::endl;
//    //std::cout << "            *** 手眼标定已完成 ***" << std::endl;
//    //std::cout << "=================================================" << std::endl;
//    //std::cout << "\n[最终结果] 手眼标定矩阵:\n" << T_A_P << std::endl;
//    //std::cout << "=================================================" << std::endl;
//
//    calibrateHandEye(R_A_moves, t_A_moves, R_P_moves, t_P_moves, R_A_P, t_A_P, cv::CALIB_HAND_EYE_TSAI);
//    cv::Mat T_A_P = R_T2HomogeneousMatrix(R_A_P, t_A_P);
//    std::cout << "CALIB_HAND_EYE_TSAI" << std::endl;
//    std::cout << T_A_P << std::endl;
//
//    calibrateHandEye(R_A_moves, t_A_moves, R_P_moves, t_P_moves, R_A_P, t_A_P, cv::CALIB_HAND_EYE_PARK);
//    T_A_P = R_T2HomogeneousMatrix(R_A_P, t_A_P);
//    std::cout << "CALIB_HAND_EYE_PARK" << std::endl;
//    std::cout << T_A_P << std::endl;
//
//    calibrateHandEye(R_A_moves, t_A_moves, R_P_moves, t_P_moves, R_A_P, t_A_P, cv::CALIB_HAND_EYE_HORAUD);
//    T_A_P = R_T2HomogeneousMatrix(R_A_P, t_A_P);
//    std::cout << "CALIB_HAND_EYE_HORAUD" << std::endl;
//    std::cout << T_A_P << std::endl;
//
//    calibrateHandEye(R_A_moves, t_A_moves, R_P_moves, t_P_moves, R_A_P, t_A_P, cv::CALIB_HAND_EYE_ANDREFF);
//    T_A_P = R_T2HomogeneousMatrix(R_A_P, t_A_P);
//    std::cout << "CALIB_HAND_EYE_ANDREFF" << std::endl;
//    std::cout << T_A_P << std::endl;
//
//    calibrateHandEye(R_A_moves, t_A_moves, R_P_moves, t_P_moves, R_A_P, t_A_P, cv::CALIB_HAND_EYE_DANIILIDIS);
//    T_A_P = R_T2HomogeneousMatrix(R_A_P, t_A_P);
//    std::cout << "CALIB_HAND_EYE_DANIILIDIS" << std::endl;
//    std::cout << T_A_P << std::endl;
//
//
//    T_CamA_Board_ = T_A_P;
//
//    return true;
//}



//bool CalibrationManager::handEyeCalibration(const std::vector<std::string>& imageFiles_A,
//    const std::vector<std::string>& imageFiles_B) {
//    std::cout << "\n--- Starting Hand-Eye Calibration (AX=ZB) ---" << std::endl;
//
//    // 检查相机内参是否已加载
//    if (cameraA_.getCameraMatrix().empty() || cameraA_.getDistCoeffs().empty() ||
//        cameraB_.getCameraMatrix().empty() || cameraB_.getDistCoeffs().empty()) {
//        std::cerr << "Error: Camera internal parameters must be loaded before hand-eye calibration." << std::endl;
//        return false;
//    }
//
//    if (imageFiles_A.empty() || imageFiles_B.empty()) {
//        std::cerr << "Error: Hand-eye calibration requires images, input paths cannot be empty." << std::endl;
//        return false;
//    }
//    if (imageFiles_A.size() != imageFiles_B.size()) {
//        std::cerr << "Error: Number of images for Camera A and Camera B must match." << std::endl;
//        return false;
//    }
//
//    std::cout << "\n--- Calculating all poses using camera intrinsics... ---" << std::endl;
//    std::vector<cv::Mat> collected_poses_A_World, collected_poses_B_Plate; // 存储 (rvec, tvec) 向量
//
//    // 获取相机A和相机B的内参和畸变系数
//    cv::Mat cameraMatrix_A = cameraA_.getCameraMatrix();
//    cv::Mat distCoeffs_A = cameraA_.getDistCoeffs();
//    cv::Mat cameraMatrix_B = cameraB_.getCameraMatrix();
//    cv::Mat distCoeffs_B = cameraB_.getDistCoeffs();
//
//
//    std::cout << "\n--- 正在使用相机内参计算所有位姿... ---" << std::endl;
//
//    // collected_poses_A 和 collected_poses_B 存储的是OpenCV的Mat类型
//    std::vector<cv::Mat> collected_poses_A, collected_poses_B;
//
//    const cv::Size chessboardSize_World(11, 8);
//    const float squareSize_World = 15.0f;
//    const cv::Size chessboardSize_Plate(11, 8);
//    const float squareSize_Plate = 6.0f;
//
//    for (size_t i = 0; i < imageFiles_A.size(); ++i) {
//        cv::Mat poseA = calculatePoseFromImage(imageFiles_A[i], cameraMatrix_A, distCoeffs_A, chessboardSize_World, squareSize_World);
//        cv::Mat poseB = calculatePoseFromImage(imageFiles_B[i], cameraMatrix_B, distCoeffs_B, chessboardSize_Plate, squareSize_Plate);
//
//        if (!poseA.empty() && !poseB.empty()) {
//            collected_poses_A.push_back(poseA);
//            collected_poses_B.push_back(poseB);
//        }
//    }
//
//    if (collected_poses_A.size() < 3) {
//        std::cerr << "错误: 没有找到足够数量的有效位姿对用于手眼标定。至少需要3对。" << std::endl;
//        return -1;
//    }
//
//    cv::Mat poses_A_sees_World, poses_B_sees_Plate;
//    cv::vconcat(collected_poses_A, poses_A_sees_World);
//    cv::vconcat(collected_poses_B, poses_B_sees_Plate);
//
//    std::cout << "\n--- 正在执行手眼标定 (AX=ZB)... ---" << std::endl;
//
//    std::vector<cv::Mat> R_A_moves, t_A_moves;
//    std::vector<cv::Mat> R_P_moves, t_P_moves;
//
//    // 在 for 循环内部
//    for (int i = 0; i < poses_A_sees_World.rows - 1; ++i)
//    {
//        // 获取绝对位姿的齐次矩阵
//        // poses_A_sees_World 存储 T_CamA_World
//        // poses_B_sees_Plate 存储 T_CamB_Plate
//
//        cv::Mat T_CamA_W_i = attitudeVectorToMatrix(poses_A_sees_World.row(i), false, "");
//        cv::Mat T_CamA_W_i_1 = attitudeVectorToMatrix(poses_A_sees_World.row(i + 1), false, "");
//
//        cv::Mat T_CamB_P_i = attitudeVectorToMatrix(poses_B_sees_Plate.row(i), false, "");
//        cv::Mat T_CamB_P_i_1 = attitudeVectorToMatrix(poses_B_sees_Plate.row(i + 1), false, "");
//
//        // A_motion: 相机A相对于世界坐标系的运动 (T_W_CA_i.inv() * T_W_CA_i+1)
//        // A_motion = (T_CamA_W_i).inv() * (T_CamA_W_i_1); // WRONG, should be T_W_CA_i+1 * T_CA_W_i
//        // The rotation is R_CA_W from World to Camera A. We want R_W_CA
//        // R_W_CA = R_CA_W.t()
//        // T_W_CA_i = T_CamA_W_i.inv()
//        // A_motion = T_W_CA_i+1 * T_CA_W_i = T_CamA_W_i_1.inv() * T_CamA_W_i
//
//        cv::Mat A_motion = T_CamA_W_i_1.inv() * T_CamA_W_i; // This represents (T_W_CA_i+1) * (T_CA_W_i)
//
//        // B_motion: 标定板P相对于相机B的运动 (T_CB_P_i.inv() * T_CB_P_i+1)
//        cv::Mat B_motion = T_CamB_P_i_1 * T_CamB_P_i.inv(); // This represents (T_CB_P_i+1) * (T_P_CB_i)
//
//        // 分解并存储
//        cv::Mat R_A, t_A, R_B, t_B;
//        HomogeneousMtr2RT(A_motion, R_A, t_A);
//        HomogeneousMtr2RT(B_motion, R_B, t_B);
//
//        R_A_moves.push_back(R_A); // 用于calibrateHandEye的第一个运动序列A
//        t_A_moves.push_back(t_A);
//
//        R_P_moves.push_back(R_B); // 用于calibrateHandEye的第二个运动序列B
//        t_P_moves.push_back(t_B);
//    }
//
//    cv::Mat R_CA_P, t_CA_P; // 对应 AX=ZB 中的 X，即 T_CameraA_Plate
//    cv::Mat R_W_CB, t_W_CB; // 对应 AX=ZB 中的 Z，即 T_World_CameraB
//
//    cv::calibrateHandEye(R_A_moves, t_A_moves, R_P_moves, t_P_moves, R_CA_P, t_CA_P, cv::CALIB_HAND_EYE_TSAI);
//
//    // T_CamA_Plate 就是您想要的结果
//    cv::Mat T_CamA_Plate = R_T2HomogeneousMatrix(R_CA_P, t_CA_P);
//    std::cout << "T_CamA_Plate (相机A到标定板Plate的变换):\n" << T_CamA_Plate << std::endl;
//
//    T_CamA_Board_ = T_CamA_Plate; // 保存最终结果
//
//
//    /* 手眼标定的方法
//    CALIB_HAND_EYE_TSAI         = 0, //!< A New Technique for Fully Autonomous and Efficient 3D Robotics Hand/Eye Calibration @cite Tsai89
//    CALIB_HAND_EYE_PARK         = 1, //!< Robot Sensor Calibration: Solving AX = XB on the Euclidean Group @cite Park94
//    CALIB_HAND_EYE_HORAUD       = 2, //!< Hand-eye Calibration @cite Horaud95
//    CALIB_HAND_EYE_ANDREFF      = 3, //!< On-line Hand-Eye Calibration @cite Andreff99
//    CALIB_HAND_EYE_DANIILIDIS   = 4  //!< Hand-Eye Calibration Using Dual Quaternions @cite Daniilidis98
//    */
//
//
//
//    //std::cout << "\n\n=================================================" << std::endl;
//    //std::cout << "            *** 手眼标定已完成 ***" << std::endl;
//    //std::cout << "=================================================" << std::endl;
//    //std::cout << "\n[最终结果] 手眼标定矩阵:\n" << T_A_P << std::endl;
//    //std::cout << "=================================================" << std::endl;
//
//    calibrateHandEye(R_A_moves, t_A_moves, R_P_moves, t_P_moves, R_CA_P, t_CA_P, cv::CALIB_HAND_EYE_TSAI);
//    cv::Mat T_A_P = R_T2HomogeneousMatrix(R_CA_P, t_CA_P);
//    std::cout << "CALIB_HAND_EYE_TSAI" << std::endl;
//    std::cout << T_A_P << std::endl;
//
//    calibrateHandEye(R_A_moves, t_A_moves, R_P_moves, t_P_moves, R_CA_P, t_CA_P, cv::CALIB_HAND_EYE_PARK);
//    T_A_P = R_T2HomogeneousMatrix(R_CA_P, t_CA_P);
//    std::cout << "CALIB_HAND_EYE_PARK" << std::endl;
//    std::cout << T_A_P << std::endl;
//
//    calibrateHandEye(R_A_moves, t_A_moves, R_P_moves, t_P_moves, R_CA_P, t_CA_P, cv::CALIB_HAND_EYE_HORAUD);
//    T_A_P = R_T2HomogeneousMatrix(R_CA_P, t_CA_P);
//    std::cout << "CALIB_HAND_EYE_HORAUD" << std::endl;
//    std::cout << T_A_P << std::endl;
//
//    calibrateHandEye(R_A_moves, t_A_moves, R_P_moves, t_P_moves, R_CA_P, t_CA_P, cv::CALIB_HAND_EYE_ANDREFF);
//    T_A_P = R_T2HomogeneousMatrix(R_CA_P, t_CA_P);
//    std::cout << "CALIB_HAND_EYE_ANDREFF" << std::endl;
//    std::cout << T_A_P << std::endl;
//
//    calibrateHandEye(R_A_moves, t_A_moves, R_P_moves, t_P_moves, R_CA_P, t_CA_P, cv::CALIB_HAND_EYE_DANIILIDIS);
//    T_A_P = R_T2HomogeneousMatrix(R_CA_P, t_CA_P);
//    std::cout << "CALIB_HAND_EYE_DANIILIDIS" << std::endl;
//    std::cout << T_A_P << std::endl;
//
//
//    T_CamA_Board_ = T_A_P;
//
//    return true;
//}



// 功能：输入图像路径，计算标定板相对于相机的位姿 (T_Camera_Board)
// 返回：4x4 齐次变换矩阵
cv::Mat calculatePoseFromImage(const std::string& imagePath,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    const cv::Size& boardSize,
    float squareSize)
{
    cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cerr << "Error: Cannot load image " << imagePath << std::endl;
        return cv::Mat();
    }

    // 1. 寻找角点 (直接在原图上操作)
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(image, boardSize, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK + cv::CALIB_CB_NORMALIZE_IMAGE);

    if (!found) return cv::Mat();

    // 2. 亚像素精确化
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));

    // 3. 生成3D点
    std::vector<cv::Point3f> objectPoints = createObjectPoints(boardSize, squareSize);

    // 4. SolvePnP 计算位姿
    cv::Mat rvec, tvec;
    // 使用 SOLVEPNP_ITERATIVE 精度较好
    bool success = cv::solvePnPRansac(objectPoints, corners, cameraMatrix, distCoeffs, rvec, tvec,
        false, 100, 3.0, 0.99, cv::noArray(), cv::SOLVEPNP_ITERATIVE);

    if (!success) {
        std::cerr << "Warning: SolvePnP failed for " << imagePath << std::endl;
        return cv::Mat();
    }

    // 5. 转换为 4x4 矩阵
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    return R_T2HomogeneousMatrix(R, tvec); // 返回 T_Camera_Board
}


bool CalibrationManager::handEyeCalibration(const std::vector<std::string>& imageFiles_A,
    const std::vector<std::string>& imageFiles_B) {
    std::cout << "\n--- Starting Hand-Eye Calibration (AX=ZB) ---" << std::endl;

    // 0. 基础检查
    if (cameraA_.getCameraMatrix().empty() || cameraB_.getCameraMatrix().empty()) {
        std::cerr << "Error: Camera intrinsics not loaded." << std::endl;
        return false;
    }
    if (imageFiles_A.size() != imageFiles_B.size() || imageFiles_A.empty()) {
        std::cerr << "Error: Image lists mismatch or empty." << std::endl;
        return false;
    }

    // 准备容器存储绝对位姿
    // R_gripper2base: 对应 T_World_CamA (相机A是移动的"手")
    // R_target2cam:   对应 T_CamB_Plate (相机B是固定的"眼"，看的是Plate)
    std::vector<cv::Mat> R_gripper2base, t_gripper2base;
    std::vector<cv::Mat> R_target2cam, t_target2cam;

    // 标定板参数配置
    const cv::Size chessboardSize_World(11, 8);
    const float squareSize_World = 30.0f;
    const cv::Size chessboardSize_Plate(11, 8);
    const float squareSize_Plate = 6.0f;

    std::cout << "Processing " << imageFiles_A.size() << " image pairs..." << std::endl;

    cv::Mat m = cameraA_.getCameraMatrix();

    for (size_t i = 0; i < imageFiles_A.size(); ++i) {
        // 计算 CamA 看到的位姿: T_CamA_World
        cv::Mat T_CamA_World = calculatePoseFromImage(imageFiles_A[i], cameraA_.getCameraMatrix(), cameraA_.getDistCoeffs(), chessboardSize_World, squareSize_World);

        // 计算 CamB 看到的位姿: T_CamB_Plate
        cv::Mat T_CamB_Plate = calculatePoseFromImage(imageFiles_B[i], cameraB_.getCameraMatrix(), cameraB_.getDistCoeffs(), chessboardSize_Plate, squareSize_Plate);

        if (!T_CamA_World.empty() && !T_CamB_Plate.empty()) {

            // --- 关键坐标系转换 ---

            // 1. 处理移动端 (Gripper): 相机A
            // SolvePnP 给出的是 T_CamA_World (点从World转到CamA)
            // 标定算法需要 Gripper相对于Base的位姿，即 T_World_CamA
            // 关系: T_World_CamA = (T_CamA_World)^-1
            cv::Mat T_World_CamA = T_CamA_World.inv();

            cv::Mat R_g, t_g;
            HomogeneousMtr2RT(T_World_CamA, R_g, t_g);
            R_gripper2base.push_back(R_g);
            t_gripper2base.push_back(t_g);

            // 2. 处理固定端 (Eye): 相机B
            // SolvePnP 给出的是 T_CamB_Plate
            // 标定算法需要 Target相对于Cam的位姿，即 T_CamB_Plate
            // 关系: 直接使用，无需求逆
            cv::Mat T_Plate_CamB = T_CamB_Plate.inv();

            cv::Mat R_t, t_t;
            HomogeneousMtr2RT(T_Plate_CamB, R_t, t_t); // 使用求逆后的矩阵
            R_target2cam.push_back(R_t);
            t_target2cam.push_back(t_t);
        }
        else {
            std::cout << "  - Skipped pair " << i << " (chessboard not found)" << std::endl;
        }
    }

    if (R_gripper2base.size() < 3) {
        std::cerr << "Error: Not enough valid poses (" << R_gripper2base.size() << "). Need at least 3." << std::endl;
        return false;
    }

    std::cout << "Valid poses collected: " << R_gripper2base.size() << std::endl;
    std::cout << "\n=== Comparing Different Hand-Eye Calibration Methods ===" << std::endl;

    // 定义所有可用的标定方法及其名称
    std::vector<std::pair<cv::HandEyeCalibrationMethod, std::string>> methods = {
        {cv::CALIB_HAND_EYE_TSAI,       "TSAI (Default)"},
        {cv::CALIB_HAND_EYE_PARK,       "PARK"},
        {cv::CALIB_HAND_EYE_HORAUD,     "HORAUD"},
        {cv::CALIB_HAND_EYE_ANDREFF,    "ANDREFF"},
        {cv::CALIB_HAND_EYE_DANIILIDIS, "DANIILIDIS"}
    };

    cv::Mat R_final, t_final; // 用于存储最终选定的结果
    cv::Mat T_final;

    for (const auto& method : methods) {
        cv::Mat R_est, t_est;

        // 调用标定函数
        cv::calibrateHandEye(R_gripper2base, t_gripper2base,
            R_target2cam, t_target2cam,
            R_est, t_est,
            method.first);

        // 转换为 4x4 矩阵
        cv::Mat T_est = R_T2HomogeneousMatrix(R_est, t_est);


        // 1. 打印方法名
        // .noquote() 用于去掉输出字符串两边的双引号，让显示更干净
        // 如果 method.second 是 std::string，建议转为 QString 或 .c_str()
        qDebug().noquote() << "\n--- Method:" << QString::fromStdString(method.second) << "---";

        // 2. 打印 T_est 矩阵
        // 因为 qDebug 无法直接漂亮地打印 cv::Mat/Eigen，我们需要借用 stringstream
        std::stringstream ss_T;
        ss_T << T_est;
        qDebug().noquote() << QString::fromStdString(ss_T.str());

        // 3. 打印平移向量
        std::stringstream ss_t;
        ss_t << "Translation (x, y, z): " << t_est.t();
        qDebug().noquote() << QString::fromStdString(ss_t.str());


        // 我们默认使用 TSAI 或 DANIILIDIS 作为最终保存的结果
        if (method.first == cv::CALIB_HAND_EYE_DANIILIDIS) {
            T_final = T_est.clone();
        }   

    }

    // 保存到成员变量
    T_CamA_Board_ = T_final;

    return true;
}