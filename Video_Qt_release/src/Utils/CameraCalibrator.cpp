#include "CameraCalibrator.h"
#include <iostream>

CameraCalibrator::CameraCalibrator() {
    // 构造函数
}

bool CameraCalibrator::calibrate(const std::vector<std::string>& imagePaths,
    const Chessboard& chessboard,
    Camera& camera) {
    std::vector<std::vector<cv::Point2f>> allImagePoints;
    std::vector<std::vector<cv::Point3f>> allObjectPoints;
    cv::Mat rvecs, tvecs; // 存储每次位姿估计结果

    cv::Mat currentImage;
    std::vector<cv::Point2f> currentCorners;

    // 获取棋盘格的3D点
    std::vector<cv::Point3f> objectPoints = chessboard.getObjectPoints();

    // 遍历所有标定图像
    for (const std::string& path : imagePaths) {
        currentImage = cv::imread(path);
        if (currentImage.empty()) {
            std::cerr << "Warning: Could not read image " << path << ". Skipping." << std::endl;
            continue;
        }

        // 查找棋盘格角点
        bool found = imageProcessor_.detectChessboardCorners(currentImage, chessboard, currentCorners);
        if (found) {
            allImagePoints.push_back(currentCorners);
            allObjectPoints.push_back(objectPoints); // 每次都添加相同的3D点模型
            // 可选：在图像上绘制角点并显示
            // cv::Mat displayImage;
            // currentImage.copyTo(displayImage);
            // imageProcessor_.drawPoints(displayImage, currentCorners, cv::Scalar(0, 255, 0));
            // cv::imshow("Corners Found", displayImage);
            // cv::waitKey(100);
        }
        else {
            std::cerr << "Warning: Could not find chessboard corners in " << path << ". Skipping." << std::endl;
        }
    }

    if (allImagePoints.empty()) {
        std::cerr << "Error: No valid images for calibration found." << std::endl;
        return false;
    }

    // 执行相机标定
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    // 初始猜测相机内参，如果没有提供，OpenCV会自动估计
    cameraMatrix = cv::initCameraMatrix2D(allObjectPoints, allImagePoints, camera.getImageSize(), 0);

    double rms = cv::calibrateCamera(allObjectPoints, allImagePoints, camera.getImageSize(),
        cameraMatrix, distCoeffs, rvecs, tvecs, cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5); // 可以调整标定参数

    std::cout << "Calibration RMS error: " << rms << std::endl;
    std::cout << "Calibrated Camera Matrix:\n" << cameraMatrix << std::endl;
    std::cout << "Calibrated Distortion Coefficients:\n" << distCoeffs << std::endl;

    // 将结果设置到Camera对象中
    camera.setParams(cameraMatrix, distCoeffs, camera.getImageSize());
    
    // --- 添加参数保存逻辑 ---
   // 确保目录存在
    std::string configPath = ".\\data\\config\\";

    // 构建完整的文件名
    std::string filename = configPath + camera.getName() + ".yml";

    // 调用Camera类的saveParams方法保存参数
    if (camera.saveParams(filename)) {
        std::cout << "Camera parameters successfully saved to: " << filename << std::endl;
    }
    else {
        std::cerr << "Error: Failed to save camera parameters to: " << filename << std::endl;
        return false; // 保存失败也返回false
    }

    return true;
}