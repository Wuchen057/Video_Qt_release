#include "Camera.h"
#include <iostream>
#include <opencv2/imgproc.hpp> // 必须包含，用于 cv::remap
#include <opencv2/calib3d.hpp> // 必须包含，用于 cv::initUndistortRectifyMap

Camera::Camera(CameraType type, const std::string& name, const cv::Size& imageSize)
    : type_(type), name_(name), imageSize_(imageSize) {
    // 默认初始化
    cameraMatrix_ = cv::Mat::eye(3, 3, CV_64F);
    distCoeffs_ = cv::Mat::zeros(5, 1, CV_64F);

    // 如果构造时已经提供了尺寸，尝试初始化映射表
    if (imageSize_.area() > 0) {
        // initUndistortMaps(); // 可以调用，但通常此时内参还是空的，所以意义不大
    }
}

bool Camera::loadParams(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Error: Could not open camera parameters file " << filename << std::endl;
        return false;
    }

    fs["camera_matrix"] >> cameraMatrix_;
    fs["dist_coeffs"] >> distCoeffs_;
    fs["image_width"] >> imageSize_.width;
    fs["image_height"] >> imageSize_.height;

    fs.release();

    // 优化关键点：参数加载完成后，立即预计算去畸变映射表
    initUndistortMaps();

    std::cout << "Camera " << name_ << " parameters loaded from " << filename << std::endl;
    return true;
}

bool Camera::saveParams(const std::string& filename) const {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "Error: Could not create camera parameters file " << filename << std::endl;
        return false;
    }

    fs << "camera_matrix" << cameraMatrix_;
    fs << "dist_coeffs" << distCoeffs_;
    fs << "image_width" << imageSize_.width;
    fs << "image_height" << imageSize_.height;

    fs.release();
    std::cout << "Camera " << name_ << " parameters saved to " << filename << std::endl;
    return true;
}

void Camera::setParams(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, const cv::Size& imageSize) {
    cameraMatrix_ = cameraMatrix.clone();
    distCoeffs_ = distCoeffs.clone();
    imageSize_ = imageSize;

    // 优化关键点：参数变更后，更新映射表
    initUndistortMaps();
}

// ---------------------------------------------------------
// 核心优化函数：initUndistortMaps
// ---------------------------------------------------------
void Camera::initUndistortMaps() {
    if (cameraMatrix_.empty() || distCoeffs_.empty() || imageSize_.area() == 0) {
        return;
    }

    // 计算映射表
    // CV_16SC2 格式通常比 CV_32FC1 更快且省内存，适合定点数运算
    cv::initUndistortRectifyMap(
        cameraMatrix_,
        distCoeffs_,
        cv::Mat(), // 无矫正旋转
        cameraMatrix_, // 新相机矩阵保持一致
        imageSize_,
        CV_16SC2,
        map1_,
        map2_
    );
}

// ---------------------------------------------------------
// 核心优化函数：undistortImage (使用 remap)
// ---------------------------------------------------------
void Camera::undistortImage(const cv::Mat& src, cv::Mat& dst) const {
    // 检查映射表是否已就绪
    if (map1_.empty() || map2_.empty()) {
        // 如果没有映射表（可能是没加载参数），尝试使用旧方法作为回退，或直接复制
        if (!cameraMatrix_.empty() && !distCoeffs_.empty()) {
            cv::undistort(src, dst, cameraMatrix_, distCoeffs_);
        }
        else {
            src.copyTo(dst);
        }
        return;
    }

    // 检查输入尺寸是否与映射表匹配，如果不匹配可能导致崩溃
    if (src.size() != imageSize_) {
        // 尺寸不匹配，无法使用预计算的表，回退到慢速方法
        // std::cerr << "Warning: Input image size does not match calibrated size." << std::endl;
        cv::undistort(src, dst, cameraMatrix_, distCoeffs_);
        return;
    }

    // 使用查表法进行重映射
    // INTER_LINEAR 是速度和质量的最佳平衡
    cv::remap(src, dst, map1_, map2_, cv::INTER_LINEAR);
}