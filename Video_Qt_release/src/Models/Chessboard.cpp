#include "Chessboard.h"
#include <iostream>

Chessboard::Chessboard(cv::Size boardSize, float squareSize_mm, const std::string& name)
    : boardSize_(boardSize),          // <--- 确保这里将传入的 boardSize 赋给成员变量 boardSize_
    squareSize_(squareSize_mm),
    name_(name)
{
    // 在这里调用生成3D点的函数
    generateObjectPoints();
    std::cout << "Chessboard '" << name_ << "' initialized with internal corners "
        << boardSize_.width << "x" << boardSize_.height
        << " and square size " << squareSize_ << "mm." << std::endl;
}

// generateObjectPoints 函数的实现
void Chessboard::generateObjectPoints() {
    objectPoints_.clear();
    for (int i = 0; i < boardSize_.height; ++i) {
        for (int j = 0; j < boardSize_.width; ++j) {
            objectPoints_.push_back(cv::Point3f(j * squareSize_, i * squareSize_, 0.0f));
        }
    }
}


bool Chessboard::findCorners(const cv::Mat& image, std::vector<cv::Point2f>& corners) const {
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    else {
        gray = image;
    }

    // 检查 boardSize_ 是否有效
    if (boardSize_.width <= 0 || boardSize_.height <= 0) {
        std::cerr << "Error: Chessboard boardSize_ is invalid ("
            << boardSize_.width << "x" << boardSize_.height << "). Cannot find corners." << std::endl;
        return false;
    }

    // 关键：这里使用的 boardSize_ 必须是正确的内角点尺寸
    bool found = cv::findChessboardCorners(gray, boardSize_, corners, // <--- 这里使用了 boardSize_
        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

    if (found) {
        // 亚像素角点精炼
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
    }
    return found;
}

void Chessboard::drawCorners(cv::Mat& image, const std::vector<cv::Point2f>& corners) const {
    cv::drawChessboardCorners(image, boardSize_, cv::Mat(corners), true);
}