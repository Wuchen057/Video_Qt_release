#ifndef CAMERA_CALIBRATOR_HPP
#define CAMERA_CALIBRATOR_HPP

#include "../CommonTypes.h"
#include "../Models/Camera.h"
#include "../Models/Chessboard.h"
#include "ImageProcessor.h" // 用于查找角点

class CameraCalibrator {
public:
    CameraCalibrator();

    // 执行相机内参标定
    // imagePaths: 标定图像文件路径列表
    // chessboard: 棋盘格模型
    // camera: 将标定结果存储到此Camera对象中
    bool calibrate(const std::vector<std::string>& imagePaths,
        const Chessboard& chessboard,
        Camera& camera);

private:
    ImageProcessor imageProcessor_; // 辅助进行图像处理
};

#endif // CAMERA_CALIBRATOR_HPP