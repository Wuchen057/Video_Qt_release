#ifndef CALIBRATION_MANAGER_HPP
#define CALIBRATION_MANAGER_HPP

#include "../CommonTypes.h"
#include "../Models/Camera.h"
#include "../Models/Chessboard.h"
#include "../Utils/CameraCalibrator.h" // 辅助进行相机标定
#include "../Utils/PoseEstimator.h"    // 辅助进行手眼标定中的PnP

class CalibrationManager {
public:
    CalibrationManager(Camera& camA, Camera& camB, Chessboard& chessboardA, Chessboard& chessboardB);

    // 加载所有标定结果
    bool loadCalibrationResults();
    // 保存所有标定结果
    bool saveCalibrationResults();

    // 标定相机A内参 (使用 chessboardA_)
    bool calibrateCameraA(const std::vector<std::string>& imagePaths);
    // 标定相机B内参 (使用 chessboardB_)
    bool calibrateCameraB(const std::vector<std::string>& imagePaths);

    // 手眼标定：获取相机A到其上棋盘格的变换T_CamA_Board
    // 这通常意味着你需要提供一系列的图像以及对应的相机A的位姿或者棋盘格的位姿信息。
    // 在你的描述中，棋盘格是固定在相机A上的，所以这里我们是求一个固定变换。
    // 这可以通过让相机A（带着棋盘格）相对于另一个固定相机（或已知世界坐标系）进行多次运动，
    // 然后利用这些数据求解。如果相机B是用来观测相机A上的棋盘格，那这就是一个典型的
    // "eye-on-hand" (如果相机A是手) 或 "eye-to-hand" (如果相机B是手) 问题，
    // 实际是求 A -> Board 的固定变换。
    // 更简单直接的方法：直接用相机A拍摄其上棋盘格的图像，然后解PnP
    bool handEyeCalibration(const std::vector<std::string>& imagePaths_CamA,
        const std::vector<std::string>& imagePaths_CamB);

    bool saveMatToTxt(const cv::Mat& matrix, const std::string& filename);

    cv::Mat loadMatFromTxt(const std::string& filename);

    // 获取手眼标定结果
    cv::Mat getHandEyeTransform() const { return T_CamA_Board_; }

private:
    Camera& cameraA_;
    Camera& cameraB_;
    Chessboard& chessboardA_;     // 新增：用于相机A的棋盘格
    Chessboard& chessboardB_;     // 新增：用于相机B的棋盘格

    CameraCalibrator camCalibrator_;
    PoseEstimator poseEstimator_;

    cv::Mat T_CamA_Board_; // 手眼标定结果：棋盘格坐标系到相机A坐标系的变换
    // (T_CamA_Board: from Board to CamA)
};

#endif // CALIBRATION_MANAGER_HPP