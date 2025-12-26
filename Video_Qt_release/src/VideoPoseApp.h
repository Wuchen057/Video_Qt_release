#ifndef VIDEOPOSEAPP_H
#define VIDEOPOSEAPP_H

#include <QMainWindow>
#include <QTimer>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QMessageBox>
#include <QFileDialog> // 新增：用于选择视频文件
#include <QIntValidator>

#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>

// 假设这些自定义头文件在你的项目中存在
#include "Utils/Utils.h"
#include "Models/Camera.h"      // 如果这些类在栈上实例化(非指针)，则必须包含头文件
#include "Models/Chessboard.h"
#include "Models/TargetObject.h"
#include "Managers/CalibrationManager.h"
#include "Managers/MeasurementManager.h"

class VideoPoseApp : public QMainWindow {
    Q_OBJECT

public:
    explicit VideoPoseApp(QWidget* parent = nullptr);
    ~VideoPoseApp();

private slots:
    void openCamera();       // 已修改：打开视频文件
    void closeCamera();
    void toggleTracking();
    void setBrightness();    // 已修改：禁用
    void updateFrame();      // 已修改：包含循环播放逻辑

private:
    void setupUI();
    bool detectMotion(const cv::Mat& newFrame);
    void resetData();
    QImage cvMatToQImage(const cv::Mat& mat);
    void drawAxisAndInfo(cv::Mat& img, const Camera& cam, const PoseResult& pose);

    // === UI 组件 ===
    QLabel* videoDisplayCamA;
    QLabel* videoDisplayCamB;
    QLabel* connectionStatusLabel;
    QLineEdit* brightnessEdit;
    QPushButton* btnToggleTracking;

    // 数据显示
    QLineEdit* xPosEdit;
    QLineEdit* yPosEdit;
    QLineEdit* zPosEdit;
    QLineEdit* rollEdit;
    QLineEdit* pitchEdit;
    QLineEdit* yawEdit;

    QTimer* timer;

    // === OpenCV 与 图像数据 ===
    cv::VideoCapture capA;
    cv::VideoCapture capB;

    cv::Mat frameA_raw, frameA_display;
    cv::Mat frameB_raw, frameB_display;

    // 运动检测辅助变量
    cv::Mat prevGraySmall;

    // === 业务逻辑对象 ===
    Chessboard chessboardA;
    Chessboard chessboardB;
    Camera cameraA;
    Camera cameraB;
    TargetObject targetObject;

    CalibrationManager* calibManager;
    MeasurementManager* measManager;

    // === 状态标志 ===
    bool camerasConnected;
    bool isTracking;

    // 稳定性检测逻辑
    bool isStatic;
    int stabilityCounter;
    const int STABILITY_THRESHOLD = 3; // 连续N帧无运动视为静止

    // 当前计算出的位姿结果
    PoseResult currentPoseA;
    PoseResult currentPoseB;
};

#endif