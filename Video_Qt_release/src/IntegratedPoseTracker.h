#ifndef INTEGRATEDPOSETRACKER_H
#define INTEGRATEDPOSETRACKER_H

#include <QMainWindow>
#include <QTimer>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QGroupBox>
#include <QFileDialog>
#include <vector>

// 核心库
#include <opencv2/opencv.hpp>
#include "Models/Camera.h"
#include "Models/Chessboard.h"
#include "Models/TargetObject.h"
#include "Managers/CalibrationManager.h"
#include "Managers/MeasurementManager.h"
#include "trajectory3dplotter.h" // OpenGL 绘图窗口

class IntegratedPoseTracker : public QMainWindow {
    Q_OBJECT

public:
    explicit IntegratedPoseTracker(QWidget* parent = nullptr);
    ~IntegratedPoseTracker();

private slots:
    void openVideoFiles();      // 合并后的打开视频逻辑
    void updateFrame();         // 定时刷新函数（包含实时解算和轨迹更新）
    void toggleTracking();      // 切换实时测量状态
    void resetTrajectory();     // 重置轨迹和基准点
    void closeSystem();

private:
    void setupUI();
    bool detectMotion(const cv::Mat& frame);
    QImage cvMatToQImage(const cv::Mat& mat);
    void drawAxis(cv::Mat& img, const Camera& cam, const PoseResult& pose);

    // === UI 组件 ===
    QLabel* videoDisplayA, * videoDisplayB;
    Trajectory3DPlotter* trajectory3D; // OpenGL 窗口
    QLineEdit* xEdit, * yEdit, * zEdit, * rollEdit, * pitchEdit, * yawEdit;
    QPushButton* btnStart;
    QLabel* statusLabel;
    QLineEdit* errorEdit;

    // === 逻辑控制 ===
    cv::VideoCapture capA, capB;
    QTimer* timer;
    bool isTracking;

    // === 算法对象 ===
    CalibrationManager* calibManager;
    MeasurementManager* measManager;
    Camera cameraA, cameraB;
    Chessboard chessboardA, chessboardB;
    TargetObject targetObject;

    // === 轨迹数据 ===
    bool hasInitialPose;
    cv::Mat T0_inv; // 初始时刻的逆矩阵
    std::vector<TrajectoryPoint> trajectoryHistory;

    // 运动检测辅助
    cv::Mat prevGraySmall;
    int stabilityCounter;
};

#endif