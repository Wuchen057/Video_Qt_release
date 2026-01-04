#ifndef CAMERAFILEAPP_H
#define CAMERAFILEAPP_H

#include <QMainWindow>
#include <QLabel>
#include <QLineEdit>
#include <QImage>
#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QMessageBox>
#include <QFileDialog> // 新增：文件选择对话框
#include <QDebug>

// OpenCV 包含
#include <opencv2/opencv.hpp>

// 前置声明
class Camera;
class Chessboard;
class TargetObject;
class CalibrationManager;
class MeasurementManager;
struct PoseResult;

// 包含 Utils 和 Models
#include "Utils/Utils.h"
#include "Models/Camera.h"
#include "Models/Chessboard.h"
#include "Models/TargetObject.h"
#include "Managers/CalibrationManager.h"
#include "Managers/MeasurementManager.h"

class CameraFileApp : public QMainWindow {
    Q_OBJECT

public:
    CameraFileApp(QWidget* parent = nullptr);
    ~CameraFileApp();

private slots:
    // --- 新增：加载图片槽函数 ---
    void loadT1Images(); // 加载初始时刻(T1)的两张图
    void loadT2Images(); // 加载当前时刻(T2)的两张图

    // --- 计算槽函数 ---
    void calculatePoseChange(); // 计算 T1 -> T2 的位姿变化

private:
    // --- UI 初始化与辅助 ---
    void setupUI();
    void updatePoseDisplay();
    void resetSystem();

    // 辅助：读取带有中文路径的图片
    cv::Mat readImageSafe(const QString& filePath);

    // 辅助：Mat 转 QImage 用于显示
    QImage cvMatToQImage(const cv::Mat& mat);

    // 辅助：在界面上显示缩略图
    void displayImage(const cv::Mat& img, QLabel* label);

private:
    // --- UI 控件 ---
    QLabel* displayT1_CamA; // 显示 T1 时刻 相机A
    QLabel* displayT1_CamB; // 显示 T1 时刻 相机B
    QLabel* displayT2_CamA; // 显示 T2 时刻 相机A
    QLabel* displayT2_CamB; // 显示 T2 时刻 相机B

    QLabel* resultDisplay;  // 显示特征提取或计算结果的可视化

    // 显示位姿数据的控件
    QLineEdit* xPosEdit, * yPosEdit, * zPosEdit;
    QLineEdit* alphaEdit, * betaEdit, * gammaEdit;
    QLabel* statusLabel;

    // --- 核心对象 ---
    Chessboard chessboardA;
    Chessboard chessboardB;
    Camera cameraA;
    Camera cameraB;
    TargetObject targetObject;

    CalibrationManager* calibManager;
    MeasurementManager* measManager;

    // --- 图像数据 ---
    // 存储加载进来的图片
    cv::Mat img_t1_camA, img_t1_camB; // 初始时刻
    cv::Mat img_t2_camA, img_t2_camB; // 当前时刻

    // 计算结果
    PoseResult poseA_t1, poseB_t1;
    PoseResult poseA_t2, poseB_t2;

    cv::Mat angle;
    cv::Mat t_relative;

    // 状态标志
    bool t1Loaded;
    bool t2Loaded;
};

#endif // CAMERASYSTEMAPP_H