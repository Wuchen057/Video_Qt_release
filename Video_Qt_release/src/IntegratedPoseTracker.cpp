#include "IntegratedPoseTracker.h"
#include <QDebug>
#include <QGridLayout>
#include <QMessageBox>
#include <QGroupBox>

// OpenCV 依赖
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

// ==========================================
// 构造函数与析构函数
// ==========================================

IntegratedPoseTracker::IntegratedPoseTracker(QWidget* parent)
    : QMainWindow(parent),
    // 初始化相机模型与标定板参数
    chessboardA(cv::Size(11, 8), 30.0f, "chessboardA"),
    chessboardB(cv::Size(11, 8), 6.0f, "chessboardB"),
    cameraA(CameraType::CAM_A, "Camera A", cv::Size(2448, 2048)),
    cameraB(CameraType::CAM_B, "Camera B", cv::Size(2448, 2048)),
    targetObject(),
    // 初始化状态变量
    isTracking(false),
    hasInitialPose(false),
    stabilityCounter(0)
{
    // 1. 初始化业务管理器
    calibManager = new CalibrationManager(cameraA, cameraB, chessboardA, chessboardB);
    measManager = new MeasurementManager(cameraA, cameraB, targetObject, chessboardA, chessboardB);

    // 2. 设置日志等级，避免OpenCV输出过多信息
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);

    // 3. 构建界面
    setupUI();

    // 4. 设置定时器 (33ms ≈ 30 FPS)
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &IntegratedPoseTracker::updateFrame);

    // 5. 加载标定数据
    if (calibManager->loadCalibrationResults()) {
        if (!measManager->initialize()) {
            QMessageBox::warning(this, "Init Error", "MeasurementManager init failed.");
        }
    }
    else {
        QMessageBox::warning(this, "Calibration Error", "Failed to load calibration files.");
    }
}

IntegratedPoseTracker::~IntegratedPoseTracker() {
    closeSystem(); // 确保释放资源
    delete timer;
    delete calibManager;
    delete measManager;
    // trajectory3D 由 Qt 父子对象机制自动管理，无需手动 delete
}

// ==========================================
// UI 布局
// ==========================================

void IntegratedPoseTracker::setupUI() {
    this->setWindowTitle(QString::fromLocal8Bit("目标位姿测量与3D轨迹可视化系统"));
    this->resize(1800, 900); // 宽屏布局

    QWidget* centralWidget = new QWidget(this);
    this->setCentralWidget(centralWidget);
    QHBoxLayout* mainLayout = new QHBoxLayout(centralWidget);

    // ------------------------------------------------
    // 左侧：视频显示区域
    // ------------------------------------------------
    QVBoxLayout* leftLayout = new QVBoxLayout();

    // 相机 A
    QGroupBox* boxA = new QGroupBox(QString::fromLocal8Bit("相机A (主视图)"));
    QVBoxLayout* layA = new QVBoxLayout(boxA);
    videoDisplayA = new QLabel();
    videoDisplayA->setFixedSize(480, 360);
    videoDisplayA->setStyleSheet("background-color: #000; border: 1px solid #555;");
    videoDisplayA->setScaledContents(true);
    layA->addWidget(videoDisplayA);
    leftLayout->addWidget(boxA);

    // 相机 B
    QGroupBox* boxB = new QGroupBox(QString::fromLocal8Bit("相机B (辅视图)"));
    QVBoxLayout* layB = new QVBoxLayout(boxB);
    videoDisplayB = new QLabel();
    videoDisplayB->setFixedSize(480, 360);
    videoDisplayB->setStyleSheet("background-color: #000; border: 1px solid #555;");
    videoDisplayB->setScaledContents(true); 
    layB->addWidget(videoDisplayB);
    leftLayout->addWidget(boxB);

    mainLayout->addLayout(leftLayout);

    // ------------------------------------------------
    // 中间：OpenGL 3D 轨迹显示
    // ------------------------------------------------
    QGroupBox* glBox = new QGroupBox(QString::fromLocal8Bit("目标实时位姿轨迹"));
    QVBoxLayout* glLayout = new QVBoxLayout(glBox);

    trajectory3D = new Trajectory3DPlotter(this);
    trajectory3D->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    trajectory3D->setMinimumWidth(500);

    glLayout->addWidget(trajectory3D);
    mainLayout->addWidget(glBox, 1); // 权重设为1，让它尽可能占位

    // ------------------------------------------------
    // 右侧：数据与控制
    // ------------------------------------------------
    QVBoxLayout* rightLayout = new QVBoxLayout();
    rightLayout->setSpacing(10);

    // 1. 数据显示面板
    QGroupBox* dataBox = new QGroupBox(QString::fromLocal8Bit("实时位姿数据"));
    QGridLayout* grid = new QGridLayout(dataBox);

    grid->addWidget(new QLabel(QString::fromLocal8Bit("Roll (°)")), 3, 0);
    rollEdit = new QLineEdit("0.00"); rollEdit->setReadOnly(true);
    grid->addWidget(rollEdit, 3, 1);

    grid->addWidget(new QLabel(QString::fromLocal8Bit("Pitch (°)")), 4, 0);
    pitchEdit = new QLineEdit("0.00"); pitchEdit->setReadOnly(true);
    grid->addWidget(pitchEdit, 4, 1);

    grid->addWidget(new QLabel(QString::fromLocal8Bit("Yaw (°)")), 5, 0);
    yawEdit = new QLineEdit("0.00"); yawEdit->setReadOnly(true);
    grid->addWidget(yawEdit, 5, 1);

    grid->addWidget(new QLabel("dX (mm):"), 0, 0);
    xEdit = new QLineEdit("0.00"); xEdit->setReadOnly(true);
    grid->addWidget(xEdit, 0, 1);

    grid->addWidget(new QLabel("dY (mm):"), 1, 0);
    yEdit = new QLineEdit("0.00"); yEdit->setReadOnly(true);
    grid->addWidget(yEdit, 1, 1);

    grid->addWidget(new QLabel("dZ (mm):"), 2, 0);
    zEdit = new QLineEdit("0.00"); zEdit->setReadOnly(true);
    grid->addWidget(zEdit, 2, 1);

    grid->addWidget(new QLabel("RMSE (px):"), 6, 0);
    errorEdit = new QLineEdit("0.000");
    errorEdit->setReadOnly(true);
    grid->addWidget(errorEdit, 6, 1);

    rightLayout->addWidget(dataBox);

    // 2. 状态信息
    statusLabel = new QLabel(QString::fromLocal8Bit("状态：等待加载视频"));
    statusLabel->setStyleSheet("color: gray; font-weight: bold; font-size: 14px;");
    statusLabel->setAlignment(Qt::AlignCenter);
    rightLayout->addWidget(statusLabel);

    // 3. 控制按钮
    QPushButton* btnOpen = new QPushButton(QString::fromLocal8Bit("1. 打开视频文件"));
    btnStart = new QPushButton(QString::fromLocal8Bit("2. 开始/停止 位姿轨迹测量"));
    btnStart->setCheckable(true);

    QPushButton* btnReset = new QPushButton(QString::fromLocal8Bit("3. 重置轨迹"));
    QPushButton* btnClose = new QPushButton(QString::fromLocal8Bit("4. 关闭视频文件"));

    rightLayout->addWidget(btnOpen);
    rightLayout->addWidget(btnStart);
    rightLayout->addWidget(btnReset);
    rightLayout->addWidget(btnClose);
    rightLayout->addStretch();

    mainLayout->addLayout(rightLayout);

    // 信号连接
    connect(btnOpen, &QPushButton::clicked, this, &IntegratedPoseTracker::openVideoFiles);
    connect(btnStart, &QPushButton::clicked, this, &IntegratedPoseTracker::toggleTracking);
    connect(btnReset, &QPushButton::clicked, this, &IntegratedPoseTracker::resetTrajectory);
    connect(btnClose, &QPushButton::clicked, this, &IntegratedPoseTracker::closeSystem);
}

// ==========================================
// 核心逻辑：视频流处理循环
// ==========================================

void IntegratedPoseTracker::updateFrame() {
    cv::Mat frameA_raw, frameB_raw;

    // 1. 读取视频帧
    bool retA = capA.read(frameA_raw);
    bool retB = capB.read(frameB_raw);

    // 2. 循环播放逻辑
    if (!retA || !retB || frameA_raw.empty() || frameB_raw.empty()) {
        capA.set(cv::CAP_PROP_POS_FRAMES, 0);
        capB.set(cv::CAP_PROP_POS_FRAMES, 0);
        prevGraySmall.release(); // 重置运动检测背景，防止跳帧误报
        return;
    }

    // 3. 运动检测 (Motion Detection)
    // 只有当物体静止时才进行高精度测量，避免运动模糊导致的误差
    bool isMoving = detectMotion(frameA_raw);

    if (isMoving) {
        stabilityCounter = 0;
        statusLabel->setText(QString::fromLocal8Bit("检测到运动..."));
        statusLabel->setStyleSheet("color: orange;");
    }
    else {
        stabilityCounter++;
    }

    // 4. 位姿解算与轨迹更新
    bool isStatic = (stabilityCounter > 3); // 连续5帧静止视为稳定

    if (isTracking && isStatic) {
        statusLabel->setText(QString::fromLocal8Bit("测量中..."));
        statusLabel->setStyleSheet("color: green; font-weight: bold;");

        PoseResult poseResA, poseResB;
        cv::Mat debugImg; // 这里可以获取特征点检测图，如果需要显示的话

        // 调用测量管理器进行姿态解算
        double rmse;
        bool success = measManager->processMeasurement(frameA_raw, frameB_raw, poseResA, poseResB, debugImg, rmse);

        if (success && !poseResA.T_matrix.empty() && !poseResB.T_matrix.empty()) {

            // 4.1 在原图上绘制坐标轴 (Visual Feedback)
            drawAxis(frameA_raw, cameraA, poseResA);
            drawAxis(frameB_raw, cameraB, poseResB);

            // 4.2 计算目标绝对位姿 (相对于基准坐标系)
            cv::Mat T_curr;
            measManager->calculateFinalPoseChange(
                poseResA.T_matrix,
                poseResB.T_matrix,
                calibManager->getHandEyeTransform(),
                T_curr
            );

            // 确保是 CV_64F 类型
            T_curr.convertTo(T_curr, CV_64F);

            cv::Mat T_final = T_curr; // 直接使用绝对位姿

            TrajectoryPoint pt;
            // 1. 赋值旋转矩阵 (关键：用于绘制目标的坐标轴)
            pt.R = T_final(cv::Rect(0, 0, 3, 3)).clone();
            pt.t = T_final(cv::Rect(3, 0, 1, 3)).clone();

            // 2. 赋值位置 (相对于相机原点的位置)
            pt.dx = T_final.at<double>(0, 3);
            pt.dy = T_final.at<double>(1, 3);
            pt.dz = T_final.at<double>(2, 3);

            // 3. 计算欧拉角 (仅用于UI显示，OpenGL画图不需要这个)
            cv::Mat euler = measManager->rotationMatrixToEulerAngles(pt.R);
            pt.roll  = euler.at<double>(0);
            pt.pitch = euler.at<double>(1);
            pt.yaw   = euler.at<double>(2);

            // 4. 如果你希望 UI 上的文本依然显示"相对于起点"的变化量，
            //    可以保留 hasInitialPose 逻辑单独给 UI 用，
            //    但 pt (传给OpenGL的点) 必须是绝对坐标。
            //    为了简单起见，这里 UI 也显示绝对坐标：
            xEdit->setText(QString::number(pt.dx, 'f', 2));
            yEdit->setText(QString::number(pt.dy, 'f', 2));
            zEdit->setText(QString::number(pt.dz, 'f', 2));
            rollEdit->setText(QString::number(pt.roll, 'f', 2));
            pitchEdit->setText(QString::number(pt.pitch, 'f', 2));
            yawEdit->setText(QString::number(pt.yaw, 'f', 2));
            errorEdit->setText(QString::number(rmse, 'f', 4));

            // 发送给 OpenGL
            trajectory3D->addPoint(pt);

        }
    }
    else if (isTracking && isMoving) {
        // 正在追踪但物体在动，不记录数据
    }

    // 5. 显示图像到界面
    // 缩放以适应 UI 大小，避免过大导致卡顿
    cv::Mat displayA, displayB;
    cv::resize(frameA_raw, displayA, cv::Size(480, 360));
    cv::resize(frameB_raw, displayB, cv::Size(480, 360));

    videoDisplayA->setPixmap(QPixmap::fromImage(cvMatToQImage(displayA)));
    videoDisplayB->setPixmap(QPixmap::fromImage(cvMatToQImage(displayB)));
}

// ==========================================
// 辅助功能实现
// ==========================================

void IntegratedPoseTracker::openVideoFiles() {
    QString filter = "Video Files (*.avi *.mp4 *.mkv);;All Files (*.*)";

    QString fileA = QFileDialog::getOpenFileName(this, QString::fromLocal8Bit("选择相机A的视频文件"), "", filter);
    if (fileA.isEmpty()) return;

    QString fileB = QFileDialog::getOpenFileName(this, QString::fromLocal8Bit("选择相机B的视频文件"), "", filter);
    if (fileB.isEmpty()) return;

    // 先关闭之前的
    closeSystem();

    capA.open(fileA.toStdString());
    capB.open(fileB.toStdString());

    if (capA.isOpened() && capB.isOpened()) {
        resetTrajectory(); // 清空旧轨迹
        timer->start(333);  // 启动循环
        statusLabel->setText(QString::fromLocal8Bit("视频已加载 - 暂停中"));
    }
    else {
        QMessageBox::critical(this, "Error", QString::fromLocal8Bit("无法打开视频文件！"));
    }
}

void IntegratedPoseTracker::closeSystem() {
    if (timer->isActive()) timer->stop();

    if (capA.isOpened()) capA.release();
    if (capB.isOpened()) capB.release();

    videoDisplayA->clear();
    videoDisplayB->clear();
    prevGraySmall.release();

    isTracking = false;
    btnStart->setChecked(false);
    btnStart->setText(QString::fromLocal8Bit("2. 开始/停止 测量"));
    statusLabel->setText(QString::fromLocal8Bit("未连接"));
}

void IntegratedPoseTracker::toggleTracking() {
    isTracking = btnStart->isChecked();
    if (isTracking) {
        statusLabel->setText(QString::fromLocal8Bit("测量已启动 - 等待静止..."));
    }
    else {
        statusLabel->setText(QString::fromLocal8Bit("测量暂停"));
    }
}

void IntegratedPoseTracker::resetTrajectory() {
    hasInitialPose = false;
    trajectory3D->clearTrajectory(); // 清空OpenGL绘图

    xEdit->setText("0.00"); yEdit->setText("0.00"); zEdit->setText("0.00");
    rollEdit->setText("0.00"); pitchEdit->setText("0.00"); yawEdit->setText("0.00");
    errorEdit->setText("0.00");

    QMessageBox::information(this, "Info", QString::fromLocal8Bit("轨迹已重置，下一次测量将作为新起点。"));
}

bool IntegratedPoseTracker::detectMotion(const cv::Mat& frame) {
    if (frame.empty()) return false;

    cv::Mat currentSmall;
    // 缩小图像加快处理速度 (320x240)
    cv::resize(frame, currentSmall, cv::Size(320, 240));
    cv::cvtColor(currentSmall, currentSmall, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(currentSmall, currentSmall, cv::Size(21, 21), 0);

    if (prevGraySmall.empty()) {
        prevGraySmall = currentSmall.clone();
        return true;
    }

    cv::Mat diff;
    cv::absdiff(currentSmall, prevGraySmall, diff);
    cv::threshold(diff, diff, 25, 255, cv::THRESH_BINARY);

    int changedPixels = cv::countNonZero(diff);
    prevGraySmall = currentSmall.clone();

    // 阈值：如果变动像素超过 3000 (约占图像 4%) 则认为在运动
    return changedPixels > 3000;
}

void IntegratedPoseTracker::drawAxis(cv::Mat& img, const Camera& cam, const PoseResult& pose) {
    if (pose.T_matrix.empty()) return;

    // 提取旋转向量和平移向量
    cv::Mat rvec, tvec;
    cv::Rodrigues(pose.T_matrix(cv::Rect(0, 0, 3, 3)), rvec);
    tvec = pose.T_matrix(cv::Rect(3, 0, 1, 3));

    // 使用 OpenCV 绘制坐标轴 (长度 80mm)
    try {
        cv::drawFrameAxes(img, cam.getCameraMatrix(), cam.getDistCoeffs(), rvec, tvec, 80.0f, 10);
    }
    catch (...) {
        // 防止 OpenCV 版本兼容问题导致的崩溃
    }
}

QImage IntegratedPoseTracker::cvMatToQImage(const cv::Mat& mat) {
    if (mat.empty()) return QImage();

    if (mat.type() == CV_8UC3) {
        cv::Mat rgb;
        cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
        // 深拷贝数据，防止 mat 析构后 QImage 访问非法内存
        return QImage(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), QImage::Format_RGB888).copy();
    }
    else if (mat.type() == CV_8UC1) {
        return QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_Grayscale8).copy();
    }
    return QImage();
}