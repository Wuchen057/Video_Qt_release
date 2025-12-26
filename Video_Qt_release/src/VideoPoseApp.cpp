#include "VideoPoseApp.h"
#include <QDebug>

VideoPoseApp::VideoPoseApp(QWidget* parent)
    : QMainWindow(parent),
    chessboardA(cv::Size(11, 8), 30.0f, "chessboardA"),
    chessboardB(cv::Size(11, 8), 6.0f, "chessboardB"),
    cameraA(CameraType::CAM_A, "Camera A", cv::Size(2448, 2048)),
    cameraB(CameraType::CAM_B, "Camera B", cv::Size(2448, 2048)),
    targetObject(),
    camerasConnected(false),
    isTracking(false),
    stabilityCounter(0),
    isStatic(false)
{
    calibManager = new CalibrationManager(cameraA, cameraB, chessboardA, chessboardB);
    measManager = new MeasurementManager(cameraA, cameraB, targetObject, chessboardA, chessboardB);
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);

    setupUI();

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &VideoPoseApp::updateFrame);

    if (calibManager->loadCalibrationResults()) {
        if (!measManager->initialize()) {
            QMessageBox::warning(this, "Init Error", "MeasurementManager init failed.");
        }
    }
}

VideoPoseApp::~VideoPoseApp() {
    closeCamera();
    delete timer;
    delete calibManager;
    delete measManager;
}

void VideoPoseApp::setupUI() {
    this->setWindowTitle(QString::fromLocal8Bit("视频位姿追踪系统"));
    this->resize(1280, 850);

    QWidget* centralWidget = new QWidget(this);
    this->setCentralWidget(centralWidget);
    QHBoxLayout* mainLayout = new QHBoxLayout(centralWidget);

    // === 左侧：视频显示 ===
    QVBoxLayout* leftLayout = new QVBoxLayout();

    // Camera A
    QGroupBox* boxA = new QGroupBox(QString::fromLocal8Bit("相机A (主视角 - 视频)"));
    QVBoxLayout* layoutA = new QVBoxLayout(boxA);
    videoDisplayCamA = new QLabel();
    videoDisplayCamA->setFixedSize(560, 420);
    videoDisplayCamA->setStyleSheet("background-color: #111; border: 1px solid #333;");
    videoDisplayCamA->setAlignment(Qt::AlignCenter);
    layoutA->addWidget(videoDisplayCamA);
    leftLayout->addWidget(boxA);

    // Camera B
    QGroupBox* boxB = new QGroupBox(QString::fromLocal8Bit("相机B (辅助视角 - 视频)"));
    QVBoxLayout* layoutB = new QVBoxLayout(boxB);
    videoDisplayCamB = new QLabel();
    videoDisplayCamB->setFixedSize(560, 420);
    videoDisplayCamB->setStyleSheet("background-color: #111; border: 1px solid #333;");
    videoDisplayCamB->setAlignment(Qt::AlignCenter);
    layoutB->addWidget(videoDisplayCamB);
    leftLayout->addWidget(boxB);

    mainLayout->addLayout(leftLayout);

    // === 右侧：控制与数据 ===
    QVBoxLayout* rightLayout = new QVBoxLayout();
    rightLayout->setSpacing(15);

    // Controls
    QGroupBox* ctrlBox = new QGroupBox(QString::fromLocal8Bit("系统控制"));
    QVBoxLayout* ctrlLayout = new QVBoxLayout(ctrlBox);

    QHBoxLayout* brightLayout = new QHBoxLayout();
    brightLayout->addWidget(new QLabel(QString::fromLocal8Bit("亮度(禁用):")));
    brightnessEdit = new QLineEdit("50");
    brightnessEdit->setValidator(new QIntValidator(0, 100, this));
    brightnessEdit->setFixedWidth(50);
    brightnessEdit->setEnabled(false); // 视频模式下禁用
    QPushButton* btnSetBright = new QPushButton(QString::fromLocal8Bit("设置"));
    btnSetBright->setEnabled(false);   // 视频模式下禁用
    brightLayout->addWidget(brightnessEdit);
    brightLayout->addWidget(btnSetBright);
    ctrlLayout->addLayout(brightLayout);

    QPushButton* btnOpen = new QPushButton(QString::fromLocal8Bit("打开视频文件"));
    btnToggleTracking = new QPushButton(QString::fromLocal8Bit("开始实时测量"));
    btnToggleTracking->setCheckable(true);
    btnToggleTracking->setStyleSheet("QPushButton:checked { background-color: #90EE90; color: black; }"); // LightGreen
    QPushButton* btnClose = new QPushButton(QString::fromLocal8Bit("停止/关闭"));

    ctrlLayout->addWidget(btnOpen);
    ctrlLayout->addWidget(btnToggleTracking);
    ctrlLayout->addWidget(btnClose);
    rightLayout->addWidget(ctrlBox);

    // Pose Data
    QGroupBox* dataBox = new QGroupBox(QString::fromLocal8Bit("位姿数据"));
    QGridLayout* dataLayout = new QGridLayout(dataBox);
    dataLayout->setSpacing(10);

    dataLayout->addWidget(new QLabel("Position (mm)"), 0, 1);
    dataLayout->addWidget(new QLabel("Rotation (deg)"), 0, 3);

    dataLayout->addWidget(new QLabel("X:"), 1, 0);
    xPosEdit = new QLineEdit("0.00"); xPosEdit->setReadOnly(true);
    dataLayout->addWidget(xPosEdit, 1, 1);

    dataLayout->addWidget(new QLabel("Roll:"), 1, 2);
    rollEdit = new QLineEdit("0.00"); rollEdit->setReadOnly(true);
    dataLayout->addWidget(rollEdit, 1, 3);

    dataLayout->addWidget(new QLabel("Y:"), 2, 0);
    yPosEdit = new QLineEdit("0.00"); yPosEdit->setReadOnly(true);
    dataLayout->addWidget(yPosEdit, 2, 1);

    dataLayout->addWidget(new QLabel("Pitch:"), 2, 2);
    pitchEdit = new QLineEdit("0.00"); pitchEdit->setReadOnly(true);
    dataLayout->addWidget(pitchEdit, 2, 3);

    dataLayout->addWidget(new QLabel("Z:"), 3, 0);
    zPosEdit = new QLineEdit("0.00"); zPosEdit->setReadOnly(true);
    dataLayout->addWidget(zPosEdit, 3, 1);

    dataLayout->addWidget(new QLabel("Yaw:"), 3, 2);
    yawEdit = new QLineEdit("0.00"); yawEdit->setReadOnly(true);
    dataLayout->addWidget(yawEdit, 3, 3);

    rightLayout->addWidget(dataBox);

    connectionStatusLabel = new QLabel(QString::fromLocal8Bit("状态：等待加载视频"));
    connectionStatusLabel->setStyleSheet("color: gray; font-weight: bold; font-size: 14px;");
    connectionStatusLabel->setAlignment(Qt::AlignCenter);
    rightLayout->addWidget(connectionStatusLabel);

    rightLayout->addStretch();
    mainLayout->addLayout(rightLayout);

    connect(btnSetBright, &QPushButton::clicked, this, &VideoPoseApp::setBrightness);
    connect(btnOpen, &QPushButton::clicked, this, &VideoPoseApp::openCamera);
    connect(btnClose, &QPushButton::clicked, this, &VideoPoseApp::closeCamera);
    connect(btnToggleTracking, &QPushButton::clicked, this, &VideoPoseApp::toggleTracking);
}

void VideoPoseApp::openCamera() {
    if (camerasConnected) return;

    // === 修改：选择文件而不是打开硬件索引 ===
    QString filter = "Video Files (*.avi *.mp4 *.mkv);;All Files (*.*)";

    // 1. 选择相机A视频
    QString fileNameA = QFileDialog::getOpenFileName(this,
        QString::fromLocal8Bit("请选择【相机A】的录像文件"), "", filter);
    if (fileNameA.isEmpty()) return;

    // 2. 选择相机B视频
    QString fileNameB = QFileDialog::getOpenFileName(this,
        QString::fromLocal8Bit("请选择【相机B】的录像文件"), "", filter);
    if (fileNameB.isEmpty()) return;

    // 3. 打开视频流
    capA.open(fileNameA.toStdString());
    capB.open(fileNameB.toStdString());

    if (capA.isOpened() && capB.isOpened()) {
        // === 注意：文件模式下不需要设置缓冲区和分辨率，OpenCV会自动读取 ===
        // capA.set(cv::CAP_PROP_FRAME_WIDTH, ...); // 删除
        // capA.set(cv::CAP_PROP_BUFFERSIZE, 1);    // 删除

        camerasConnected = true;
        stabilityCounter = 0;

        connectionStatusLabel->setText(QString::fromLocal8Bit("状态：视频已加载，播放中"));

        // 设置定时器频率，33ms 约等于 30FPS。如果是60FPS视频可设为16
        timer->start(33);
    }
    else {
        capA.release(); capB.release();
        QMessageBox::critical(this, "Error", QString::fromLocal8Bit("无法打开选定的视频文件！"));
    }
}

void VideoPoseApp::closeCamera() {
    if (!camerasConnected) return;
    timer->stop();
    capA.release();
    capB.release();

    // 清空辅助变量
    prevGraySmall.release();

    camerasConnected = false;
    isTracking = false;
    btnToggleTracking->setChecked(false);
    btnToggleTracking->setText(QString::fromLocal8Bit("开始实时测量"));
    connectionStatusLabel->setText(QString::fromLocal8Bit("状态：未连接"));
    videoDisplayCamA->clear();
    videoDisplayCamB->clear();
    resetData();
}

void VideoPoseApp::toggleTracking() {
    if (!camerasConnected) return;
    isTracking = btnToggleTracking->isChecked();
    if (isTracking) {
        btnToggleTracking->setText(QString::fromLocal8Bit("停止实时测量"));
    }
    else {
        btnToggleTracking->setText(QString::fromLocal8Bit("开始实时测量"));
        connectionStatusLabel->setText(QString::fromLocal8Bit("状态：待机"));
        resetData();
    }
}

/**
 * @brief 简单的帧差法检测运动
 */
bool VideoPoseApp::detectMotion(const cv::Mat& newFrame) {
    if (newFrame.empty()) return false;

    // 1. 缩小图像
    cv::Mat currentSmall;
    cv::resize(newFrame, currentSmall, cv::Size(320, 240));
    cv::cvtColor(currentSmall, currentSmall, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(currentSmall, currentSmall, cv::Size(21, 21), 0);

    // 如果是第一帧，初始化
    if (prevGraySmall.empty()) {
        prevGraySmall = currentSmall.clone();
        return true;
    }

    // 4. 计算差异
    cv::Mat diff;
    cv::absdiff(currentSmall, prevGraySmall, diff);
    cv::threshold(diff, diff, 25, 255, cv::THRESH_BINARY);
    int changedPixels = cv::countNonZero(diff);

    // 更新上一帧
    prevGraySmall = currentSmall.clone();

    // 阈值判断
    return changedPixels > 5000;
}

void VideoPoseApp::updateFrame() {
    if (!camerasConnected) return;

    // 1. 读取视频帧
    // 使用 read() 对文件更安全，可以检测返回值
    bool retA = capA.read(frameA_raw);
    bool retB = capB.read(frameB_raw);

    // === 循环播放逻辑 ===
    // 如果读完了或者帧为空，重置到开头
    if (!retA || !retB || frameA_raw.empty() || frameB_raw.empty()) {
        capA.set(cv::CAP_PROP_POS_FRAMES, 0);
        capB.set(cv::CAP_PROP_POS_FRAMES, 0);
        // 重置运动检测的基准帧，防止跳变导致误报运动
        prevGraySmall.release();
        return;
    }

    // 2. 运动检测 (基于主相机 A)
    bool isMoving = detectMotion(frameA_raw);

    // 3. 状态判定机
    if (isMoving) {
        stabilityCounter = 0;
        isStatic = false;
        if (isTracking) {
            connectionStatusLabel->setText(QString::fromLocal8Bit("状态：检测到物体运动..."));
            connectionStatusLabel->setStyleSheet("color: red; font-weight: bold;");
        }
    }
    else {
        stabilityCounter++;
        // 视频流如果很稳，可以适当调小 STABILITY_THRESHOLD
        if (stabilityCounter > STABILITY_THRESHOLD) {
            isStatic = true;
            if (isTracking) {
                connectionStatusLabel->setText(QString::fromLocal8Bit("状态：物体静止 - 计算中"));
                connectionStatusLabel->setStyleSheet("color: green; font-weight: bold;");
            }
        }
    }

    // 4. 位姿解算与标注
    if (isTracking && isStatic) {
        cv::Mat dummyResult;
        double rmse;
        bool success = measManager->processMeasurement(frameA_raw, frameB_raw, currentPoseA, currentPoseB, dummyResult, rmse);

        if (success) {
            cv::Mat angle, t_rel;
            measManager->calculateFinalPoseChange(
                currentPoseA.T_matrix, currentPoseB.T_matrix,
                calibManager->getHandEyeTransform(), angle, t_rel);

            // 更新 UI
            if (!t_rel.empty() && !angle.empty()) {
                xPosEdit->setText(QString::number(t_rel.at<double>(0, 0), 'f', 2));
                yPosEdit->setText(QString::number(t_rel.at<double>(1, 0), 'f', 2));
                zPosEdit->setText(QString::number(t_rel.at<double>(2, 0), 'f', 2));
                rollEdit->setText(QString::number(angle.at<double>(0, 0), 'f', 2));
                pitchEdit->setText(QString::number(angle.at<double>(1, 0), 'f', 2));
                yawEdit->setText(QString::number(angle.at<double>(2, 0), 'f', 2));
            }

            // 在视频帧上画线
            drawAxisAndInfo(frameA_raw, cameraA, currentPoseA);
            drawAxisAndInfo(frameB_raw, cameraB, currentPoseB);
        }
    }

    // 5. 显示图像
    if (videoDisplayCamA) {
        cv::resize(frameA_raw, frameA_display, cv::Size(560, 420));
        videoDisplayCamA->setPixmap(QPixmap::fromImage(cvMatToQImage(frameA_display)));
    }
    if (videoDisplayCamB) {
        cv::resize(frameB_raw, frameB_display, cv::Size(560, 420));
        videoDisplayCamB->setPixmap(QPixmap::fromImage(cvMatToQImage(frameB_display)));
    }
}

// 辅助函数：绘制坐标轴
void VideoPoseApp::drawAxisAndInfo(cv::Mat& img, const Camera& cam, const PoseResult& pose) {
    if (pose.T_matrix.empty()) return;
    const cv::Mat& K = cam.getCameraMatrix();
    const cv::Mat& D = cam.getDistCoeffs();
    cv::Mat R = pose.T_matrix(cv::Rect(0, 0, 3, 3));
    cv::Mat tvec = pose.T_matrix(cv::Rect(3, 0, 1, 3));
    cv::Mat rvec;
    cv::Rodrigues(R, rvec);
    try {
        cv::drawFrameAxes(img, K, D, rvec, tvec, 80.0f, 15);
    }
    catch (...) {}
}

void VideoPoseApp::setBrightness() {
    // 视频文件模式下无法调节硬件亮度，直接返回
    return;
}

void VideoPoseApp::resetData() {
    xPosEdit->setText("0.00"); yPosEdit->setText("0.00"); zPosEdit->setText("0.00");
    rollEdit->setText("0.00"); pitchEdit->setText("0.00"); yawEdit->setText("0.00");
}

QImage VideoPoseApp::cvMatToQImage(const cv::Mat& mat) {
    if (mat.empty()) return QImage();
    if (mat.type() == CV_8UC3) {
        cv::Mat rgb;
        cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
        return QImage(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), QImage::Format_RGB888).copy();
    }
    else if (mat.type() == CV_8UC1) {
        return QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_Grayscale8).copy();
    }
    return QImage();
}