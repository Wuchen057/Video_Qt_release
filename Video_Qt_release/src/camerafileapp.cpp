#include "camerafileapp.h"
#include <QBuffer>

// 构造函数
CameraFileApp::CameraFileApp(QWidget* parent)
    : QMainWindow(parent),
    chessboardA(cv::Size(11, 8), 30.0f, "chessboardA"),
    chessboardB(cv::Size(11, 8), 6.0f, "chessboardB"),
    // 虽然不连相机，但 Camera 对象仍用于存储内参矩阵等信息
    cameraA(CameraType::CAM_A, "Camera A", cv::Size(2448, 2048)),
    cameraB(CameraType::CAM_B, "Camera B", cv::Size(2448, 2048)),
    targetObject(),
    t1Loaded(false),
    t2Loaded(false)
{
    // 初始化管理器
    calibManager = new CalibrationManager(cameraA, cameraB, chessboardA, chessboardB);
    measManager = new MeasurementManager(cameraA, cameraB, targetObject, chessboardA, chessboardB);

    setupUI();

    // 尝试加载标定参数
    if (calibManager->loadCalibrationResults()) {
        statusLabel->setText(QString::fromLocal8Bit("标定参数已加载，请导入图片进行计算"));
        statusLabel->setStyleSheet("color: green;");
        if (!measManager->initialize()) {
            QMessageBox::warning(this, "Error", "Measurement Manager init failed.");
        }
    }
    else {
        statusLabel->setText(QString::fromLocal8Bit("未找到标定参数，计算可能不准确"));
        statusLabel->setStyleSheet("color: red;");
    }
}

CameraFileApp::~CameraFileApp() {
    delete calibManager;
    delete measManager;
}

void CameraFileApp::setupUI() {
    this->setWindowTitle(QString::fromLocal8Bit("多相机位姿测量系统 - 离线文件模式"));
    this->resize(1280, 900);

    QWidget* centralWidget = new QWidget(this);
    this->setCentralWidget(centralWidget);
    QHBoxLayout* mainLayout = new QHBoxLayout(centralWidget);

    // ================= 左侧：图像显示区 =================
    QVBoxLayout* leftLayout = new QVBoxLayout();

    // --- T1 时刻区域 ---
    QGroupBox* groupT1 = new QGroupBox(QString::fromLocal8Bit("初始时刻 (T1)"));
    QHBoxLayout* layoutT1 = new QHBoxLayout(groupT1);

    displayT1_CamA = new QLabel("Cam A (T1)");
    displayT1_CamA->setFixedSize(320, 240);
    displayT1_CamA->setStyleSheet("background: #333; color: #888; border: 1px solid #555;");
    displayT1_CamA->setAlignment(Qt::AlignCenter);

    displayT1_CamB = new QLabel("Cam B (T1)");
    displayT1_CamB->setFixedSize(320, 240);
    displayT1_CamB->setStyleSheet("background: #333; color: #888; border: 1px solid #555;");
    displayT1_CamB->setAlignment(Qt::AlignCenter);

    layoutT1->addWidget(displayT1_CamA);
    layoutT1->addWidget(displayT1_CamB);
    leftLayout->addWidget(groupT1);

    // --- T2 时刻区域 ---
    QGroupBox* groupT2 = new QGroupBox(QString::fromLocal8Bit("当前时刻 (T2)"));
    QHBoxLayout* layoutT2 = new QHBoxLayout(groupT2);

    displayT2_CamA = new QLabel("Cam A (T2)");
    displayT2_CamA->setFixedSize(320, 240);
    displayT2_CamA->setStyleSheet("background: #333; color: #888; border: 1px solid #555;");
    displayT2_CamA->setAlignment(Qt::AlignCenter);

    displayT2_CamB = new QLabel("Cam B (T2)");
    displayT2_CamB->setFixedSize(320, 240);
    displayT2_CamB->setStyleSheet("background: #333; color: #888; border: 1px solid #555;");
    displayT2_CamB->setAlignment(Qt::AlignCenter);

    layoutT2->addWidget(displayT2_CamA);
    layoutT2->addWidget(displayT2_CamB);
    leftLayout->addWidget(groupT2);

    // --- 结果可视化区域 ---
    QGroupBox* groupRes = new QGroupBox(QString::fromLocal8Bit("计算结果可视化"));
    QHBoxLayout* layoutRes = new QHBoxLayout(groupRes);
    resultDisplay = new QLabel(QString::fromLocal8Bit("等待计算..."));
    resultDisplay->setFixedHeight(240);
    resultDisplay->setStyleSheet("background: #222; color: #aaa; border: 1px solid #555;");
    resultDisplay->setAlignment(Qt::AlignCenter);
    layoutRes->addWidget(resultDisplay);
    leftLayout->addWidget(groupRes);

    mainLayout->addLayout(leftLayout);

    // ================= 右侧：控制与数据区 =================
    QVBoxLayout* rightLayout = new QVBoxLayout();
    QGroupBox* controlGroupBox = new QGroupBox("操作面板");
    QVBoxLayout* controlLayout = new QVBoxLayout(controlGroupBox);

    QPushButton* btnLoadT1 = new QPushButton(QString::fromLocal8Bit("1. 导入初始时刻图片 (A & B)"));
    QPushButton* btnLoadT2 = new QPushButton(QString::fromLocal8Bit("2. 导入当前时刻图片 (A & B)"));
    QPushButton* btnCalc = new QPushButton(QString::fromLocal8Bit("3. 计算位姿变化"));
    QPushButton* btnReset = new QPushButton(QString::fromLocal8Bit("重置系统"));

    btnCalc->setFixedHeight(50);
    QFont font = btnCalc->font(); font.setBold(true); font.setPointSize(12);
    btnCalc->setFont(font);

    controlLayout->addWidget(btnLoadT1);
    controlLayout->addWidget(btnLoadT2);
    controlLayout->addSpacing(10);
    controlLayout->addWidget(btnCalc);
    controlLayout->addStretch();
    controlLayout->addWidget(btnReset);
    rightLayout->addWidget(controlGroupBox);

    // 数据显示区
    QGroupBox* targetPosGroupBox = new QGroupBox(QString::fromLocal8Bit("平移 T (mm)"));
    QGridLayout* targetPosLayout = new QGridLayout(targetPosGroupBox);
    xPosEdit = new QLineEdit("0.00"); xPosEdit->setReadOnly(true);
    yPosEdit = new QLineEdit("0.00"); yPosEdit->setReadOnly(true);
    zPosEdit = new QLineEdit("0.00"); zPosEdit->setReadOnly(true);

    targetPosLayout->addWidget(new QLabel("dx:"), 0, 0); targetPosLayout->addWidget(xPosEdit, 0, 1);
    targetPosLayout->addWidget(new QLabel("dy:"), 1, 0); targetPosLayout->addWidget(yPosEdit, 1, 1);
    targetPosLayout->addWidget(new QLabel("dz:"), 2, 0); targetPosLayout->addWidget(zPosEdit, 2, 1);
    rightLayout->addWidget(targetPosGroupBox);

    QGroupBox* targetPoseGroupBox = new QGroupBox(QString::fromLocal8Bit("旋转 R (°)"));
    QGridLayout* targetPoseLayout = new QGridLayout(targetPoseGroupBox);
    alphaEdit = new QLineEdit("0.00"); alphaEdit->setReadOnly(true);
    betaEdit = new QLineEdit("0.00"); betaEdit->setReadOnly(true);
    gammaEdit = new QLineEdit("0.00"); gammaEdit->setReadOnly(true);

    targetPoseLayout->addWidget(new QLabel("Roll:"), 0, 0); targetPoseLayout->addWidget(alphaEdit, 0, 1);
    targetPoseLayout->addWidget(new QLabel("Pitch:"), 1, 0); targetPoseLayout->addWidget(betaEdit, 1, 1);
    targetPoseLayout->addWidget(new QLabel("Yaw:"), 2, 0); targetPoseLayout->addWidget(gammaEdit, 2, 1);
    rightLayout->addWidget(targetPoseGroupBox);

    statusLabel = new QLabel("Ready");
    rightLayout->addWidget(statusLabel);

    rightLayout->addStretch();
    mainLayout->addLayout(rightLayout);

    // 信号连接
    connect(btnLoadT1, &QPushButton::clicked, this, &CameraFileApp::loadT1Images);
    connect(btnLoadT2, &QPushButton::clicked, this, &CameraFileApp::loadT2Images);
    connect(btnCalc, &QPushButton::clicked, this, &CameraFileApp::calculatePoseChange);
    connect(btnReset, &QPushButton::clicked, this, &CameraFileApp::resetSystem);
}

// 辅助：安全读取图片（支持中文路径）
cv::Mat CameraFileApp::readImageSafe(const QString& filePath) {
    if (filePath.isEmpty()) return cv::Mat();

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        return cv::Mat();
    }

    // 将整个文件读入内存
    QByteArray byteArray = file.readAll();
    // 从内存解码图像
    std::vector<char> data(byteArray.data(), byteArray.data() + byteArray.size());
    cv::Mat img = cv::imdecode(data, cv::IMREAD_COLOR);

    return img;
}

// 辅助：显示图片到 Label
void CameraFileApp::displayImage(const cv::Mat& img, QLabel* label) {
    if (img.empty() || !label) return;

    cv::Mat resized;
    // 保持纵横比缩放
    float ratio = (float)img.cols / (float)img.rows;
    int w = label->width();
    int h = w / ratio;
    if (h > label->height()) {
        h = label->height();
        w = h * ratio;
    }

    cv::resize(img, resized, cv::Size(w, h));
    label->setPixmap(QPixmap::fromImage(cvMatToQImage(resized)));
}

// 加载 T1 时刻的两张图
void CameraFileApp::loadT1Images() {
    // 我们可以让用户一次选两张，或者分开选。这里演示一次选两张（按住Ctrl）
    // 或者弹出两次对话框。为了明确哪个是A哪个是B，分开选最稳妥。

    QString pathA = QFileDialog::getOpenFileName(this,
        QString::fromLocal8Bit("选择 初始时刻(T1) - 相机A 图像"), "", "Images (*.png *.jpg *.bmp)");
    if (pathA.isEmpty()) return;

    QString pathB = QFileDialog::getOpenFileName(this,
        QString::fromLocal8Bit("选择 初始时刻(T1) - 相机B 图像"), "", "Images (*.png *.jpg *.bmp)");
    if (pathB.isEmpty()) return;

    img_t1_camA = readImageSafe(pathA);
    img_t1_camB = readImageSafe(pathB);

    if (img_t1_camA.empty() || img_t1_camB.empty()) {
        QMessageBox::critical(this, "Error", QString::fromLocal8Bit("图片读取失败"));
        t1Loaded = false;
        return;
    }

    displayImage(img_t1_camA, displayT1_CamA);
    displayImage(img_t1_camB, displayT1_CamB);

    t1Loaded = true;
    statusLabel->setText(QString::fromLocal8Bit("T1 图片已加载"));
}

// 加载 T2 时刻的两张图
void CameraFileApp::loadT2Images() {
    if (!t1Loaded) {
        QMessageBox::warning(this, "Info", QString::fromLocal8Bit("建议先加载 T1 图片"));
    }

    QString pathA = QFileDialog::getOpenFileName(this,
        QString::fromLocal8Bit("选择 当前时刻(T2) - 相机A 图像"), "", "Images (*.png *.jpg *.bmp)");
    if (pathA.isEmpty()) return;

    QString pathB = QFileDialog::getOpenFileName(this,
        QString::fromLocal8Bit("选择 当前时刻(T2) - 相机B 图像"), "", "Images (*.png *.jpg *.bmp)");
    if (pathB.isEmpty()) return;

    img_t2_camA = readImageSafe(pathA);
    img_t2_camB = readImageSafe(pathB);

    if (img_t2_camA.empty() || img_t2_camB.empty()) {
        QMessageBox::critical(this, "Error", QString::fromLocal8Bit("图片读取失败"));
        t2Loaded = false;
        return;
    }

    displayImage(img_t2_camA, displayT2_CamA);
    displayImage(img_t2_camB, displayT2_CamB);

    t2Loaded = true;
    statusLabel->setText(QString::fromLocal8Bit("T1 和 T2 图片均已就绪"));
}

// 执行计算
void CameraFileApp::calculatePoseChange() {
    if (!t1Loaded || !t2Loaded) {
        QMessageBox::warning(this, "Warning", QString::fromLocal8Bit("请先加载 T1 和 T2 时刻的所有图片"));
        return;
    }

    if (calibManager->getHandEyeTransform().empty()) {
        QMessageBox::warning(this, "Error", "未加载手眼标定矩阵 (Hand-Eye Matrix Missing)");
        return;
    }

    cv::Mat plot_img_t1, plot_img_t2;
    double error_t1 = 0, error_t2 = 0;

    // 1. 计算 T1 位姿
    bool successT1 = measManager->processMeasurement(img_t1_camA, img_t1_camB, poseA_t1, poseB_t1, plot_img_t1, error_t1);
    if (!successT1) {
        QMessageBox::warning(this, "Fail", QString::fromLocal8Bit("T1 时刻目标检测失败"));
        return;
    }

    // 2. 计算 T2 位姿
    bool successT2 = measManager->processMeasurement(img_t2_camA, img_t2_camB, poseA_t2, poseB_t2, plot_img_t2, error_t2);
    if (!successT2) {
        QMessageBox::warning(this, "Fail", QString::fromLocal8Bit("T2 时刻目标检测失败"));
        return;
    }

    // 3. 计算相对变化
    measManager->calculateFinalPoseChange(
        poseA_t1.T_matrix, poseA_t2.T_matrix,
        poseB_t1.T_matrix, poseB_t2.T_matrix,
        calibManager->getHandEyeTransform(), angle, t_relative);

    // 4. 更新 UI 和结果图
    updatePoseDisplay();

    // 显示其中一张处理后的结果图（这里选择 T2 时刻的特征图作为展示）
    if (!plot_img_t2.empty()) {
        displayImage(plot_img_t2, resultDisplay);
    }

    statusLabel->setText(QString::fromLocal8Bit("计算完成"));
    QMessageBox::information(this, "Success", QString::fromLocal8Bit("计算成功！\n位姿变化已更新。"));
}

void CameraFileApp::resetSystem() {
    img_t1_camA.release(); img_t1_camB.release();
    img_t2_camA.release(); img_t2_camB.release();

    displayT1_CamA->clear(); displayT1_CamA->setText("Cam A (T1)");
    displayT1_CamB->clear(); displayT1_CamB->setText("Cam B (T1)");
    displayT2_CamA->clear(); displayT2_CamA->setText("Cam A (T2)");
    displayT2_CamB->clear(); displayT2_CamB->setText("Cam B (T2)");
    resultDisplay->clear(); resultDisplay->setText("Result");

    xPosEdit->setText("0.00"); yPosEdit->setText("0.00"); zPosEdit->setText("0.00");
    alphaEdit->setText("0.00"); betaEdit->setText("0.00"); gammaEdit->setText("0.00");

    t1Loaded = false;
    t2Loaded = false;
    statusLabel->setText("System Reset");
}

void CameraFileApp::updatePoseDisplay() {
    if (!t_relative.empty() && t_relative.rows >= 3) {
        xPosEdit->setText(QString::number(t_relative.at<double>(0, 0), 'f', 2));
        yPosEdit->setText(QString::number(t_relative.at<double>(1, 0), 'f', 2));
        zPosEdit->setText(QString::number(t_relative.at<double>(2, 0), 'f', 2));
    }
    if (!angle.empty() && angle.rows >= 3) {
        alphaEdit->setText(QString::number(angle.at<double>(0, 0), 'f', 2));
        betaEdit->setText(QString::number(angle.at<double>(1, 0), 'f', 2));
        gammaEdit->setText(QString::number(angle.at<double>(2, 0), 'f', 2));
    }
}

QImage CameraFileApp::cvMatToQImage(const cv::Mat& mat) {
    if (mat.empty()) return QImage();
    if (mat.type() == CV_8UC3) {
        // 使用 rgbSwapped 避免手动 cvtColor，效率更高
        // 注意：这里必须 copy() 否则数据随 mat 释放
        QImage img((const uchar*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_BGR888);
        return img.rgbSwapped();
    }
    else if (mat.type() == CV_8UC1) {
        QImage img((const uchar*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
        return img.copy();
    }
    return QImage();
}