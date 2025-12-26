#include "PoseEstimator.h"
#include <iostream>
#include "../Models/Chessboard.h"
#include <opencv2/calib3d.hpp> // For cv::solvePnP, Rodrigues

PoseEstimator::PoseEstimator() {
    // 构造函数
}

// ---------------------------------------------------------------------------
// 通用 PnP 求解
// ---------------------------------------------------------------------------
bool PoseEstimator::solvePnP(const std::vector<cv::Point3f>& objectPoints,
    const std::vector<cv::Point2f>& imagePoints,
    const Camera& camera,
    cv::Mat& rvec, cv::Mat& tvec,
    int pnpMethod) { // 支持外部指定算法

    // 基本检查
    if (objectPoints.size() < 4 || imagePoints.size() < 4 || objectPoints.size() != imagePoints.size()) {
        // std::cerr << "[Error] PnP input points invalid or mismatch." << std::endl;
        return false;
    }

    // 优化建议：
    // 1. 如果是平面目标（棋盘格），推荐使用 cv::SOLVEPNP_IPPE (极快且稳定)
    // 2. 如果是非平面3D目标，且点数较少，SOLVEPNP_EPNP 或 SOLVEPNP_SQPNP
    // 3. 如果含有噪声/误匹配，在外部使用 solvePnPRansac

    bool success = cv::solvePnP(objectPoints, imagePoints,
        camera.getCameraMatrix(), camera.getDistCoeffs(),
        rvec, tvec, false, pnpMethod);

    if (!success) {
        std::cerr << "[Warn] PnP failed to converge." << std::endl;
    }
    return success;
}

// ---------------------------------------------------------------------------
// SRPnP 占位符 (保持原样，但在内部复用 solvePnP)
// ---------------------------------------------------------------------------
bool PoseEstimator::solveSRPnP(const std::vector<cv::Point3f>& objectPoints,
    const std::vector<cv::Point2f>& imagePoints,
    const Camera& camera,
    cv::Mat& rvec, cv::Mat& tvec) {
    // 实际项目中应在此处调用第三方 SRPnP 库
    // 暂时回退到 EPNP，因为它对一般 3D 点云比较稳健
    return solvePnP(objectPoints, imagePoints, camera, rvec, tvec, cv::SOLVEPNP_EPNP);
}

// ---------------------------------------------------------------------------
// 工具：生成 4x4 变换矩阵 (Static)
// ---------------------------------------------------------------------------
cv::Mat PoseEstimator::getTransformMatrix(const cv::Mat& rvec, const cv::Mat& tvec) {
    cv::Mat R;
    // Rodrigues 运算量适中，无法避免，但在 CV_64F 下精度最高
    cv::Rodrigues(rvec, R);

    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);

    // 使用 ROI 赋值，避免额外的内存分配
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    tvec.copyTo(T(cv::Rect(3, 0, 1, 3)));

    return T;
}

// ---------------------------------------------------------------------------
// 工具：计算相对位姿 (Static)
// ---------------------------------------------------------------------------
cv::Mat PoseEstimator::getPoseChange(const cv::Mat& T1, const cv::Mat& T2) {
    if (T1.empty() || T2.empty()) return cv::Mat::eye(4, 4, CV_64F);

    // T_delta = T2 * T1_inv
    // OpenCV 的矩阵乘法和求逆已经针对 SIMD 优化过，直接调用即可
    return T2 * T1.inv();
}

// ---------------------------------------------------------------------------
// 估算位姿 (直接使用 cv::Mat，移除磁盘 I/O)
// ---------------------------------------------------------------------------
bool PoseEstimator::estimatePose(const cv::Mat& image, const Camera& camera,
    const Chessboard& chessboard, cv::Mat& rvec, cv::Mat& tvec) {
    if (image.empty()) return false;

    // 1. 快速检查模式检测角点
    std::vector<cv::Point2f> corners;
    int flags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK;

    bool found = cv::findChessboardCorners(image, chessboard.getBoardSize(), corners, flags);

    if (!found) return false;

    // 2. 亚像素精确化 (只在检测到之后做，且需要灰度图)
    cv::Mat gray;
    if (image.channels() == 3) cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    else gray = image;

    // 减小搜索窗口 (11->5) 和迭代次数 (30->20) 以提高速度，通常对于PnP精度足够
    cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 20, 0.01));

    std::vector<cv::Point3f> objectPoints = chessboard.getObjectPoints();

    if (objectPoints.size() != corners.size()) return false;

    // 3. 求解 PnP
    // 对于棋盘格（平面），IPPE 是最快且最准确的非迭代算法
    bool pnp_solved = cv::solvePnP(objectPoints, corners,
        camera.getCameraMatrix(), camera.getDistCoeffs(),
        rvec, tvec, false, cv::SOLVEPNP_IPPE);

    return pnp_solved;
}