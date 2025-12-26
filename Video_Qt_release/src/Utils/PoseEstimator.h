#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include "../CommonTypes.h"
#include "../Models/Camera.h"
#include <opencv2/calib3d.hpp> // 包含 SOLVEPNP 宏定义

// 前置声明，减少头文件依赖，加快编译
class Chessboard;

class PoseEstimator {
public:
    PoseEstimator();

    // 标准PnP求解器
    // 优化：增加了 pnpMethod 参数，允许外部指定更快的算法 (如 SOLVEPNP_IPPE)
    bool solvePnP(const std::vector<cv::Point3f>& objectPoints,
        const std::vector<cv::Point2f>& imagePoints,
        const Camera& camera,
        cv::Mat& rvec, cv::Mat& tvec,
        int pnpMethod = cv::SOLVEPNP_ITERATIVE);

    // SRPnP 接口
    bool solveSRPnP(const std::vector<cv::Point3f>& objectPoints,
        const std::vector<cv::Point2f>& imagePoints,
        const Camera& camera,
        cv::Mat& rvec, cv::Mat& tvec);

    // 优化：不再接收 imagePath (string)，改为直接接收图像数据 (Mat)
    // 避免在位姿估算核心循环中进行磁盘 I/O
    bool estimatePose(const cv::Mat& image, const Camera& camera,
        const Chessboard& chessboard, cv::Mat& rvec, cv::Mat& tvec);

    // --- 工具函数 (声明为 static，无需实例化即可调用) ---

    // 将旋转向量和平移向量转换为4x4变换矩阵
    static cv::Mat getTransformMatrix(const cv::Mat& rvec, const cv::Mat& tvec);

    // 计算从T1到T2的位姿变化量 (T_delta = T2 * T1.inv())
    static cv::Mat getPoseChange(const cv::Mat& T1, const cv::Mat& T2);
};

#endif // POSE_ESTIMATOR_HPP