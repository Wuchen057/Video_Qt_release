#ifndef MEASUREMENT_MANAGER_HPP
#define MEASUREMENT_MANAGER_HPP

#include "../CommonTypes.h"
#include "../Models/Camera.h"
#include "../Models/TargetObject.h"
#include "../Models/Chessboard.h"
#include "../Utils/ImageProcessor.h"
#include "../Utils/PoseEstimator.h"

#include <vector>
#include <opencv2/opencv.hpp>

// --- 优化建议：逐步淘汰 Custom 结构体，直接在项目中使用 cv::Point3f ---
// 为了兼容旧代码保留定义，但建议后续重构 Models/TargetObject.h 直接存储 cv 类型
struct Point3D_Custom {
    float x, y, z;
};

struct Point2D_Custom {
    float x, y;
};

// 优化1：改为 inline 函数，减少函数调用开销
inline std::vector<cv::Point3f> toCvPoint3f_Custom(const std::vector<Point3D_Custom>& points) {
    std::vector<cv::Point3f> cvPoints;
    cvPoints.reserve(points.size()); // 预分配内存
    for (const auto& p : points) {
        cvPoints.emplace_back(p.x, p.y, p.z);
    }
    return cvPoints;
}

inline std::vector<cv::Point2f> toCvPoint2f_Custom(const std::vector<Point2D_Custom>& points) {
    std::vector<cv::Point2f> cvPoints;
    cvPoints.reserve(points.size());
    for (const auto& p : points) {
        cvPoints.emplace_back(p.x, p.y);
    }
    return cvPoints;
}

class MeasurementManager {
public:
    MeasurementManager(Camera& camA, Camera& camB, TargetObject& targetObj, Chessboard& chessboardA, Chessboard& chessboardB);

    bool initialize();

    // 核心处理函数
    // 优化：result_img 作为可选参数（默认空），因为绘图非常耗时，如果是自动运行模式可以不传
    bool processMeasurement(const cv::Mat& camA_image, const cv::Mat& camB_image,
        PoseResult& camA_pose_curr, PoseResult& camB_pose_curr, cv::Mat& result_img, double& error);

    // 计算相对位姿变化
    bool calculateFinalPoseChange(const cv::Mat& T_CamA_Obj_curr, const cv::Mat& T_CamB_Board_curr,
        const cv::Mat& T_CamA_Board_fixed, cv::Mat& angle, cv::Mat& t_relative);

    bool calculateFinalPoseChange(const cv::Mat& T_CamA_Obj_prev, const cv::Mat& T_CamA_Obj_curr,
        const cv::Mat& T_CamB_Board_prev, const cv::Mat& T_CamB_Board_curr,
        const cv::Mat& T_CamA_Board_fixed, cv::Mat& angle, cv::Mat& t_relative);

    bool calculateFinalPoseChange(const cv::Mat& T_CamA_Obj_curr, const cv::Mat& T_CamB_Board_curr,
        const cv::Mat& T_CamA_Board_fixed, cv::Mat& T);

    void setHandEyeTransform(const cv::Mat& transform) { T_CamA_Board_fixed_ = transform.clone(); }

    void setRansacParams(int iterations, double reproj_thresh, int min_sample);

    cv::Mat rotationMatrixToEulerAngles(const cv::Mat& R); // 加 const 引用

private:
    Camera& cameraA_;
    Camera& cameraB_;
    TargetObject& targetObject_;
    Chessboard& chessboardA_;
    Chessboard& chessboardB_;

    ImageProcessor imageProcessor_;
    PoseEstimator poseEstimator_;

    int ransac_iterations_;
    double reprojection_threshold_;
    int min_sample_size_;

    cv::Mat T_CamA_Board_fixed_;

    // --- 优化2：添加成员变量作为内存缓存 (Memory Cache) ---
    // 避免在 processMeasurement 循环中反复 new/delete std::vector
    std::vector<cv::Point2f> buffer_cornersA_; // 缓存相机A检测到的角点
    std::vector<cv::Point2f> buffer_cornersB_; // 缓存相机B检测到的角点
    std::vector<cv::Point3f> buffer_objPoints_; // 缓存对应的3D点

    // 内部使用的快速PnP函数
    bool solvePosePnP(
        const std::vector<cv::Point3d>& objectPoints, // 使用 double 精度更高
        const std::vector<cv::Point2f>& imagePoints,
        const cv::Mat& cameraMatrix,
        const cv::Mat& distCoeffs,
        cv::Mat& rvec, cv::Mat& tvec
    );

    // 保留你的旧接口定义，但在 CPP 中我们会优化它的实现
    bool customRANSACPnP(
        const std::vector<cv::Point3f>& input_3d_points,
        const std::vector<cv::Point2f>& detected_2d_points,
        const Camera& camera,
        int ransac_iterations, double reprojection_threshold, int min_sample_size,
		cv::Mat& best_rvec, cv::Mat& best_tvec, double& error
    );
};

#endif // MEASUREMENT_MANAGER_HPP