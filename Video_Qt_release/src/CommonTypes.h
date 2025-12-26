#ifndef COMMON_TYPES_HPP
#define COMMON_TYPES_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

// 定义相机类型枚举
enum class CameraType {
    CAM_A, // 测量相机
    CAM_B  // 参考相机
};

// 存储位姿结果的结构体
struct PoseResult {
    cv::Mat T_matrix; // 4x4 变换矩阵 (R|t)
    cv::Mat rvec;     // 旋转向量
    cv::Mat tvec;     // 平移向量
    double timestamp; // 时间戳

    PoseResult() : timestamp(0.0) {}
};

// 定义文件路径常量或枚举，方便管理
namespace ConfigPaths {
    const std::string CAMERA_A_PARAMS = "data/config/Camera A.yml";
    const std::string CAMERA_B_PARAMS = "data/config/Camera B.yml";
    const std::string HAND_EYE_TRANSFORM = "data/config/handeye_transform.yml";
    const std::string TARGET_OBJECT_3D_POINTS = "data/config/target_object_3d_points.yml";
}

#endif // COMMON_TYPES_HPP