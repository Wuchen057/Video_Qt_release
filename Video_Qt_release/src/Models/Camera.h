#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <string>
#include <opencv2/core.hpp> // 只包含必要的头文件，加快编译
#include "../CommonTypes.h"

class Camera {
public:
    Camera(CameraType type = CameraType::CAM_A, const std::string& name = "Camera", const cv::Size& imageSize = cv::Size(0, 0));

    // 从文件加载相机参数
    bool loadParams(const std::string& filename);
    // 保存相机参数到文件
    bool saveParams(const std::string& filename) const;

    // 图像去畸变 (优化：使用预计算的映射表)
    void undistortImage(const cv::Mat& src, cv::Mat& dst) const;

    // --- Getters ---
    const cv::Mat& getCameraMatrix() const { return cameraMatrix_; } // 返回 const 引用避免拷贝
    const cv::Mat& getDistCoeffs() const { return distCoeffs_; }
    cv::Size getImageSize() const { return imageSize_; }
    CameraType getType() const { return type_; }
    std::string getName() const { return name_; }

    // 设置相机参数（通常由标定过程设置）
    void setParams(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, const cv::Size& imageSize);

private:
    // 初始化去畸变映射表 (内部辅助函数)
    void initUndistortMaps();

private:
    CameraType type_;
    std::string name_;
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    cv::Size imageSize_;

    // --- 优化新增：去畸变映射缓存 ---
    // 避免每次去畸变时重新计算映射关系
    cv::Mat map1_, map2_;
};

#endif // CAMERA_HPP