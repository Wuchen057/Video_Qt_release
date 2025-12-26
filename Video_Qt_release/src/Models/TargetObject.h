#ifndef TARGET_OBJECT_HPP
#define TARGET_OBJECT_HPP

#include "../CommonTypes.h"

class TargetObject {
public:
    TargetObject();
    std::vector<cv::Point3f> getReflectivePoints3D() {
        return reflectivePoints3D_;
    }

private:
    const std::vector<cv::Point3f> reflectivePoints3D_ = {
            {-0.0f,  -120.0f, 0.0f}, // ID 1
            {-0.0f,  -100.0f, 0.0f}, // ID 2
            {-0.0f,   -80.0f, 0.0f}, // ID 3

            // b部分 (ID 4-8): 沿着Y轴排列 (+Y向下)
            {  0.0f, -20.0f,  50.0f}, // ID 4
            {-20.0f,   0.0f,  50.0f}, // ID 5
            { -0.0f,   0.0f,  50.0f}, // ID 6 (中心点)
            { 20.0f,   0.0f,  50.0f}, // ID 7
            {  0.0f,  20.0f,  50.0f}, // ID 8

            // c部分 (ID 9-14): 位于+Y方向 (下方)，且有50mm深度
            {  0.0f,  80.0f, 0.0f}, // ID 9
            {-30.0f, 100.0f, 0.0f}, // ID 10
            {-10.0f, 100.0f, 0.0f}, // ID 11
            { 10.0f, 100.0f, 0.0f}, // ID 12
            { 30.0f, 100.0f, 0.0f}, // ID 13
            { -0.0f, 120.0f, 0.0f}  // ID 14
    }; // 14个圆形标记点的3D坐标
    float markerRadius_; // 圆形标记点的半径，可能用于图像处理
};

#endif // TARGET_OBJECT_HPP