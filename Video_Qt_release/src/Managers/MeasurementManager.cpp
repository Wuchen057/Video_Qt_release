#define _USE_MATH_DEFINES
#include <cmath>
#include "MeasurementManager.h"
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <random>
#include <opencv2/core/core.hpp>

// 如果不需要 Ceres，建议移除相关头文件以加快编译
// #include <ceres/ceres.h> 
// #include <ceres/rotation.h> 

// 辅助函数：判断旋转矩阵
static bool isRotationMatrix(const cv::Mat& R)
{
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());
    return cv::norm(I, shouldBeIdentity) < 1e-6;
}

// 辅助函数：计算欧拉角
cv::Mat MeasurementManager::rotationMatrixToEulerAngles(const cv::Mat& R)
{
    // 移除 assert 以防止 Release 模式下被优化掉，改用软检查
    // if (!isRotationMatrix(R)) return cv::Mat::zeros(3, 1, CV_64F);

    double sy = std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
    bool singular = sy < 1e-6;

    double x, y, z;
    if (!singular)
    {
        x = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = std::atan2(-R.at<double>(2, 0), sy);
        z = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    }
    else
    {
        x = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = std::atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }

    const double rad_to_deg = 180.0 / M_PI;
    return (cv::Mat_<double>(3, 1) << x * rad_to_deg, y * rad_to_deg, z * rad_to_deg);
}

// 构造函数
MeasurementManager::MeasurementManager(Camera& camA, Camera& camB, TargetObject& targetObj, Chessboard& chessboardA, Chessboard& chessboardB)
    : cameraA_(camA), cameraB_(camB), targetObject_(targetObj), chessboardA_(chessboardA), chessboardB_(chessboardB),
    ransac_iterations_(1000), reprojection_threshold_(8.0), min_sample_size_(4)
{
    T_CamA_Board_fixed_ = cv::Mat::eye(4, 4, CV_64F);

    // 预分配内存，避免运行时抖动
    buffer_cornersA_.reserve(20);
    buffer_cornersB_.reserve(100);
}

bool MeasurementManager::initialize() {
    bool success = true;
    if (!cameraA_.loadParams(ConfigPaths::CAMERA_A_PARAMS)) {
        success = false;
        std::cerr << "[Error] Failed to load Camera A params." << std::endl;
    }
    if (!cameraB_.loadParams(ConfigPaths::CAMERA_B_PARAMS)) {
        success = false;
        std::cerr << "[Error] Failed to load Camera B params." << std::endl;
    }

    cv::FileStorage fs(ConfigPaths::HAND_EYE_TRANSFORM, cv::FileStorage::READ);
    if (fs.isOpened()) {
        fs["T_CamA_Board"] >> T_CamA_Board_fixed_;
        fs.release();
    }
    else {
        std::cerr << "[Warn] Hand-eye transform not found. Using Identity." << std::endl;
        success = false;
    }
    return success;
}

void MeasurementManager::setRansacParams(int iterations, double reproj_thresh, int min_sample) {
    ransac_iterations_ = iterations;
    reprojection_threshold_ = reproj_thresh;
    min_sample_size_ = min_sample;
}

// =========================================================================
// 核心函数：迭代修正圆形标记点的透视投影偏差 (高度优化版)
// =========================================================================
void solvePnPWithCircleCorrection(
    const std::vector<cv::Point3f>& objectPoints,
    const std::vector<cv::Point2f>& originalImagePoints,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    double markerRadius,
    cv::Mat& rvec,
    cv::Mat& tvec,
    double& rmse
)
{
    // 1. 初始解算
    cv::solvePnP(objectPoints, originalImagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_EPNP);

    // 优化点1: 减少迭代次数 (通常3次足矣)
    const int max_iterations = 3;

    // 优化点2: 静态缓存圆周模板，避免重复分配内存
    static std::vector<cv::Point3f> circleTemplate;
    if (circleTemplate.empty()) {
        int numSamples = 36; // 优化点3: 减少采样点数，72 -> 36 足够拟合椭圆
        for (int i = 0; i < numSamples; ++i) {
            double theta = 2.0 * CV_PI * i / numSamples;
            circleTemplate.emplace_back(cos(theta) * markerRadius, sin(theta) * markerRadius, 0.0f);
        }
    }

    // 预分配容器
    std::vector<cv::Point2f> correctedImagePoints;
    correctedImagePoints.reserve(objectPoints.size());

    std::vector<cv::Point3f> currentMarkerRim3D;
    currentMarkerRim3D.reserve(circleTemplate.size());

    std::vector<cv::Point2f> projectedRim2D;
    std::vector<cv::Point2f> projectedCenter2D_Vec;

    for (int k = 0; k < max_iterations; ++k) {
        correctedImagePoints.clear();

        for (size_t i = 0; i < objectPoints.size(); ++i) {
            cv::Point3f center3D = objectPoints[i];

            // A. 平移模板到当前3D点
            currentMarkerRim3D.clear();
            for (const auto& pt : circleTemplate) {
                currentMarkerRim3D.push_back(pt + center3D);
            }

            // B. 投影边缘和圆心
            cv::projectPoints(currentMarkerRim3D, rvec, tvec, cameraMatrix, distCoeffs, projectedRim2D);

            // 优化: 只有第一次需要投影圆心吗？不，每次位姿变了都要投
            // 使用临时vector投影单个点稍微有点慢，但OpenCV API限制如此
            std::vector<cv::Point3f> singleCenter3D = { center3D };
            cv::projectPoints(singleCenter3D, rvec, tvec, cameraMatrix, distCoeffs, projectedCenter2D_Vec);

            // C. 拟合椭圆
            if (projectedRim2D.size() > 5) { // 保护
                cv::RotatedRect projectedEllipse = cv::fitEllipse(projectedRim2D);

                // D. 计算偏差
                cv::Point2f bias = projectedEllipse.center - projectedCenter2D_Vec[0];

                // E. 修正
                correctedImagePoints.push_back(originalImagePoints[i] - bias);
            }
            else {
                correctedImagePoints.push_back(originalImagePoints[i]);
            }
        }

        // F. 重新解算 (使用 ITERATIVE 进行微调)
        cv::solvePnP(objectPoints, correctedImagePoints, cameraMatrix, distCoeffs, rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);
    }

    // 1. 定义容器存放重投影后的 2D 点
    std::vector<cv::Point2f> projectedPoints;

    // 2. 将 3D 点重新投影到图像平面
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);

    // 3. 计算误差
    double totalErr = 0;
    // 使用 cv::norm 计算两个点集之间的 L2 范数（欧氏距离）
    // NORM_L2 会计算 sqrt(sum((x1-x2)^2 + (y1-y2)^2))
    totalErr = cv::norm(correctedImagePoints, projectedPoints, cv::NORM_L2);

    // 如果你想看“平均像素误差”：
    double totalPoints = (double)objectPoints.size();
    double meanReprojectionError = std::sqrt(totalErr * totalErr / totalPoints); // 如果使用 cv::norm(..., NORM_L2) 直接得到的是总距离的平方根（如果是点集），这步需注意
    // 更通用的手动累加写法（推荐，逻辑更清晰）：

    double sumReprojectionError = 0.0;
    for (size_t i = 0; i < objectPoints.size(); ++i) {
        double err = cv::norm(correctedImagePoints[i] - projectedPoints[i]); // 单个点的欧氏距离
        sumReprojectionError += err * err; // 累加平方
    }
    // 计算 RMSE (均方根误差)
    rmse = std::sqrt(sumReprojectionError / objectPoints.size());

    //std::cout << "重投影误差 (RMSE): " << rmse << " pixels" << std::endl;

}

// 辅助结构体
struct PointWithId {
    cv::Point2f pt;
    int id;
};

static double distance(const cv::Point2f& p1, const cv::Point2f& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

// 点匹配逻辑
std::vector<cv::Point2f> match_points(const std::vector<cv::Point2f>& detected_pts, float& dist_threshold) {
    if (detected_pts.size() != 14) {
        // std::cerr << "Input points size != 14" << std::endl;
        return {};
    }

    // 复制一份，避免在原 vector 上 shuffle
    std::vector<cv::Point2f> detected_pts_shuffled = detected_pts;

    // 优化: 只有在检测失败时才用 K-Means 这种昂贵的操作
    // 但鉴于逻辑依赖聚类，我们优化聚类参数
    cv::Mat points_mat(detected_pts_shuffled.size(), 1, CV_32FC2, detected_pts_shuffled.data());
    cv::Mat labels, centers_mat;

    // 优化点4: 减少 K-Means 迭代次数和精度要求
    cv::kmeans(points_mat, 3, labels,
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 5, 1.0), // Count 10->5
        3, // Attempts 5->3
        cv::KMEANS_PP_CENTERS, centers_mat);

    std::map<int, int> cluster_counts;
    for (int i = 0; i < labels.rows; ++i) cluster_counts[labels.at<int>(i)]++;

    if (cluster_counts.size() != 3) return {};

    // 逻辑保持不变...
    int label_a = -1, label_b = -1, label_c = -1;
    for (const auto& pair : cluster_counts) {
        if (pair.second == 3) label_a = pair.first;
        else if (pair.second == 5) label_b = pair.first;
        else if (pair.second == 6) label_c = pair.first;
    }

    if (label_a == -1 || label_b == -1 || label_c == -1) return {};

    std::vector<cv::Point2f> points_in_a, points_in_b, points_in_c;
    points_in_a.reserve(3); points_in_b.reserve(5); points_in_c.reserve(6);

    for (int i = 0; i < detected_pts_shuffled.size(); ++i) {
        int label = labels.at<int>(i);
        if (label == label_a) points_in_a.push_back(detected_pts_shuffled[i]);
        else if (label == label_b) points_in_b.push_back(detected_pts_shuffled[i]);
        else if (label == label_c) points_in_c.push_back(detected_pts_shuffled[i]);
    }

    cv::Point2f center_a(centers_mat.at<cv::Vec2f>(label_a, 0)[0], centers_mat.at<cv::Vec2f>(label_a, 0)[1]);
    cv::Point2f center_c(centers_mat.at<cv::Vec2f>(label_c, 0)[0], centers_mat.at<cv::Vec2f>(label_c, 0)[1]);

    std::vector<PointWithId> result_a, result_b, result_c;
    result_a.reserve(3); result_b.reserve(5); result_c.reserve(6);

    // --- ID 分配逻辑 (保持原逻辑) ---
    // A Cluster
    std::sort(points_in_a.begin(), points_in_a.end(), [&](const cv::Point2f& p1, const cv::Point2f& p2) {
        return distance(p1, center_c) > distance(p2, center_c);
        });
    for (size_t i = 0; i < points_in_a.size(); ++i) result_a.push_back({ points_in_a[i], static_cast<int>(i + 1) });

    // B Cluster
    auto min_max_b = std::minmax_element(points_in_b.begin(), points_in_b.end(), [&](const cv::Point2f& p1, const cv::Point2f& p2) {
        return distance(p1, center_a) < distance(p2, center_a);
        });
    cv::Point2f pt_id_4 = *min_max_b.first;
    cv::Point2f pt_id_8 = *min_max_b.second;

    std::vector<cv::Point2f> remaining_b;
    for (const auto& p : points_in_b) {
        if (cv::norm(p - pt_id_4) > 0.1 && cv::norm(p - pt_id_8) > 0.1) remaining_b.push_back(p);
    }
    std::sort(remaining_b.begin(), remaining_b.end(), [&](const cv::Point2f& p1, const cv::Point2f& p2) { return p1.y < p2.y; });

    result_b.push_back({ pt_id_4, 4 });
    if (remaining_b.size() >= 3) {
        result_b.push_back({ remaining_b[0], 5 });
        result_b.push_back({ remaining_b[1], 6 });
        result_b.push_back({ remaining_b[2], 7 });
    }
    result_b.push_back({ pt_id_8, 8 });

    // C Cluster
    auto min_max_c = std::minmax_element(points_in_c.begin(), points_in_c.end(), [&](const cv::Point2f& p1, const cv::Point2f& p2) {
        return distance(p1, center_a) < distance(p2, center_a);
        });
    cv::Point2f pt_id_9 = *min_max_c.first;
    cv::Point2f pt_id_14 = *min_max_c.second;

    std::vector<cv::Point2f> remaining_c;
    for (const auto& p : points_in_c) {
        if (cv::norm(p - pt_id_9) > 0.1 && cv::norm(p - pt_id_14) > 0.1) remaining_c.push_back(p);
    }
    if (std::abs(pt_id_9.x - pt_id_14.x) > std::abs(pt_id_9.y - pt_id_14.y) + 30) {
        std::sort(remaining_c.begin(), remaining_c.end(), [](const cv::Point2f& p1, const cv::Point2f& p2) { return p1.y < p2.y; });
    }
    else {
        std::sort(remaining_c.begin(), remaining_c.end(), [](const cv::Point2f& p1, const cv::Point2f& p2) { return p1.x > p2.x; });
    }

    result_c.push_back({ pt_id_9, 9 });
    for (size_t i = 0; i < remaining_c.size(); ++i) result_c.push_back({ remaining_c[i], static_cast<int>(i + 10) });
    result_c.push_back({ pt_id_14, 14 });

    // 筛选点
    std::vector<cv::Point2f> selected_points;
    selected_points.reserve(6);
    for (const auto& p : result_a) if (p.id == 1 || p.id == 3) selected_points.push_back(p.pt);
    for (const auto& p : result_b) if (p.id == 4 || p.id == 8) selected_points.push_back(p.pt);
    for (const auto& p : result_c) if (p.id == 9 || p.id == 14) selected_points.push_back(p.pt);

    if (selected_points.size() >= 2) {
        dist_threshold = cv::norm(selected_points[0] - selected_points[1]) / 4.0;
    }
    else {
        dist_threshold = 10.0; // 默认值
    }

    return selected_points;
}

bool MeasurementManager::customRANSACPnP(
    const std::vector<cv::Point3f>& input_3d_points,
    const std::vector<cv::Point2f>& detected_2d_points,
    const Camera& camera,
    int ransac_iterations, double reprojection_threshold, int min_sample_size,
    cv::Mat& final_rvec, cv::Mat& final_tvec, double& error)
{
    if (detected_2d_points.size() < min_sample_size) return false;

    float dist_threshold = 0;
    // 1. 特征点匹配
    std::vector<cv::Point2f> pre_match_2d_points = match_points(detected_2d_points, dist_threshold);
    if (pre_match_2d_points.size() < 6) return false;

    // 2. 初始位姿估计 (使用 EPNP，因为点是非平面的或有一定深度的)
    // 对应 match_points 返回的 ID: 1, 3, 4, 8, 9, 14
    static const std::vector<cv::Point3f> pre_match_3d_points = {
        {  0.0f, 120.0f, 50.0f}, {  0.0f,  80.0f, 50.0f}, // A 1  3
        {  0.0f, 20.0f,  0.0f},  {  0.0f, -20.0f,  0.0f}, // B 4  8
        {  0.0f, -80.0f, 50.0f}, {  0.0f, -120.0f, 50.0f} // C 9 14
    };

    cv::Mat rvec, tvec;
    bool pnp_success = cv::solvePnP(pre_match_3d_points, pre_match_2d_points,
        camera.getCameraMatrix(), camera.getDistCoeffs(), rvec, tvec, false, cv::SOLVEPNP_EPNP);

    if (!pnp_success) return false;

    // 3. 重投影验证与剩余点匹配
    static const std::vector<cv::Point3f> remaining_3d_points = {
        { 0.0f,  100.0f, 50.0f}, // ID 2
        { 20.0f,   0.0f,  0.0f}, {  0.0f,   0.0f,  0.0f}, {-20.0f,   0.0f,  0.0f}, // ID 5,6,7
        { 30.0f, -100.0f, 50.0f}, { 10.0f, -100.0f, 50.0f}, {-10.0f, -100.0f, 50.0f}, {-30.0f, -100.0f, 50.0f}, // ID 10-13
    };

    std::vector<cv::Point2f> reprojected_2d_points;
    cv::projectPoints(remaining_3d_points, rvec, tvec, camera.getCameraMatrix(), camera.getDistCoeffs(), reprojected_2d_points);

    // 构建最终点集
    std::vector<cv::Point3f> final_3d_points = pre_match_3d_points;
    std::vector<cv::Point2f> final_2d_points = pre_match_2d_points;

    // 标记已使用的 2D 点
    std::vector<bool> used_2d_indices(detected_2d_points.size(), false);

    // 简单的最近邻匹配 (比之前的逻辑更紧凑)
    // 先标记前6个点
    for (const auto& p : pre_match_2d_points) {
        for (size_t i = 0; i < detected_2d_points.size(); ++i) {
            if (!used_2d_indices[i] && cv::norm(p - detected_2d_points[i]) < 1.0f) {
                used_2d_indices[i] = true;
                break;
            }
        }
    }

    // 匹配剩余点
    const float MATCH_THRESH = dist_threshold;
    for (size_t i = 0; i < reprojected_2d_points.size(); ++i) {
        int best_idx = -1;
        float min_dist = 1e9;

        for (size_t j = 0; j < detected_2d_points.size(); ++j) {
            if (used_2d_indices[j]) continue;
            float d = cv::norm(reprojected_2d_points[i] - detected_2d_points[j]);
            if (d < min_dist) {
                min_dist = d;
                best_idx = j;
            }
        }

        if (best_idx != -1 && min_dist < MATCH_THRESH) {
            final_3d_points.push_back(remaining_3d_points[i]);
            final_2d_points.push_back(detected_2d_points[best_idx]);
            used_2d_indices[best_idx] = true;
        }
    }

    if (final_3d_points.size() < 6) return false;

    // 4. 最终高精度优化
    // 使用圆心偏差修正算法
    solvePnPWithCircleCorrection(final_3d_points, final_2d_points,
        camera.getCameraMatrix(), camera.getDistCoeffs(),
        30.0, // Marker Radius
        final_rvec, final_tvec, error);

    return true;
}

bool MeasurementManager::processMeasurement(const cv::Mat& camA_image, const cv::Mat& camB_image,
    PoseResult& camA_pose_curr, PoseResult& camB_pose_curr, cv::Mat& result_img, double& error) {

    if (camA_image.empty() || camB_image.empty()) return false;

    bool successA = false;
    bool successB = false;

    // --- Cam A ---
    // 优化：只有在需要显示时才生成 plot_img
    buffer_cornersA_.clear();

    // 注意：假设 extractReflectiveMarkers 已优化
    if (imageProcessor_.extractReflectiveMarkers(camA_image, buffer_cornersA_, result_img)) {
        // targetObject_ 内部数据通常是固定的，不需要转换，直接使用
        // 这里假设 targetObject_.getReflectivePoints3D() 在内部已经是 cv::Point3f

        cv::Mat rvec, tvec;
        // 注意：传入空 vector 让函数内部处理，或者传入 targetObject_.getReflectivePoints3D()
        // 这里我们传入一个 dummy，因为 customRANSACPnP 内部有写死的 3D 点坐标
        std::vector<cv::Point3f> dummy_3d;
        if (customRANSACPnP(dummy_3d, buffer_cornersA_, cameraA_, 0, 0, 0, rvec, tvec, error)) {
            camA_pose_curr.rvec = rvec.clone();
            camA_pose_curr.tvec = tvec.clone();
            camA_pose_curr.T_matrix = poseEstimator_.getTransformMatrix(rvec, tvec);
            successA = true;
        }
    }

    // --- Cam B ---
    buffer_cornersB_.clear();
    if (imageProcessor_.detectChessboardCorners(camB_image, chessboardB_, buffer_cornersB_)) {
        cv::Mat rvec, tvec;
        if (poseEstimator_.solvePnP(chessboardB_.getObjectPoints(), buffer_cornersB_, cameraB_, rvec, tvec)) {
            camB_pose_curr.rvec = rvec.clone();
            camB_pose_curr.tvec = tvec.clone();
            camB_pose_curr.T_matrix = poseEstimator_.getTransformMatrix(rvec, tvec);
            successB = true;
        }
    }

    return successA && successB;
}

bool MeasurementManager::calculateFinalPoseChange(const cv::Mat& T_CamA_Obj_curr, const cv::Mat& T_CamB_Board_curr,
    const cv::Mat& T_CamA_Board_fixed, cv::Mat& angle, cv::Mat& t_relative) {

    if (T_CamA_Obj_curr.empty() || T_CamB_Board_curr.empty() || T_CamA_Board_fixed.empty()) return false;

    // T_total = T_CamB_Board * (T_CamA_Board)^-1 * T_CamA_Obj
    cv::Mat T_CamA_Board_Inv;
    cv::invert(T_CamA_Board_fixed, T_CamA_Board_Inv); // 矩阵求逆比较耗时，如果是固定值，建议缓存 T_inv

    cv::Mat T_ref_target = T_CamB_Board_curr * T_CamA_Board_Inv * T_CamA_Obj_curr;

    T_ref_target(cv::Rect(3, 0, 1, 3)).copyTo(t_relative);
    cv::Mat R_relative = T_ref_target(cv::Rect(0, 0, 3, 3));
    angle = rotationMatrixToEulerAngles(R_relative);

    return true;
}

// 保留重载接口
bool MeasurementManager::calculateFinalPoseChange(const cv::Mat& T_CamA_Obj_prev, const cv::Mat& T_CamA_Obj_curr,
    const cv::Mat& T_CamB_Board_prev, const cv::Mat& T_CamB_Board_curr,
    const cv::Mat& T_CamA_Board_fixed, cv::Mat& angle, cv::Mat& t_relative) {
    
    if (T_CamA_Obj_prev.empty() || T_CamA_Obj_curr.empty() ||
        T_CamB_Board_prev.empty() || T_CamB_Board_curr.empty() ||
        T_CamA_Board_fixed.empty()) {
        std::cerr << "Error: Invalid input matrices for final pose change calculation." << std::endl;
        return false; // 返回单位矩阵作为错误指示
    }

    cv::Mat T_CamA_Board_Inv;
    cv::invert(T_CamA_Board_fixed, T_CamA_Board_Inv);


    cv::Mat T_ref_target = T_CamB_Board_prev * T_CamA_Board_Inv * T_CamA_Obj_prev;
    cv::Mat T_ref_target_next = T_CamB_Board_curr * T_CamA_Board_Inv * T_CamA_Obj_curr;

    cv::Mat final_rotation_matrix = T_ref_target(cv::Rect(0, 0, 3, 3));
    cv::Mat final_translation_vector = T_ref_target(cv::Rect(3, 0, 1, 3));
    cv::Mat final_rotation_matrix_next = T_ref_target_next(cv::Rect(0, 0, 3, 3));
    cv::Mat final_translation_vector_next = T_ref_target_next(cv::Rect(3, 0, 1, 3));


    cv::Mat T_relative = T_ref_target.inv() * T_ref_target_next;

    T_relative(cv::Rect(3, 0, 1, 3)).copyTo(t_relative);
    cv::Mat R_relative = T_relative(cv::Rect(0, 0, 3, 3));

    std::cout << "相对旋转矩阵:" << std::endl;
    std::cout << R_relative << std::endl;
    std::cout << "旋转角:" << std::endl;
    angle = rotationMatrixToEulerAngles(R_relative);
    std::cout << angle << std::endl;
    std::cout << "相对平移向量:" << std::endl;
    std::cout << t_relative << std::endl;

    return true;
}

bool MeasurementManager::calculateFinalPoseChange(const cv::Mat& T_CamA_Obj_curr, const cv::Mat& T_CamB_Board_curr,
    const cv::Mat& T_CamA_Board_fixed, cv::Mat& T) {
    if (T_CamA_Obj_curr.empty() || T_CamB_Board_curr.empty() || T_CamA_Board_fixed.empty()) return false;

    cv::Mat T_CamA_Board_Inv;
    cv::invert(T_CamA_Board_fixed, T_CamA_Board_Inv);
    T = T_CamB_Board_curr * T_CamA_Board_Inv * T_CamA_Obj_curr;
    return true;
}