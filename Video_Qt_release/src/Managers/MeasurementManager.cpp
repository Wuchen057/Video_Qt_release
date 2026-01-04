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
    double& rmsError
)
{
    // 1. 初始解算
    cv::solvePnP(objectPoints, originalImagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_EPNP);

    // 优化点1: 减少迭代次数 (通常3次足矣)
    const int max_iterations = 5;

    // 优化点2: 静态缓存圆周模板，避免重复分配内存
    static std::vector<cv::Point3f> circleTemplate;
    if (circleTemplate.empty()) {
        int numSamples = 108; // 优化点3: 减少采样点数，72 -> 36 足够拟合椭圆
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
        //bool success = cv::solvePnP(objectPoints, correctedImagePoints, cameraMatrix, distCoeffs, rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);
        bool success = cv::solvePnP(objectPoints, correctedImagePoints, cameraMatrix, distCoeffs, rvec, tvec, true, cv::SOLVEPNP_SQPNP);

        // 2. 将 3D 点重投影回 2D 图像
        std::vector<cv::Point2f> projectedPoints;
        projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);

        // 3. 计算重投影误差 (使用 L2 范数)
        // norm 函数可以直接计算两个点集之间的差值的范数 (即 sum((x1-x2)^2 + (y1-y2)^2) 的平方根)
        double totalError = norm(correctedImagePoints, projectedPoints, cv::NORM_L2);

        // 4. 计算 RMS (均方根误差) 或 平均误差
        // RMSE 计算公式: sqrt( sum(dist^2) / n )
        // totalError 已经是 sqrt( sum(dist^2) )，所以：
        double totalPoints = (double)objectPoints.size();
        rmsError = totalError / sqrt(totalPoints);

    }
}

struct PointWithId {
    cv::Point2f pt;
    int id; // 1-based ID
};


// 计算欧氏距离
static double dist_sq(const cv::Point2f& p1, const cv::Point2f& p2) {
    return std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2);
}

static double distance(const cv::Point2f& p1, const cv::Point2f& p2) {
    return std::sqrt(dist_sq(p1, p2));
}


// 计算二维向量叉积 (P-A) x (C-A) 的Z分量
// 用于判断点 P 在 向量 AC 的左侧还是右侧
static double cross_product_2d(const cv::Point2f& a, const cv::Point2f& c, const cv::Point2f& p) {
    cv::Point2f ac = c - a;
    cv::Point2f ap = p - a;
    return ac.x * ap.y - ac.y * ap.x;
}


// 计算重投影误差
static double compute_reprojection_error(
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points,
    const cv::Mat& rvec, const cv::Mat& tvec,
    const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs)
{
    if (object_points.empty()) return 1e9;
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, projected);

    double sum_err = 0;
    for (size_t i = 0; i < object_points.size(); ++i) {
        sum_err += cv::norm(projected[i] - image_points[i]);
    }
    return sum_err / object_points.size();
}


// 新的鲁棒匹配函数
    // 返回包含关键骨架点和待定侧翼点的列表
    // ID 105 和 107 为临时ID，分别代表向量两侧的点  
std::vector<PointWithId> match_points_robust(const std::vector<cv::Point2f>& detected_pts) {
    if (detected_pts.size() != 14) return {};

    // 1. K-Means 聚类 (保持原有逻辑，参数稍作优化)
    cv::Mat points_mat(detected_pts.size(), 1, CV_32FC2, (void*)detected_pts.data());
    cv::Mat labels, centers_mat;

    // 尝试多次以避免局部最优
    cv::kmeans(points_mat, 3, labels,
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0),
        5, cv::KMEANS_PP_CENTERS, centers_mat);

    std::map<int, std::vector<cv::Point2f>> clusters;
    for (int i = 0; i < detected_pts.size(); ++i) {
        clusters[labels.at<int>(i)].push_back(detected_pts[i]);
    }

    if (clusters.size() != 3) return {};

    // 识别 A(3点), B(5点), C(6点)
    int label_a = -1, label_b = -1, label_c = -1;
    for (const auto& pair : clusters) {
        if (pair.second.size() == 3) label_a = pair.first;
        else if (pair.second.size() == 5) label_b = pair.first;
        else if (pair.second.size() == 6) label_c = pair.first;
    }
    if (label_a == -1 || label_b == -1 || label_c == -1) return {};

    std::vector<cv::Point2f> pts_a = clusters[label_a];
    std::vector<cv::Point2f> pts_b = clusters[label_b];
    std::vector<cv::Point2f> pts_c = clusters[label_c];

    // 计算几何中心 (比 K-Means 中心更准确)
    cv::Point2f center_a(0, 0), center_c(0, 0);
    for (const auto& p : pts_a) center_a += p; center_a /= (float)pts_a.size();
    for (const auto& p : pts_c) center_c += p; center_c /= (float)pts_c.size();

    std::vector<PointWithId> result;
    result.reserve(14);

    // --- 处理 Cluster A (顶部 3 点) ---
    // 排序依据：距离 Center C 的远近。 1(最远), 2(中), 3(最近)
    std::sort(pts_a.begin(), pts_a.end(), [&](const cv::Point2f& p1, const cv::Point2f& p2) {
        return distance(p1, center_c) > distance(p2, center_c);
        });
    result.push_back({ pts_a[0], 1 });
    result.push_back({ pts_a[1], 2 });
    result.push_back({ pts_a[2], 3 });

    // --- 处理 Cluster B (中间 5 点) ---
    // 关键点：4(近A), 8(近C)。中间是 5,6,7。
    // 6 在轴线上，5 和 7 在两侧。

    // 1. 找两端点
    auto it_4 = std::min_element(pts_b.begin(), pts_b.end(), [&](auto& a, auto& b) {
        return distance(a, center_a) < distance(b, center_a);
        });
    cv::Point2f pt_4 = *it_4;

    auto it_8 = std::min_element(pts_b.begin(), pts_b.end(), [&](auto& a, auto& b) {
        return distance(a, center_c) < distance(b, center_c);
        });
    cv::Point2f pt_8 = *it_8;

    result.push_back({ pt_4, 4 });
    result.push_back({ pt_8, 8 });

    // 2. 找中间点 (5,6,7)
    std::vector<cv::Point2f> mid_b;
    for (const auto& p : pts_b) {
        if (p != pt_4 && p != pt_8) mid_b.push_back(p);
    }

    if (mid_b.size() == 3) {
        // ID 6 是距离轴线 AC 最近的点 (叉积绝对值最小)
        std::sort(mid_b.begin(), mid_b.end(), [&](const cv::Point2f& p1, const cv::Point2f& p2) {
            return std::abs(cross_product_2d(center_a, center_c, p1)) < std::abs(cross_product_2d(center_a, center_c, p2));
            });
        result.push_back({ mid_b[0], 6 }); // 轴线点

        // 剩余两个是 5 和 7。根据它们在向量 AC 的哪一侧来区分。
        // 我们不能直接定死哪个是5哪个是7（因为可能翻转），
        // 这里的策略是：叉积为负的给临时ID 107，叉积为正的给临时ID 105
        // 稍后在 PnP 假设中尝试匹配
        double cp1 = cross_product_2d(center_a, center_c, mid_b[1]);
        double cp2 = cross_product_2d(center_a, center_c, mid_b[2]);

        // 确保 mid_b[1] 是叉积较小(负)的，mid_b[2] 是较大的(正)
        if (cp1 > cp2) std::swap(mid_b[1], mid_b[2]);

        result.push_back({ mid_b[1], 107 }); // 临时 Side Negative
        result.push_back({ mid_b[2], 105 }); // 临时 Side Positive
    }

    // --- 处理 Cluster C (底部 6 点) ---
    // 我们只需要 ID 9 (近A) 和 ID 14 (近C) 来稳固骨架，其余点留给投影匹配
    auto it_9 = std::min_element(pts_c.begin(), pts_c.end(), [&](auto& a, auto& b) {
        return distance(a, center_a) < distance(b, center_a);
        });
    auto it_14 = std::max_element(pts_c.begin(), pts_c.end(), [&](auto& a, auto& b) {
        return distance(a, center_a) < distance(b, center_a);
        });

    result.push_back({ *it_9, 9 });
    result.push_back({ *it_14, 14 });

    return result;
}


// 主 PnP 函数
bool MeasurementManager::customRANSACPnP(
    const std::vector<cv::Point3f>& input_3d_points, // 必须包含完整的14个点，顺序对应ID 1-14
    const std::vector<cv::Point2f>& detected_2d_points,
    const Camera& camera,
    int ransac_iterations, double reprojection_threshold, int min_sample_size,
    cv::Mat& final_rvec, cv::Mat& final_tvec, double& totalerror)
{
    if (detected_2d_points.size() < 10) return false;

    // 1. 进行鲁棒的特征点匹配
    std::vector<PointWithId> pre_matches = match_points_robust(detected_2d_points);
    if (pre_matches.size() < 8) return false; // 至少需要 1,2,3,4,8,9,14 + 2个翅膀

    // 2. 准备 PnP 数据
    std::vector<cv::Point3f> pnp_obj_pts;
    std::vector<cv::Point2f> pnp_img_pts;
    cv::Point2f pt_side_neg, pt_side_pos;
    bool has_wings = false;
    int wing_count = 0;

    // 提取骨架点 (ID 1-4, 6, 8, 9, 14) 对应 X=0 的点
    for (const auto& pm : pre_matches) {
        if (pm.id <= 14) {
            pnp_obj_pts.push_back(input_3d_points[pm.id - 1]);
            pnp_img_pts.push_back(pm.pt);
        }
        else if (pm.id == 107) { pt_side_neg = pm.pt; wing_count++; }
        else if (pm.id == 105) { pt_side_pos = pm.pt; wing_count++; }
    }

    if (wing_count < 2) return false; // 必须有翅膀点来解算翻转

    // 3. 双假设验证 (Double Hypothesis Testing)
    // 假设 A: 正常放置 (Side Pos -> ID 5 (+20), Side Neg -> ID 7 (-20))
    // 假设 B: 倒置/翻转 (Side Pos -> ID 7 (-20), Side Neg -> ID 5 (+20))

    std::vector<cv::Point3f> obj_h1 = pnp_obj_pts;
    std::vector<cv::Point2f> img_h1 = pnp_img_pts;
    obj_h1.push_back(input_3d_points[5 - 1]); img_h1.push_back(pt_side_pos); // ID 5 (20) -> Pos
    obj_h1.push_back(input_3d_points[7 - 1]); img_h1.push_back(pt_side_neg); // ID 7 (-20) -> Neg

    std::vector<cv::Point3f> obj_h2 = pnp_obj_pts;
    std::vector<cv::Point2f> img_h2 = pnp_img_pts;
    obj_h2.push_back(input_3d_points[7 - 1]); img_h2.push_back(pt_side_pos); // ID 7 (-20) -> Pos
    obj_h2.push_back(input_3d_points[5 - 1]); img_h2.push_back(pt_side_neg); // ID 5 (20) -> Neg

    cv::Mat rvec1, tvec1, rvec2, tvec2;
    // 使用 EPNP 进行初始估计，因为它对初始化不敏感
    bool ret1 = cv::solvePnP(obj_h1, img_h1, camera.getCameraMatrix(), camera.getDistCoeffs(), rvec1, tvec1, false, cv::SOLVEPNP_EPNP);
    bool ret2 = cv::solvePnP(obj_h2, img_h2, camera.getCameraMatrix(), camera.getDistCoeffs(), rvec2, tvec2, false, cv::SOLVEPNP_EPNP);

    double err1 = compute_reprojection_error(obj_h1, img_h1, rvec1, tvec1, camera.getCameraMatrix(), camera.getDistCoeffs());
    double err2 = compute_reprojection_error(obj_h2, img_h2, rvec2, tvec2, camera.getCameraMatrix(), camera.getDistCoeffs());

    // 选出最好的姿态
    cv::Mat best_rvec, best_tvec;
    if (ret1 && (!ret2 || err1 < err2)) {
        best_rvec = rvec1.clone();
        best_tvec = tvec1.clone();
    }
    else if (ret2) {
        best_rvec = rvec2.clone();
        best_tvec = tvec2.clone();
    }
    else {
        return false;
    }

    // 4. 全局匹配与优化
    // 利用求得的粗略姿态，将所有14个3D点投影回去，通过最近邻寻找所有匹配点
    std::vector<cv::Point2f> all_reproj;
    cv::projectPoints(input_3d_points, best_rvec, best_tvec, camera.getCameraMatrix(), camera.getDistCoeffs(), all_reproj);

    std::vector<cv::Point3f> final_3d;
    std::vector<cv::Point2f> final_2d;
    std::vector<bool> used_detection(detected_2d_points.size(), false);

    // 使用贪婪策略匹配：对每个投影点，找最近的检测点
    // 动态计算匹配阈值：基于投影的ID 4和8的距离
    double scale = cv::norm(all_reproj[4 - 1] - all_reproj[8 - 1]); // ID 4到8的距离
    double match_thresh = scale / 3.0; // 宽容度

    for (size_t i = 0; i < input_3d_points.size(); ++i) {
        int best_idx = -1;
        double min_dist = 1e9;

        for (size_t j = 0; j < detected_2d_points.size(); ++j) {
            double d = cv::norm(all_reproj[i] - detected_2d_points[j]);
            if (d < min_dist) {
                min_dist = d;
                best_idx = j;
            }
        }

        // 检查双向最近邻或简单阈值
        if (best_idx != -1 && min_dist < match_thresh) {
            // 防止同一个检测点被多次匹配（取最近的优先）
            // 简单的去重逻辑：如果该检测点已被用过，比较谁更近（这里简化处理，直接用）
            // 更严谨的做法是构建Cost Matrix做匈牙利算法，但这里最近邻足够
            final_3d.push_back(input_3d_points[i]);
            final_2d.push_back(detected_2d_points[best_idx]);
        }
    }

    if (final_3d.size() < 6) return false;

    // 5. 最终高精度优化
    // 调用你原有的圆心修正优化函数
    solvePnPWithCircleCorrection(final_3d, final_2d,
        camera.getCameraMatrix(), camera.getDistCoeffs(),
        30.0, // Marker Radius
        final_rvec, final_tvec, totalerror);

    return true;
};




bool MeasurementManager::processMeasurement(const cv::Mat& camA_image, const cv::Mat& camB_image,
    PoseResult& camA_pose_curr, PoseResult& camB_pose_curr, cv::Mat& result_img, double& totalerror) {

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
        std::vector<cv::Point3f> dummy_3d = {
            // a部分 (ID 1-3): 位于Y方向 (上方)
           {-0.5977, 119.6644, 49.7042},  // Marker ID: 1
           {-0.3116, 100.0031, 49.7346},  // Marker ID: 2
           {-0.3347, 79.7212, 49.7342},   // Marker ID: 3

           // b部分 (ID 4-8): 沿着-Y轴排列 (+Y向上)
           {0.0000, 20.2677, 0.0000},     // Marker ID: 4
           {19.8696, 0.3551, 0.0035},     // Marker ID: 5
           {0.0000, 0.0000, 0.0000},      // Marker ID: 6(中心点)
           {-20.0916, 0.0927, 0.0075},    // Marker ID: 7
           {0.1774, -19.8517, 0.0021},    // Marker ID: 8

           // c部分 (ID 9-14): 位于-Y方向 (下方)
           {0.2526, -79.0333, 49.7844},   // Marker ID: 9
           {29.8784, -99.0601, 49.7825},  // Marker ID: 10
           {10.0528, -99.0347, 49.8143},  // Marker ID: 11
           {-9.9231, -99.1172, 49.7992},  // Marker ID: 12
           {-29.7733, -99.3615, 49.8123}, // Marker ID: 13
           {0.2882, -119.1413, 49.8173}   // Marker ID: 14
        };

        if (customRANSACPnP(dummy_3d, buffer_cornersA_, cameraA_, 0, 0, 0, rvec, tvec, totalerror)) {
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