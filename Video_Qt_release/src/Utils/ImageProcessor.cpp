#include "ImageProcessor.h"
#include <iostream>
#include <opencv2/opencv.hpp> 
#include <vector>
#include <numeric>
#include <algorithm>
#include <limits> // 确保包含 limits

// 构造函数：初始化一些固定参数
ImageProcessor::ImageProcessor() 
    : bin_threshold_(100), min_area_(100.0), max_area_(1000.0) 
{
    // 预先初始化形态学核，避免在循环中重复创建
    element_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
}

void ImageProcessor::setThresholdParams(int threshold, double minArea, double maxArea) {
    bin_threshold_ = threshold;
    min_area_ = minArea;
    max_area_ = maxArea;
}

cv::Mat removeSmallRegions(const cv::Mat& binaryImage, int minArea)
{
    if (binaryImage.empty() || binaryImage.type() != CV_8UC1) {
        std::cerr << "错误: 输入必须是单通道8位二值图像 (CV_8UC1)。" << std::endl;
        return cv::Mat();
    }

    cv::Mat labels, stats, centroids;
    // 使用8连通
    int numComponents = cv::connectedComponentsWithStats(binaryImage, labels, stats, centroids, 8, CV_32S);

    // keepLabels[i] 表示第 i 个组件是否保留
    // 优化：使用 vector<uchar> 并在栈上操作通常很快
    std::vector<uchar> keepLabels(numComponents, 0);
    for (int i = 1; i < numComponents; ++i) {
        if (stats.at<int>(i, cv::CC_STAT_AREA) >= minArea) {
            keepLabels[i] = 255;
        }
    }

    cv::Mat result = cv::Mat::zeros(binaryImage.size(), CV_8UC1);

    int rows = result.rows;
    int cols = result.cols;

    if (result.isContinuous() && labels.isContinuous()) {
        cols *= rows;
        rows = 1;
    }

    // 这是一个内存密集型操作，当前指针遍历方式已经是 OpenCV 中较优的写法
    for (int i = 0; i < rows; ++i) {
        const int* labelPtr = labels.ptr<int>(i);
        uchar* resPtr = result.ptr<uchar>(i);
        for (int j = 0; j < cols; ++j) {
            int lbl = labelPtr[j];
            if (lbl > 0) { 
                resPtr[j] = keepLabels[lbl];
            }
        }
    }

    return result;
}

cv::Mat fillInternalContours(const cv::Mat& binaryImage)
{
    if (binaryImage.empty() || binaryImage.type() != CV_8UC1) {
        return cv::Mat();
    }

    cv::Mat imageToProcess = binaryImage.clone();

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    // 优化建议：如果只关心孔洞填充，RETR_CCOMP 比 RETR_TREE 更轻量，
    // 但为了不改变算法逻辑（TREE 包含完整的树结构），此处保留 RETR_TREE
    cv::findContours(imageToProcess, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours.size(); i++) {
        if (hierarchy[i][3] != -1 && contours[i].size() < 100) {
            cv::drawContours(imageToProcess, contours, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
        }
    }

    return imageToProcess;
}


struct PointInfo {
    int grayValue;
    cv::Point2f point;
};

/**
 * @brief 优化后的亮度检测函数
 * 
 * 优化点：
 * 1. 接收外部传入的 maskBuffer，避免每次调用都重新分配内存。
 * 2. 只有在需要时才对 buffer 进行清零和绘制。
 */
bool isContourBrighterThanBackground(const cv::Mat& srcGray, 
                                     const std::vector<cv::Point>& contour, 
                                     cv::Mat& maskBuffer, // 新增：复用缓冲区
                                     int padding = 10, 
                                     double diff_thresh = 10.0) 
{
    cv::Rect bbox = cv::boundingRect(contour);

    // 扩大 ROI
    cv::Rect roiRect = (bbox + cv::Size(padding * 2, padding * 2)) - cv::Point(padding, padding);
    roiRect &= cv::Rect(0, 0, srcGray.cols, srcGray.rows);

    if (roiRect.empty()) return false;

    // 引用 ROI 数据 (O(1))
    cv::Mat roiImg = srcGray(roiRect);

    // --- 内存优化核心 ---
    // 重新调整 buffer 大小 (如果尺寸变大会重新分配，尺寸变小或不变则复用)
    maskBuffer.create(roiRect.size(), CV_8UC1); 
    maskBuffer.setTo(0); // 快速清零

    // 绘制内部 Mask (offset 设置为 -roiRect.tl() 以匹配 ROI 坐标系)
    // 这里的 vector构造有轻微开销，但 drawContours 接口限制必须传 ArrayOfArrays
    // 使用 std::vector<std::vector<cv::Point>> 的临时包装是标准做法
    // 只有 contour[0] 被绘制
    const cv::Point* pts = contour.data();
    int npts = (int)contour.size();
    // 使用 fillPoly 可能比 drawContours 略快，因为它不需要处理 hierarchy
    cv::fillPoly(maskBuffer, &pts, &npts, 1, cv::Scalar(255), 8, 0, -roiRect.tl());

    // 计算统计量
    // countNonZero 获取内部面积
    int innerArea = cv::countNonZero(maskBuffer);
    if (innerArea == 0) return false;

    // 计算内部均值
    double innerMean = cv::mean(roiImg, maskBuffer)[0];

    // 数学推导计算外部均值
    double totalSum = cv::sum(roiImg)[0];
    double totalArea = (double)roiRect.area();
    double outerArea = totalArea - innerArea;

    if (outerArea <= 0) return false;

    double innerSum = innerMean * innerArea;
    double outerMean = (totalSum - innerSum) / outerArea;

    return (innerMean > outerMean + diff_thresh);
}


void filterPointsByGraySimilarity(const cv::Mat& grayImg,
    const std::vector<cv::Point2f>& centers,
    std::vector<cv::Point2f>& centers_filter)
{
    const int TARGET_COUNT = 14;
    centers_filter.clear();

    if (centers.size() <= TARGET_COUNT) {
        centers_filter = centers;
        return;
    }
    if (grayImg.empty() || grayImg.channels() != 1) {
        return;
    }

    // 优化：一次性分配内存
    std::vector<PointInfo> pointData;
    pointData.reserve(centers.size());

    const int cols = grayImg.cols;
    const int rows = grayImg.rows;
    // 获取原始指针以加速访问（仅在极其频繁调用时有显著差异，此处保持 safe 的 at 或 ptr 即可）
    // 为了极致性能，若保证坐标在范围内，可用 ptr
    for (const auto& pt : centers) {
        int x = cvRound(pt.x);
        int y = cvRound(pt.y);

        if (x >= 0 && x < cols && y >= 0 && y < rows) {
            int val = grayImg.at<uchar>(y, x);
            pointData.push_back({ val, pt });
        }
    }

    if (pointData.size() < TARGET_COUNT) {
        for (const auto& pd : pointData) centers_filter.push_back(pd.point);
        return;
    }

    std::sort(pointData.begin(), pointData.end(),
        [](const PointInfo& a, const PointInfo& b) {
            return a.grayValue < b.grayValue;
        });

    int min_range = std::numeric_limits<int>::max();
    int best_start_idx = 0;

    const int end_idx = static_cast<int>(pointData.size()) - TARGET_COUNT;
    for (int i = 0; i <= end_idx; ++i) {
        int range = pointData[i + TARGET_COUNT - 1].grayValue - pointData[i].grayValue;
        if (range < min_range) {
            min_range = range;
            best_start_idx = i;
        }
    }

    centers_filter.reserve(TARGET_COUNT);
    for (int i = 0; i < TARGET_COUNT; ++i) {
        centers_filter.push_back(pointData[best_start_idx + i].point);
    }
}


static std::vector<cv::Point2f> filterOutliers_NearestNeighbor(
    const std::vector<cv::Point2f>& points, int k)
{
    if (points.size() <= static_cast<size_t>(k)) return points;

    static const int threshold_lut[] = {
        1000, 10000, 40000, 160000, 250000, 250000, 250000, 
        360000, 640000, 810000, 810000, 810000, 810000, 1000000 
    };

    int dist_threshold = 1000;
    if (k > 0 && k <= 13) {
        dist_threshold = threshold_lut[k];
    }
    long dist_threshold_sq = (long)dist_threshold * dist_threshold;

    std::vector<cv::Point2f> filtered_points;
    filtered_points.reserve(points.size());

    // 优化：将临时向量移出循环，避免每次外层循环都重新分配内存
    std::vector<float> dists_sq; 
    dists_sq.reserve(points.size());

    for (size_t i = 0; i < points.size(); ++i) {
        dists_sq.clear(); // 仅重置 size，不释放 capacity
        
        // 此处循环可以考虑 SIMD 优化，但编译器通常对这种简单的平方和做了优化
        for (size_t j = 0; j < points.size(); ++j) {
            if (i == j) continue;
            float dx = points[i].x - points[j].x;
            float dy = points[i].y - points[j].y;
            dists_sq.push_back(dx * dx + dy * dy);
        }

        if (dists_sq.size() < static_cast<size_t>(k)) {
            filtered_points.push_back(points[i]);
            continue;
        }

        std::nth_element(dists_sq.begin(), dists_sq.begin() + k - 1, dists_sq.end());

        if (dists_sq[k - 1] <= dist_threshold_sq) {
            filtered_points.push_back(points[i]);
        }
    }
    return filtered_points;
}


bool ImageProcessor::extractReflectiveMarkers(const cv::Mat& image, std::vector<cv::Point2f>& centers, cv::Mat& result) {
    centers.clear();
    if (image.empty()) {
        std::cerr << "Error: Input image is empty for marker extraction." << std::endl;
        return false;
    }


    if (image.channels() == 3) {
        cv::cvtColor(image, gray_cache_, cv::COLOR_BGR2GRAY);
    }
    else {
        image.copyTo(gray_cache_); // 确保不修改原图
    }

    // 2. 高斯模糊
    cv::GaussianBlur(gray_cache_, blurred_cache_, cv::Size(9, 9), 2, 2);

    // 3. 二值化
    cv::adaptiveThreshold(blurred_cache_, binary_cache_, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 17, -1);

    // 形态学处理 (getStructuringElement 内部有缓存，不需要 static)
    cv::erode(binary_cache_, binary_cache_, element_kernel_);

    // 4. 寻找轮廓
    // RETR_EXTERNAL 比 LIST 快一点点，因为不建立层级
    cv::findContours(binary_cache_, contours_cache_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    result = image.clone(); // 这里的深拷贝是必须的，因为要输出结果图

    std::vector<cv::Point2f> centers_find;
    centers_find.reserve(contours_cache_.size());

    // --- 优化：将凸包和拟合曲线的容器移出循环 ---
    std::vector<cv::Point> hull;
    std::vector<cv::Point> approxCurve;

    // 5. 遍历轮廓进行筛选
    for (size_t i = 0; i < contours_cache_.size(); i++) {
        const auto& contour = contours_cache_[i]; // 引用别名

        // --- 级联筛选 ---
        double area = cv::contourArea(contour);
        // 使用成员变量 min_area_ / max_area_ 代替硬编码 (如果需要保持原逻辑不变，请改回 100/1000)
        if (area < min_area_ || area > max_area_) continue;

        double perimeter = cv::arcLength(contour, true);
        if (perimeter == 0) continue;

        double circularity = (4 * CV_PI * area) / (perimeter * perimeter);
        if (circularity <= 0.7) continue;

        cv::convexHull(contour, hull);
        double hullArea = cv::contourArea(hull);
        double solidity = (hullArea > 0) ? (area / hullArea) : 0;
        if (solidity <= 0.92) continue;

        double epsilon = 0.02 * perimeter;
        cv::approxPolyDP(contour, approxCurve, epsilon, true);
        int numVertices = static_cast<int>(approxCurve.size());
        if (numVertices <= 6 || numVertices >= 11) continue;

        // --- 亮度检测 (关键优化) ---
        // 传入成员变量 mask_buffer_ 进行复用
        if (!isContourBrighterThanBackground(blurred_cache_, contour, mask_buffer_, 2, 5.0)) {
            continue;
        }

        // --- 提取 ---
        cv::RotatedRect ellipse = cv::fitEllipse(contour);
        centers_find.push_back(ellipse.center);

        // 绘图
        cv::drawContours(result, contours_cache_, (int)i, cv::Scalar(0, 255, 0), 2);
        cv::circle(result, ellipse.center, 3, cv::Scalar(0, 0, 255), -1);
    }

    // 后续筛选逻辑
    size_t found_count = centers_find.size();
    if (found_count < 14) return false;

    //cv::imwrite("output.jpg", result);

    
    // 如果正好14个，直接返回，避免后续不必要的拷贝和计算
    if (found_count == 14) {
        centers = std::move(centers_find); // 移动语义
        return true;
    }

    std::vector<cv::Point2f> centers_filter_1;
    filterPointsByGraySimilarity(blurred_cache_, centers_find, centers_filter_1);

    for (const auto& c : centers_filter_1) {
        cv::circle(result, c, 10, cv::Scalar(0, 0, 255), -1);
    }

    if (centers_filter_1.size() == 14) {
        centers = std::move(centers_filter_1);
        return true;
    }

    if (centers_filter_1.size() > 14) {
        int k = static_cast<int>(centers_filter_1.size()) - 14;
        std::vector<cv::Point2f> centers_filter_2 = filterOutliers_NearestNeighbor(centers_filter_1, k);

        for (const auto& c : centers_filter_2) {
            cv::circle(result, c, 30, cv::Scalar(0, 0, 255), -1);
        }

        if (centers_filter_2.size() == 14) {
            centers = std::move(centers_filter_2);
            return true;
        }
    }

    return false;
}

bool ImageProcessor::detectChessboardCorners(const cv::Mat& image, const Chessboard& chessboard, std::vector<cv::Point2f>& corners, bool useFastCheck) {
    // 这里的优化取决于 Chessboard 类内部实现，外部无法优化
    return chessboard.findCorners(image, corners);
}

void ImageProcessor::drawPoints(cv::Mat& image, const std::vector<cv::Point2f>& points, const cv::Scalar& color, int radius, int thickness) const {
    for (const auto& p : points) {
        cv::circle(image, p, radius, color, thickness);
    }
}

void ImageProcessor::drawCircles(cv::Mat& image, const std::vector<cv::Point2f>& centers, float radius, const cv::Scalar& color, int thickness) const {
    int r = static_cast<int>(radius);
    for (const auto& c : centers) {
        cv::circle(image, c, r, color, thickness);
    }
}