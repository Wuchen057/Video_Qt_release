#ifndef CHESSBOARD_HPP
#define CHESSBOARD_HPP

#include "../CommonTypes.h" // 确保 CommonTypes.h 包含了 cv::Size 等OpenCV类型

class Chessboard {
public:
    // 构造函数：将参数 boardSize 明确地赋值给成员变量 boardSize_
    // 更改参数名为 boardSize，避免与成员变量名完全相同导致的潜在困惑
    Chessboard(cv::Size boardSize, float squareSize_mm, const std::string& name = "Chessboard");

    // 根据boardSize和squareSize生成3D角点坐标（Z=0）
    void generateObjectPoints();

    // 在图像中查找棋盘格角点
    bool findCorners(const cv::Mat& image, std::vector<cv::Point2f>& corners) const;
    // 在图像上绘制角点
    void drawCorners(cv::Mat& image, const std::vector<cv::Point2f>& corners) const;

    // 获取棋盘格内角点行列数
    cv::Size getBoardSize() const { return boardSize_; } // 使用 boardSize_ 作为内角点尺寸
    // 获取棋盘格方格边长
    float getSquareSize() const { return squareSize_; }

    // 如果 patternSize_ 和 boardSize_ 实际上是同一个东西，建议只保留一个。
    // 如果它们语义不同，请明确。目前来看，boardSize_ 更符合 findChessboardCorners 的 patternSize。
    // 如果你坚持要 patternSize_，那么它应该在构造函数中被初始化。
    // cv::Size getPatternSize() const { return patternSize_; } 

    // 获取棋盘格的3D点 (以棋盘格自身坐标系为原点)
    std::vector<cv::Point3f> getObjectPoints() const { return objectPoints_; }
    // 获取棋盘格名称
    std::string getName() const { return name_; }

private:
    // 建议只保留一个表示棋盘格内角点尺寸的成员变量，比如 boardSize_
    // cv::Size patternSize_; // 如果和 boardSize_ 意义重复，可以删除
    cv::Size boardSize_;    // 内角点行列数，例如 9x6 (这个才是你传给 findChessboardCorners 的)
    float squareSize_;      // 方格边长，单位（毫米或米）
    std::vector<cv::Point3f> objectPoints_; // 棋盘格在世界坐标系下的3D角点坐标
    std::string name_;               // 棋盘格的名称

};

#endif // CHESSBOARD_HPP