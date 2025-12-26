#ifndef TRAJECTORY3DPLOTTER_H
#define TRAJECTORY3DPLOTTER_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <vector>
#include <opencv2/core.hpp>

// 定义一个结构体存储轨迹点信息
struct TrajectoryPoint {
    int id;
    cv::Mat t; // 平移向量 (3x1)
    cv::Mat R; // 旋转矩阵 (3x3)
    double dx, dy, dz; // 相对于初始点的位移
    double roll, pitch, yaw; // 相对于初始点的旋转
};

class Trajectory3DPlotter : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit Trajectory3DPlotter(QWidget* parent = nullptr);

    // 添加数据
    void addPoint(const TrajectoryPoint& point);
    // 清空
    void clearTrajectory();

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    // 鼠标交互
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    void drawGrid();
    void drawCoordinates(const TrajectoryPoint& pt, float axisLen);
    void drawCameraSymbol(float size);

    std::vector<TrajectoryPoint> m_points;

    // 摄像机/视图控制变量
    GLfloat m_xRot;
    GLfloat m_yRot;
    GLfloat m_zRot;
    QPoint m_lastPos;
    float m_zoom;     // 缩放距离
    float m_xPan;     // 水平平移
    float m_yPan;     // 垂直平移
};

#endif // TRAJECTORY3DPLOTTER_H