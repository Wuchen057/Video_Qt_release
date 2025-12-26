#include "trajectory3dplotter.h"
#include <GL/gl.h> 
#include <cmath>
#include <opencv2/core.hpp> // 确保能处理 cv::Mat

Trajectory3DPlotter::Trajectory3DPlotter(QWidget* parent)
    : QOpenGLWidget(parent)
{
    // 初始化视图参数 (测量相机视角)
    //m_xRot = -75.0f;   // 稍微俯视
    //m_yRot = 180.0f;   // 稍微侧视
    //m_zRot = 0.0f;

    // 初始化视图参数 (俯瞰视角)
    m_xRot = -0.0f;   
    m_yRot = 180.0f;   
    m_zRot = 0.0f;

    // 关键：初始拉远距离
    m_zoom = -6000.0f;

    // 初始平移：由于OpenCV坐标系Z向前，Y向下，
    // 为了让相机在屏幕左边或中间，目标在右边或深处，通常不需要太大偏移
    m_xPan = 0.0f;
    m_yPan = 0.0f;
}

// 新增辅助函数：绘制相机视锥体模型
void Trajectory3DPlotter::drawCameraSymbol(float size)
{
    glLineWidth(2.0f);
    glColor3f(1.0f, 1.0f, 1.0f); // 白色线条代表相机

    glBegin(GL_LINES);

    // 假设相机镜头在原点 (0,0,0)，朝向 +Z 方向 (OpenCV坐标系特性)
    // 绘制一个金字塔形状表示视场

    // 4条棱
    glVertex3f(0, 0, 0); glVertex3f(size, size, size);
    glVertex3f(0, 0, 0); glVertex3f(-size, size, size);
    glVertex3f(0, 0, 0); glVertex3f(-size, -size, size);
    glVertex3f(0, 0, 0); glVertex3f(size, -size, size);

    // 底面矩形框
    glVertex3f(size, size, size); glVertex3f(-size, size, size);
    glVertex3f(-size, size, size); glVertex3f(-size, -size, size);
    glVertex3f(-size, -size, size); glVertex3f(size, -size, size);
    glVertex3f(size, -size, size); glVertex3f(size, size, size);

    // 顶部标记 (表示相机的"上方"，通常是 -Y 方向，画个小三角)
    //glVertex3f(-size / 2, -size, size); glVertex3f(0, -size * 1.5f, size);
    //glVertex3f(0, -size * 1.5f, size); glVertex3f(size / 2, -size, size);

    glEnd();

    // 在原点画一个小球代表镜头中心
    glPointSize(8.0f);
    glColor3f(0.0f, 1.0f, 1.0f); // 青色
    glBegin(GL_POINTS);
    glVertex3f(0, 0, 0);
    glEnd();

    // 绘制相机自身的坐标系 (World Origin)
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f); glVertex3f(0, 0, 0); glVertex3f(-size * 1.5f, 0, 0); // X 红
    glColor3f(0.0f, 1.0f, 0.0f); glVertex3f(0, 0, 0); glVertex3f(0, -size * 1.5f, 0); // Y 绿
    glColor3f(0.0f, 0.0f, 1.0f); glVertex3f(0, 0, 0); glVertex3f(0, 0, size * 1.5f); // Z 蓝
    glEnd();
}

void Trajectory3DPlotter::addPoint(const TrajectoryPoint& point)
{
    m_points.push_back(point);
    update(); // 触发重绘
}

void Trajectory3DPlotter::clearTrajectory()
{
    m_points.clear();
    // 重置视角（可选，根据需求决定是否重置）
    //m_xRot = -75.0f;   // 稍微俯视
    //m_yRot = 180.0f;   
    //m_zRot = 0.0f;

    // 初始化视图参数 (俯瞰视角)
    m_xRot = -0.0f;  
    m_yRot = 180.0f;   
    m_zRot = 0.0f;


    update();


}

void Trajectory3DPlotter::initializeGL()
{
    initializeOpenGLFunctions();

    // 背景色：深灰色，类似CAD软件风格
    glClearColor(0.15f, 0.15f, 0.15f, 1.0f);

    glEnable(GL_DEPTH_TEST); // 开启深度测试 (Z-Buffer)

    // 简单的光照或颜色材质设置（此处只用颜色）
    glEnable(GL_COLOR_MATERIAL);
}

void Trajectory3DPlotter::resizeGL(int w, int h)
{
    if (h == 0) h = 1;
    glViewport(0, 0, w, h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // 透视投影
    GLfloat aspect = GLfloat(w) / h;
    float zNear = 1.0f;
    float zFar = 50000.0f; // 远裁剪面设大一点，防止远处的轨迹消失
    float fov = 45.0f;

    // 手动计算透视矩阵 (替代 gluPerspective)
    float fH = tan(fov / 360.0f * 3.14159f) * zNear;
    float fW = fH * aspect;
    glFrustum(-fW, fW, -fH, fH, zNear, zFar);

    glMatrixMode(GL_MODELVIEW);
}


void Trajectory3DPlotter::paintGL()
{
    // 1. 清理
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // 2. 摄像机视角变换
    // 注意：因为目标现在是绝对坐标，Z值通常很大 (比如 500mm ~ 2000mm)
    // 所以这里的 m_zoom 需要拉得更远才能看到全貌，或者你可以根据数据自动调整
    glTranslatef(m_xPan, m_yPan, m_zoom);

    glRotatef(m_xRot, 1.0f, 0.0f, 0.0f);
    glRotatef(m_yRot, 0.0f, 1.0f, 0.0f);

    // --- 新增：绘制原点处的相机 ---
    drawCameraSymbol(200.0f); // 绘制大小为100mm的相机模型

    // 绘制地面网格 (可选，辅助观察)
    //drawGrid(); 

    if (m_points.empty()) return;

    // 3. 绘制轨迹线 (黄色)
    glLineWidth(2.0f);
    glColor3f(1.0f, 1.0f, 0.0f);
    glBegin(GL_LINE_STRIP);
    for (const auto& pt : m_points) {
        glVertex3f(pt.dx, pt.dy, pt.dz);
    }
    glEnd();

    // 4. 绘制历史点
    glPointSize(3.0f);
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < m_points.size(); ++i) {
        glVertex3f(m_points[i].dx, m_points[i].dy, m_points[i].dz);
    }
    glEnd();

    // 5. 绘制当前点的坐标轴
    //const auto& lastPt = m_points.back();
    //drawCoordinates(lastPt, 80.0f);

    for (auto Pt : m_points) {
        drawCoordinates(Pt, 200.0f);
    }

}


void Trajectory3DPlotter::drawGrid()
{
    glLineWidth(1.0f);
    glColor3f(0.4f, 0.4f, 0.4f); // 浅灰网格线

    float size = 3000.0f; // 网格范围
    float step = 200.0f;  // 网格间距

    glBegin(GL_LINES);
    // Z轴方向线
    for (float x = -size; x <= size; x += step) {
        glVertex3f(x, 0, -size);
        glVertex3f(x, 0, size);
    }
    // X轴方向线
    for (float z = -size; z <= size; z += step) {
        glVertex3f(-size, 0, z);
        glVertex3f(size, 0, z);
    }
    glEnd();

    // 绘制世界原点坐标轴 (细线，作为绝对参考)
    float axisLen = 200.0f;
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    glColor3f(0.5f, 0.0f, 0.0f); glVertex3f(0, 0, 0); glVertex3f(axisLen, 0, 0); // 暗红 X
    glColor3f(0.0f, 0.5f, 0.0f); glVertex3f(0, 0, 0); glVertex3f(0, axisLen, 0); // 暗绿 Y
    glColor3f(0.0f, 0.0f, 0.5f); glVertex3f(0, 0, 0); glVertex3f(0, 0, axisLen); // 暗蓝 Z
    glEnd();
}

void Trajectory3DPlotter::drawCoordinates(const TrajectoryPoint& pt, float axisLen)
{
    if (pt.R.empty()) return;

    // --- 关键修正：类型安全转换 ---
    // OpenCV 的 Mat 可能是 float (CV_32F) 也可能是 double (CV_64F)。
    // 如果不匹配直接用 at<double> 会导致数据错乱或崩溃。
    // 这里强制转为 double 类型矩阵进行计算。
    cv::Mat R_double;
    if (pt.R.type() != CV_64F) {
        pt.R.convertTo(R_double, CV_64F);
    }
    else {
        R_double = pt.R;
    }

    // 计算旋转后的轴向量
    // 假设 R 是旋转矩阵，列向量代表了新坐标系的轴在世界系下的方向
    // x_axis = R * [1,0,0]^T = R的第一列
    // y_axis = R * [0,1,0]^T = R的第二列
    // z_axis = R * [0,0,1]^T = R的第三列

    double xx = R_double.at<double>(0, 0) * axisLen;
    double xy = R_double.at<double>(1, 0) * axisLen;
    double xz = R_double.at<double>(2, 0) * axisLen;

    double yx = R_double.at<double>(0, 1) * axisLen;
    double yy = R_double.at<double>(1, 1) * axisLen;
    double yz = R_double.at<double>(2, 1) * axisLen;

    double zx = R_double.at<double>(0, 2) * axisLen;
    double zy = R_double.at<double>(1, 2) * axisLen;
    double zz = R_double.at<double>(2, 2) * axisLen;

    glLineWidth(4.0f); // 坐标轴画粗一点
    glBegin(GL_LINES);

    // 局部 X 轴 (鲜红)
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(pt.dx, pt.dy, pt.dz);
    glVertex3f(pt.dx + xx, pt.dy + xy, pt.dz + xz);

    // 局部 Y 轴 (鲜绿)
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(pt.dx, pt.dy, pt.dz);
    glVertex3f(pt.dx + yx, pt.dy + yy, pt.dz + yz);

    // 局部 Z 轴 (鲜蓝)
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(pt.dx, pt.dy, pt.dz);
    glVertex3f(pt.dx + zx, pt.dy + zy, pt.dz + zz);

    glEnd();

    // 绘制中心白点
    glPointSize(6.0f);
    glColor3f(1.0f, 1.0f, 1.0f);
    glBegin(GL_POINTS);
    glVertex3f(pt.dx, pt.dy, pt.dz);
    glEnd();
}

// --- 鼠标交互部分 ---

void Trajectory3DPlotter::mousePressEvent(QMouseEvent* event)
{
    m_lastPos = event->pos();
}

void Trajectory3DPlotter::mouseMoveEvent(QMouseEvent* event)
{
    int dx = event->x() - m_lastPos.x();
    int dy = event->y() - m_lastPos.y();

    if (event->buttons() & Qt::LeftButton) {
        // 左键旋转
        m_xRot += dy;
        m_yRot += dx;
        update();
    }
    else if (event->buttons() & Qt::RightButton) {
        // 右键平移
        // 调节系数 1.0f 可以改变平移灵敏度
        // 注意：平移受 zoom 影响，离得越远平移看起来越慢是正常的透视效果
        m_xPan += dx * 1.0f;
        m_yPan -= dy * 1.0f;
        update();
    }
    m_lastPos = event->pos();
}

void Trajectory3DPlotter::wheelEvent(QWheelEvent* event)
{
    int numDegrees = event->angleDelta().y() / 8;
    int numSteps = numDegrees / 15;

    // 滚轮缩放：根据当前距离动态调整缩放速度，体验更好
    float zoomSpeed = std::abs(m_zoom) * 0.1f;
    if (zoomSpeed < 10.0f) zoomSpeed = 10.0f;

    m_zoom += numSteps * zoomSpeed;

    // 限制最近距离，防止穿模到背面
    if (m_zoom > -10.0f) m_zoom = -10.0f;

    update();
}