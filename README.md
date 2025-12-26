项目配置所需环境：
opencv的release库
Ceres的release库 https://www.cnblogs.com/zsgyx/p/10920418.html
Qt：先安装Qt（见百度网盘），然后visual studio安装Qt的插件
OpenGL:https://zhuanlan.zhihu.com/p/283696937


项目修改日志：
11.28 修改了camerasystemapp.cpp文件
将相机A和B的图像采集修改为了软件同步

12.5 
针对已知直径的圆形标记点进行位姿解算时，透视偏差（Perspective Bias） 是导致近距离测量不准的核心原因。
解决这个问题的核心思路是**“迭代修正法”**：
先假设图像上检测到的椭圆中心就是圆心的投影，解算出一个粗略的位姿。
利用这个粗略位姿，在虚拟空间中重建这个3D圆，并将其投影回图像平面。
比较“虚拟投影出的椭圆中心”与“虚拟投影出的圆心”之间的偏差向量。
利用这个偏差向量反向修正原始检测坐标，然后重新解算PnP。
新增了 refineCircularPointsAttributes 函数，用于计算并修正偏差。
详见main00函数

12.9
加入了OpenGL库的配置，实现了查看靶标的三维运动轨迹的功能，详见main02函数

12.10
项目高性能优化
架构层面 (CameraSystemApp):
并行化: 修改了 updateFrame，同时触发两台相机的 grab()，利用硬件并行性。
显示优化: 引入 Pre-Resize，在转 QImage 前先缩放图片，大幅降低 UI 线程的 CPU 占用。
解耦: 将图像采集与处理逻辑解耦，为未来可能的子线程处理打下基础。
图像处理层面 (ImageProcessor, Camera):
查表法去畸变: 在 Camera 类中使用 initUndistortRectifyMap 和 remap 替代实时的 undistort，速度提升显著。
内存池化: 在 ImageProcessor 中引入成员变量缓存（gray_cache_, contours_cache_），消除了每帧频繁的 malloc/free。
算法精简: 优化了阈值处理、轮廓查找参数（RETR_EXTERNAL），并移除了不必要的深拷贝。
计算层面 (MeasurementManager, PoseEstimator):
静态缓存: 缓存了圆心修正算法中的模板数据。
算法升级: 在 PoseEstimator 中针对平面目标引入了 SOLVEPNP_IPPE，替代了慢速的迭代法。
移除 I/O: 删除了核心计算路径上的所有文件读写和繁重的控制台打印 (cout)。
图像处理层面，加入了内亮外暗的判断

12.11
修改了ImageProcessor .cpp文件，删除了内亮外暗的判断，修正了标记点提取的流程
包括第一阶段的筛选流程（圆度，边数，凸度，轮廓面积）
以及第二阶段的筛选（灰度离群，距离离群）
现阶段的标记点提取可以在高曝光的情况下进行，相机亮度可以设置在100-500之间

12.12
修改了ImageProcessor .cpp文件，将灰度离群筛选和距离离群筛选的顺序作了调整
加入了内亮外暗的判断，从而排除背景石膏像上的标记点干扰

12.15
修改了ImageProcessor .cpp文件，提高图像处理和标记点提取的计算效率
新增 blurred_cache_：因为你的算法用了高斯模糊，这个中间结果也需要缓存。
新增 mask_buffer_：这是这次优化中提速最关键的地方，用于 isContourBrighterThanBackground 函数，避免成百上千次的内存分配。
新增 element_kernel_：缓存腐蚀操作的核，避免每帧重复调用 getStructuringElement。

12.16
新增了视频目标位姿测量功能，在文件VideoPoseApp.cpp和VideoPoseApp.h中，可以实现在打开的视频文件中测量目标的位姿
将视频目标位姿测量的功能和轨迹记录的功能相结合，在文件IntegratedPoseTracker.cpp和IntegratedPoseTracker.h中，
同时也修改了trajectory3dplotter.h和trajectory3dplotter.cpp中部分程序

12.18
加入了重投影误差的显示功能，可以显示pnp算法的重投影误差

12.22
加入了将测量的位姿信息和重投影误差信息保存到txt文件的功能，重置轨迹可以清空txt文件

12.26
修正了点匹配的程序逻辑，之前的点匹配程序由于14个标记点是关于Y轴对称，导致其出现匹配失误X轴方向出错
去除绝对坐标依赖：不再使用 p.y 或 p.x 进行排序，而是基于 “骨架向量” (Cluster A中心 -> Cluster C中心) 的相对几何关系（距离和叉积）。
这使得算法支持任意角度旋转（包括倒置）。
双假设 PnP (Hypothesis Testing)：由于初始骨架点（1,3,4,8,9,14）在3D空间中 X=0 共面，导致 PnP 存在镜像模糊。
算法现在会提取非共线的“翅膀点”（ID 5 和 7），分别假设“正放”和“镜像”两种情况进行求解，选择重投影误差最小的一组。
全点回溯匹配：确定正确姿态后，将所有14个3D点投影回图像，通过最近邻匹配修正剩余所有点的ID（特别是 ID 10-13 这种密集点）。
