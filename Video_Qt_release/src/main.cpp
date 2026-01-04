#include "VideoPoseApp.h"
#include "IntegratedPoseTracker.h"
#include "camerafileapp.h"
#include <QtWidgets/QApplication>


int main00(int argc, char* argv[])
{
    QApplication a(argc, argv);

    // 使用新类
    VideoPoseApp w;
    w.show();

    return a.exec();
}


int main01(int argc, char* argv[])
{
    QApplication a(argc, argv);

    // 使用新类
    IntegratedPoseTracker w;
    w.show();

    return a.exec();
}


int main(int argc, char* argv[])
{
    QApplication a(argc, argv);

    // 使用新类
    CameraFileApp w;
    w.show();

    return a.exec();
}