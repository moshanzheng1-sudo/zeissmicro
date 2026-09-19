#include "mainwindow.h"
#include <QApplication>
#include "opencv2/opencv.hpp"


#if _MSC_VER >= 1600
#pragma execution_character_set("utf-8")

#endif

using namespace cv;
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QApplication::setStyle(QStyleFactory::create("Fusion"));//设置界面风格

    MainWindow w;


    w.show();

    return a.exec();
}
