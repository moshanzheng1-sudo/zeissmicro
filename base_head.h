#ifndef BASE_HEAD_H
#define BASE_HEAD_H





//public

#include <QObject>
#include <iostream>
#include <QThread>
#include <QtCharts>
#include <QString>
#include <QPointF>
#include <QQueue>
#include <QDebug>
#include <math.h>
#include <QMetaType>
#include <QTimer>
#include <iostream>
#include <QTimer>
#include <QtNetwork>


//function
#include "servo_function.h"
#include "sbaslercameracontrol.h"
#include "opencv2/opencv.hpp"
#include "nditracking.h"
#include "transform_format.h"
#include "chi.h"
#include "algorithms.h"

#define cameraOpen 1
#define actOpen 1 //0:关闭；1:开1微操；2:开双微操

#define pumpOpen 1 //1：开启界面驱动 0：关闭界面驱动
#define motorOpenFlagBase 0  //1：开泵 0：可选波形发射器或膜片钳
#define voltageFromWaveGenerator 1 //1：开波形发生器 0：开膜片钳

#define auto_positioning_flag 0 //1:自动完成坐标转换 0：手动完成

#define penetrationPathSelection 2   //2:快速刺入 3：压迫式刺入


using namespace std;


#endif // BASE_HEAD_H
