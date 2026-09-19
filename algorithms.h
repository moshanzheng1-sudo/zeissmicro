#ifndef ALGORITHMS_H
#define ALGORITHMS_H


#include "sbaslercameracontrol.h"
#include <QFile>
#include <QProcess>
#include <iostream>
#include <QThread>
#include <QtCharts>
#include <QString>
#include <QPointF>
#include <QQueue>
#include <queue>
#include <QDebug>
#include <math.h>
#include "opencv2/opencv.hpp"
#include <QMetaType>
#include "transform_format.h"
#include <QTimer>

using namespace cv;

struct voteDynamic
{
    int index;
    int votePoseHead;
    int votePoseTail;
    int voteNumber;
    voteDynamic *pVote;

};

class algorithms : public Transform_Format
{
    Q_OBJECT

public:
    algorithms();

    float InvSqrt(float x);
    uint8_t Line_Fitting(float * Point,uint8_t num,float * k,float * b);
    uint8_t Curve_Fitting(queue<QPointF>& points, int n, cv::Mat& A);
    uint8_t cvtAdjust(queue<QPointF>& curve, int threshold, int range, int &voteEnd, vector<Point3f> &voteMax);

    void GammaTransform(cv::Mat &image, double gamma);
    void sharpnessFunction(Mat fiboImg,Mat &outImg,int funSelect,double & y);
    void unevenLightCompensate(Mat &image, int blockSize);
    double sourceTriangleCompare( const Mat& _src ,double * max_num, int * max_index, double * lengthValue,double  threshold);

signals:
    void drawCurve(double *paraCurve);
    void voteSuccess(int votePose);

};

#endif // ALGORITHMS_H
