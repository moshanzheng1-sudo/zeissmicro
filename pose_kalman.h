#ifndef POSE_KALMAN_H
#define POSE_KALMAN_H


#include "positioning_cell.h"
#include "servo_function.h"
#include <QThread>
#include <QMutex>
#include <sbaslercameracontrol.h>
#include <opencv2/opencv.hpp>
#include <QImage>
#include <transform_format.h>






typedef struct
{
    float x;
    float y;
    float similarity;
}Pose_XY;


class Pose_Kalman : public Transform_Format
{
    Q_OBJECT

public:
    Pose_Kalman(SBaslerCameraControl *image_control,params_struct &params);

    void decision();
    void serverInit();
    void server5();
    void server500();
    void uiShow();
    void uiShowOpen();
    void whiteAndExposure();


    void videoRecord();
    void planeMove();
    void verticalMove();
    void receiveTargetP(int receiveP);

    void receiveNowInfo(double timeControl,double current,int x,int y,int z,int d,double sendVol,double sendFre);

    void receivePositions(float centerX, float centerY);
    void penetrationSiteShow(float centerX, float centerY);

    vector<Point2f> receiveBarycenterP;
    Pose_XY * pose_xy;
    SBaslerCameraControl *Image_Control;
    int8_t manipulationSelection=1; //夹持针的微操作器选择

    QImage temporary_image;
    Transform_Format *cvtimg;
    Mat TrainImg;

    params_struct kalmanParams;
    int targetP;
    vector<Point2f> injectionSites; //质心位置
    Point2f tipPosition=Point2f(0.0,0.0);

    int funSelect=100;
    int iteratorNum=0;
    int recordFlag=-1;

    double timeNow=0,currentNow=0,sendVoltage=0,sendFrequency=0;
    int xNow=0,yNow=0,zNow=0,dNow=0;

    QTimer *timerKalman;
    QTimer* timeDecision;



signals:
    void sendImage(const QImage &img );



};

#endif // POSE_KALMAN_H
