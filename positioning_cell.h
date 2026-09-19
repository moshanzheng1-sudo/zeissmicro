#ifndef AUTO_FOCUS_H
#define AUTO_FOCUS_H


#include "servo_function.h"
#include "sbaslercameracontrol.h"
#include <QFile>
#include <QProcess>
#include <iostream>
#include <QThread>
#include <QtCharts>
#include <QString>
#include <QPointF>
#include <QQueue>
#include <QDebug>
#include <math.h>
#include "opencv2/opencv.hpp"
#include <QMetaType>
#include "transform_format.h"
#include "algorithms.h"
#include <QTimer>
#include "base_head.h"



using namespace cv;

#define ImageNum 150
#define Auto_Focus_Rows 1200
#define Auto_Focus_Cols 2

//尖端的x/y目标位置
#define xTarget -79.98
#define yTarget -107.36

//磁传感器到尖端的偏移量
#define xOffset -5.28
#define yOffset 28.71
#define zOffset -18.14

#define shrinkRate 0 //路径规划缩小比例

#define  imgPath  "D:/QT_space/MicroSystem/image/image_autofocusing_set/image_autofocusing_set10/%d.bmp"
#define  imgBevel "D:/QT_space/MicroSystem/image/image_autofocusing_set/bevel_model_1/%d.bmp"
//#define  imgCell "D:/QT_space/MicroSystem/image/adherent_cells/%d.bmp"
//#define  imgCell "D:/QT_space/MicroSystem/image/adad_cells/%d.bmp"
//#define  imgCell "D:/QT_space/MicroSystem/image/20X3/%d.bmp"
//#define  imgCell "D:/QT_space/MicroSystem/image/ad_cells/20X_1/%d.bmp"

#define  imgCell "D:/QT_space/MicroSystem/image/07220/0_%d.bmp"

//#define  imgPath  "F:/Huweikang/auto_focusing/10.19set/image_autofocusing_set10/%d.bmp"
const std::string autofocusVideo= "D:/QT_space/MicroSystem/image/ad_cells/20X_1.mp4";





#define tBegin 10000    //初始温度
#define EPS   1e-8    //终止温度
#define DELTA 0.98    //温度衰减率

#define LIMIT 20000   //概率选择上限
#define OLOOP 20000    //外循环次数
#define ILOOP 20000   //内循环次数
//定义路线结构体
struct injectPath
{
    vector<int> citys;
    double len;
};

enum focus_States
{   
    state_coarseAdjust=2,
    state_fineAdjust=3,
    state_cellAutofocus=4,
    state_pathPlaning=5,
    state_segmentTransform=6,
    state_imageStitching=7,
    state_cellDetection=8,
    state_penetrationSleep=9,
    state_idle=-1,
    state_init=100

};




class Auto_Focus : public algorithms
{
    Q_OBJECT
public:
    Auto_Focus(SBaslerCameraControl *Image_Control, params_struct &params);
    ~Auto_Focus();

    int8_t Auto_Focus_Slect=state_init;

    params_struct cellParamsFirst;
    params_struct verticalPosition;//针的位置
    params_struct cellPosition;
    params_struct vaguePosition;//模糊细胞位置

    params_struct cellMark;//移出位置

    int outDistance=20000;

    void decision();


    float dist(Point2f A, Point2f B);
    void GetDist(vector<Point2f> p, int n);
    injectPath GetNext(injectPath p, int n);


    void fineGetSharpness(int funSelect,int xPose, double &y);

    int8_t barycentersWrite();

    void goOut();
    void comeBack();
    void goToDefocus();
    Mat getCellImg();
    int8_t penetrationSleepChange();
    int8_t penetrationSleep();
    void mousePosition(float x,float y);
    void showImgUI(Mat imageInput);

    Point2f maxSimilarity;

    Point2f mouseRatio;
    int penetrationCollectFlag = 0;

    int AutoOpen=0;
    int8_t manipulationSelection=1; //夹持针的微操作器选择
    int flourescence=0;


    int penetrationTime=3;


    int voteEnd_;
    vector<Point3f> voteMax_;

    int tipFlag;
    vector<Point2f> barycenterAll;
    vector<Point2f> barycenterP;
    vector<Point2f> bestPathPoint;
    vector<Point2f> barycenterBare;
    vector<vector<float>> pathL;
    int nCase;
    injectPath bestPath;        //记录最优路径
    std::vector<float> shortestPath;//记录最优路径

    void send(int Show_Slect)
    {
        switch (Show_Slect) {
        case 1:
            /***斜轴检测时为线性关系***/
            /***平面检测时为清晰度曲线***/
            Chart_Param(&Array_Sharpness[0][0],len_rows,Show_Slect);
            break;
        case 2:
            /***只用于斜轴检测：显示横坐标与梯度强度关系***/
            Chart_Param(&Sobel_Win[0],len_rows,Show_Slect);
            break;
        default:
            break;
        }

    }
signals:
    void Chart_Param(const float * P_Sharpness,int i,int Show_Slect);
    void sendState(QString sState);
    void goAndGetImg(int xPose_1,Mat & fiboImg);
    void afOver(int select);
    void sendImage(const QImage & img );
    void sendPenetration();
    void sendPositions(float centerX, float centerY);




private:

    void matrixInit(Mat &matrix);


    int8_t coarseAdjust(int functionSlect, int stepS, int beginPose, int range, int threshold, int fitNumber);
    int8_t fineAdjust(int searchSelect);
    int8_t cellAutofocus();
    int8_t cellDetection();

    void barycentersGet(Mat &showImg, Mat imageRead);
    int8_t pathPlaning();
    int8_t touchPreparation();
    int8_t segmentTransform();
    int8_t imageStitching();
    void Init(int n);

    float Array_Sharpness[Auto_Focus_Rows][Auto_Focus_Cols]={{0}};//多张图片清晰度值
    float Array_Sharpness_Save[Auto_Focus_Rows]={0};//多张图片清晰度值
    float Sobel_Win[500]={0}; //单张图片局部窗口清晰度值
    uint32_t Sobel_Cols[3000]={0};//每列灰度值之和

    SBaslerCameraControl *ImgControl;

    


    int len_rows=0;//清晰度数据列数：多张为图片张数；单张为窗口个数
    float Line_k=0,Line_b=0;//标定所得斜率与截距
    int stepSize=50;//lineK是没有考虑布局得出的，所以最终结果要乘以一个步距
    cv::Mat Image_sharpness;

    QImage temporary_image;

    int maxP;

    Mat rMatrix;

    QTimer* timeAutoF;
};

#endif // AUTO_FOCUS_H
