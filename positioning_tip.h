#ifndef POSE_PLANE
#define POSE_PLANE

#include "positioning_cell.h"
#include <QImage>
#include<opencv2/opencv.hpp>
//#include<opencv2/optflow.hpp>
#include <QMetaType>
#include<line2Dup.h>
#include "transform_format.h"
#include "algorithms.h"
#include "base_head.h"
#include "chinstrument.h"




#include"opencv2/opencv.hpp"
#include"opencv2/core/core.hpp"
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include"opencv2/video/background_segm.hpp"



#define slect_key 0

#define Right_or_Left 0

///*injectTemplate_720*/
#define origin_offset_x 280
#define origin_offset_y 20
/*injectTemplate_720*/
//#define origin_offset_x 180
//#define origin_offset_y 20
/*injectTemplate*/
//#define origin_offset_x 325
//#define origin_offset_y 67
/*template*/
//#define origin_offset_x  490
//#define origin_offset_y  185


#define ampFactorX 1.138*0.68*(4096/720)/4
#define ampFactorY 1.121*0.675*(2168/480)/4

enum tip_States
{
    state_nonOvershootPositioning=1,
    state_coordinateTransformation=2,
    state_touchDetection=3,
    state_penetration=4,
    state_manualmove=5,
    state_angleMeasure=6,
    state_tipFocusing=7,
    state_bevelPositioning=8,
    state_moveOut=9,
    state_moveIn=10,
    state_semiAutoBiopsy=11,
    state_mogInit=12,
    state_electrochemicalMapping=13,
    state_manualmoveStep=14
//    state_idle=-1,
//    state_init=100

};


using namespace std;
using namespace line2Dup;

//template matching
const int feature_numbers=100;//特征点个数
const double matching_thread=90;//相似度阈值
const vector<int> T={4};//金字塔梯度扩散大小
static std::string prefix= "D:/QT_space/build-MicroSystem-Desktop_Qt_5_12_0_MSVC2017_64bit-Debug/";
const std::string templateImageAddress= "D:/QT_space/MicroSystem/image/line2d_probe/injectTemplate_720.bmp";
//const std::string templateImageAddress= "D:/QT_space/MicroSystem/image/line2d_probe/template.jpg";
//const std::string videoAddress= "D:/QT_space/MicroSystem/image/line2d_probe/injection_720.mp4";
//const std::string videoAddress= "D:/QT_space/MicroSystem/image/line2d_probe/Basler1025.avi";
//const std::string videoAddress= "D:/QT_space/MicroSystem/image/injection_video.avi";
//const std::string videoAddress= "D:/QT_space/MicroSystem/image/needle_double2.avi";
const std::string videoAddress= "D:/qt_space/Microsystem/image/planeMove2.avi";
//const std::string videoAddress= "D:/QT_space/MicroSystem/image/needle_psbeads.mp4";
//const std::string videoAddress= "D:/QT_space/MicroSystem/image/20x_1.mp4";
//const std::string videoAddress= "D:/QT_space/MicroSystem/image/touch_test.mp4";
const std::string sampleAddress= "D:/QT_space/MicroSystem/image/temp04193.jpg";

#define  imgmicropipette "D:/QT_space/MicroSystem/image/0310/%d.bmp"

class Pose_Plane : public algorithms
{
    Q_OBJECT

public:
    Pose_Plane( Auto_Focus *A_Focus, SBaslerCameraControl *image_control, params_struct &params, Ch_Instrument *ch_instrument);
    ~Pose_Plane(void);

    void templateTrain();
    int8_t nonOvershootPositioning();
    Mat tipPositioning_otsu_pump(double &Time_Focus,Mat originalImg);
    Mat tipPositioning_otsu(double &Time_Focus, Mat originalImg);
    Mat interfacePositioning(double &Time_Focus, Mat originalImg, Mat backgroundImg, int orientation);
    Mat tipPositioning_mog2(double &Time_Focus, Mat originalImg, int orientation, int updateFlag);//mog2算法
    Mat tipPositioningBK(double &Time_Focus, Mat originalImg, Mat backImg, int orientation);//背景相减法
    void decision();
    void touchCurrentMutation();
    void sicmData(float current, float time);
    int8_t touchDetection();
    int8_t coordinateTransformation();
    int8_t penetration();
    int8_t manualmove();
    int8_t angleMeasure();
    int8_t tipFocusing(int range, int threshold, int fitNumber);
    Mat getBackImg();

    void Data_processing(Mat * Input_Image, uint32_t * sobel_cols, float * sobel_win, int *Sobel_Max_Num);//绘制梯度曲线
    void Image_processing(Mat & src, Mat & dst, int &subNum, int8_t fineTuning);//图像处理
    int8_t bevelPositioning();
    void Go_Focus();
    int8_t semiAutoBiopsy();
    int8_t mogInit();
    int8_t electrochemicalmapping();
    int8_t penetrationSleepChange();
    void generateScanPath(int curveFlag);


    Mat Get3DR_TransMatrix(const std::vector<Point3d>& srcPoints, const std::vector<Point3d>&  dstPoints);
    void mousePosition(float x,float y);
    int8_t manualMoveControl(int axis, int direction);


    int8_t moveOut(int distance);
    int8_t moveIn(int distance);

    void showImgUI(Mat image);
    void GammaTransform(cv::Mat &image, double gamma);
    Mat updateMotionHistory(Mat src,Mat lastSrc,int magnify,double diffnum, int duration );
    Mat snakeImage(Mat image, Mat xs, Mat ys, double alpha, double beta, double gamma, double kappa, double wl, double we, double wt,int iterations);

    void zeroSet();
    void zeroReturn();


    int AutoOpen=0;
    int voteEnd_tip;
    int axisSelect;
    vector<Point3f> voteMax_tip;
    int reciprocatingStrat=0;

    int scanningStrat=0;
    int scanningPointN =0;
    double scanning_stepLength[3] = {10.0, 10.0, 10.0};// 步长
    int scanning_numSteps[3] = {3, 3, 3};   // 步数
    int scanning_axisOrder[3] = {2, 1, 0}; // 轴顺序
    vector <Point3f> scanningPoints;
    Point3f scanningPointLast = Point3f(0,0,0);

    int scanningSleepFlag = 0;
    Mat imageMonaLisa, paitingImg;


    int8_t keepState=1;
    int8_t planeSelection=state_init;
    int8_t currentMutationFlag=0;
    int8_t touchFlag=-1;//默认手动
    int8_t servoSelection = penetrationPathSelection; //PBVS 或 IBVS
    int8_t manipulationSelection=1; //夹持针的微操作器选择
    int8_t manipulationSelection_last=1;
    int mulTouchSelection=0; //夹持针的微操作器选择
    float lensMagnify=1.0; //放大倍率的选择
    int get_position_flag=0;
    int biopsyStartFlag=0;
    int MPC_Initial=1;


    double manualDistace=1;
    int manualSpeed=100;
    int manualAcc=6000;

    int outDistance=100000;

    int zUpAdd=0,zDownAdd=3;

    int penetrationTime=3;
    int penetrationSleepFlag=1;

    Mat transformMatrix;

    Mat templateImg;
    Mat backImg_original,backImg;
    int originalFlag=1;
    Ptr<BackgroundSubtractorMOG2> bg_model;

    // parameter of camera
    double min_exposure_time,max_exposure_time;
    float min_gain,max_gain;
    int image_width,image_height;
    PylonAutoInitTerm autoInitTerm;

   //parameter of track
    bool tracking_signal;
  // Detector basler_detector;
     cv::Size mask_size;
    int angle_step;
    double probe_x,probe_y;
//    double angleD=0.931306;//0.935878
    double angleD=55.0*3.1415926/180.0;//水平夹角25度安装，竖直夹角65，所以结果是65*3.14/180
    vector<Point> tipPoint;
    Point tipPointFirst;
    vector<Point> angleInfo;
    float injectionDis=18000;
    float injectionDis2=3500;

    Point2f tipViaPBVS;//更新位置：1.鼠标点击后换算得到，2.刺入动作换算得到

    Point2f penetrationSiteShow;

    params_struct sicmParams;
    params_struct line2DParams;
    params_struct line2DParamsFirst;
    params_struct  mappingParams;
    params_struct zeroParams;

    params_struct poseReciprocating;
    params_struct poseScanning;

    params_struct markParams;//离开前的位置

    params_struct touchPosition;//接触检测时针尖微操作器的位置
    params_struct tipClearPosition;//接触检测时针尖微操作器的位置
    params_struct vagueCell;//细胞最优平面时细胞微操作器的位置

    vector<Point2f> barycenterPFinal; //质心位置
    vector<Point2f> barycenterBFinal; //质心位置
    injectPath bestPathFinal;        //记录最优路径

    Ch_Instrument *ch_instrument_pose;

    Point2f mouseRatio;
    QImage imgUI;

    Point2f tipRight;

    QQueue<double> deltaDisQueue;
    double minDis=0;
    double maxDis=1000;
    double sumDeltaDis=0;
    double aveDeltaDis=0;

    //半自动电润湿
    double interfacePosition=0.0;
    int interfacePositionChange=0;
    //Motor
    int8 motorOpenFlag=motorOpenFlagBase;
    int onlyInterfaceShowFlag=0;


    //图片保存
    int8 imageSavaFlag=0;

signals:
    void sendImage(const QImage &img );
    void sendState(QString sState);
    void sendTarget(int targetP);
    void sendVoltage(double sendVoltage);
    void sendDigiVoltage(int port, double sendVoltage);
    void sendPenetrationSleep();
    void sendPosShow(float centerX, float centerY);
    void sendNowInfo(double timeControl,double current,int x,int y,int z,int d,double sendVol,double sendFre);
    void sendTargetPose(int x, int y, int z, int d);
    void manualZMoveCompleted(int direction, int manipulator);

//protected:
//    void run();

private:
    SBaslerCameraControl *Image_Control;
    bool CameraRuning;
    bool CameraOpening;

    QTimer * timePoseXY;
    QImage temporary_image;
    cv::Mat OpencvImage;
    cv::Mat mask;
    vector<cv::Point2f> radius;
    double noodel_x_position,noodel_y_position;



};

#endif
