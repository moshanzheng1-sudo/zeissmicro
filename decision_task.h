#ifndef DECISION_TASK_H
#define DECISION_TASK_H


#include "positioning_cell.h"
#include "sbaslercameracontrol.h"
#include "opencv2/opencv.hpp"
#include "positioning_tip.h"
#include "ui_frame.h"
#include <QThread>
#include "pose_kalman.h"
#include "chinstrument.h"
#include "patchclamp.h"

class Decision_Task: public QObject
{
    Q_OBJECT
public:
    Decision_Task();
    void decision_task_init();
    void decision_task_run();
    void decisionSelect(int8_t select);

//    void upMotorControl();
//    void downMotorControl();
    void penetrationDepthControl(int depth);
    void penetrationTimeControl(int time);
    void liftheightControl(int height);
    void manualMoveControl(int axis, int direction);
    void ZeroSet();
    void ZeroReturn();


////    /*全自动: 自动聚焦*/
//    vector<Point2i> stateHandle={{state_idle,state_tipFocusing},{state_idle,state_coordinateTransformation},
//                                 {state_idle,state_moveOut},{state_coarseAdjust,state_idle},
//                                 {state_pathPlaning,state_idle},
//                                 {state_idle,state_moveIn},{state_idle,state_touchDetection},{state_idle,state_coordinateTransformation},
//                                 {state_idle,state_penetration},{state_idle,state_idle}};//其中x是细胞的状态，y是针尖的状态

    ////    /*全自动: 手动聚焦*/
        vector<Point2i> stateHandle={{state_idle,state_coordinateTransformation},{state_idle,state_moveOut},
                                     {state_pathPlaning,state_idle},{state_idle,state_moveIn},
                                     {state_idle,state_manualmove},{state_idle,state_touchDetection},
                                     {state_idle,state_coordinateTransformation},{state_idle,state_penetration},
                                     {state_idle,state_idle}};//其中x是细胞的状态，y是针尖的状态
//        ////    /*全自动: 手动聚焦*/
//            vector<Point2i> stateHandle={{state_pathPlaning,state_idle},{state_idle,state_moveIn},
//                                         {state_idle,state_manualmove},{state_idle,state_touchDetection},
//                                         {state_idle,state_coordinateTransformation},{state_idle,state_penetration},
//                                         {state_idle,state_idle}};//其中x是细胞的状态，y是针尖的状态

    ////    /*全自动: 手动聚焦*/
//        vector<Point2i> stateHandle={{state_idle,state_touchDetection},{state_idle,state_idle}};//其中x是细胞的状态，y是针尖的状态

    /*全自动: 自动聚焦*/
//    vector<Point2i> stateHandle={{state_idle,state_penetration},{state_idle,state_idle}};//其中x是细胞的状态，y是针尖的状态


    /*全自动：基于手动聚焦*/
    /*vector<Point2i> stateHandle={{state_idle,state_coordinateTransformation},{state_idle,state_moveOut},{state_pathPlaning,state_idle},
                                 {state_idle,state_moveIn},{state_idle,state_touchDetection},{state_idle,state_penetration},
                                 {state_idle,state_idle}};*///其中x是细胞的状态，y是针尖的状态
    /*路径规划调试*/
//    vector<Point2i> stateHandle={{state_coarseAdjust,state_idle},{state_fineAdjust,state_idle},
//                                 {state_pathPlaning,state_idle},{state_idle,state_idle}};//其中x是细胞的状态，y是针尖的状态


    /*测试*/
//    vector<Point2i> stateHandle={{state_pathPlaning,state_idle},{state_idle,state_idle}};//其中x是细胞的状态，y是针尖的状态

    /*测试*/
//    vector<Point2i> stateHandle={{state_idle,state_manualmove},{state_idle,state_idle}};//其中x是细胞的状态，y是针尖的状态


    /*测试*/
//    vector<Point2i> stateHandle={{state_idle,state_angleMeasure},{state_idle,state_idle}};//其中x是细胞的状态，y是针尖的状态


//        //    /*半自动注射*/
//            vector<Point2i> stateHandleSemi={{state_idle,state_manualmove},{state_idle,state_idle}};//其中x是细胞的状态，y是针尖的状态

    /*半自动注射*/
    vector<Point2i> stateHandleSemi={{state_idle,state_coordinateTransformation},{state_idle,state_manualmove},
                                 {state_idle,state_touchDetection},{state_idle,state_coordinateTransformation},{state_idle,state_penetration},
                                 {state_idle,state_touchDetection},{state_idle,state_penetration},
                                 {state_idle,state_touchDetection},{state_idle,state_penetration},
                                 {state_idle,state_touchDetection},{state_idle,state_penetration},
                                 {state_idle,state_touchDetection},{state_idle,state_penetration},
                                 {state_idle,state_touchDetection},{state_idle,state_penetration},
                                 {state_idle,state_idle}};//其中x是细胞的状态，y是针尖的状态
//    vector<Point2i> stateHandleSemi={{state_idle,state_coordinateTransformation},{state_idle,state_touchDetection},{state_idle,state_penetration},
//                                 {state_idle,state_touchDetection},{state_idle,state_penetration},
//                                 {state_idle,state_touchDetection},{state_idle,state_penetration},
//                                 {state_idle,state_touchDetection},{state_idle,state_penetration},
//                                 {state_idle,state_touchDetection},{state_idle,state_penetration},
//                                 {state_idle,state_touchDetection},{state_idle,state_penetration},
//                                 {state_idle,state_idle}};//其中x是细胞的状态，y是针尖的状态


//        vector<Point2i> stateHandleSemi={{state_idle,state_coordinateTransformation},{state_idle,state_manualmove},
//                                     {state_idle,state_touchDetection},{state_idle,state_coordinateTransformation},{state_idle,state_electrochemicalMapping},
//                                     {state_idle,state_touchDetection},{state_idle,state_penetration},
//                                     {state_idle,state_touchDetection},{state_idle,state_penetration},
//                                     {state_idle,state_touchDetection},{state_idle,state_penetration},
//                                     {state_idle,state_touchDetection},{state_idle,state_penetration},
//                                     {state_idle,state_touchDetection},{state_idle,state_penetration},
//                                     {state_idle,state_idle}};//其中x是细胞的状态，y是针尖的状态



    
    Pose_Plane * pose_plane;
    SBaslerCameraControl *m_control;
    Auto_Focus *A_Focus;
    Ch_Instrument *ch_instrument;
    patchclamp *patchClampX;
    void * Window=nullptr;
    Pose_Kalman *imageCollect;
  
    QTimer *timer;
    cv::VideoCapture capture;
    ui_frame *U_Frame;
    params_struct params;

    int8_t stopBit=state_idle;
    int8_t funSelect=state_init;
    QTimer* timeDecision;

    std::string com;
    
};

#endif // DECISION_TASK_H
