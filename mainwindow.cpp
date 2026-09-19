#if _MSC_VER >= 1600
#pragma execution_character_set("utf-8")

#endif

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "opencv2/opencv.hpp"
#include "sbaslercameracontrol.h"
#include <QtSerialPort/QSerialPortInfo>
#include <QMouseEvent>
#include "mylabel.h"
#include "experimentrecorder.h"


#include <QtGui>
#include <QtWidgets>
#include <QtCharts>
#include <iostream>
#include <QJsonObject>
#include <limits>






MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    /*decision线程*/
    QThread* subThread = new QThread;
    decision_task=new Decision_Task;
    decision_task->moveToThread(subThread);
    connect(subThread, &QThread::finished, subThread, &QThread::deleteLater);
    connect(subThread,&QThread::started,decision_task,&Decision_Task::decision_task_run);
    subThread->start();
    /*状态栏显示*/
    stateLabel = new QLabel;
    ui->statusBar->addWidget(stateLabel);
    connect(ui->video,&MyLabel::sendPose,this,&MainWindow::showPose);
    connect(ui->video,&MyLabel::midButtonPress,this,&MainWindow::midButtonAct);
    /*鼠标坐标计算*/
    connect(this,&MainWindow::sendRatio,decision_task->pose_plane,&Pose_Plane::mousePosition);
    connect(this,&MainWindow::sendRatio,decision_task->A_Focus,&Auto_Focus::mousePosition);
    /*鼠标控制升降*/
    connect(this,&MainWindow::penetrationDepthControl,decision_task,&Decision_Task::penetrationDepthControl);
    connect(this,&MainWindow::penetrationTimeControl,decision_task,&Decision_Task::penetrationTimeControl);
    connect(this,&MainWindow::liftheightControl,decision_task,&Decision_Task::liftheightControl);
    connect(this,&MainWindow::sendManualControl,decision_task,&Decision_Task::manualMoveControl);
    connect(this,&MainWindow::sendZeroSet,decision_task,&Decision_Task::ZeroSet);
    connect(this,&MainWindow::sendZeroReturn,decision_task,&Decision_Task::ZeroReturn);

    /*图片显示*/
    connect(decision_task->pose_plane,&Pose_Plane::sendImage,this,&MainWindow::showImage);
    connect(decision_task->A_Focus,&Auto_Focus::sendImage,this,&MainWindow::showImage);
    connect(decision_task->imageCollect,&Pose_Kalman::sendImage,this,&MainWindow::showImage);
    connect(decision_task->A_Focus,&Auto_Focus::sendPositions,decision_task->imageCollect,&Pose_Kalman::receivePositions);
    connect(decision_task->pose_plane,&Pose_Plane::sendPosShow,decision_task->imageCollect,&Pose_Kalman::penetrationSiteShow);

    connect(decision_task->pose_plane,&Pose_Plane::sendDigiVoltage,decision_task->ch_instrument,&Ch_Instrument::sendDigidataVol);
    connect(decision_task->patchClampX,&patchclamp::sendDigidataVoltage,decision_task->ch_instrument,&Ch_Instrument::sendDigidataVol);

    /*状态显示*/
    connect(decision_task->A_Focus,&Auto_Focus::sendState,this,&MainWindow::showState);
    connect(decision_task->pose_plane,&Pose_Plane::sendState,this,&MainWindow::showState);

    /*刺入时不卡住针尖程序*/
    connect(decision_task->pose_plane,&Pose_Plane::sendPenetrationSleep,decision_task->A_Focus,&Auto_Focus::penetrationSleepChange);
    connect(decision_task->A_Focus,&Auto_Focus::sendPenetration,decision_task->pose_plane,&Pose_Plane::penetrationSleepChange);
    connect(decision_task->A_Focus,&Auto_Focus::sendPenetration,decision_task->imageCollect,&Pose_Kalman::uiShowOpen);

    /*实验原始数据记录：采集线程只复制并投递帧，磁盘写入在独立线程完成。*/
    experimentRecorder=new ExperimentRecorder(this);
    connect(decision_task->imageCollect,&Pose_Kalman::mhiFrameReady,
            experimentRecorder,&ExperimentRecorder::recordMhiFrame,
            Qt::DirectConnection);
    connect(decision_task->imageCollect,&Pose_Kalman::keyFrameReady,
            experimentRecorder,&ExperimentRecorder::recordKeyFrame,
            Qt::DirectConnection);
    connect(decision_task->pose_plane,&Pose_Plane::sendTargetPose,
            experimentRecorder,
            [this](int, int, int z, int){
                const double zCommand = z == 0
                    ? std::numeric_limits<double>::quiet_NaN()
                    : static_cast<double>(z) / 1000.0;
                experimentRecorder->recordMotionCommand(zCommand);
            }, Qt::DirectConnection);
    connect(decision_task->ch_instrument,&Ch_Instrument::sendNowInfo,
            experimentRecorder,
            [this](double, double, int x, int y, int z, int, double, double){
                experimentRecorder->recordMotionData(
                    static_cast<double>(z) / 1000.0,
                    static_cast<double>(x) / 1000.0,
                    static_cast<double>(y) / 1000.0);
            }, Qt::DirectConnection);
    connect(decision_task->pose_plane,&Pose_Plane::manualZMoveCompleted,
            this,[this](int direction,int manipulator){
                const bool movedDown=(manipulator==1 && direction==-1) ||
                                     (manipulator==2 && direction==1);
                if(movedDown && experimentRecorder->isRecording()){
                    QMetaObject::invokeMethod(decision_task->imageCollect,
                                              "requestExperimentFrame",
                                              Qt::QueuedConnection);
                }
            },Qt::QueuedConnection);
    connect(experimentRecorder,&ExperimentRecorder::recorderError,
            this,&MainWindow::handleRecorderError,Qt::QueuedConnection);
    recordingUiTimer=new QTimer(this);
    recordingUiTimer->setInterval(100);
    connect(recordingUiTimer,&QTimer::timeout,this,&MainWindow::updateRecordingUi);
    recordingUiTimer->start();
    ui->stopRecording->setEnabled(false);
    QShortcut *contactShortcut=new QShortcut(QKeySequence(Qt::Key_Space),this);
    contactShortcut->setContext(Qt::WindowShortcut);
    connect(contactShortcut,&QShortcut::activated,
            this,&MainWindow::markContactEvent);


    /*颜色调整*/
    ui->stackedWidget->setCurrentWidget(ui->page);
//    ui->Page_1->setEnabled(false);

    ui->toZero->setEnabled(false);

//    ui->video->sizePolicy();
//    ui->video->setGeometry(208,9,1024,542);


//    QWidget * ppp =new QWidget(nullptr);
//    QPalette pal1(ppp->palette());
//    pal1.setColor(QPalette::Background, QColor(200, 200, 255));
//    ppp->setAutoFillBackground(true);
//    ppp->setPalette(pal1);




}

MainWindow::~MainWindow()
{
    if(experimentRecorder && experimentRecorder->isRecording()){
        QMetaObject::invokeMethod(decision_task->imageCollect,
                                  "setExperimentRecording",
                                  Qt::QueuedConnection,
                                  Q_ARG(bool,false));
        experimentRecorder->stopTrial();
    }
    delete ui;
}


void MainWindow::showState(QString sState)
{
    stateLabel->setText(sState);

}

void MainWindow::midButtonAct()
{
    decision_task->A_Focus->penetrationCollectFlag=2;

}


void MainWindow::showPose(float x,float y)
{
    float ratioX,ratioY;
    float lengthX,lengthY;
    stateLabel->setText(QString("Site: x:%1,y:%2").arg(x).arg(y));
    lengthX=ui->video->size().width();
    lengthY=ui->video->size().height();
    ratioX=x/ui->video->size().width();
    ratioY=y/ui->video->size().height();
    emit sendRatio(ratioX,ratioY);

}

void MainWindow::showImage(const QImage &image)
{
    QImage  imageOut = image.scaled(ui->video->size(), Qt::IgnoreAspectRatio, Qt::FastTransformation);
//    QImage  imageOut = image.scaled(image.size(), Qt::KeepAspectRatio, Qt::FastTransformation);
//    QImage  imageOut = image.scaled(image.size(), Qt::IgnoreAspectRatio, Qt::FastTransformation);
    ui->video->setPixmap(QPixmap::fromImage(imageOut));

}

//void MainWindow::mouseMoveEvent(QMouseEvent* event)
//{
//     QPoint mousepos = event->pos();

//     //在坐标（0 ~ width，0 ~ height）范围内改变鼠标形状
//     if(1)
//     {
//         this->setCursor(Qt::CrossCursor);
//     }
//     else
//     {
//         this->setCursor(Qt::ArrowCursor);      //范围之外变回原来形状
//     }
//}



/**半自动按键区**/

void MainWindow::on_Manual_clicked()
{
    decision_task->pose_plane->touchFlag=4;

}


void MainWindow::on_penetration_time_valueChanged(int arg1)
{
    emit penetrationTimeControl(arg1);

}


void MainWindow::on_penetration_depth_valueChanged(int arg1)
{
    emit penetrationDepthControl(arg1);
}

void MainWindow::on_lift_height_valueChanged(int arg1)
{
    emit liftheightControl(arg1);

}


void MainWindow::on_ImageAdjust_clicked()
{
    decision_task->funSelect=7;//图片自动校准
}


void MainWindow::on_NextAction_clicked()
{
    decision_task->funSelect=4;//针尖定位独立测试
    ui->fullyNextAct->setDisabled(1);

}

/**全自动按键区**/

void MainWindow::on_fullyStart_clicked()
{
     decision_task->funSelect=1;//开启自动化流程
}

void MainWindow::on_fullyStop_clicked()
{
    decision_task->funSelect=2;//中断自动化流程
}


void MainWindow::on_fullyNextAct_clicked()
{
    decision_task->funSelect=3;//独立测试
    ui->NextAction->setDisabled(1);
}


void MainWindow::on_saveImage_clicked()
{
//    decision_task->funSelect=5;//图像采集
    decision_task->imageCollect->recordFlag=1;
}


void MainWindow::on_Close_Camera_clicked()
{
//    decision_task->funSelect=6;//图像采集结束
    decision_task->imageCollect->recordFlag=0;
}




/**空 区**/


void MainWindow::on_None_clicked()
{
//    Mat imageRead;
//    imageRead=imread("D:/qt_space/1004/Microsystem/image/1.bmp");
//    imwrite("image.jpg",imageRead);
//    imshow("image",imageRead);

//    decision_task->patchClampX->patchSelection=1;
    decision_task->pose_plane->planeSelection=6;

}


void MainWindow::on_IBVS_clicked()
{
    if(ui->IBVS->text()=="Close"){
        ui->IBVS->setText("Open");
        decision_task->pose_plane->servoSelection=2;//转变为PBVS

    }else if(ui->IBVS->text()=="Open"){
        ui->IBVS->setText("Close");
        decision_task->pose_plane->servoSelection=1;//转变为IBVS
    }

}

double MainWindow::volumeToDistance(double arg1)
{
    /***基于二分法将体积转换为μm长度***/
    double m=100,n=0;//搜索区间
    double disResult = 0.0,i,sResult,f1,f2,eps=1e-6;//eps是精度控制，此处为10^-6
    double a=0.0001969,b=0.04215,c=0.0,d=-arg1;//输入方程参数

    f1=a*pow(m,3)+b*pow(m,2)+c*m+d;//pow(x,y)=x^y，幂函数
    f2=a*pow(n,3)+b*pow(n,2)+c*n+d;
    //判断f1*f2<0是主要代码
    if(f1*f2<0)
    {
        while(fabs(m-n)>eps)
        {
            i=(m+n)/2;
            sResult=a*pow(i,3)+b*pow(i,2)+c*i+d;
            if(fabs(sResult)<eps)//如果函数f(i)的绝对值|sum|小于无限小
            {
                disResult=i;
                break;
            }
            else if(f1*sResult<0)
            {
                n=i;
            }
            else if(f2*sResult<0)
            {
                m=i;
            }
        }
        disResult=i;
    }
    else if(f1*f2==0)     //如果刚好区间取值为方程解
    {
        if(f1==0)
        { disResult=m;}
        if(f2==0)
        { disResult=n;}
    }
    /***将μm长度转换为像素长度***/
//    cout<<"Distance: "<<interfacePositionSave<<endl;
    disResult=disResult*0.906401249f/(0.68f*decision_task->pose_plane->lensMagnify);
    return disResult;
}


void MainWindow::on_EwStart_valueChanged(double arg1)
{
    interfacePositionSave=volumeToDistance(arg1);
    if(decision_task->pose_plane->onlyInterfaceShowFlag==1){
        decision_task->pose_plane->onlyInterfaceShowFlag=2;
    }
    cout<<"Pixel Distance: "<<interfacePositionSave<<endl;

}

//电润湿自动化
void MainWindow::on_distanceSend_clicked()
{
    decision_task->pose_plane->interfacePosition=interfacePositionSave;
    decision_task->pose_plane->motorOpenFlag=0;
    decision_task->pose_plane->interfacePositionChange=1;
    decision_task->funSelect=9;
//    //MPC test
//    static int MPC_Initial=1;
//    if(MPC_Initial==1){
//           mpcTest.mpcControllerInit();
//           MPC_Initial=0;
//    }
//    mpcTest.mpcController(interfacePositionSave,1,0,0);



}


void MainWindow::on_doubleSpinBox_valueChanged(double arg1)
{
    voltagekk=arg1;

}

void MainWindow::on_volInputOver_clicked()
{
//    decision_task->pose_plane->mpcInterfaceControl.kAdjust=voltagekk;
    decision_task->patchClampX->motorOpenFlag=0;
    decision_task->patchClampX->pcSetVoltage(voltagekk);
    decision_task->ch_instrument->sendVoltage = voltagekk;
    decision_task->ch_instrument->sendFrequency = decision_task->patchClampX->volTagefre;

}

void MainWindow::on_touchBox_currentIndexChanged(int index)
{
    decision_task->pose_plane->touchFlag=index;
}

//manipulation selection
void MainWindow::on_RorLSelection_clicked()
{
    if(ui->RorLSelection->text()=="toRight"){
        ui->RorLSelection->setText("toLeft");
        decision_task->pose_plane->manipulationSelection=1; //夹持针的微操作器选择
        decision_task->ch_instrument->manipulationSelection=1; //夹持针的微操作器选择
        decision_task->m_control->manipulationSelection=1; //夹持针的微操作器选择
        decision_task->imageCollect->manipulationSelection=1; //夹持针的微操作器选择
        stateLabel->setText(QString("Manipulator Now: 1"));

    }else if(ui->RorLSelection->text()=="toLeft"){
        ui->RorLSelection->setText("toRight");
        decision_task->pose_plane->manipulationSelection=2; //夹持针的微操作器选择
        decision_task->ch_instrument->manipulationSelection=2; //夹持针的微操作器选择
        decision_task->m_control->manipulationSelection=2; //夹持针的微操作器选择
        decision_task->imageCollect->manipulationSelection=2; //夹持针的微操作器选择
        stateLabel->setText(QString("Manipulator Now: 2"));
    }



}


void MainWindow::on_lensMagnify_currentIndexChanged(int index)
{
    switch (index) {
    case 0:
      decision_task->pose_plane->lensMagnify=1.0;
        break;
    case 1:
      decision_task->pose_plane->lensMagnify=0.5;
        break;
    case 2:
      decision_task->pose_plane->lensMagnify=4.0;
        break;
    case 3:
      decision_task->pose_plane->lensMagnify=2.0;
        break;
    default:
        break;
    }
    decision_task->funSelect=7;//图片自动校准

}


void MainWindow::on_testOpen_clicked()
{
    if(ui->testOpen->text()=="Test_Open"){
        ui->testOpen->setText("Test_Close");
        decision_task->pose_plane->mulTouchSelection=1;

    }else if(ui->testOpen->text()=="Test_Close"){
        ui->testOpen->setText("Test_Open");
        decision_task->pose_plane->mulTouchSelection=0;
    }

}


//void MainWindow::on_Distance_valueChanged(int arg1)
//{
//    decision_task->pose_plane->manualDistace=arg1;
//}



void MainWindow::on_Speed_valueChanged(int arg1)
{
    decision_task->pose_plane->manualSpeed=arg1;
}

void MainWindow::on_Max_acc_valueChanged(int arg1)
{
    decision_task->pose_plane->manualAcc=arg1;
    cout<<decision_task->pose_plane->manualAcc<<endl;

}


void MainWindow::on_X_down_clicked()
{
    emit sendManualControl(1,-1);

}


void MainWindow::on_X_up_clicked()
{
    emit sendManualControl(1,1);
}


void MainWindow::on_Y_down_clicked()
{
    emit sendManualControl(2,-1);
}


void MainWindow::on_Y_up_clicked()
{
    emit sendManualControl(2,1);
}


void MainWindow::on_Z_down_clicked()
{
//    if(decision_task->pose_plane->manipulationSelection==1){
        emit sendManualControl(3,-1);
//    }else{
//        emit sendManualControl(3,1);
//    }

}


void MainWindow::on_Z_up_clicked()
{
//    if(decision_task->pose_plane->manipulationSelection==1){
        emit sendManualControl(3,1);
//    }else{
//        emit sendManualControl(3,-1);
//    }
}

void MainWindow::on_D_up_clicked()
{
    emit sendManualControl(4,1);
}

void MainWindow::on_D_down_clicked()
{
    emit sendManualControl(4,-1);
}


void MainWindow::on_Reciprocating_clicked()
{
    if(ui->Reciprocating->text()=="Start"){
        ui->Reciprocating->setText("Stop");
        decision_task->pose_plane->planeSelection=state_manualmoveStep;
        decision_task->pose_plane->reciprocatingStrat=0;
        emit sendManualControl(5,1);        
    }else{
        decision_task->pose_plane->planeSelection=state_idle;
        ui->Reciprocating->setText("Start");
    }


}








void MainWindow::on_fullyAboveAct_clicked()
{
    decision_task->funSelect=10;//独立测试

}

void MainWindow::on_Page_1_clicked()
{
//    ui->stackedWidget->setCurrentIndex(1);
    int currentIndex = ui->stackedWidget->currentIndex();
    int nextPageIndex = currentIndex - 1;
    // 如果当前页是最后一页，则跳转到第一页
    if (nextPageIndex < 0) {
        nextPageIndex = ui->stackedWidget->count()-1;
    }
    ui->stackedWidget->setCurrentIndex(nextPageIndex);
//    ui->stackedWidget->setCurrentWidget(ui->page);
    ui->Page_1->setEnabled(true);
    ui->Page_2->setEnabled(true);



}


void MainWindow::on_Page_2_clicked()
{
//    ui->stackedWidget->setCurrentIndex(2);
    int currentIndex = ui->stackedWidget->currentIndex();
    int nextPageIndex = currentIndex + 1;
    // 如果当前页是最后一页，则跳转到第一页
    if (nextPageIndex >= ui->stackedWidget->count()) {
        nextPageIndex = 0;
    }
    ui->stackedWidget->setCurrentIndex(nextPageIndex);
//    ui->stackedWidget->setCurrentWidget(ui->page_2);
    ui->Page_1->setEnabled(true);
    ui->Page_2->setEnabled(true);

}


void MainWindow::on_pumpPulse_valueChanged(double arg1)
{
    motorPulse=arg1;

}


void MainWindow::on_pumpPulseSend_clicked()
{
    decision_task->patchClampX->motorOpenFlag=1;
    decision_task->patchClampX->initPoseMark=decision_task->patchClampX->device->getCurPos();//相对位置运动
    decision_task->patchClampX->pcSetVoltage(motorPulse);
//    for (int j=0;j<50;j++) {
//        motorPulse=-motorPulse;
//        for (int i=0;i<20*10;i++) {
//            decision_task->patchClampX->pcSetVoltage(motorPulse);
////            cout<<"now speed: "<<motorPulse<<endl;
//            Sleep(50);
//        }
//    }

}


void MainWindow::on_pumpDistance_valueChanged(double arg1)
{
    interfacePositionSave=volumeToDistance(arg1);
    cout<<"Pixel Distance: "<<interfacePositionSave<<endl;
//    interfacePositionSave=arg1;

//    if(decision_task->pose_plane->onlyInterfaceShowFlag==1){
//        decision_task->pose_plane->onlyInterfaceShowFlag=2;
//    }

}


void MainWindow::on_pumpDistanceSend_clicked()
{
    int initFlag=1;
    decision_task->pose_plane->interfacePosition=interfacePositionSave;
    decision_task->pose_plane->motorOpenFlag=1;
    decision_task->pose_plane->MPC_Initial=1;
    decision_task->pose_plane->interfacePositionChange=1;
    decision_task->funSelect=9;
    if(initFlag){
        initFlag=0;
        decision_task->patchClampX->initPoseMark=decision_task->patchClampX->device->getCurPos();//绝对位置运动
    }
    if(decision_task->pose_plane->onlyInterfaceShowFlag==1){
        decision_task->pose_plane->onlyInterfaceShowFlag=2;
    }

}



void MainWindow::on_zeroSet_clicked()
{
    emit sendZeroSet();
    ui->toZero->setEnabled(true);


}

void MainWindow::on_toZero_clicked()
{
    emit sendZeroReturn();

}


void MainWindow::keyPressEvent(QKeyEvent * event)
{
    // 普通键
    switch (event->key())
    {
        // ESC键
    case Qt::Key_Q:
        if(decision_task->pose_plane->manipulationSelection==1){
            emit sendManualControl(3,1);
        }else{
            emit sendManualControl(3,-1);
        }
//         emit sendManualControl(3,1);
        qDebug()<<"zUp"<<endl;
    break;
    case Qt::Key_D:
        emit sendManualControl(1,1);
        qDebug()<<"yUp"<<endl;
    break;
    case Qt::Key_E:
        if(decision_task->pose_plane->manipulationSelection==1){
            emit sendManualControl(3,-1);
        }else{
            emit sendManualControl(3,1);
        }
//        emit sendManualControl(3,-1);
        qDebug()<<"zDown"<<endl;
    break;
    case Qt::Key_W:
        emit sendManualControl(2,1);
        qDebug()<<"xLeft"<<endl;
    break;
    case Qt::Key_A:
        emit sendManualControl(1,-1);
        qDebug()<<"yDown"<<endl;
    break;
    case Qt::Key_S:
        emit sendManualControl(2,-1);
        qDebug()<<"xRight"<<endl;
    break;
    case Qt::Key_F:
        emit sendManualControl(4,-1);
        qDebug()<<"fDown"<<endl;
    break;
    case Qt::Key_G:
        emit sendManualControl(4,1);
        qDebug()<<"fUp"<<endl;
    break;
    case Qt::Key_1:
         decision_task->pose_plane->manualDistace=1;
         decision_task->pose_plane->manualSpeed=10;
     break;
    case Qt::Key_2:
         decision_task->pose_plane->manualDistace=5;
         decision_task->pose_plane->manualSpeed=50;
     break;
    case Qt::Key_3:
         decision_task->pose_plane->manualDistace=10;
         decision_task->pose_plane->manualSpeed=50;
     break;
    case Qt::Key_4:
         decision_task->pose_plane->manualDistace=50;
         decision_task->pose_plane->manualSpeed=100;
     break;
    case Qt::Key_5:
         decision_task->pose_plane->manualDistace=100;
         decision_task->pose_plane->manualSpeed=500;
     break;
    }
}

void MainWindow::on_startRecording_clicked()
{
    if(experimentRecorder->isRecording()){
        return;
    }

    QJsonObject metadata;
    metadata.insert(QStringLiteral("cell_id"),QString());
    metadata.insert(QStringLiteral("cell_type"),QStringLiteral("HeLa"));
    metadata.insert(QStringLiteral("tip_id"),QString());
    metadata.insert(QStringLiteral("tip_diameter_um"),QJsonValue::Null);
    metadata.insert(QStringLiteral("magnification"),ui->lensMagnify->currentText());
    metadata.insert(QStringLiteral("camera_fps"),QJsonValue::Null);
    metadata.insert(QStringLiteral("exposure"),QJsonValue::Null);
    metadata.insert(QStringLiteral("mhi_target_fps"),10);
    metadata.insert(QStringLiteral("mhi_width"),1024);
    metadata.insert(QStringLiteral("mhi_height"),542);
    metadata.insert(QStringLiteral("mhi_channels"),1);
    metadata.insert(QStringLiteral("mhi_format"),QStringLiteral("BMP"));
    metadata.insert(QStringLiteral("keyframe_format"),QStringLiteral("PNG"));
    metadata.insert(QStringLiteral("keyframe_trigger"),
                    QStringLiteral("First camera frame after a completed E descent"));
    metadata.insert(QStringLiteral("approach_speed_um_s"),
                    decision_task->pose_plane->manualSpeed);
    metadata.insert(QStringLiteral("tip_angle_deg"),
                    decision_task->pose_plane->angleD*180.0/3.14159265358979323846);
    metadata.insert(QStringLiteral("contact_position"),QString());
    metadata.insert(QStringLiteral("z_command_semantics"),
                    QStringLiteral("Raw Pose_Plane sendTargetPose Z value; manual moves are relative steps"));
    metadata.insert(QStringLiteral("encoder_source"),
                    QStringLiteral("Ch_Instrument sendNowInfo; NaN until feedback is received"));
    experimentRecorder->setMetadata(metadata);

    const QString root=QStringLiteral("MicroSystemExperiments");
    if(!experimentRecorder->startTrial(root)){
        handleRecorderError(QStringLiteral("Unable to start experiment recording."));
        return;
    }

    QMetaObject::invokeMethod(decision_task->imageCollect,
                              "setExperimentRecording",
                              Qt::QueuedConnection,
                              Q_ARG(bool,true));
    ui->recordingStatusLabel->setText(QStringLiteral("ON"));
    ui->trialIdValueLabel->setText(experimentRecorder->trialId());
    ui->frameCountValueLabel->setText(QStringLiteral("0"));
    ui->elapsedTimeValueLabel->setText(QStringLiteral("0.000 s"));
    ui->startRecording->setEnabled(false);
    ui->stopRecording->setEnabled(true);
    stateLabel->setText(QStringLiteral("Recording: %1")
                        .arg(experimentRecorder->trialDirectory()));
}

void MainWindow::on_stopRecording_clicked()
{
    if(!experimentRecorder->isRecording()){
        return;
    }

    QMetaObject::invokeMethod(decision_task->imageCollect,
                              "setExperimentRecording",
                              Qt::QueuedConnection,
                              Q_ARG(bool,false));
    ui->recordingStatusLabel->setText(QStringLiteral("Stopping..."));
    ui->stopRecording->setEnabled(false);
    experimentRecorder->stopTrial();
    ui->recordingStatusLabel->setText(QStringLiteral("OFF"));
    ui->startRecording->setEnabled(true);
    stateLabel->setText(QStringLiteral("Recording saved: %1")
                        .arg(experimentRecorder->trialDirectory()));
}

void MainWindow::updateRecordingUi()
{
    if(experimentRecorder->isRecording()){
        ui->elapsedTimeValueLabel->setText(
            QStringLiteral("%1 s").arg(experimentRecorder->elapsedSeconds(),0,'f',3));
        const qulonglong count=experimentRecorder->frameCount();
        const qulonglong dropped=experimentRecorder->droppedFrameCount();
        ui->frameCountValueLabel->setText(
            dropped==0
                ? QString::number(count)
                : QStringLiteral("%1 (dropped %2)").arg(count).arg(dropped));
    }
}

void MainWindow::handleRecorderError(const QString &message)
{
    stateLabel->setText(QStringLiteral("Recorder error: %1").arg(message));
}

void MainWindow::markContactEvent()
{
    if(!experimentRecorder->isRecording()){
        return;
    }
    experimentRecorder->markEvent(QStringLiteral("manual_contact"));
    ui->recordingStatusLabel->setText(QStringLiteral("CONTACT MARKED"));
    QTimer::singleShot(800,this,[this](){
        if(experimentRecorder->isRecording()){
            ui->recordingStatusLabel->setText(QStringLiteral("ON"));
        }
    });
}


void MainWindow::on_Distance_double_valueChanged(double arg1)
{
        decision_task->pose_plane->manualDistace=arg1;        
}



void MainWindow::on_StepNumX_valueChanged(int arg1)
{
   decision_task->pose_plane->scanning_numSteps[0] = arg1;
   decision_task->pose_plane->generateScanPath(lineCombo);

}


void MainWindow::on_StepNumY_valueChanged(int arg1)
{
    decision_task->pose_plane->scanning_numSteps[1] = arg1;
    decision_task->pose_plane->generateScanPath(lineCombo);

}


void MainWindow::on_StepNumZ_valueChanged(int arg1)
{
    decision_task->pose_plane->scanning_numSteps[2] = arg1;
    decision_task->pose_plane->generateScanPath(lineCombo);

}


void MainWindow::on_StepX_valueChanged(double arg1)
{
    decision_task->pose_plane->scanning_stepLength[0] = arg1;
    decision_task->pose_plane->generateScanPath(lineCombo);

}


void MainWindow::on_StepY_valueChanged(double arg1)
{
    decision_task->pose_plane->scanning_stepLength[1] = arg1;
    decision_task->pose_plane->generateScanPath(lineCombo);

}


void MainWindow::on_StepZ_valueChanged(double arg1)
{
    decision_task->pose_plane->scanning_stepLength[2] = arg1;
    decision_task->pose_plane->generateScanPath(lineCombo);

}


void MainWindow::on_sanOrder_currentIndexChanged(int index)
{

//    double s_stepLength[3] = {0.0, 0.0, 0.0};// 步长
//    int s_numSteps[3] = {0, 0, 0};   // 步数
    switch (index) {
    case 0:
        // X Y Z
        decision_task->pose_plane->scanning_axisOrder[0] = 2;
        decision_task->pose_plane->scanning_axisOrder[1] = 1;
        decision_task->pose_plane->scanning_axisOrder[2] = 0;
        break;
    case 1:
        // X Z Y
        decision_task->pose_plane->scanning_axisOrder[0] = 1;
        decision_task->pose_plane->scanning_axisOrder[1] = 2;
        decision_task->pose_plane->scanning_axisOrder[2] = 0;
        break;
    case 2:
        // Y X Z
        decision_task->pose_plane->scanning_axisOrder[0] = 2;
        decision_task->pose_plane->scanning_axisOrder[1] = 0;
        decision_task->pose_plane->scanning_axisOrder[2] = 1;
        break;
    case 3:
        // Y Z X
        decision_task->pose_plane->scanning_axisOrder[0] = 0;
        decision_task->pose_plane->scanning_axisOrder[1] = 2;
        decision_task->pose_plane->scanning_axisOrder[2] = 1;
        break;
    case 4:
        // Z X Y
        decision_task->pose_plane->scanning_axisOrder[0] = 1;
        decision_task->pose_plane->scanning_axisOrder[1] = 0;
        decision_task->pose_plane->scanning_axisOrder[2] = 2;
        break;
    case 5:
        // Z Y X
        decision_task->pose_plane->scanning_axisOrder[0] = 0;
        decision_task->pose_plane->scanning_axisOrder[1] = 1;
        decision_task->pose_plane->scanning_axisOrder[2] = 2;
        break;
    default:
        break;
    }
    decision_task->pose_plane->generateScanPath(lineCombo);

}


void MainWindow::on_Scanning_clicked()
{

    if(ui->Scanning->text()=="Start"){
        ui->Scanning->setText("Stop");
        decision_task->pose_plane->planeSelection=state_manualmoveStep;
        decision_task->pose_plane->scanningStrat=0;
        decision_task->pose_plane->scanningPointN=0;
        switch (lineCombo) {
        case 0:
            emit sendManualControl(6,1);
            break;
        case 1:
            emit sendManualControl(8,1);
            break;
        case 2:
            emit sendManualControl(9,1);
            break;
        default:
            break;
        }

    }else{
        decision_task->pose_plane->planeSelection=state_idle;
        ui->Scanning->setText("Start");
    }


}





void MainWindow::on_volFrequency_valueChanged(double arg1)
{
    volFrequency=arg1;
    decision_task->patchClampX->volTagefre=(int)volFrequency*freUnit;

}

void MainWindow::on_volFreComb_currentIndexChanged(int index)
{
    switch (index) {
    case 0:
        //MHZ
        freUnit = 1000000;
        break;
    case 1:
        //KHZ
        freUnit = 1000;
        break;
    case 2:
        //HZ
        freUnit = 1;
        break;
    default:
        break;
    }
    decision_task->patchClampX->volTagefre=(int)volFrequency*freUnit;

}


void MainWindow::on_lineComboBox_currentIndexChanged(int index)
{
   lineCombo=index;

}
