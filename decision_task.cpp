#include "decision_task.h"
#include "mainwindow.h"

Decision_Task::Decision_Task()
{
    parse_args(&params);//微操作器初始化
    if(actOpen)
    {
        Ump_Init(&params);//启动连接
    }
    m_control=new SBaslerCameraControl(this);//相机模块
//    ndiTrack=new ndiTracking;

    A_Focus=new Auto_Focus(m_control,params);//自动聚焦模块
    QThread* afocusThread = new QThread;
    A_Focus->moveToThread(afocusThread);
    connect(afocusThread, &QThread::finished, afocusThread, &QThread::deleteLater);
    connect(afocusThread,&QThread::started,A_Focus,&Auto_Focus::decision);
    afocusThread->start();

    U_Frame=new ui_frame(A_Focus);//自动聚焦独立界面

    ch_instrument=new Ch_Instrument(params);//电化学模块
    QThread* chiThread = new QThread;
    ch_instrument->moveToThread(chiThread);
    connect(chiThread, &QThread::finished, chiThread, &QThread::deleteLater);
    connect(chiThread,&QThread::started,ch_instrument,&Ch_Instrument::decision);
    chiThread->start();

    pose_plane=new Pose_Plane(A_Focus,m_control,params,ch_instrument);//平面定位模块
    QThread* planeThread = new QThread;
    pose_plane->moveToThread(planeThread);
    connect(planeThread, &QThread::finished, planeThread, &QThread::deleteLater);
    connect(planeThread,&QThread::started,pose_plane,&Pose_Plane::decision);
    connect(ch_instrument,&Ch_Instrument::currentmutation,pose_plane,&Pose_Plane::touchCurrentMutation);
    connect(ch_instrument,&Ch_Instrument::dataGet,pose_plane,&Pose_Plane::sicmData);
     connect(pose_plane,&Pose_Plane::sendTargetPose,ch_instrument,&Ch_Instrument::getTargetPose);
    planeThread->start();


    patchClampX=new patchclamp();//膜片钳模块
    QThread* patchThread = new QThread;
    patchClampX->moveToThread(patchThread);
    connect(patchThread, &QThread::finished, patchClampX, &patchclamp::pcStop);
    connect(patchThread, &QThread::finished, patchThread, &QThread::deleteLater);
    connect(patchThread,&QThread::started,patchClampX,&patchclamp::decision);
    connect(pose_plane,&Pose_Plane::sendVoltage,patchClampX,&patchclamp::pcSetVoltage);
//    connect(ch_instrument,&Ch_Instrument::dataGet,pose_plane,&Pose_Plane::sicmData);
    patchThread->start();

    imageCollect=new Pose_Kalman(m_control,params);//驱动采集图像
    QThread* imageCollectThread = new QThread;
    imageCollect->moveToThread(imageCollectThread);
    connect(imageCollectThread, &QThread::finished, imageCollectThread, &QThread::deleteLater);
    connect(imageCollectThread,&QThread::started,imageCollect,&Pose_Kalman::decision);
    connect(pose_plane,&Pose_Plane::sendTarget,imageCollect,&Pose_Kalman::receiveTargetP);
    connect(pose_plane,&Pose_Plane::sendNowInfo,imageCollect,&Pose_Kalman::receiveNowInfo);
    connect(ch_instrument,&Ch_Instrument::sendNowInfo,imageCollect,&Pose_Kalman::receiveNowInfo);
    imageCollectThread->start();

    connect(A_Focus, &Auto_Focus::afOver, this, &Decision_Task::decisionSelect);
    /***相机初始化***/
//    connect(m_control, &SBaslerCameraControl::sigCurrentImage,(MainWindow*)Window,&MainWindow::showImage);
    connect(m_control,&SBaslerCameraControl::sigCameraUpdate, [=](QStringList list){
        m_control->OpenCamera(m_control->cameras().first());
        m_control->setFeatureTriggerSourceType("Freerun");//设置为软件触发
        m_control->getFeatureTriggerSourceType();//读取触发模式
        m_control->setFeatureTriggerModeType(1);//on
        m_control->setPixelFormat("Bayer_RG8");//固定图片格式
        m_control->StartAcquire();
    });
    if(cameraOpen){
        m_control->initSome();
    }

}
void Decision_Task::decisionSelect(int8_t select)
{
    funSelect=select;
}

void Decision_Task::decision_task_init()
{

}





void Decision_Task::decision_task_run()
{
    static int actionStep=0,segmentNum=0;
        switch (funSelect) {
        case 100:
            timer=new QTimer();
            timer->setInterval(50);
            connect(timer,&QTimer::timeout,this,&Decision_Task::decision_task_run);
            timer->start();
            funSelect=state_idle;
            cout<<"waiting for decision "<<endl;
            break;
        case 1:
            A_Focus->AutoOpen=1;
            pose_plane->AutoOpen=1;
//            pose_plane->servoSelection=1;
            if(A_Focus->Auto_Focus_Slect<0&&pose_plane->planeSelection<0)//等待空闲
            {
                if(stateHandle[actionStep].x==state_idle && stateHandle.at(actionStep).y==state_idle){
                    if(segmentNum<5){
                        actionStep=7;//从更换检测区域继续
                        segmentNum++;//累计检测区域
                    } else{
                        funSelect=0;//检测结束
                    }
                }

                A_Focus->Auto_Focus_Slect=stateHandle[actionStep].x;//赋予新任务
                pose_plane->planeSelection=stateHandle[actionStep].y;//赋予新任务

                if(stateHandle[actionStep].y==state_touchDetection)//当要接触检测时，保存细胞位置
                {
                    pose_plane->vagueCell=A_Focus->vaguePosition;//细胞最优平面时微操作器的位置
                    pose_plane->barycenterPFinal=A_Focus->barycenterP;//存储质心
                    pose_plane->bestPathFinal=A_Focus->bestPath;//最优路径
                }
                actionStep++;
            }
            imageCollect->funSelect=4;//控制UI稳定显示图像
             funSelect=1;
            break;
        case 2:
            funSelect=0;
            break;
        case 3://全自动，手动点击
            A_Focus->AutoOpen=1;
            pose_plane->AutoOpen=1;
//            pose_plane->servoSelection=1;
            if(A_Focus->penetrationCollectFlag==1)//当要接触检测时，保存细胞位置
            {
                A_Focus->penetrationCollectFlag++;
            }else{
                A_Focus->Auto_Focus_Slect=stateHandle[actionStep].x;//手动赋予新任务
            }
            pose_plane->planeSelection=stateHandle[actionStep].y;//手动赋予新任务

            if(stateHandle[actionStep].y==state_touchDetection)//当要接触检测时，保存细胞位置
            {
                pose_plane->vagueCell=A_Focus->vaguePosition;//细胞最优平面时微操作器的位置
                pose_plane->barycenterPFinal=A_Focus->barycenterP;//存储质心
                pose_plane->barycenterBFinal=A_Focus->barycenterBare;//存储裸露区质心
                pose_plane->bestPathFinal=A_Focus->bestPath;//最优路径
            }
            if(stateHandle[actionStep].y==state_penetration)//当要接触检测时，保存细胞位置
            {
                 imageCollect->injectionSites=A_Focus->bestPathPoint;//存储质心
            }
            actionStep++;
            imageCollect->funSelect=4;//控制UI稳定显示图像
            funSelect=0;
            break;
        case 4://半自动，手动点击
            A_Focus->AutoOpen=0;
            pose_plane->AutoOpen=0;
//            pose_plane->servoSelection=1;
            A_Focus->Auto_Focus_Slect=stateHandleSemi[actionStep].x;//手动赋予新任务
            pose_plane->planeSelection=stateHandleSemi[actionStep].y;//手动赋予新任务

//            pose_plane->planeSelection=state_planePosition;

            if(stateHandleSemi[actionStep].y==state_touchDetection)//当要接触检测时，保存细胞位置
            {
                pose_plane->vagueCell=A_Focus->vaguePosition;//细胞最优平面时微操作器的位置
                pose_plane->barycenterPFinal=A_Focus->barycenterP;//存储质心
                pose_plane->barycenterBFinal=A_Focus->barycenterBare;//存储裸露区质心
                pose_plane->bestPathFinal=A_Focus->bestPath;//最优路径
            }
            actionStep++;
            imageCollect->funSelect=4;//控制UI稳定显示图像
            funSelect=0;
            break;
        case 5:
             imageCollect->funSelect=3;
             funSelect=0;//开始采集图像
            break;
        case 6:
             imageCollect->funSelect=0;
             funSelect=0;//停止采集图像
            break;
        case 7:
             imageCollect->funSelect=5;//自动曝光+自动白平衡
             funSelect=0;//
            break;
        case 8:
             funSelect=0;//停止
            break;
        case 9:
//            if(pose_plane->planeSelection==state_idle && A_Focus->Auto_Focus_Slect==state_idle)
//            {
            if(pose_plane->planeSelection==state_idle || pose_plane->planeSelection==state_manualmove)
            {
                pose_plane->planeSelection=state_semiAutoBiopsy;
                imageCollect->funSelect=0;//控制UI不重复
                funSelect=0;
            }
            break;
        case 10:
            A_Focus->AutoOpen=1;
            pose_plane->AutoOpen=1;
//            pose_plane->servoSelection=1;
            actionStep--;
            A_Focus->Auto_Focus_Slect=stateHandle[actionStep].x;//手动赋予新任务
            pose_plane->planeSelection=stateHandle[actionStep].y;//手动赋予新任务

            if(stateHandle[actionStep].y==state_touchDetection)//当要接触检测时，保存细胞位置
            {
                pose_plane->vagueCell=A_Focus->vaguePosition;//细胞最优平面时微操作器的位置
                pose_plane->barycenterPFinal=A_Focus->barycenterP;//存储质心
                pose_plane->barycenterBFinal=A_Focus->barycenterBare;//存储裸露区质心
                pose_plane->bestPathFinal=A_Focus->bestPath;//最优路径
            }
            if(stateHandle[actionStep].y==state_penetration)//当要接触检测时，保存细胞位置
            {
                 imageCollect->injectionSites=A_Focus->bestPathPoint;//存储质心
            }
            imageCollect->funSelect=4;//控制UI稳定显示图像
            funSelect=0;
            break;
        default:
            break;
        }
}

void Decision_Task::penetrationDepthControl(int depth)
{
    pose_plane->zDownAdd=depth*1000;
    cout<<"zDown: "<<pose_plane->zDownAdd<<endl;

}

void Decision_Task::penetrationTimeControl(int peneTime)
{
    pose_plane->penetrationTime=peneTime;
    A_Focus->penetrationTime=peneTime;
    cout<<"penetrationTime: "<<pose_plane->penetrationTime<<endl;

}

void Decision_Task::liftheightControl(int height)
{
    pose_plane->zUpAdd=height*1000;
    cout<<"zUpAdd: "<<pose_plane->zUpAdd<<endl;

}

void Decision_Task::manualMoveControl(int axis, int direction)
{
    pose_plane->manualMoveControl(axis,direction);


}


void Decision_Task::ZeroSet()
{
    pose_plane->zeroSet();


}

void Decision_Task::ZeroReturn()
{
    pose_plane->zeroReturn();


}


