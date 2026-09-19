

#include "pose_kalman.h"



Pose_Kalman::Pose_Kalman(SBaslerCameraControl *image_control, params_struct &params)
{
    Image_Control=image_control;
    kalmanParams=params;    

}



void Pose_Kalman::serverInit()
{
}

void Pose_Kalman::server500()//设定初始位置
{

}
void Pose_Kalman::planeMove()
{
    /***摄像头方式***/
    static params_struct initialPosition;
    static int yNumber=0,numFlag=0;
    char * filename=new char[100];
    sprintf(filename,"D:/QT_space/MicroSystem/image/07221/%d_%d.bmp",yNumber,iteratorNum);
//    while(Image_Control->GrabImage(TrainImg,5)!=0)
//    {
//        std::cout << "Grab image failed " << std::endl;
//    }
//    imshow("TrainImg11",TrainImg);
//    waitKey();
    Image_Control->GrabImage(TrainImg,50);
    Sleep(100);//等待
    if(Image_Control->GrabImage(TrainImg,50)!=0){
        std::cout << "Transfor: Grab failed " << std::endl;
        return ;
    }
    if(numFlag==0)
    {
        Ump_Read_Position(&kalmanParams);
        initialPosition=kalmanParams;
    }
    imwrite(filename,TrainImg);
    iteratorNum++;
    if(iteratorNum>6)//共0.3mm距离
    {
        yNumber++;
        iteratorNum=0;
        if(yNumber>4)
        {
          funSelect=0;
        }
    }
    kalmanParams.target_d=initialPosition.home_d;
    kalmanParams.target_y=initialPosition.home_y+yNumber*180*1000;
    kalmanParams.target_z=initialPosition.home_z;
    kalmanParams.target_x=initialPosition.home_x+iteratorNum*150*1000;
    kalmanParams.speed=3000;
    Ump_Goto_Position(&kalmanParams);
    Sleep(2000);
    cout<<"y:"<<yNumber<<"x:"<<iteratorNum<<endl;


//    if(++iteratorNum>5)//共0.3mm距离
//    {
//        kalmanParams.target_d=initialPosition.home_d;
//        kalmanParams.target_y=initialPosition.home_y;
//        kalmanParams.target_z=initialPosition.home_z+150*1000;
//        kalmanParams.target_x=initialPosition.home_x;
//        kalmanParams.speed=1000;
//        Ump_Goto_Position(&kalmanParams);
//        funSelect=0;
//    }
    numFlag++;

}





void Pose_Kalman::videoRecord()
{
    /***摄像头方式***/
    Mat imageRead,testImg;
    static int yNumber=0,numFlag=0;
    char * filename=new char[100];
    sprintf(filename,"D:/QT_space/MicroSystem/image/07221/%d_%d.bmp",yNumber,iteratorNum);
    static VideoWriter videoCreate("planeMove.avi", CV_FOURCC('D', 'I', 'V', 'X'), 25, Size(1024,542),1);

    Image_Control->GrabImage(imageRead,50);
    Sleep(100);//等待
    if(Image_Control->GrabImage(imageRead,50)!=0){
        std::cout << "Transfor: Grab failed " << std::endl;
        return ;
    }

    resize(imageRead,testImg,Size(1024,542));

    videoCreate<<testImg;
    numFlag++;
    cout<<"video number:"<<numFlag<<endl;

    if(numFlag>400)
    {
        funSelect=0;
        videoCreate.release();
    }
}


void Pose_Kalman::verticalMove()
{
    /***摄像头方式***/
    static params_struct initialPosition;
    static int numFlag=0;
    char * filename=new char[100];
    sprintf(filename,"D:/HWKPROGRAMS/MicroSystem/image/01072_40/%d.bmp",numFlag);

    Image_Control->GrabImage(TrainImg,50);
    Sleep(100);//等待
    if(Image_Control->GrabImage(TrainImg,50)!=0){
        std::cout << "Transfor: Grab failed " << std::endl;
        return ;
    }


    if(numFlag==0)
    {
        kalmanParams.dev=2;
        Ump_Select_Dev(&kalmanParams);
        kalmanParams.target_d=0;
        kalmanParams.target_y=0;
        kalmanParams.target_z=-150*1000;
        kalmanParams.target_x=0;
        kalmanParams.speed=1000;
        Ump_Take_Step(&kalmanParams);
        Sleep(1000);
        Ump_Read_Position(&kalmanParams);
        initialPosition=kalmanParams;
    }
    imwrite(filename,TrainImg);
    numFlag++;
    kalmanParams.target_d=initialPosition.home_d;
    kalmanParams.target_y=initialPosition.home_y;
    kalmanParams.target_z=initialPosition.home_z+numFlag*1000;
    kalmanParams.target_x=initialPosition.home_x;
    kalmanParams.speed=10;
    Ump_Goto_Position(&kalmanParams);
    Sleep(100);//等待
    cout<<"iteratorNum:"<<numFlag<<endl;
    if(numFlag>300)//共0.3mm距离
    {
        kalmanParams.target_d=initialPosition.home_d;
        kalmanParams.target_y=initialPosition.home_y;
        kalmanParams.target_z=initialPosition.home_z+150*1000;
        kalmanParams.target_x=initialPosition.home_x;
        kalmanParams.speed=1000;
        Ump_Goto_Position(&kalmanParams);
        funSelect=0;
    }

}

void Pose_Kalman::receiveTargetP(int receiveP)
{
    targetP=receiveP;
}

void Pose_Kalman::receivePositions(float centerX, float centerY)
{
//    receiveBarycenterP=barycenterP;
    injectionSites.push_back(Point2f(centerX,centerY));

}

void Pose_Kalman::penetrationSiteShow(float centerX, float centerY)
{
    tipPosition.x=centerX;
    tipPosition.y=centerY;
}

void Pose_Kalman::setExperimentRecording(bool enabled)
{
    experimentRecording=enabled;
    if(enabled && funSelect<0){
        funSelect=4;
    }
}


void Pose_Kalman::receiveNowInfo(double timeControl,double current,int x,int y,int z,int d,double sendVol,double sendFre){
    timeNow=timeControl;
    currentNow=current;
    xNow=x;
    yNow=y;
    zNow=z;
    dNow=d;
    sendVoltage=sendVol;
    sendFrequency=sendFre;

}


void Pose_Kalman::uiShow()
{
    double Time_Focus;
    Mat tansImg,sendImg;
    static int numFlag=1,startFlag=0;
    QImage outImg;
    static VideoWriter videoCreate("recording.avi", CV_FOURCC('D', 'I', 'V', 'X'), 25, Size(4096,2168),1);

//    static int timeMark= QDateTime::currentDateTime().toTime_t();

    static char * filename1=new char[100];
    static QDateTime timeMark= QDateTime::currentDateTime();
    sprintf(filename1,"%d_%d_%d_%d_videoNormal.avi",timeMark.date().dayOfYear(),timeMark.time().hour(),timeMark.time().minute(),timeMark.time().second());
    static VideoWriter videoNormal(filename1, CV_FOURCC('D', 'I', 'V', 'X'), 20, Size(1024,542),1);
    static char * filename=new char[100];
    sprintf(filename,"D:/HWKPROGRAMS/MicroSystem/image/0222_1/%d.bmp",numFlag);
    static char * text = new char[100];
    static char * textNum = new char[10];

    static QFile handle("manipulator_pose.txt");
    if(startFlag==0){
        handle.open(QIODevice::WriteOnly);
        startFlag++;
    }
    static QTextStream write_(&handle);

    if(cameraOpen){

//        Image_Control->GrabImage(TrainImg,50);
//        Sleep(50);//等待
//        if(Image_Control->GrabImage(TrainImg,50)!=0){
//            std::cout << "Transfor: Grab failed " << std::endl;
//        }
//        Time_Focus=getTickCount();
        while(Image_Control->GrabImage(TrainImg,5)!=0)
        {
//            std::cout << "Grab image failed " << std::endl;
            /*break;*/
        }

        // Keep experiment data separate from the resized/annotated preview.
        // copy() detaches the QImage from the cv::Mat buffer before it is sent
        // to the asynchronous recorder thread.
        if(experimentRecording){
            const QImage rawFrame=MatToQImage(TrainImg);
            if(!rawFrame.isNull()){
                emit rawFrameReady(rawFrame.copy());
            }
        }
//        Time_Focus=((double)getTickCount()-Time_Focus)/getTickFrequency()*1000;

        if(recordFlag==1){
//            if(Ump_Read_Position(&kalmanParams)==1){
                tansImg=TrainImg.clone();
                resize(tansImg,sendImg,Size(1024,542));
//                imwrite(filename,sendImg);
                videoCreate<<tansImg;
//                videoCreate<<sendImg;
//                write_<<"numFlag:"<<numFlag<<"; x:"<<kalmanParams.home_x<<"; y:"<<kalmanParams.home_y<<"; z:"<<kalmanParams.home_z<<"; d:"<<kalmanParams.home_d<<endl;
                cout<<numFlag<<endl;
                numFlag++;
//            }
        }else if(recordFlag==0){
//            videoCreate.release();
//            handle.close();
            recordFlag=-1;
            tansImg=TrainImg.clone();
            resize(tansImg,sendImg,Size(1024,542));
        }else{
            tansImg=TrainImg.clone();
            resize(tansImg,sendImg,Size(1024,542));
        }
//        sprintf(text,"Time: %f, I: %f, x: %d, y: %d, z: %d, d: %d, vol: %f, fre: %d Hz",timeNow,currentNow,xNow, yNow, zNow, dNow,sendVoltage,(int)sendFrequency);

        sprintf(text,"Time: %f, I: %f, x: %d, y: %d, z: %d, d: %d",timeNow,currentNow,xNow, yNow, zNow, dNow);
        putText(sendImg,text,cvPoint(10,15),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255),2);

        sprintf(text,"vol: %f, fre: %f KHz",sendVoltage,sendFrequency/1000.0);
        putText(sendImg,text,cvPoint(10,35),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255),2);

        videoNormal<<sendImg;

        if(!injectionSites.empty())
        {
            for( int i = 0; i<injectionSites.size()-1; i++ )
            {
                if(i==targetP)
                {
                   circle(sendImg, injectionSites[i],1, Scalar(0,0,255));
                }else{
                   circle(sendImg, injectionSites[i],2, Scalar(255,0,0));
                }

                sprintf(textNum,"%d",i+1);
                putText(sendImg,textNum,injectionSites[i],FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255),1);

                line( sendImg, injectionSites[i],injectionSites[i+1], Scalar(0,0,255));
            }
        }

//        if(tipPosition.x>0.1&&tipPosition.y>0.1){
//            cvtColor(sendImg,sendImg,COLOR_RGB2BGR);
//            line( sendImg, Point2f(tipPosition.x,tipPosition.y-50),Point2f(tipPosition.x,tipPosition.y+50), Scalar(0,0,255));
//            line( sendImg, Point2f(tipPosition.x-30,tipPosition.y-50),Point2f(tipPosition.x-30,tipPosition.y+50), Scalar(0,0,255));
//            line( sendImg, Point2f(tipPosition.x-40,tipPosition.y-50),Point2f(tipPosition.x-40,tipPosition.y+50), Scalar(0,0,255));
//            line( sendImg, Point2f(tipPosition.x-50,tipPosition.y-50),Point2f(tipPosition.x-50,tipPosition.y+50), Scalar(0,0,255));
//            line( sendImg, Point2f(tipPosition.x-60,tipPosition.y-50),Point2f(tipPosition.x-60,tipPosition.y+50), Scalar(0,0,255));
//            line( sendImg, Point2f(tipPosition.x-70,tipPosition.y-50),Point2f(tipPosition.x-70,tipPosition.y+50), Scalar(0,0,255));
//            line( sendImg, Point2f(tipPosition.x-80,tipPosition.y-50),Point2f(tipPosition.x-80,tipPosition.y+50), Scalar(0,0,255));
//            line( sendImg, Point2f(tipPosition.x-90,tipPosition.y-50),Point2f(tipPosition.x-90,tipPosition.y+50), Scalar(0,0,255));
//            line( sendImg, Point2f(tipPosition.x-100,tipPosition.y-50),Point2f(tipPosition.x-100,tipPosition.y+50), Scalar(0,0,255));
//            line( sendImg, Point2f(tipPosition.x-110,tipPosition.y-50),Point2f(tipPosition.x-110,tipPosition.y+50), Scalar(0,0,255));
//            line( sendImg, Point2f(tipPosition.x-120,tipPosition.y-50),Point2f(tipPosition.x-120,tipPosition.y+50), Scalar(0,0,255));
//            line( sendImg, Point2f(tipPosition.x-130,tipPosition.y-50),Point2f(tipPosition.x-130,tipPosition.y+50), Scalar(0,0,255));
//            line( sendImg, Point2f(tipPosition.x-140,tipPosition.y-50),Point2f(tipPosition.x-140,tipPosition.y+50), Scalar(0,0,255));
//            line( sendImg, Point2f(tipPosition.x-150,tipPosition.y-50),Point2f(tipPosition.x-150,tipPosition.y+50), Scalar(0,0,255));
//            line( sendImg, Point2f(tipPosition.x-160,tipPosition.y-50),Point2f(tipPosition.x-160,tipPosition.y+50), Scalar(0,0,255));
//            line( sendImg, Point2f(tipPosition.x-170,tipPosition.y-50),Point2f(tipPosition.x-170,tipPosition.y+50), Scalar(0,0,255));
//            line( sendImg, Point2f(tipPosition.x-180,tipPosition.y-50),Point2f(tipPosition.x-180,tipPosition.y+50), Scalar(0,0,255));
//            line( sendImg, Point2f(tipPosition.x-190,tipPosition.y-50),Point2f(tipPosition.x-190,tipPosition.y+50), Scalar(0,0,255));
//            line( sendImg, Point2f(tipPosition.x-200,tipPosition.y-50),Point2f(tipPosition.x-200,tipPosition.y+50), Scalar(0,0,255));
//        }


        outImg=MatToQImage(sendImg);
        emit sendImage(outImg);
    }

//    cout<<"Time_K:"<<Time_Focus<<endl;

}


void Pose_Kalman::whiteAndExposure()
{
    if(cameraOpen){
        Image_Control->autoImageAdjust(10000);
        cout<<"auto image adjust over"<<endl;
    }
}

void Pose_Kalman::uiShowOpen()
{
    funSelect=4;
}

void Pose_Kalman::decision()
{
    switch (funSelect) {
    case 100:
        timerKalman=new QTimer();
        timerKalman->setInterval(25);
        connect(timerKalman,&QTimer::timeout,this,&Pose_Kalman::decision);
        timerKalman->start();
        funSelect=-1;
        cout<<"waiting for imageCollection "<<endl;
        break;
    case 1:
        verticalMove();
        break;
    case 2:
//        server500();//离开聚焦平面
        funSelect=3;
        break;
    case 3:
        verticalMove();
//        videoRecord(); //开始采集图像
        break;
    case 4:
        uiShow();
//         funSelect=4;//稳定图像传输至UI
        break;
    case 5:
        whiteAndExposure();
         funSelect=4;
        break;
    default:
        break;
    }
}


//void Pose_Kalman::run()
//{
//    timeDecision=new QTimer();
//    timeDecision->setInterval(100);
//    connect(timeDecision,&QTimer::timeout,this,&Pose_Kalman::imageGetTask);
//    timeDecision->start(1);
//    funSelect=1;
//    exec();
//}
