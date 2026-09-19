#include "chinstrument.h"

Ch_Instrument::Ch_Instrument(params_struct &params)
{

    poseSave=params;
//    server = new QTcpServer();
//    server->listen(QHostAddress::LocalHost, 1900);
//    connect(server,&QTcpServer::newConnection,[=](){socket= server->nextPendingConnection();});

    //    connect(server, &QTcpServer::newConnection, this, &Ch_Instrument::acceptConnection);


}

void Ch_Instrument::acceptConnection()
{
//    socket = server->nextPendingConnection();
//    connect(socket, SIGNAL(readyRead()), this, SLOT(readClient()));
}

void Ch_Instrument::getTargetPose(int x, int y, int z, int d){
    targetX=x;
    targetY=y;
    targetZ=z;
    targetD=d;

}


void Ch_Instrument::readClient()
{
    static char * filename1=new char[100];
    static int manipulationSelection_last=1;
    static int infoSaveInit=1;
    static QFile handle;
    static QTextStream write_;
    static double timeControl_Init=(double)getTickCount();
    static int numFlag=0;
    static QString str;
    static QStringList strlist;
    Point2f data;
    double fluctuation=0;

    if(infoSaveInit==1){
        infoSaveInit=0;
        QDateTime timeMark= QDateTime::currentDateTime();
        sprintf(filename1,"%d_%d_%d_%d_current_and_pose.txt",timeMark.date().dayOfYear(),timeMark.time().hour(),timeMark.time().minute(),timeMark.time().second());
        handle.setFileName(filename1);
        handle.open(QIODevice::Append);
        write_.setDevice(&handle);
    }




//    static QFile handle(filename1);
//    if(biopsyStartFlag==0){handle.open(QIODevice::WriteOnly);}
//    static QTextStream write_(&handle);




    str = socket->readAll();
    if(!str.isEmpty()){
        strlist = str.split(",");
        if(!strlist.isEmpty()){
            data.x=strlist[0].toFloat();
            data.y=strlist[1].toFloat();
        }
    }


//    emit dataGet(data.y, data.x);
//    if(abs(data.y)<1e-11 && data.y!=0)//0.01
//    {
//        emit currentmutation();
//    }


//    fluctuation=abs((averageCurrent-data.y)/averageCurrent);//判断刷新速度为1000Hz
//    emit dataGet(data.y, fluctuation);
//    if(fluctuation>0.25&&chiPointBack.size()>=100)
//    {
//        emit currentmutation();
//    }

//    if(numFlag%1==0)//数据缓存速度为100Hz，缓存时间长度为1S
//    {
//        numFlag=0;
//        if(chiPoint.size()>=100){
//            if(chiPointBack.size()>=100){
//                sumCurrent-=chiPointBack.front().y;//去头
//                chiPointBack.pop();//去头
//            }
//            chiPointBack.push(chiPoint.front());//加尾
//            sumCurrent+=chiPoint.front().y;//加尾
//            chiPoint.pop();//去头
//        }
//        chiPoint.push(data);//加尾

        if(actOpen){
            if(manipulationSelection!=manipulationSelection_last){
                poseSave.dev=manipulationSelection;//设置控制对象
                Ump_Select_Dev(&poseSave);
                manipulationSelection_last=manipulationSelection;
            }
            Ump_Read_Position(&poseSave);
        }


        double timeControl=((double)getTickCount()-timeControl_Init)/getTickFrequency();
//        cout<<"Time:"<<timeControl<<", Ion_Current:"<<data.y<<", x:"<<poseSave.home_x<<", y:"<<poseSave.home_y<<", z:"<<poseSave.home_z<<", d:"<<poseSave.home_d<<endl;

        write_<<"Time "<<timeControl<<"  Time_Current "<<data.x<<"  Ion_Current "<<data.y
             <<"  x "<<poseSave.home_x<<"  y "<<poseSave.home_y<<"  z "<<poseSave.home_z<<"  d "<<poseSave.home_d
            <<"  voltage "<<sendVoltage<<"  frequency "<<sendFrequency<<"\n";
//        write_<<"Time "<<timeControl<<"  Time_Current "<<data.x<<"  Ion_Current "<<data.y
//             <<"  x "<<poseSave.home_x<<"  y "<<poseSave.home_y<<"  z "<<poseSave.home_z<<"  d "<<poseSave.home_d
//             <<"  targetX "<<targetX<<"  targetY "<<targetY<<"  targetZ "<<targetZ<<"  targetD "<<targetD
//            <<"  voltage "<<sendVoltage<<"  frequency "<<sendFrequency<<"\n";
        sendNowInfo(timeControl,data.y,poseSave.home_x,poseSave.home_y,poseSave.home_z,poseSave.home_d,sendVoltage,sendFrequency);
//        handle.close();

//    cout<<"x:"<<data.x<<", y:"<<data.y<<endl;
//    }

    numFlag++;

}

void Ch_Instrument::reConnection()
{

//    chiPoint = queue<Point2f> ();//清空数据
    cout<<"Waiting for CHInstrument Connection"<<endl;

}

void Ch_Instrument::chConnected()
{

    cout<<"CHInstrument Connected"<<endl;

}

void Ch_Instrument::averageSum()
{

    averageCurrent=sumCurrent/chiPoint.size();
//    cout<<"averageCurrent:"<<averageCurrent<<endl;

}

void Ch_Instrument::sendDigidataVol(int port, float voltage)
{
    if(socketW->state()==QAbstractSocket::ConnectedState){
        cout<<"port: "<<port<<"; voltage: "<<voltage<<endl;
        char str[50]={0};
        sprintf(str,"%d,%f",port,voltage);
        socketW->write(str, sizeof(str));
    }


}

void Ch_Instrument::decision()
{
    switch (chiSelection) {
    case 100:

//        if(actOpen){
//            poseSave.dev=manipulationSelection;//设置控制对象
//            Ump_Select_Dev(&poseSave);
//            Ump_Read_Position(&poseSave);
//        }

        timeChi=new QTimer();
        timeChi->setInterval(10);
        timeSum=new QTimer();
        timeSum->setInterval(2000);
        server = new QTcpServer(this);
        socketW = new QTcpSocket(this);
        server->listen(QHostAddress::LocalHost,1900);
        connect(server,&QTcpServer::newConnection,[=](){socketW= server->nextPendingConnection();});


        socket = new QTcpSocket();
        socket->abort();
        connect(socket, SIGNAL(readyRead()), this, SLOT(readClient()));
        connect(socket, SIGNAL(disconnected()), this, SLOT(reConnection()));
        connect(socket, SIGNAL(connected()), this, SLOT(chConnected()));
        connect(timeChi,&QTimer::timeout,this,&Ch_Instrument::decision);
//        connect(timeSum,&QTimer::timeout,this,&Ch_Instrument::averageSum);
        timeChi->start();
        timeSum->start();
        chiSelection=0;
        cout<<"waiting for CHInstruments "<<endl;
        break;
    case 0:
        if(socket->state()==0)
        {
            socket->connectToHost(QHostAddress::LocalHost, 1800);
            socket->waitForConnected(10);
        }
        break;
    case 1:
        break;
    case 2:
        break;
    case 3:
        break;
    case 4:
        break;
    case 5:
        break;
    case 6:
        break;
    default:
        chiSelection=-1;
        break;
    }
}
