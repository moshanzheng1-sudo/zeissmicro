#ifndef CHINSTRUMENT_H
#define CHINSTRUMENT_H

#include <QObject>
#include <queue>
#include "base_head.h"

using namespace std;

class Ch_Instrument : public QObject
{
    Q_OBJECT
public:
//    explicit Ch_Instrument(QObject *parent = nullptr);
    Ch_Instrument(params_struct &params);

    void decision();


    void acceptConnection();
    void averageSum();

    void sendDigidataVol(int port, float voltage);

    void getTargetPose(int x, int y, int z, int d);

    int targetX=0,targetY=0,targetZ=0,targetD=0;
    int8_t chiSelection=100;
    QTimer* timeChi;
    QTimer* timeSum;
    double averageCurrent=666666;
    int8_t manipulationSelection=1;

    params_struct poseSave;

    QTcpServer *server;
    QTcpSocket *socket;
    QTcpSocket *socketW;

    queue<Point2f> chiPoint;
    queue<Point2f> chiPointBack;
    double sumCurrent=0;

    double sendVoltage = 0.0;
    double sendFrequency = 1000000;

signals:
    void currentmutation();
    void dataGet(float current, float time);
    void sendNowInfo(double timeControl,double current,int x,int y,int z,int d,double sendVol,double sendFre);

public slots:
    void reConnection();
    void chConnected();
    void readClient();
};

#endif // CHINSTRUMENT_H
