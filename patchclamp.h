#ifndef PATCHCLAMP_H
#define PATCHCLAMP_H

#include <windows.h>
#include <QObject>
#include <queue>
#include "base_head.h"
#include "AxMultiClampMsg.h"

#include <visa.h>
#include "visatype.h"
#include "vsmdlib.h"


class patchclamp: public QObject
{
    Q_OBJECT
public:
    patchclamp(QObject *parent = nullptr);
    void decision();
    int pcStart();
    int pcDataGet();
    int pcSetVoltage(double voltage);

    int pcWaveGeneratorStart();
    int pcSetVoltageWaveGenerator(double voltage);
    int pcStop();

    int8_t motor_init();
    int8_t motor_control(int8 controlFlag, double distance);

    void openDev();
    void writeDev(int Vfre,int Vamp,int Voff);
    void closeDev();
    char funcpara[6][50]={"",
                          "",
                          "",
                          "",
                          "OUTPut1:STATe ON",
                         "OUTPut1:STATe OFF"};//波形选择，频率，幅值 偏执 输出口开关设置
    char getpara[2][50]={"SOURce1:VOLTage:LEVel:IMMediate:AMPLitude?",
                        "SOURce1:VOLTage:LEVel:IMMediate:OFFSet?"};
    void get_amp();
    void get_off();
    double volTageoff=3;
    double volTageamp=2;
    int volTagefre=1000000;//默认1 MHz

    ViRsrc DevName; //设备名称
    ViStatus status; //状态
    ViSession defaultRM,instrument;//会话
    ViUInt32 retCount;
    ViUInt32 writeCount;
    unsigned char buffer[100];
    char VISA_ADDRESS[2][50]={"USB0::0x0699::0x0353::1725304::INSTR","USB0::0x0699::0x035C::C012051::INSTR"};//instrument的连接地址 第一个为AFG1062 USB 地址 第二个为AFG31152USB 地址


    //Motor
    int8 motorOpenFlag=motorOpenFlagBase;
    Vsmd* vsmd;
    VsmdDevice* device;

    int countN=0;
//    int voltageFromWaveGenerator=1;
    int initPoseMark=0;
    int initPose=0;

    double Vol=0.85;
    int nError = MCCMSG_ERROR_NOERROR;
    HMCCMSG hMCCmsg;
    int patchSelection=100;
    QTimer* timePatch;

signals:
    void dataShow(QPointF current);
    void sendDigidataVoltage(int port, float voltage);
};

#endif // PATCHCLAMP_H
