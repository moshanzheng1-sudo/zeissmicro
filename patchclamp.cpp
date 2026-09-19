
#include "patchclamp.h"


patchclamp::patchclamp(QObject *parent) : QObject(parent)
{

}


/*********************************/
// FUNCTION: DisplayErrorMsg
// PURPOSE: Display error as text string
/*********************************/
void DisplayErrorMsg(HMCCMSG hMCCmsg, int nError)
{
    char szError[256] = "";
    MCCMSG_BuildErrorText(hMCCmsg, nError, szError, sizeof(szError));
    cout<<szError<<endl;
    //AfxMessageBox(szError, MB_ICONSTOP);
}

int patchclamp::pcStart()
{

    // check the API version matches the expected value
    if( !MCCMSG_CheckAPIVersion(MCCMSG_APIVERSION_STR) )
    {
        //AfxMessageBox("Version mismatch: AXCLAMPEXMSG.DLL", MB_ICONSTOP);
        cout<<"Version mismatch: AXCLAMPEXMSG.DLL"<<endl;
        return -1;
    }
    // create DLL handle

    hMCCmsg = MCCMSG_CreateObject(&nError);
    if( !hMCCmsg )
    {
        DisplayErrorMsg(hMCCmsg, nError);
        return -1;
    }

    // find the first MultiClamp
    char szError[256] = "";
    char szSerialNum[16] = ""; // Serial number of MultiClamp 700B
    UINT uModel = 0; // Identifies MultiClamp 700A or 700B model
    UINT uCOMPortID = 0; // COM port ID of MultiClamp 700A (1-16)
    UINT uDeviceID = 0; // Device ID of MultiClamp 700A (1-8)
    UINT uChannelID = 0; // Headstage channel ID

    if( !MCCMSG_FindFirstMultiClamp(hMCCmsg, &uModel, szSerialNum,sizeof(szSerialNum), &uCOMPortID,&uDeviceID, &uChannelID, &nError) )
    {
        DisplayErrorMsg(hMCCmsg, nError);
        return -1;
    }

    // select this MultiClamp
    if( !MCCMSG_SelectMultiClamp(hMCCmsg, uModel, szSerialNum,uCOMPortID, uDeviceID, uChannelID, &nError) )
    {
        DisplayErrorMsg(hMCCmsg, nError);
        return -1;
    }

    // set voltage clamp mode
    if( !MCCMSG_SetMode(hMCCmsg, MCCMSG_MODE_VCLAMP, &nError) )
    {
        DisplayErrorMsg(hMCCmsg, nError);
        return -1;
    }

    BOOL bEnable = TRUE;
    if( !MCCMSG_SetHoldingEnable(hMCCmsg, bEnable, &nError) )
    {
        DisplayErrorMsg(hMCCmsg, nError);
        return -1;
    }

//    // set the holding level
//    double dHolding = 0.3;
//    if( !MCCMSG_SetHolding(hMCCmsg, dHolding, &nError) )
//    {
//        DisplayErrorMsg(hMCCmsg, nError);
//        return -1;
//    }

//    bEnable = TRUE;
//    if( !MCCMSG_SetMeterResistEnable(hMCCmsg, bEnable, &nError) )
//    {
//        DisplayErrorMsg(hMCCmsg, nError);
//        return -1;
//    }

    cout<<"Multiclamp amplifier connected!!!"<<endl;

    // execute auto fast compensation
//    if( !MCCMSG_AutoFastComp(hMCCmsg, &nError) )
//    {
//        DisplayErrorMsg(hMCCmsg, nError);
//        return 0;
//    }

    // execute auto slow compensation
//    if( !MCCMSG_AutoSlowComp(hMCCmsg, &nError) )
//    {
//        DisplayErrorMsg(hMCCmsg, nError);
//        return 0;
//    }


    // destroy DLL handle
//    MCCMSG_DestroyObject(hMCCmsg);
//    hMCCmsg = NULL;
    return 2;

}


int patchclamp::pcStop()
{
    BOOL bEnable = FALSE;
    if( !MCCMSG_SetMeterResistEnable(hMCCmsg, bEnable, &nError) )
    {
        DisplayErrorMsg(hMCCmsg, nError);
        return -1;
    }
    // destroy DLL handle
    MCCMSG_DestroyObject(hMCCmsg);
    hMCCmsg = NULL;
    return -1;

}


int patchclamp::pcDataGet()//没用
{
    // get the primary output
    double currentData=0;
    if( !MCCMSG_SetPrimarySignal(hMCCmsg, MCCMSG_PRI_SIGNAL_VC_MEMBCURRENT, &nError) )
    {
        DisplayErrorMsg(hMCCmsg, nError);
        return -1;
    }
    if( !MCCMSG_GetMeterValue(hMCCmsg, &currentData, MCCMSG_METER1, &nError) )
    {
        DisplayErrorMsg(hMCCmsg, nError);
        return -1;
    }
    cout<<currentData<<endl;
    return 2;//继续读取数据

}


int patchclamp::pcSetVoltageWaveGenerator(double voltage)
{

    // 获取用户输入的操作符和控制参数
    QString pyPath= "../Microsystem/py_wave_generator/tektronix_func_gen.py";
    QString pythonCommand = "D:/ANACONDA/envs/wave_generator/python";

    QString op= "SIN";
    double WaveFrequency = 10.0, WaveAmplitude = voltage, WaveOffset = 10.0;
    if(op=="SIN")
    {
        WaveFrequency = 0.001;//UNIT:Hz
        WaveAmplitude = 0.001;//UNIT:V
        WaveOffset = voltage;//UNIT:V
    } else if(op=="DC"){
        WaveFrequency = 0;//UNIT:Hz
        WaveAmplitude = voltage;//UNIT:V
        WaveOffset = voltage;//UNIT:V
    }
    QStringList pythonArguments;
    pythonArguments << pyPath << op <<QString::number(WaveFrequency)<<QString::number(WaveAmplitude)<<QString::number(WaveOffset);
    QProcess process;
    process.start(pythonCommand, pythonArguments);
    process.waitForFinished(-1);
    // 获取Python输出结果并显示
    QString output = process.readAllStandardOutput();
    float result = output.toFloat();
    cout << "The result is:" << result <<endl;
    process.kill();

//    /*调用py计算器*/
//    // 获取用户输入的操作符和两个数字
//    QString op= "+";
//    QString pyPath= "../Microsystem/py_wave_generator/calc.py";
//    float num1 = 10.0, num2 = 1.0;
//    // 调用Python脚本进行运算
//    QString pythonCommand = "D:/ANACONDA/envs/wave_generator/python";
//    QStringList pythonArguments;
//    pythonArguments << pyPath << op << QString::number(num1) << QString::number(num2);
//    QProcess process;
//    process.start(pythonCommand, pythonArguments);
//    process.waitForFinished(-1);
//    // 获取Python输出结果并显示
//    QString output = process.readAllStandardOutput();
//    float result = output.toFloat();
//    cout << "The result is:" << result <<endl;
//    process.kill();

    return 0;

}


void patchclamp::writeDev(int Vfre,int Vamp,int Voff)//写 频率 幅值 偏执 单位分别为mHz，mV，mVpp
{

    //模式设置
    sprintf(funcpara[0],"SOURce1:FUNCtion:SHAPe %s","SIN");
    status = viWrite (instrument, (ViBuf)funcpara[0], (ViUInt32)strlen(funcpara[0]), &writeCount);
    //频率设置
     sprintf(funcpara[1],"SOURce1:FREQuency:FIXed %dHz",Vfre);
     status=viWrite(instrument, (ViBuf)funcpara[1], (ViUInt32)strlen(funcpara[1]), &writeCount);
    //幅值设置
    sprintf(funcpara[2],"SOURce1:VOLTage:LEVel:IMMediate:OFFSet %dmV",Voff);// 只能用int类型
    status = viWrite (instrument, (ViBuf)funcpara[2], (ViUInt32)strlen(funcpara[2]), &writeCount);
    //偏置设置
    sprintf(funcpara[3],"SOURce1:VOLTage:LEVel:IMMediate:AMPLitude %dmVpp",Vamp);//只能用int类型
    status = viWrite (instrument, (ViBuf)funcpara[3], (ViUInt32)strlen(funcpara[3]), &writeCount);
    //通道打开
    status = viWrite (instrument, (ViBuf)funcpara[4], (ViUInt32)strlen(funcpara[4]), &writeCount);
}
void patchclamp::closeDev()//关闭仪器
{
    status = viWrite (instrument, (ViBuf)funcpara[5], (ViUInt32)strlen(funcpara[5]), &writeCount);
    viClose(instrument);
    viClose(defaultRM);
}
void patchclamp:: get_amp()//获取幅值
{

//    status=viRead(instrument, (ViBuf)getpara[0], (ViUInt32)strlen(getpara[0]), &retCount);
//    return (float)retCount;
    status = viWrite (instrument, (ViBuf)getpara[0], (ViUInt32)strlen(getpara[0]), &writeCount);
    status = viRead (instrument, buffer, 100, &retCount);
       if (status < VI_SUCCESS)
       {
          printf("Error reading a response from the device\n");
       }
       else
       {
          qDebug("%*s",retCount,buffer);
       }
//       double midnum=strtod(buffer,NULL);
//       double f=atof(buffer);

//       return (float)retCount;
       //HexPrint(buffer,sizeof(buffer));
}
void patchclamp::get_off()//获取偏置
{
    status = viWrite (instrument, (ViBuf)getpara[1], (ViUInt32)strlen(getpara[1]), &writeCount);
    status = viRead (instrument, buffer, 100, &retCount);
       if (status < VI_SUCCESS)
       {
          printf("Error reading a response from the device\n");
       }
       else
       {
          qDebug("%*s",retCount,buffer);
       }
}


int patchclamp::pcWaveGeneratorStart()
{
    status = viOpenDefaultRM (&defaultRM);//启动visa
    if(status < VI_SUCCESS){
        qDebug()<<"Could not open a session to the VISA Resource Manager";
        return -1;
    }
    status=viOpen(defaultRM,VISA_ADDRESS[0],VI_NULL,VI_NULL,&instrument);//打开我设备与软件的通道
    if(status < VI_SUCCESS){
            qDebug()<<"Could not open Device";
            viClose(defaultRM);
            return -1;
        }
    return 2;

}

int8_t patchclamp::motor_init(){

    vsmd = Vsmd::createVsmd((char*)"COM4", 9600);
    device = vsmd->createDevice(1);
    device->autoStsOn();

    if (vsmd->open())
    {
        qDebug() << "open serial port ok...\n\r";
        return 2;
    }
    else
    {
        qDebug() << "open serial port error...\n\r";
        vsmd->close();
        vsmd = nullptr;
        device = nullptr;
        return -1;
    }

}

int8_t patchclamp::motor_control(int8 controlFlag, double distance){

    static int enableFlag=1;

    double mcsNum=16.0;

    switch (controlFlag) {
    case 1:
        if(enableFlag){
            if(device)
            {
                device->enable();
                device->cfgMcs(mcsNum);//16细分，3200脉冲一圈

                device->cfgSpd(mcsNum*200.0*(360.0/360.0)*20);
                device->cfgAcc(mcsNum*200.0*(360.0/360.0)*20);
                device->cfgDec(mcsNum*200.0*(360.0/360.0)*20);
                initPose=device->getCurPos();
//                device->cfgSpd(mcsNum*200.0*(270.0/360.0));//90°每秒
            }
            enableFlag=0;
        }
        if(device)
        {
            //控制位置
            distance=distance*mcsNum*200.0/360.0;//由角度变为脉冲数
            device->rmove(distance);//相对位置
//            initPoseMark=device->getCurPos();
//            device->moveto(distance+initPoseMark);//绝对位置
//            cout<<"initPose: "<<initPose<<"distance: "<<distance<<"initPoseMark: "<<initPoseMark<<endl;

            //控制速度
//            distance=distance*mcsNum*200.0/360.0;//单位为度每秒
//            device->cfgSpd(distance);//单位为脉冲每秒
//            device->move();
//            cout<<"now speed: "<<distance<<endl;
        }
        break;
    case 2:
        if(device)
        {
            device->stop(1);
        }
        break;
    case 3:
        if(device)
        {
            device->disable();
        }
        enableFlag=1;

        break;
    default:
        break;
    }

    return 0;

}

int patchclamp::pcSetVoltage(double voltage)
{
    if(pumpOpen){
        if(motorOpenFlag){
//            if(abs(voltage)<0.001){
//                motor_control(2, 0);
//            }else{
                motor_control(1, voltage);
//            }

        }else if(voltageFromWaveGenerator){
            //信号发生器
//           pcSetVoltageWaveGenerator(voltage);//基于Python

//            writeDev(1,1,(int)(voltage*1000));//基于C++ 单位为毫伏
            writeDev(volTagefre,(int)(voltage*1000),0);//基于C++ 单位为毫伏

    //        get_off();
    //        cout<<retCount<<endl;


        }else{
            //膜片钳
//            if( !MCCMSG_SetHolding(hMCCmsg, voltage, &nError))
//            {
//                DisplayErrorMsg(hMCCmsg, nError);
//                return -1;
//            }
            emit sendDigidataVoltage(7, voltage);

        }
    }

    return 2;//继续读取数据
}


void patchclamp::decision()
{
    switch (patchSelection) {
    case 100:
        timePatch=new QTimer();
        timePatch->setInterval(10);
        connect(timePatch,&QTimer::timeout,this,&patchclamp::decision);
        timePatch->start();
        patchSelection=1;
        cout<<"waiting for patchClamp "<<endl;
        break;
    case -1:
        countN++;
        if(countN>6000)//60s自动重连
        {
            patchSelection=1;
            countN=0;
        }
        break;
    case 1:
        if(motorOpenFlag){
            patchSelection=motor_init();
        }else if(voltageFromWaveGenerator){
            patchSelection=pcWaveGeneratorStart();
        }else{
//            patchSelection=pcStart();
        }
        break;
    case 2:
//        pcDataGet();
        break;
    case 3:
        patchSelection=pcSetVoltage(Vol);
        break;
    case 4:
        break;
    case 5:
        break;
    case 6:
        break;
    default:
        patchSelection=-2;
        break;
    }
}
