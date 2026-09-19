#ifndef NDITRACKING_H
#define NDITRACKING_H

#include<stdlib.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <QThread>
#include <QTimer>


#include "CombinedApi.h"
#include "PortHandleInfo.h"
#include "ToolData.h"

#include <Windows.h> //sleep


class ndiTracking: public QThread
{
    Q_OBJECT
public:
    ndiTracking();
    void onErrorPrintDebugMessage(std::string methodName, int errorCode);
    std::string getToolInfo(std::string toolHandle);
    std::string toolDataToCSV(const ToolData& toolData);
    void writeCSV(std::string fileName, int numberOfLines);
    void printToolData(const ToolData& toolData);
    void printTrackingData();
    void initializeAndEnableTools();
    void loadTool(const char* toolDefinitionFilePath);
    void configurePassiveTools();
    void configureActiveTools(std::string scuHostname);
    void configureActiveWirelessTools();
    void configureDummyTools();
    void configureUserParameters();
    void simulateAlerts(uint32_t simulatedAlerts = 0x00000000);
    void determineApiSupportForBX2();
    int ndiInit();
    ToolData *ndiGetData();

    void run() Q_DECL_OVERRIDE;

    CombinedApi *capi;
    bool apiSupportsBX2 = false;
    QTimer *ndiTimer;
    int controlFlag=0;
    std::string com;
    std::vector<ToolData> toolData;

signals:
    void dataShow(ToolData *data);
};

#endif // NDITRACKING_H
