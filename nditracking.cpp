#include "nditracking.h"

using namespace std;

ndiTracking::ndiTracking()
{
    moveToThread(this);

}



/**
 * @brief Prints a debug message if a method call failed.
 * @details To use, pass the method name and the error code returned by the method.
 *          Eg: onErrorPrintDebugMessage("this->capi->initialize()", this->capi->initialize());
 *          If the call succeeds, this method does nothing.
 *          If the call fails, this method prints an error message to stdout.
 */
void ndiTracking::onErrorPrintDebugMessage(std::string methodName, int errorCode)
{
    if (errorCode < 0)
    {
        std::cout << methodName << " failed: " << this->capi->errorToString(errorCode) << std::endl;
    }
}

/**
* @brief Returns the string: "[tool.id] s/n:[tool.serialNumber]" used in CSV output
*/
std::string ndiTracking::getToolInfo(std::string toolHandle)
{
    // Get the port handle info from PHINF
    PortHandleInfo info = capi->portHandleInfo(toolHandle);

    // Return the ID and SerialNumber the desired string format
    std::string outputString = info.getToolId();
    outputString.append(" s/n:").append(info.getSerialNumber());
    return outputString;
}

/**
* @brief Returns a string representation of the data in CSV format.
* @details The CSV format is: "Frame#,ToolHandle,Face,TransformStatus,q0,qx,qy,qz,tx,ty,tz,error,#markers,[Marker1:status,x,y,z],[Marker2..."
*/
std::string ndiTracking::toolDataToCSV(const ToolData& toolData)
{
    std::stringstream stream;
    stream << std::setprecision(toolData.PRECISION) << std::setfill('0');
    stream << "" << static_cast<unsigned>(toolData.frameNumber) << ","
           << "Port:" << static_cast<unsigned>(toolData.transform.toolHandle) << ",";
    stream << static_cast<unsigned>(toolData.transform.getFaceNumber()) << ",";

    if (toolData.transform.isMissing())
    {
        stream << "Missing,,,,,,,,";
    }
    else
    {
        stream << TransformStatus::toString(toolData.transform.getErrorCode()) << ","
               << toolData.transform.q0 << "," << toolData.transform.qx << "," << toolData.transform.qy << "," << toolData.transform.qz << ","
               << toolData.transform.tx << "," << toolData.transform.ty << "," << toolData.transform.tz << "," << toolData.transform.error;
    }

    // Each marker is printed as: status,tx,ty,tz
    stream << "," << toolData.markers.size();
    for ( int i = 0; i < toolData.markers.size(); i++)
    {
        stream << "," << MarkerStatus::toString(toolData.markers[i].status);
        if (toolData.markers[i].status == MarkerStatus::Missing)
        {
            stream << ",,,";
        }
        else
        {
            stream << "," << toolData.markers[i].x << "," << toolData.markers[i].y << "," << toolData.markers[i].z;
        }
    }
    return stream.str();
}

/**
 * @brief Write tracking data to a CSV file in the format: "#Tools,ToolInfo,Frame#,[Tool1],Frame#,[Tool2]..."
 * @details It's worth noting that the number lines in the file does not necessarily match the number of frames collected.
 *          NDI measurement systems support different types of tools: passive, active, and active-wireless.
 *          Because different tool types are detected in different physical ways, each tool type has a separate frame for
 *          collecting data for all tools of that type. Each line of the file has the same number of tools, but each tool
 *          may have a different frame number that corresponds to its tool type.
 * @param fileName The file to write to.
 * @param numberOfLines The number of lines to write
 */
void ndiTracking::writeCSV(std::string fileName, int numberOfLines)
{
    // Assumption: tools are not enabled/disabled during CSV output
    std::vector<PortHandleInfo> portHandles = capi->portHandleSearchRequest(PortHandleSearchRequestOption::Enabled);
    if (portHandles.size() < 1)
    {
        std::cout << "Cannot write CSV file when no tools are enabled!" << std::endl;
        return;
    }

    // Lookup and store the serial number for each enabled tool
    std::vector<ToolData> enabledTools;
    for (int i = 0; i < portHandles.size(); i++)
    {
        enabledTools.push_back(ToolData());
        enabledTools.back().transform.toolHandle = (uint16_t) capi->stringToInt(portHandles[i].getPortHandle());
        enabledTools.back().toolInfo = getToolInfo(portHandles[i].getPortHandle());
    }

    // Start tracking
    std::cout << std::endl << "Entering tracking mode and collecting data for CSV file..." << std::endl;
    onErrorPrintDebugMessage("this->capi->startTracking()", this->capi->startTracking());

    // Print header information to the first line of the output file
    std::cout << std::endl << "Writing CSV file..." << std::endl;
    std::ofstream csvFile(fileName.c_str());
    csvFile << "#Tools,ToolInfo,Frame#,PortHandle,Face#,TransformStatus,Q0,Qx,Qy,Qz,Tx,Ty,Tz,Error,Markers,State,Tx,Ty,Tz" << std::endl;

    // Loop to gather tracking data and write to the file
    int linesWritten = 0;
    int previousFrameNumber = 0; // use this variable to avoid printing duplicate data with BX
    while (linesWritten < numberOfLines)
    {
        // Get new tool data using BX2
        std::vector<ToolData> newToolData = this->apiSupportsBX2 ? this->capi->getTrackingDataBX2("--6d=tools --3d=tools --sensor=none --1d=buttons") :
                                                             this->capi->getTrackingDataBX(TrackingReplyOption::TransformData | TrackingReplyOption::AllTransforms);

        // Update enabledTools array with new data
        for (int t = 0; t < enabledTools.size(); t++)
        {
            for (int j = 0; j < newToolData.size(); j++)
            {
                if (enabledTools[t].transform.toolHandle == newToolData[j].transform.toolHandle)
                {
                    // Copy the new tool data
                    newToolData[j].toolInfo = enabledTools[t].toolInfo; // keep the serial number
                    enabledTools[t] = newToolData[j]; // use the new data
                }
            }
        }

        // If we're using BX2 there's extra work to do because BX2 and BX use opposite philosophies.
        // BX will always return data for all enabled tools, but some of the data may be old: #linesWritten == # BX commands
        // BX2 never returns old data, but cannot guarantee new data for all enabled tools with each call: #linesWritten <= # BX2 commands
        // We want a CSV file with data for all enabled tools in each line, but this requires multiple BX2 calls.
        if (this->apiSupportsBX2)
        {
            // Count the number of tools that have new data
            int newDataCount = 0;
            for (int t = 0; t < enabledTools.size(); t++)
            {
                if (enabledTools[t].dataIsNew)
                {
                    newDataCount++;
                }
            }

            // Send another BX2 if some tools still have old data
            if (newDataCount < enabledTools.size())
            {
                continue;
            }
        }
        else
        {
            if (previousFrameNumber == enabledTools[0].frameNumber)
            {
                // If the frame number didn't change, don't print duplicate data to the CSV, send another BX
                continue;
            }
            else
            {
                // This frame number is different, so we'll print a line to the CSV, but remember it for next time
                previousFrameNumber = enabledTools[0].frameNumber;
            }
        }

        // Print a line of the CSV file if all enabled tools have new data
        csvFile << std::dec << enabledTools.size();
        for (int t = 0; t < enabledTools.size(); t++)
        {
            csvFile << "," << enabledTools[t].toolInfo << "," << toolDataToCSV(enabledTools[t]);
            enabledTools[t].dataIsNew = false; // once printed, the data becomes "old"
        }
        csvFile << std::endl;
        linesWritten++;
    }

    // Stop tracking and return to configuration mode
    onErrorPrintDebugMessage("this->capi->stopTracking()", this->capi->stopTracking());
}

/**
 * @brief Prints a ToolData object to stdout
 * @param toolData The data to print
 */
void ndiTracking::printToolData(const ToolData& toolData)
{
    if (toolData.systemAlerts.size() > 0)
    {
        std::cout << "[" << toolData.systemAlerts.size() << " alerts] ";
        for (int a = 0; a < toolData.systemAlerts.size(); a++)
        {
            std::cout << toolData.systemAlerts[a].toString() << std::endl;
        }
    }

    if (toolData.buttons.size() > 0)
    {
        std::cout << "[buttons: ";
        for (int b = 0; b < toolData.buttons.size(); b++)
        {
            std::cout << ButtonState::toString(toolData.buttons[b]) << " ";
        }
        std::cout << "] ";
    }
    std::cout << toolDataToCSV(toolData) << std::endl;
}

/**
 * @brief Put the system into tracking mode, and get a few frames of data.
 */
void ndiTracking::printTrackingData()
{
    // Start tracking, output a few frames of data, and stop tracking
    std::cout << std::endl << "Entering tracking mode and collecting data..." << std::endl;
    onErrorPrintDebugMessage("this->capi->startTracking()", this->capi->startTracking());
    for (int i = 0; i < 10; i++)
    {
        // Demonstrate TX command: ASCII command sent, ASCII reply received
        std::cout << this->capi->getTrackingDataTX() << std::endl;

        // Demonstrate BX or BX2 command
        std::vector<ToolData> toolData =  this->apiSupportsBX2 ? this->capi->getTrackingDataBX2() : this->capi->getTrackingDataBX();

        // Print to stdout in similar format to CSV
        std::cout << "[alerts] [buttons] Frame#,ToolHandle,Face#,TransformStatus,Q0,Qx,Qy,Qz,Tx,Ty,Tz,Error,#Markers,State,Tx,Ty,Tz" << std::endl;
        for (int i = 0; i < toolData.size(); i++)
        {
            printToolData(toolData[i]);
        }
    }

    // Stop tracking (back to configuration mode)
    std::cout << std::endl << "Leaving tracking mode and returning to configuration mode..." << std::endl;
    onErrorPrintDebugMessage("this->capi->stopTracking()", this->capi->stopTracking());
}

/**
 * @brief Initialize and enable loaded tools. This is the same regardless of tool type.
 */
void ndiTracking::initializeAndEnableTools()
{
    std::cout << std::endl << "Initializing and enabling tools..." << std::endl;

    // Initialize and enable tools
    std::vector<PortHandleInfo> portHandles = capi->portHandleSearchRequest(PortHandleSearchRequestOption::NotInit);
    for (int i = 0; i < portHandles.size(); i++)
    {
        onErrorPrintDebugMessage("this->capi->portHandleInitialize()", this->capi->portHandleInitialize(portHandles[i].getPortHandle()));
        onErrorPrintDebugMessage("this->capi->portHandleEnable()", this->capi->portHandleEnable(portHandles[i].getPortHandle()));
    }

    // Print all enabled tools
    portHandles = capi->portHandleSearchRequest(PortHandleSearchRequestOption::Enabled);
    for (int i = 0; i < portHandles.size(); i++)
    {
        std::cout << portHandles[i].toString() << std::endl;
    }
}

/**
 * @brief Loads a tool from a tool definition file (.rom)
 */
void ndiTracking::loadTool(const char* toolDefinitionFilePath)
{
    // Request a port handle to load a passive tool into
    int portHandle = this->capi->portHandleRequest();
    onErrorPrintDebugMessage("this->capi->portHandleRequest()", portHandle);

    // Load the .rom file using the previously obtained port handle
    this->capi->loadSromToPort(toolDefinitionFilePath, portHandle);
}

/**
 * @brief Demonstrate loading passive tools.
 * @details Passive tools use NDI spheres to passively reflect IR light to the cameras.
 */
void ndiTracking::configurePassiveTools()
{
    // Load a few passive tool definitions from a .rom files
    std::cout << std::endl << "Configuring Passive Tools - Loading .rom Files..." << std::endl;
    loadTool("sroms/8700338.rom");
    loadTool("sroms/8700340.rom");
}

/**
 * @brief Demonstrate detecting active tools.
 * @details Active tools are connected through a System Control Unit (SCU) with physical wires.
 */
void ndiTracking::configureActiveTools(std::string scuHostname)
{
    // Setup the SCU connection for demonstrating active tools
    std::cout << std::endl << "Configuring Active Tools - Setup SCU Connection" << std::endl;
    onErrorPrintDebugMessage("this->capi->setUserParameter()", this->capi->setUserParameter("Param.Connect.SCU Hostname", scuHostname));
    std::cout << this->capi->getUserParameter("Param.Connect.SCU Hostname") << std::endl;

    // Wait a few seconds for the SCU to detect any wired tools plugged in
    std::cout << std::endl << "Demo Active Tools - Detecting Tools..." << std::endl;
    Sleep(2000);

    // Print all port handles
    std::vector<PortHandleInfo> portHandles = capi->portHandleSearchRequest(PortHandleSearchRequestOption::NotInit);
    for (int i = 0; i < portHandles.size(); i++)
    {
        std::cout << portHandles[i].toString() << std::endl;
    }
}

/**
 * @brief Demonstrate loading an active wireless tool.
 * @details Active wireless tools are battery powered and emit IR in response to a chirp from the illuminators.
 */
void ndiTracking::configureActiveWirelessTools()
{
    // Load an active wireless tool definitions from a .rom files
    std::cout << std::endl << "Configuring an Active Wireless Tool - Loading .rom File..." << std::endl;
    loadTool("sroms/active-wireless.rom");
}

/**
 * @brief Demonstrate loading dummy tools of each tool type.
 * @details Dummy tools are used to report 3Ds in the absence of real tools.
 *          Dummy tools should not be loaded with regular tools of the same type.
 *          TSTART will fail if real and dummy tools are enabled simultaneously.
 */
void ndiTracking::configureDummyTools()
{
    std::cout << std::endl << "Loading passive, active-wireless, and active dummy tools..." << std::endl;
    onErrorPrintDebugMessage("this->capi->loadPassiveDummyTool()", this->capi->loadPassiveDummyTool());
    onErrorPrintDebugMessage("this->capi->loadActiveWirelessDummyTool()", this->capi->loadActiveWirelessDummyTool());
    onErrorPrintDebugMessage("this->capi->loadActiveDummyTool()", this->capi->loadActiveDummyTool());
}

/**
 * @brief Demonstrate getting/setting user parameters.
 */
void ndiTracking::configureUserParameters()
{
    std::cout << this->capi->getUserParameter("Param.User.String0") << std::endl;
    onErrorPrintDebugMessage("this->capi->setUserParameter(Param.User.String0, customString)", this->capi->setUserParameter("Param.User.String0", "customString"));
    std::cout << this->capi->getUserParameter("Param.User.String0") << std::endl;
    onErrorPrintDebugMessage("this->capi->setUserParameter(Param.User.String0, emptyString)", this->capi->setUserParameter("Param.User.String0", ""));
}

/**
 * @brief Sets the user parameter "Param.Simulated Alerts" to test communication of system alerts.
 * @details This method does nothing if simulatedAlerts is set to 0x00000000.
 */
void ndiTracking::simulateAlerts(uint32_t simulatedAlerts)
{
    // Simulate alerts if any were requested
    if (simulatedAlerts > 0x0000)
    {
        std::cout << std::endl << "Simulating system alerts..." << std::endl;
        std::stringstream stream;
        stream << simulatedAlerts;
        onErrorPrintDebugMessage("this->capi->setUserParameter(Param.Simulated Alerts, alerts)", this->capi->setUserParameter("Param.Simulated Alerts", stream.str()));
        std::cout << this->capi->getUserParameter("Param.Simulated Alerts") << std::endl;
    }
}

/**
 * @brief Determines whether an NDI device supports the BX2 command by looking at the API revision
 */
void ndiTracking::determineApiSupportForBX2()
{
    // Lookup the API revision
    std::string response = this->capi->getApiRevision();

    // Refer to the API guide for how to interpret the APIREV response
    char deviceFamily = response[0];
    int majorVersion = this->capi->stringToInt(response.substr(2,3));

    // As of early 2017, the only NDI device supporting BX2 is the Vega
    // Vega is a Polaris device with API major version 003
    if ( deviceFamily == 'G' && majorVersion >= 3)
    {
        this->apiSupportsBX2 = true;
    }
}

/**
 * @brief   The entry point for the this->capisample application.
 * @details The invocation of this->capisample is expected to pass a few arguments: ./this->capisample [hostname] [scu_hostname]
 *          arg(0) - (default)  The path to this application (ignore this)
 *          arg(1) - (required) The measurement device's hostname, IP address, or serial port.
 *          Eg: Connecting to device by IP address: "169.254.8.50"
 *          Eg: Connecting to device by zeroconf hostname: "P9-B0103.local"
 *          Eg: Connecting to serial port varies by OS: "COM10" (Win), "/dev/ttyUSB0" (Linux), "/dev/cu.usbserial-001014FA" (Mac)
 *          arg(2) - (optional) A System Control Unit (SCU) hostname, used to connect active tools.
 */
int ndiTracking::ndiInit()
{
    capi=new CombinedApi;
//    // Validate the number of arguments received
//    if (argc < 2 || argc > 3)
//    {
//        std::cout << "this->capisample Ver " << this->capi->getVersion() << std::endl
//                  << "usage: ./this->capisample <hostname> [<scu_hostname>]" << std::endl
//                  << "where:" << std::endl
//                  << "    <hostname>      (required) The measurement device's hostname, IP address, or serial port." << std::endl
//                  << "    <scu_hostname>  (optional) A System Control Unit (SCU) hostname, used to connect active tools." << std::endl
//                  << "example hostnames:" << std::endl
//                  << "    Connecting to device by IP address: 169.254.8.50" << std::endl
//                  << "    Connecting to device by hostname: P9-B0103.local" << std::endl
//                  << "    Connecting to serial port varies by operating system:" << std::endl
//                  << "        COM10 (Windows), /dev/ttyUSB0 (Linux), /dev/cu.usbserial-001014FA (Mac)" << std::endl;
//        return -1;
//    }

//    // Ignore argv[0], and assign the hostname and scu_hostname accordingly
//    std::string hostname = std::string(argv[1]);
//    std::string scu_hostname = (argc == 3) ? std::string(argv[2]) : "";

    // Ignore argv[0], and assign the hostname and scu_hostname accordingly
    std::string hostname=this->com;
    std::string scu_hostname="";

    // Attempt to connect to the device
    if (capi->connect(hostname) != 0)
    {
        // Print the error and exit if we can't connect to a device
        std::cout << "Connection Failed!" << std::endl;
        std::cout << "Press Enter to continue...";

        std::cin.ignore();
        return -1;
    }
    std::cout << "Connected!" << std::endl;

    // Wait a second - needed to support connecting to LEMO Vega
    Sleep(1000);


    // Print the firmware version for debugging purposes
    std::cout << capi->getUserParameter("Features.Firmware.Version") << std::endl;

    // Determine if the connected device supports the BX2 command
    determineApiSupportForBX2();

    // Initialize the system. This clears all previously loaded tools, unsaved settings etc...
    onErrorPrintDebugMessage("capi->initialize()", capi->initialize());

    // Demonstrate error handling by asking for tracking data in the wrong mode
    std::cout << capi->getTrackingDataTX() << std::endl;

    // Demonstrate getting/setting user parameters
    configureUserParameters();

    // Various tool types are configured in slightly different ways
    configurePassiveTools();
    configureActiveWirelessTools();
    if (scu_hostname.length() > 0)
    {
        configureActiveTools(scu_hostname);
    }

    // Dummy tools are used to report 3Ds in the absence of real tools.
    // TSTART will fail if real and dummy tools of the same type are enabled simultaneously.
    // To experiment with dummy tools, comment out the previous tool configurations first.
    // configureDummyTools();

    // Once loaded or detected, tools are initialized and enabled the same way
    initializeAndEnableTools();

    // Spoofing system faults is handy to see how system alerts are handled, but faults can
    // prevent tracking data from being returned. This sample application has valued code simplicity
    // above robustness, so it does not gracefully handle every error case. You can experiment with
    // system faults, but attempting to do other tasks (eg. writing a .csv of tracking data) may fail
    // depending on what faults have been simulated.
    // simulateAlerts(0x7FFFFFFF);
    return 0;
}

ToolData *ndiTracking::ndiGetData()
{

    switch (controlFlag) {
    case 0:
        controlFlag++;

        // Start tracking, output a few frames of data, and stop tracking
        std::cout << std::endl << "Entering tracking mode and collecting data..." << std::endl;
        onErrorPrintDebugMessage("this->capi->startTracking()", capi->startTracking());

        break;
    case 1:
        // Demonstrate TX command: ASCII command sent, ASCII reply received
        capi->getTrackingDataTX();

        // Demonstrate BX or BX2 command
        toolData =  this->apiSupportsBX2 ? capi->getTrackingDataBX2() : capi->getTrackingDataBX();

        // Print to stdout in similar format to CSV
//        std::cout << "[alerts] [buttons] Frame#,ToolHandle,Face#,TransformStatus,Q0,Qx,Qy,Qz,Tx,Ty,Tz,Error,#Markers,State,Tx,Ty,Tz" << std::endl;
//        for (int i = 0; i < toolData.size(); i++)
//        {
//            printToolData(toolData[i]);
//           // emit dataShow(&toolData[i]);
//        }
        break;
    case 2:
        controlFlag++;
        // Stop tracking (back to configuration mode)
        std::cout << std::endl << "Leaving tracking mode and returning to configuration mode..." << std::endl;
        onErrorPrintDebugMessage("this->capi->stopTracking()", this->capi->stopTracking());
        break;
    default:
        quit();
        break;

    }

    // Write a CSV file
//    writeCSV("example.csv", 50);
    return toolData.data();
}

void ndiTracking::run()
{
        ndiInit();
//    {
        controlFlag=0;
        ndiGetData();
//        ndiTimer=new QTimer;
//        connect(ndiTimer,&QTimer::timeout,this,&ndiTracking::ndiGetData);
//        ndiTimer->start(50);
//        exec();//开始事件循环
//    }


}
