#-------------------------------------------------
#
# Project created by QtCreator 2020-04-18T21:20:21
#
#-------------------------------------------------

QT       += core gui

QT       += charts
QT       += serialport
QT       += network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = MicroSystem
TEMPLATE = app
CONFIG += c++11
QMAKE_CXXFLAGS += /wd"4819"

DEFINES += WIN32_LEAN_AND_MEAN


SOURCES += main.cpp\
        mainwindow.cpp \
    line2Dup.cpp \
    multiclampcurve.cpp \
    patchclamp.cpp \
    ui_frame.cpp \
    servo_function.cpp \
    pose_kalman.cpp \
    sbaslercameracontrol.cpp \
    transform_format.cpp \
    decision_task.cpp \
    mylabel.cpp \
    chinstrument.cpp \
    algorithms.cpp \
    positioning_cell.cpp \
    positioning_tip.cpp

HEADERS  += mainwindow.h \
    line2Dup.h \
    multiclampcurve.h \
    patchclamp.h \
    ui_frame.h \
    servo_function.h \
    pose_kalman.h \
    sbaslercameracontrol.h \
    transform_format.h \
    decision_task.h \
    mylabel.h \
    chinstrument.h \
    base_head.h \
    algorithms.h \
    positioning_cell.h \
    positioning_tip.h

FORMS    += mainwindow.ui \
    multiclampcurve.ui \
    ui_frame.ui


#UI_DIR =./UI

UI_DIR =../Microsystem


win32: LIBS += -L$$PWD/pylon/lib/x64/ -lGCBase_MD_VC141_v3_1_Basler_pylon -lGenApi_MD_VC141_v3_1_Basler_pylon -lPylonBase_v6_1 -lPylonC -lPylonGUI_v6_1 -lPylonUtility_v6_1


INCLUDEPATH += $$PWD/pylon/include
DEPENDPATH += $$PWD/pylon/include


win32: LIBS += -L$$PWD/lib/ -lump

INCLUDEPATH += $$PWD/lib
DEPENDPATH += $$PWD/lib

INCLUDEPATH += $$PWD/library/include \
               $$PWD/library/src/include

DEPENDPATH += $$PWD/library/src/.

win32: LIBS += -L$$PWD/./ -llibrary

INCLUDEPATH += $$PWD/.
DEPENDPATH += $$PWD/.

#win32: LIBS += -L$$PWD/./ -lchi

#win32: LIBS += -L$$PWD/cintools/ -llabview
#win32: LIBS += -L$$PWD/cintools/ -llabviewv

INCLUDEPATH += $$PWD/cintools
DEPENDPATH += $$PWD/cintools


INCLUDEPATH +=  $$PWD/../Microsystem/opencv/include\
                $$PWD/../Microsystem/opencv/debug \
                $$PWD/../Microsystem/opencv/release \
                $$PWD/../Microsystem/opencv/include/opencv \
                $$PWD/../Microsystem/opencv/include/opencv2


DEPENDPATH +=   $$PWD/../Microsystem/opencv \
                $$PWD/../Microsystem/opencv/debug \
                $$PWD/../Microsystem/opencv/release \
                $$PWD/../Microsystem/opencv/include \
                $$PWD/../Microsystem/opencv/include/opencv \
                $$PWD/../Microsystem/opencv/include/opencv2


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/opencv/release/ -lopencv_world310
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/opencv/debug/ -lopencv_world310d


win32: LIBS += -L$$PWD/../Microsystem/patchclamp/ -lAxMultiClampMsg
INCLUDEPATH +=  $$PWD/../Microsystem/patchclamp
DEPENDPATH +=   $$PWD/../Microsystem/patchclamp

LIBS += -L$$PWD/Lib_x64/msc/ -lnivisa64
INCLUDEPATH += $$PWD/Lib_x64/msc
DEPENDPATH += $$PWD/Lib_x64/msc

LIBS += -L$$PWD/Lib_x64/msc/ -lvisa64

INCLUDEPATH += $$PWD/Lib_x64/msc
DEPENDPATH += $$PWD/Lib_x64/msc


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/pump_motor_lib/ -lvsmdlib
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/pump_motor_lib/ -lvsmdlibd

INCLUDEPATH += $$PWD/pump_motor_lib
DEPENDPATH += $$PWD/pump_motor_lib

DISTFILES += \
    !README_CODE_NEWS.txt
