#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#if _MSC_VER >= 1600
#pragma execution_character_set("utf-8")

#endif


#include <QMainWindow>
#include "decision_task.h"


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    
    Decision_Task *decision_task;

    double volumeToDistance(double arg1);

signals:
     void sendRatio(float x,float y);
     void penetrationDepthControl(int depth);
     void penetrationTimeControl(int time);
     void liftheightControl(int height);
     void sendManualControl(int axis, int direction);
     void sendZeroSet();
     void sendZeroReturn();



//     void upcontrol();
//     void downcontrol();

private slots:

    void showState(QString sState);

    void showPose(float x,float y);

    void midButtonAct();

    void showImage(const QImage &image);

//    void mouseMoveEvent(QMouseEvent* event);

    void on_Close_Camera_clicked();

    void on_Manual_clicked();

    void on_penetration_time_valueChanged(int arg1);

    void on_penetration_depth_valueChanged(int arg1);

    void on_ImageAdjust_clicked();

    void on_NextAction_clicked();

    void on_fullyNextAct_clicked();

    void on_saveImage_clicked();

    void on_fullyStart_clicked();

    void on_fullyStop_clicked();

    void on_None_clicked();

    void on_lift_height_valueChanged(int arg1);

    void on_IBVS_clicked();

    void on_EwStart_valueChanged(double arg1);

    void on_doubleSpinBox_valueChanged(double arg1);

    void on_volInputOver_clicked();

    void on_distanceSend_clicked();

    void on_touchBox_currentIndexChanged(int index);

    void on_RorLSelection_clicked();

    void on_lensMagnify_currentIndexChanged(int index);

    void on_testOpen_clicked();

//    void on_Distance_valueChanged(int arg1);

    void on_Speed_valueChanged(int arg1);

    void on_X_down_clicked();

    void on_X_up_clicked();

    void on_Y_down_clicked();

    void on_Y_up_clicked();

    void on_Z_down_clicked();

    void on_Z_up_clicked();

    void on_fullyAboveAct_clicked();

    void on_Page_1_clicked();

    void on_Page_2_clicked();

    void on_pumpPulse_valueChanged(double arg1);

    void on_pumpPulseSend_clicked();

    void on_pumpDistance_valueChanged(double arg1);

    void on_pumpDistanceSend_clicked();

    void on_zeroSet_clicked();

    void on_toZero_clicked();

    void keyPressEvent(QKeyEvent *event);

    void on_D_up_clicked();

    void on_D_down_clicked();

    void on_Max_acc_valueChanged(int arg1);

    void on_Reciprocating_clicked();

    void on_Distance_double_valueChanged(double arg1);

    void on_StepNumX_valueChanged(int arg1);

    void on_StepNumY_valueChanged(int arg1);

    void on_StepNumZ_valueChanged(int arg1);

    void on_StepX_valueChanged(double arg1);

    void on_StepZ_valueChanged(double arg1);

    void on_sanOrder_currentIndexChanged(int index);

    void on_Scanning_clicked();

    void on_StepY_valueChanged(double arg1);

    void on_volFrequency_valueChanged(double arg1);

    void on_volFreComb_currentIndexChanged(int index);   

    void on_lineComboBox_currentIndexChanged(int index);

private:
    Ui::MainWindow *ui;
     QTimer* timer;
     QMatrix m_matrix;
     QLabel *stateLabel;

     double voltagekk=0.01;
     double motorPulse=0;
     int touchSelection=0;
     double interfacePositionSave=0.0;
     double volFrequency=1;
     int freUnit=1000000;

     int lineCombo =0;


    
};

#endif // MAINWINDOW_H
