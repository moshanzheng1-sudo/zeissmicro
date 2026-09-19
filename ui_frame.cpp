#if _MSC_VER >= 1600
#pragma execution_character_set("utf-8")

#endif

#include "ui_frame.h"
#include "ui_ui_frame.h"
#include <QtCharts>
#include <iostream>

QT_CHARTS_USE_NAMESPACE


ui_frame::ui_frame(Auto_Focus *a_focus, QWidget *parent):
    QFrame(parent),
    ui(new Ui::ui_frame)
{
    ui->setupUi(this);
    A_Focus=a_focus;
    connect(A_Focus,&Auto_Focus::Chart_Param,this,&ui_frame::on_auto_focus_show);

}

ui_frame::~ui_frame()
{
    delete ui;
}

/***按键控制线程启动***/
void ui_frame::on_focus_start_clicked()
{
//    A_Focus->start();
}

/***线程计算结束后进行绘图***/
void ui_frame::on_auto_focus_show(const float *P_sharp,int len_rows,int Array_Slect)
{
    float maxX=0,maxY=0;
    QChart *chart = new QChart();
    chart->setTitle("简单函数曲线");
    /***曲线标题***/
//    QScatterSeries *series=new QScatterSeries();
    QLineSeries *series=new QLineSeries();
    series->setName("清晰度曲线");   
    chart->addSeries(series);
    /***曲线序列***/
    double x = 0.0,y = 0.0;
    switch (Array_Slect) {
    case 1:
        for(int i=0;i<len_rows;i++)
        {
            x=*(P_sharp+i*2);
            y=*(P_sharp+i*2+1);
            if(abs(x)>maxX)
            {
                maxX=abs(x);
            }
            if(abs(y)>maxY)
            {
                maxY=abs(y);
            }
            Data_Series.append(QPointF(x,y));
        }
        break;
    case 2:
        for(int i=0;i<len_rows;i++)
        {
            x=i;
            y=*(P_sharp+i);
            if(abs(x)>maxX)
            {
                maxX=abs(x);
            }
            if(abs(y)>maxY)
            {
                maxY=abs(y);
            }
            Data_Series.append(QPointF(x,y));
        }
        break;
    default:
        break;
    }

    /***坐标轴***/
    QValueAxis *axisX = new QValueAxis;
    axisX->setRange(0, maxX*3/2);//设置坐标轴范围
    axisX->setTitleText("X");//标题
    axisX->setLabelFormat("%.1f"); //标签格式：每个单位保留几位小数
    axisX->setTickCount(10); //主分隔个数：0到10分成20个单位
    axisX->setMinorTickCount(1); //每个单位之间绘制了多少虚网线
//    axisX->setGridLineVisible(false);

    QValueAxis *axisY = new QValueAxis; //Y 轴
    axisY->setRange(0,maxY*3/2);
    axisY->setTitleText("Y");
    axisY->setLabelFormat("%.2f"); //标签格式
    axisY->setTickCount(10);
    axisY->setMinorTickCount(1);
//    axisX->setGridLineVisible(false);

    //为序列设置坐标轴
    series->replace(Data_Series);
    Data_Series.clear();
    chart->setAxisX(axisX, series);
    chart->setAxisY(axisY, series);

    ui->graphicsView->setChart(chart);   
}
