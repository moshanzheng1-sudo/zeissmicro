#include "sub_window.h"
#include <QtCharts>
#include "ui_frame.h"

Sub_Window::Sub_Window():
    QFrame(),
  ui(new Ui::Frame)
{
  ui->setupUi(this);

    QChartView *chartView = new QChartView(this);
    QChart *chart = new QChart();
    chart->setTitle("简单函数曲线");   //图标的名字
    chartView->setChart(chart);
   // this->setCentralWidget(chartView);

}
