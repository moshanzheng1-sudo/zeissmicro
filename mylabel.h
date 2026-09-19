#ifndef MYLABEL_H
#define MYLABEL_H

#include <QObject>
#include <QLabel>
#include<QMouseEvent>


class MyLabel :public QLabel
{
    Q_OBJECT
public:
    MyLabel(QWidget *parent = nullptr);
    ~MyLabel();


    void mousePressEvent(QMouseEvent *e);    //鼠标按下事件

signals:
    void sendPose(float x,float y);
    void midButtonPress();

};

#endif // MYLABEL_H
