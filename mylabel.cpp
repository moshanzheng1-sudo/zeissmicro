#include "mylabel.h"
#include <iostream>

MyLabel::MyLabel(QWidget* parent) :QLabel(parent)
{

}
MyLabel::~MyLabel()
{

}

void MyLabel::mousePressEvent(QMouseEvent *e)
{
    // 如果是鼠标左键按下
    if(e->button() == Qt::LeftButton){
        emit sendPose(e->x(),e->y());

    }
    // 如果是鼠标右键按下
    else if(e->button() == Qt::RightButton){


    }else if(e->button() == Qt::MiddleButton){
        emit midButtonPress();

    }




}

