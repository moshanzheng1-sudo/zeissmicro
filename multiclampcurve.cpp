#include "multiclampcurve.h"
#include "ui_multiclampcurve.h"

MulticlampCurve::MulticlampCurve(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MulticlampCurve)
{
    ui->setupUi(this);
}

MulticlampCurve::~MulticlampCurve()
{
    delete ui;
}
