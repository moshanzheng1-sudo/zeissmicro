#ifndef MULTICLAMPCURVE_H
#define MULTICLAMPCURVE_H

#include <QWidget>

namespace Ui {
class MulticlampCurve;
}

class MulticlampCurve : public QWidget
{
    Q_OBJECT

public:
    explicit MulticlampCurve(QWidget *parent = nullptr);
    ~MulticlampCurve();

private:
    Ui::MulticlampCurve *ui;
};

#endif // MULTICLAMPCURVE_H
