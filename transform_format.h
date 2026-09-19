#ifndef TRANSFORM_FORMAT_H
#define TRANSFORM_FORMAT_H

#include <QObject>
#include "opencv2/opencv.hpp"
#include "qimage.h"
#include <QDebug>

using namespace cv;




class Transform_Format : public QObject
{
    Q_OBJECT
public:
    explicit Transform_Format(QObject *parent = nullptr);
    QImage cvmatToQImage(cv::Mat mat);
    QImage MatToQImage(const Mat& mat);
    Mat  QImage2cvMat(QImage& image);


signals:

public slots:
};

#endif // TRANSFORM_FORMAT_H
