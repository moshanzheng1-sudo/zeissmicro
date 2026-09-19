#if _MSC_VER >= 1600
#pragma execution_character_set("utf-8")

#endif


#include "positioning_tip.h"
#include <QDebug>
using namespace std;
using namespace cv;



Pose_Plane::Pose_Plane(Auto_Focus *A_Focus, SBaslerCameraControl *image_control, params_struct &params, Ch_Instrument *ch_instrument)
{
    Image_Control=image_control;
    line2DParamsFirst=params;
    ch_instrument_pose=ch_instrument;

}
Pose_Plane::~Pose_Plane(void)
{

}



void Pose_Plane::showImgUI(Mat image)
{
    imgUI=MatToQImage(image);
    emit sendImage(imgUI);
}



/*********************
Function:获取坐标旋转矩阵
Abstract:基于SVD，三个对应点获取旋转矩阵
Input   :需转换坐标系点，转换后坐标系点
Author  :胡伟康
*********************/
Mat Pose_Plane::Get3DR_TransMatrix(const std::vector<Point3d>& srcPoints, const std::vector<Point3d>&  dstPoints)
{
    double srcSumX = 0.0;
    double srcSumY = 0.0;
    double srcSumZ = 0.0;

    double dstSumX = 0.0;
    double dstSumY = 0.0;
    double dstSumZ = 0.0;

    //至少三组点
    if (srcPoints.size() != dstPoints.size() || srcPoints.size() < 3)
    {
        return cv::Mat();
    }

    int pointsNum = srcPoints.size();
    for (int i = 0; i < pointsNum; ++i)
    {
        srcSumX += srcPoints[i].x;
        srcSumY += srcPoints[i].y;
        srcSumZ += srcPoints[i].z;

        dstSumX += dstPoints[i].x;
        dstSumY += dstPoints[i].y;
        dstSumZ += dstPoints[i].z;
    }

    cv::Point3d centerSrc, centerDst;

    centerSrc.x = double(srcSumX / pointsNum);
    centerSrc.y = double(srcSumY / pointsNum);
    centerSrc.z = double(srcSumZ / pointsNum);

    centerDst.x = double(dstSumX / pointsNum);
    centerDst.y = double(dstSumY / pointsNum);
    centerDst.z = double(dstSumZ / pointsNum);

    //Mat::Mat(int rows, int cols, int type)
    cv::Mat srcMat(3, pointsNum, CV_64FC1);
    cv::Mat dstMat(3, pointsNum, CV_64FC1);

    for (int i = 0; i < pointsNum; ++i)//N组点
    {
        //三行
        srcMat.at<double>(0, i) = srcPoints[i].x - centerSrc.x;
        srcMat.at<double>(1, i) = srcPoints[i].y - centerSrc.y;
        srcMat.at<double>(2, i) = srcPoints[i].z - centerSrc.z;

        dstMat.at<double>(0, i) = dstPoints[i].x - centerDst.x;
        dstMat.at<double>(1, i) = dstPoints[i].y - centerDst.y;
        dstMat.at<double>(2, i) = dstPoints[i].z - centerDst.z;

    }


//    srcMat.convertTo(srcMat, CV_64FC1);
//    dstMat.convertTo(dstMat, CV_64FC1);
    cout<<"srcMat:"<<srcMat<<endl;
    cout<<"dstMat:"<<dstMat<<endl;
    cv::Mat matS = srcMat * dstMat.t();

    cv::Mat matU, matW, matV;
    cv::SVDecomp(matS, matW, matU, matV);

    cv::Mat matTemp = matU * matV;
    double det = cv::determinant(matTemp);//行列式的值

    double datM[] = { 1, 0, 0, 0, 1, 0, 0, 0, det };
    cv::Mat matM(3, 3, CV_64FC1, datM);

    cv::Mat matR = matV.t() * matM * matU.t();

    double* datR = (double*)(matR.data);
    double delta_X = centerDst.x - (centerSrc.x * datR[0] + centerSrc.y * datR[1] + centerSrc.z * datR[2]);
    double delta_Y = centerDst.y - (centerSrc.x * datR[3] + centerSrc.y * datR[4] + centerSrc.z * datR[5]);
    double delta_Z = centerDst.z - (centerSrc.x * datR[6] + centerSrc.y * datR[7] + centerSrc.z * datR[8]);


    //生成RT齐次矩阵(4*4)
    cv::Mat R_T = (cv::Mat_<double>(4, 4) <<
        matR.at<double>(0, 0), matR.at<double>(0, 1), matR.at<double>(0, 2), delta_X,
        matR.at<double>(1, 0), matR.at<double>(1, 1), matR.at<double>(1, 2), delta_Y,
        matR.at<double>(2, 0), matR.at<double>(2, 1), matR.at<double>(2, 2), delta_Z,
        0, 0, 0, 1
        );

    return R_T;
}



uint8_t Line_Fitting(vector<Point>* point,int num,float & k,float & b)
{
    double x_mean = 0;
    double y_mean = 0;
    vector<Point>::iterator point2f=point->begin();
    for(; point2f!=point->end(); point2f++)
    {
        x_mean += point2f->x;
        y_mean += point2f->y;
    }
    x_mean /= num;
    y_mean /= num;

    double Dxx = 0, Dxy = 0, Dyy = 0;
    double A=0,B=0,C=0;

    point2f=point->begin();
    for(; point2f!=point->end(); point2f++)
    {
        Dxx += (point2f->x - x_mean) * (point2f->x - x_mean);
        Dxy += (point2f->x - x_mean) * (point2f->y - y_mean);
        Dyy += (point2f->y - y_mean) * (point2f->y - y_mean);
    }
    double lambda = ( (Dxx + Dyy) - sqrt( (Dxx - Dyy) * (Dxx - Dyy) + 4 * Dxy * Dxy) ) / 2.0;
    double den = sqrt( Dxy * Dxy + (lambda - Dxx) * (lambda - Dxx) );
    A = Dxy / den;
    B = (lambda - Dxx) / den;
    C = - A * x_mean - B * y_mean;
    k=-A/B;
    b=-C/B;
    return 0;

}

 void on_mouse(int EVENT, int x, int y, int flags, void* userdata)
{
  Mat hh;
  hh = *(Mat*)userdata;
  Point point(x,y);
  switch (EVENT)
  {
   case EVENT_LBUTTONDOWN:
   {

//    printf("b=%d\t", hh.at<Vec3b>(p)[0]);
//    printf("g=%d\t", hh.at<Vec3b>(p)[1]);
//    printf("r=%d\n", hh.at<Vec3b>(p)[2]);
    circle(hh, point, 2, Scalar(255),3);
    cout<<"tipX:"<<point.x<<",tipY:"<<point.y<<endl;
   }
   break;

  }
}

void Pose_Plane::mousePosition(float x,float y)
{
    mouseRatio.x=x;
    mouseRatio.y=y;
    cout<<x<<"  "<<y<<endl;
}

inline void circRowShift(Mat&src,int shift_m_rows)
{
    int m=shift_m_rows;
    int rows=src.rows;
    //‘行’循环移动
    if(m%rows==0)
    {
        return;
    }

    Mat mrows(abs(m),src.cols,src.type());//用于暂时保存末尾的m行数据
    if(m>0)
    {
        src(Range(rows-m,rows),Range::all()).copyTo(mrows);
        src(Range(0,rows-m),Range::all()).copyTo(src(Range(m,rows),Range::all()));
        mrows.copyTo(src(Range(0,m),Range::all()));
    }else
    {
        src(Range(0,-m),Range::all()).copyTo(mrows);
        src(Range(-m,rows),Range::all()).copyTo(src(Range(0,rows+m),Range::all()));
        mrows.copyTo(src(Range(rows+m,rows),Range::all()));
    }
}



cv::Mat Pose_Plane::snakeImage(
    cv::Mat image,
    cv::Mat xs,
    cv::Mat ys,
    double alpha,
    double beta,
    double gamma,
    double kappa,
    double wl,
    double we,
    double wt,
    int iterations
)
{
    // 相关参数
    int N = iterations;
    cv::Mat smth = image.clone();
    cv::Mat resultImg= image.clone();
    Mat saveImg;
    VideoWriter video("snakeChange.avi", CV_FOURCC('D', 'I', 'V', 'X'), 25, Size(1024,542),0);

    // 图像大小
    qDebug() << "Calculating size of image";
    cv::Size size = image.size();
    int row = size.height;
    int col = size.width;

    // 计算外部力（图像力）
    qDebug() << "Computing external forces";
    cv::Mat E_line = smth.clone(); // E_line is simply the image intensities

    cv::Mat gradx, grady;
    cv::Sobel(smth, gradx, smth.depth(), 1, 0, 1, 1, 0, cv::BORDER_CONSTANT);
    cv::Sobel(smth, grady, smth.depth(), 0, 1, 1, 1, 0, cv::BORDER_CONSTANT);

    qDebug() << "Computing gradx and grady";
    cv::Mat E_edge(row, col, CV_32FC1);
    for (int i = 0; i < gradx.rows; i++)
    {
        for (int j = 0; j < gradx.cols; j++)
        {
            float v_gradx = gradx.at<float>(i, j);
            float v_grady = grady.at<float>(i, j);

            E_edge.at<float>(i, j) = -1 * std::sqrt(v_gradx * v_gradx + v_grady * v_grady); // E_edge is measured by gradient in the image
        }
    }

    // 导数mask
    qDebug() << "masks for taking various derivatives";
    cv::Mat m1 = (cv::Mat_<float>(1, 2) << -1, 1);
    cv::Mat m2 = (cv::Mat_<float>(2, 1) << -1, 1);
    cv::Mat m3 = (cv::Mat_<float>(1, 3) << 1, -2, 1);
    cv::Mat m4 = (cv::Mat_<float>(3, 1) << 1, -2, 1);
    cv::Mat m5 = (cv::Mat_<float>(2, 2) << 1, -1, -1, 1);

    cv::Mat cx, cy, cxx, cyy, cxy;
    filter2D(smth, cx, -1, m1);
    filter2D(smth, cy, -1, m2);
    filter2D(smth, cxx, -1, m3);
    filter2D(smth, cyy, -1, m4);
    filter2D(smth, cxy, -1, m5);

    // 计算 E_term
    cv::Mat E_term(row, col, CV_32FC1);
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            int v_cx = cx.at<float>(i, j);
            int v_cy = cy.at<float>(i, j);
            int v_cxx = cxx.at<float>(i, j);
            int v_cyy = cyy.at<float>(i, j);
            int v_cxy = cxy.at<float>(i, j);

            E_term.at<float>(i, j) = (v_cyy*v_cx*v_cx - 2 * v_cxy*v_cx*v_cy + v_cxx * v_cy*v_cy) / (std::pow((1 + v_cx * v_cx + v_cy * v_cy), 1.5));
        }
    }

    // 计算E_ext
    cv::Mat E_ext = (wl*E_line + we * E_edge - wt * E_term);

    // 计算梯度
    cv::Mat fx, fy;
    cv::Sobel(E_ext, fx, E_ext.depth(), 1, 0, 1, 0.5, 0, cv::BORDER_CONSTANT);
    cv::Sobel(E_ext, fy, E_ext.depth(), 0, 1, 1, 0.5, 0, cv::BORDER_CONSTANT);

    cv::transpose(xs, xs);
    cv::transpose(ys, ys);

    int m = xs.rows;
    int n = 1;

    int mm = fx.cols;
    int nn = fx.rows;

    // 计算五对角状矩阵，b(i)表示vi系数(i = i - 2 到 i + 2)
    double b[5];
    b[0] = beta;
    b[1] = -(alpha + 4 * beta);
    b[2] = 2 * alpha + 6 * beta;
    b[3] = b[1];
    b[4] = b[0];

    cv::Mat A = cv::Mat::eye(m, m, CV_32FC1);
    cv::Mat eyeMat0 = cv::Mat::eye(m, m, CV_32FC1);
    circRowShift(eyeMat0, 2);
    eyeMat0.convertTo(eyeMat0, CV_32FC1);
    A = b[0] * eyeMat0;

    cv::Mat eyeMat1 = cv::Mat::eye(m, m, CV_32FC1);
    circRowShift(eyeMat1, 1);
    eyeMat1.convertTo(eyeMat1, CV_32FC1);
    A = A + b[1] * eyeMat1;

    cv::Mat eyeMat2 = cv::Mat::eye(m, m, CV_32FC1);
    circRowShift(eyeMat2, 0);
    eyeMat2.convertTo(eyeMat2, CV_32FC1);
    A = A + b[2] * eyeMat2;

    cv::Mat eyeMat3 = cv::Mat::eye(m, m, CV_32FC1);
    circRowShift(eyeMat3, -1);
    eyeMat3.convertTo(eyeMat3, CV_32FC1);
    A = A + b[3] * eyeMat3;

    cv::Mat eyeMat4 = cv::Mat::eye(m, m, CV_32FC1);
    circRowShift(eyeMat4, -2);
    eyeMat4.convertTo(eyeMat4, CV_32FC1);
    A = A + b[4] * eyeMat4;

    // 计算矩阵的逆
    cv::Mat Ainv(A.size(), CV_32FC1);
    A = A + gamma * cv::Mat::eye(m, m, CV_32FC1);
    cv::invert(A, Ainv); //  Computing Ainv

//    cv::Mat srcImg = cv::imread("D:/endo_image.bmp");
    // 迭代更新曲线
    for (int i = 0; i < N; i++)
    {
        cv::Mat intFx(fx.size(), CV_32FC1);
        cv::Mat intFy(fy.size(), CV_32FC1);

        cv::remap(fx, intFx, xs, ys, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        cv::remap(fy, intFy, xs, ys, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

        cv::Mat ssx(xs.size(), CV_32FC1);
        cv::Mat ssy(ys.size(), CV_32FC1);
        for (int k = 0; k < xs.rows; k++)
        {
            for (int l = 0; l < xs.cols; l++)
            {
                ssx.at<float>(k, l) = gamma * xs.at<float>(k, l) - kappa * intFx.at<float>(k, l);
                ssy.at<float>(k, l) = gamma * ys.at<float>(k, l) - kappa * intFy.at<float>(k, l);
            }
        }

        // 更新曲线位置
        xs = Ainv * ssx;
        ys = Ainv * ssy;

//        resultImg = srcImg.clone();
        for (int j = 0; j < xs.rows; j++)
        {
            cv::Point center = cv::Point(xs.at<float>(j, 0), ys.at<float>(j, 0));
            cv::circle(resultImg, center, 4, cv::Scalar(0, 255, 255));
        }
        resultImg.convertTo(saveImg,CV_8U);
        video<<saveImg;

        // 显示
//        cv::imshow("result", resultImg);
//        cv::waitKey(30);
    }
    video.release();
    return resultImg;
}


Mat Pose_Plane::interfacePositioning(double &Time_Focus, Mat originalImg, Mat backImg, int orientation)//背景相减法
{
    Time_Focus=getTickCount();

    static int numFlag=0;
    tipRight.x=-1;
    tipRight.y=-1;

    int iNumber=-1,iNumber_2=-1,pointNum=0;
//    vector<Point> lines;//存储拟合多边形点集

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Mat motionImg,testImg,resultImg,imageContours, xs, ys;
    Mat templateImgNow,bkImg,sobelBackX,sobelTestX,sobelX;
    Mat elementOpen = getStructuringElement(MORPH_ELLIPSE,Size(5, 5));
//    backgroundImg=backgroundImg(Rect(0,0,backgroundImg.size().width-5,backgroundImg.size().height));
//    resize(backgroundImg,bkImg,Size(1024,542));
    bkImg=backImg.clone();
//    imwrite("1backkImg.jpg",bkImg);
//    imwrite("1testImg.jpg",originalImg);

    if(originalImg.type()!=CV_8UC1)
    {
        cvtColor(originalImg,testImg,COLOR_BGR2GRAY);
    }else
    {
        testImg=originalImg;
    }
    if(bkImg.type()!=CV_8UC1)
    {
        cvtColor(bkImg,bkImg,COLOR_BGR2GRAY);
    }



    double thresholdElectro=0.0;
    double minValue=0.0,maxValue=0.0;


    if(orientation){
//        bkImg=bkImg-testImg;
        bkImg=(testImg-bkImg)+(bkImg-testImg);
        minMaxLoc(bkImg,&minValue,&maxValue);
//        cout<<"minValue:"<<minValue<<"; maxValue:"<<maxValue<<endl;
        threshold(bkImg,motionImg,100,255,CV_THRESH_TRIANGLE);
    }else{
//        bkImg=bkImg-testImg;//在用
//        bkImg=testImg-bkImg;

        bkImg=(testImg-bkImg)+(bkImg-testImg);
//        bkImg=testImg;
        if(imageSavaFlag){
            imwrite("111bkImg.jpg",bkImg);
        }
//        resultImg=bkImg.clone();

//        threshold(resultImg,resultImg,0,255,CV_THRESH_OTSU);
        morphologyEx(bkImg,bkImg,MORPH_OPEN,elementOpen);//去噪

//          bg_model->apply(bkImg,bkImg);
//        bg_model->apply(testImg,bkImg);
//        resultImg=bkImg.clone();

//        bkImg=testImg-bkImg;
//        Sobel(testImg,sobelTestX,CV_8U,1,0);

        Sobel(bkImg,sobelBackX,CV_8U,1,0);//再用
        sobelX=sobelBackX;

//        sobelX=bkImg;
        if(imageSavaFlag){
            imwrite("111sobelX.jpg",sobelX);
        }

//        sobelX=sobelTestX-sobelBackX;
//        sobelX=sobelTestX;

//        resultImg=sobelX.clone();
        minMaxLoc(sobelX,&minValue,&maxValue);

        cout<<"minValue:"<<minValue<<"; maxValue:"<<maxValue<<endl;
//        thresholdElectro=threshold(sobelX,motionImg,100,255,CV_THRESH_OTSU);
//        thresholdElectro=threshold(sobelX,motionImg,maxValue-30,255,CV_THRESH_BINARY);

        /**大津阈值法+三角去噪**/
        double lengthValue,max_H,thresholdTRI;
        int max_index,thref=255;
        Mat saveImg=sobelX.clone();

        thresholdTRI=sourceTriangleCompare(saveImg,&max_H,&max_index,&lengthValue,thref);//基于大津阈值法的阈值判断左右
        if(thresholdTRI+20<=255){
            threshold(sobelX,motionImg,thresholdTRI+20,255,CV_THRESH_BINARY);//当阈值大于峰值，使用三角法滤波
        }else{
            threshold(sobelX,motionImg,thresholdTRI,255,CV_THRESH_BINARY);//当阈值大于峰值，使用三角法滤波
        }


        resultImg=motionImg.clone();

        if(maxValue<40)//屏蔽没有界面时的误检测
        {
            return  resultImg;
        }
    }

//    imwrite("3threshold.jpg",motionImg);
//
//    morphologyEx(motionImg,motionImg,MORPH_OPEN,elementOpen);//去噪
//    imwrite("4morphologyEx.jpg",motionImg);
    if(orientation){
        resultImg=motionImg.clone();
    }
//    imwrite("resultImg.jpg",resultImg);
    findContours(motionImg,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);//提取轮廓
    imageContours = Mat::zeros(motionImg.size(), CV_8UC1);
    double cArea,cArea_2;
    if(orientation){
        for(int i=0;i<contours.size();i++){
            cArea=contourArea(contours.at(i),false);
            if(pointNum<cArea&& contours.at(i).size()>100)//确保初始模板不出问题
            {
                pointNum=cArea;
                iNumber=i;
            }
        }//找最长
    }else{
        for(int i=0;i<contours.size();i++){
            cArea=contourArea(contours.at(i),false);//找面积最大
            if(pointNum<cArea)
            {
                pointNum=cArea;
                iNumber=i;
            }

        }//找最长
    }


//    drawContours(imageContours, contours, iNumber, Scalar(255,255,255), -1);//绘制轮廓

//    resultImg=imageContours.clone();

    if(imageSavaFlag){
        imwrite("111imageContours.jpg",imageContours);
    }


    int maxNum=0;
    int averageCount_x=0;
    int averageCount_y=0;
    Point2f averageP;
    if(iNumber>=0)
    {
        if(orientation){
            tipRight.x=-1;
            for (int j=0;j<contours[iNumber].size();j++) {
                if(tipRight.x<contours[iNumber].at(j).x){
                    tipRight=contours[iNumber].at(j);
                    maxNum=j;
                }
            }
            if(maxNum>30&&(maxNum+30)<contours[iNumber].size()){
                for (int j=maxNum-30;j<maxNum+30;j++) {

                    if(abs(tipRight.x-contours[iNumber].at(j).x)<10){
                        averageP.y=averageP.y+contours[iNumber].at(j).y;
                        averageCount_y++;
                    }
                    if(abs(tipRight.x-contours[iNumber].at(j).x)<5){
//                        circle(resultImg, contours[iNumber].at(j),10, Scalar(255,255,255));
//                        circle(resultImg, contours[iNumber].at(j),3, Scalar(255,255,255));
                        averageP.x=averageP.x+contours[iNumber].at(j).x;
                        averageCount_x++;
                    }
                }
            }else{
                for (int j=0;j<contours[iNumber].size();j++) {
                    if(abs(tipRight.x-contours[iNumber].at(j).x)<10){
                        averageP.y=averageP.y+contours[iNumber].at(j).y;
                        averageCount_y++;
                    }
                    if(abs(tipRight.x-contours[iNumber].at(j).x)<5){
//                        circle(resultImg, contours[iNumber].at(j),10, Scalar(255,255,255));
//                        circle(resultImg, contours[iNumber].at(j),3, Scalar(255,255,255));
                        averageP.x=averageP.x+contours[iNumber].at(j).x;
                        averageCount_x++;
                    }
                }

            }
            tipRight.x=averageP.x/averageCount_x;
            tipRight.y=averageP.y/averageCount_y;


        }else{
            //形态学中心
//            distanceTransform(imageContours,imageContours,DIST_L2,5);
//            double maxDis, minDis;
//            Point minPoint, maxPoint;
//            minMaxLoc(imageContours,&minDis,&maxDis,&minPoint,&maxPoint);
//            tipRight=maxPoint;


            //用轮廓中心的点
            tipRight.x=0;
            tipRight.y=0;
            for (int j=0;j<contours[iNumber].size();j++) {
                tipRight.x+=contours[iNumber].at(j).x;
                tipRight.y+=contours[iNumber].at(j).y;
            }
            tipRight.x=tipRight.x/contours[iNumber].size();
            tipRight.y=tipRight.y/contours[iNumber].size();

            //边界点
//            tipRight.x=100000;
//            for (int j=0;j<contours[iNumber].size();j++) {
//                if(tipRight.x>contours[iNumber].at(j).x){
//                    tipRight=contours[iNumber].at(j);
//                    maxNum=j;
//                }
//            }
        }
    }




    numFlag++;
    Time_Focus=((double)getTickCount()-Time_Focus)/getTickFrequency()*1000;
    return  resultImg;

}



Mat Pose_Plane::tipPositioning_mog2(double &Time_Focus, Mat originalImg, int orientation, int updateFlag)//mog2算法
{
    Time_Focus=getTickCount();

    static int numFlag=0;
    tipRight.x=-1;
    tipRight.y=-1;

    int iNumber=-1,pointNum=0;
    vector<Point> lines;//存储拟合多边形点集

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Mat motionImg,testImg,resultImg,imageContours, xs, ys;
    Mat templateImgNow,mogResult;
    Mat elementOpen = getStructuringElement(MORPH_ELLIPSE,Size(5, 5));
//    backgroundImg=backgroundImg(Rect(0,0,backgroundImg.size().width-5,backgroundImg.size().height));
//    resize(backgroundImg,bkImg,Size(1024,542));
//    bkImg=backgroundImg.clone();


    cvtColor(originalImg,testImg,COLOR_BGR2GRAY);



    double thresholdElectro=0.0;
    if(updateFlag){
        bg_model->apply(testImg,mogResult);
    }
    else{
        bg_model->apply(testImg,mogResult,0);
    }

    if(orientation){
        threshold(mogResult,motionImg,100,255,CV_THRESH_TRIANGLE);
    }else{
        thresholdElectro=threshold(mogResult,motionImg,100,255,CV_THRESH_TRIANGLE);

    }

//    imwrite("motionImg.jpg",motionImg);
//    resultImg=motionImg.clone();
    morphologyEx(motionImg,motionImg,MORPH_CLOSE,elementOpen);//去噪
    resultImg=motionImg.clone();
//    imwrite("resultImg.jpg",resultImg);
    findContours(motionImg,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);//提取轮廓
    imageContours = Mat::zeros(motionImg.size(), CV_8UC1);
    for(int i=0;i<contours.size();i++){
        if(pointNum<contours.at(i).size()&& contours.at(i).size()>100)//确保初始模板不出问题
        {
            pointNum=contours.at(i).size();
            iNumber=i;
        }
    }//找最长
    if(iNumber>=0)
    {
        if(orientation){
            tipRight.x=-1;
            for (int j=0;j<contours[iNumber].size();j++) {
                if(tipRight.x<contours[iNumber].at(j).x){
                    tipRight=contours[iNumber].at(j);
                }
            }
        }else{
            tipRight.x=100000;
            for (int j=0;j<contours[iNumber].size();j++) {
                if(tipRight.x>contours[iNumber].at(j).x){
                    tipRight=contours[iNumber].at(j);
                }
            }
        }

//        drawContours(imageContours, contours, iNumber, Scalar(0,0,255), -1);//绘制轮廓
//        imwrite("imageContours.bmp",imageContours);
//        circle(testImg, tipRight,10, Scalar(0,0,255));
//        circle(testImg, tipRight,3, Scalar(0,0,255));
    }


    numFlag++;
    Time_Focus=((double)getTickCount()-Time_Focus)/getTickFrequency()*1000;
    return  resultImg;

}

Mat Pose_Plane::tipPositioning_otsu_pump(double &Time_Focus,Mat originalImg)
{
    Time_Focus=getTickCount();

    Mat TrainImg,contoursImg,imageLines;
    Mat sobel_x,sobel_y;
    Mat resultImg;
    vector<int> iFlag;
    Mat imageContours = Mat::zeros(TrainImg.size(), CV_8UC1);
    Point connerPoint(10000,10000);
    Point connerPointAverage(0,0);
//    Mat elementClose_m = getStructuringElement(MORPH_RECT,Size(5, 5));
//    Mat elementClose_c = getStructuringElement(MORPH_RECT,Size(5, 5));
//    Mat elementBot = getStructuringElement(MORPH_RECT,Size(3, 3));
    Mat elementOpen = getStructuringElement(MORPH_ELLIPSE,Size(5, 5));
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

//    GammaTransform(originalImg,2);
    if(imageSavaFlag){
        imwrite("microp_2_Gamma.bmp",originalImg);
    }
    cvtColor(originalImg,TrainImg,COLOR_BGR2GRAY);
    if(imageSavaFlag){
        imwrite("microp_3_Gray.bmp",TrainImg);
    }

//        imshow("templateGet",TrainImg);
//        setMouseCallback("templateGet",on_mouse,&TrainImg);
//        waitKey();



    Sobel(TrainImg,TrainImg,CV_8U,1,0);
    //微管定位-Adherent cells
//    threshold(TrainImg,TrainImg,0,255,CV_THRESH_OTSU);

    /**大津阈值法+三角去噪**/
    double lengthValue,max_H,thresholdTRI;
    int max_index,thref=255;
    Mat saveImg=TrainImg.clone();

    thresholdTRI=sourceTriangleCompare(saveImg,&max_H,&max_index,&lengthValue,thref);//基于大津阈值法的阈值判断左右
    if(thresholdTRI+10<=255){
        threshold(TrainImg,TrainImg,thresholdTRI,255,CV_THRESH_BINARY);//当阈值大于峰值，使用三角法滤波
//        threshold(TrainImg,TrainImg,thresholdTRI-10,255,CV_THRESH_BINARY_INV);//当阈值大于峰值，使用三角法滤波
    }else{
        threshold(TrainImg,TrainImg,thresholdTRI,255,CV_THRESH_BINARY);//当阈值大于峰值，使用三角法滤波
//        threshold(TrainImg,TrainImg,thresholdTRI-10,255,CV_THRESH_BINARY_INV);//当阈值大于峰值，使用三角法滤波
    }



    if(imageSavaFlag){
        imwrite("microp_4_OTSU.bmp",TrainImg);
    }


    morphologyEx(TrainImg,TrainImg,MORPH_OPEN,elementOpen);//去噪

    resultImg=TrainImg.clone();

    if(imageSavaFlag){
        imwrite("microp_5_morph.bmp",TrainImg);
    }

//    TrainImg=~TrainImg;

    if(imageSavaFlag){
        imwrite("microp_6_Neg.bmp",TrainImg);
    }




    findContours(TrainImg,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);//提取轮廓
    imageContours = Mat::zeros(TrainImg.size(), CV_8UC1);
    imageLines = Mat::zeros(TrainImg.size(), CV_8UC1);

    int pointSize=0,iNumber=0;
    for(int i=0;i<contours.size();i++){
        if(pointSize<contours[i].size())
        {
            pointSize=contours[i].size();
            iNumber=i;
        }
    }//找到点数最多的轮廓
    drawContours(imageContours, contours, iNumber, Scalar(255), -1);//绘制轮廓
//    resultImg=imageContours.clone();
    if(imageSavaFlag){
        imwrite("microp_7_Contours.bmp",imageContours);
    }


    int maxNum=0;
    int averageCount_x=0;
    int averageCount_y=0;
    Point2f averageP;
    if(iNumber>=0)
    {
        if(1){
            tipRight.x=-1;
            for (int j=0;j<contours[iNumber].size();j++) {
                if(tipRight.x<contours[iNumber].at(j).x){
                    tipRight=contours[iNumber].at(j);
                    maxNum=j;
                }
            }
            if(maxNum>30&&(maxNum+30)<contours[iNumber].size()){
                for (int j=maxNum-30;j<maxNum+30;j++) {

                    if(abs(tipRight.x-contours[iNumber].at(j).x)<10){
                        averageP.y=averageP.y+contours[iNumber].at(j).y;
                        averageCount_y++;
                    }
                    if(abs(tipRight.x-contours[iNumber].at(j).x)<5){
//                        circle(resultImg, contours[iNumber].at(j),10, Scalar(255,255,255));
//                        circle(resultImg, contours[iNumber].at(j),3, Scalar(255,255,255));
                        averageP.x=averageP.x+contours[iNumber].at(j).x;
                        averageCount_x++;
                    }
                }
            }else{
                for (int j=0;j<contours[iNumber].size();j++) {
                    if(abs(tipRight.x-contours[iNumber].at(j).x)<10){
                        averageP.y=averageP.y+contours[iNumber].at(j).y;
                        averageCount_y++;
                    }
                    if(abs(tipRight.x-contours[iNumber].at(j).x)<5){
//                        circle(resultImg, contours[iNumber].at(j),10, Scalar(255,255,255));
//                        circle(resultImg, contours[iNumber].at(j),3, Scalar(255,255,255));
                        averageP.x=averageP.x+contours[iNumber].at(j).x;
                        averageCount_x++;
                    }
                }

            }
            tipRight.x=averageP.x/averageCount_x;
            tipRight.y=averageP.y/averageCount_y;


        }else{
            tipRight.x=100000;
            for (int j=0;j<contours[iNumber].size();j++) {
                if(tipRight.x>contours[iNumber].at(j).x){
                    tipRight=contours[iNumber].at(j);
                    maxNum=j;
                }
            }
        }
    }

    Time_Focus=((double)getTickCount()-Time_Focus)/getTickFrequency()*1000;

    /***将探针旋转至水平位置***/
//    float k=0,b=0;
//    Line_Fitting(&contoursInject,contoursInject.size(),k, b);
//    k=atan(-k);//[-90°,+90°]   取-k是考虑图像坐标系的Y轴的反的
//    angleK=0.5*k;
//    k=k*180/M_PI;//[-90°,+90°]   取-k是考虑图像坐标系的Y轴的反的

//    Mat rotationMatrix;
//    rotationMatrix=getRotationMatrix2D((Point2f)connerPoint,-k,1);
//    warpAffine(originalImg,TrainImg,rotationMatrix,originalImg.size());
////    imshow("ROTATION",TrainImg);
//    TrainImg=TrainImg(Rect(connerPoint.x-origin_offset_x,connerPoint.y-origin_offset_y,origin_offset_x+20,origin_offset_y*2));//选取模板

//    cout<<"connerPointX:"<<connerPoint.x<<",connerPointY:"<<connerPoint.y<<",Time:"<<Time_Focus<<endl;
//    imwrite(templateImageAddress,TrainImg);
//    imshow("templateGet",TrainImg);
//    setMouseCallback("templateGet",on_mouse,&TrainImg);
//    waitKey();
    return  resultImg;


}

Mat Pose_Plane::tipPositioningBK(double &Time_Focus, Mat originalImg, Mat backImg, int orientation)//背景相减法
{
    Time_Focus=getTickCount();

    static int numFlag=0;
    tipRight.x=-1;
    tipRight.y=-1;

    int iNumber=-1,pointNum=0;
//    vector<Point> lines;//存储拟合多边形点集

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Mat motionImg,testImg,resultImg,imageContours, xs, ys;
    Mat templateImgNow,bkImg;
    Mat elementOpen = getStructuringElement(MORPH_ELLIPSE,Size(5, 5));//15,15
    Mat elementDilate = getStructuringElement(MORPH_CROSS,Size(1, 1));//17,17
//    backgroundImg=backgroundImg(Rect(0,0,backgroundImg.size().width-5,backgroundImg.size().height));
//    resize(backgroundImg,bkImg,Size(1024,542));
    bkImg=backImg.clone();


    if(originalImg.type()!=CV_8UC1)
    {
        cvtColor(originalImg,testImg,COLOR_BGR2GRAY);
    }else
    {
        testImg=originalImg;
    }
    if(bkImg.type()!=CV_8UC1)
    {
        cvtColor(bkImg,bkImg,COLOR_BGR2GRAY);
    }



    double thresholdElectro=0.0;

    bkImg=bkImg-testImg;
    if(orientation){
        threshold(bkImg,motionImg,100,255,CV_THRESH_TRIANGLE);
    }else{
        thresholdElectro=threshold(bkImg,motionImg,100,255,CV_THRESH_TRIANGLE);
    }

//    imwrite("motionImg.jpg",motionImg);
//    resultImg=motionImg.clone();
//    morphologyEx(motionImg,motionImg,MORPH_OPEN,elementOpen);//去噪
    resultImg=motionImg.clone();
    imwrite("resultImg.jpg",resultImg);
    findContours(motionImg,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);//提取轮廓
    imageContours = Mat::zeros(motionImg.size(), CV_8UC1);
    for(int i=0;i<contours.size();i++){
        if(pointNum<contours.at(i).size()&& contours.at(i).size()>100)//确保初始模板不出问题
        {
            pointNum=contours.at(i).size();
            iNumber=i;
        }
    }//找最长

//    drawContours(imageContours, contours, iNumber, Scalar(255,255,255), -1);//绘制轮廓

    int maxNum=0;
    int averageCount_x=0;
    int averageCount_y=0;
    Point2f averageP;
    if(iNumber>=0)
    {
        if(orientation){
            tipRight.x=-1;
            for (int j=0;j<contours[iNumber].size();j++) {
                if(tipRight.x<contours[iNumber].at(j).x){
                    tipRight=contours[iNumber].at(j);
                    maxNum=j;
                }
            }
            if(maxNum>30&&(maxNum+30)<contours[iNumber].size()){
                for (int j=maxNum-30;j<maxNum+30;j++) {

                    if(abs(tipRight.x-contours[iNumber].at(j).x)<15){
                        averageP.y=averageP.y+contours[iNumber].at(j).y;
                        averageCount_y++;
                    }
                    if(abs(tipRight.x-contours[iNumber].at(j).x)<10){
//                        circle(resultImg, contours[iNumber].at(j),10, Scalar(255,255,255));
//                        circle(resultImg, contours[iNumber].at(j),3, Scalar(255,255,255));
                        averageP.x=averageP.x+contours[iNumber].at(j).x;
                        averageCount_x++;
                    }
                }
            }else{
                for (int j=0;j<contours[iNumber].size();j++) {
                    if(abs(tipRight.x-contours[iNumber].at(j).x)<15){
                        averageP.y=averageP.y+contours[iNumber].at(j).y;
                        averageCount_y++;
                    }
                    if(abs(tipRight.x-contours[iNumber].at(j).x)<10){
//                        circle(resultImg, contours[iNumber].at(j),10, Scalar(255,255,255));
//                        circle(resultImg, contours[iNumber].at(j),3, Scalar(255,255,255));
                        averageP.x=averageP.x+contours[iNumber].at(j).x;
                        averageCount_x++;
                    }
                }

            }
            tipRight.x=averageP.x/averageCount_x;
            tipRight.y=averageP.y/averageCount_y;


        }else{
            tipRight.x=100000;
            for (int j=0;j<contours[iNumber].size();j++) {
                if(tipRight.x>contours[iNumber].at(j).x){
                    tipRight=contours[iNumber].at(j);
                    maxNum=j;
                }
            }
        }
    }

    float lengthX=200,lengthY=100;
    vector<vector<Point>> triangularMask(1,vector<Point>(4,Point(0,0)));
//    triangularMask[0].at(0)=tipRight;
    triangularMask[0].at(1)=Point2f(tipRight.x,tipRight.y-lengthY*0.3);
    triangularMask[0].at(0)=Point2f(tipRight.x,tipRight.y+lengthY*0.3);
    triangularMask[0].at(2)=Point2f(tipRight.x-lengthX,tipRight.y-lengthY);
    triangularMask[0].at(3)=Point2f(tipRight.x-lengthX,tipRight.y+lengthY);
    drawContours(imageContours, triangularMask, 0, Scalar(255,255,255), -1);//绘制轮廓
    triangularMask.clear();
    resultImg=imageContours.clone();
//    morphologyEx(resultImg,resultImg,MORPH_DILATE,elementDilate);//稍微增大

    numFlag++;
    Time_Focus=((double)getTickCount()-Time_Focus)/getTickFrequency()*1000;
    return  resultImg;

}


Mat Pose_Plane::tipPositioning_otsu(double &Time_Focus,Mat originalImg)
{
    Time_Focus=getTickCount();

    Mat TrainImg,contoursImg,imageLines;
    Mat sobel_x,sobel_y;
    Mat resultImg;
    vector<int> iFlag;
    Mat imageContours = Mat::zeros(TrainImg.size(), CV_8UC1);   
    Point connerPoint(10000,10000);
    Point connerPointAverage(0,0);
//    Mat elementClose_m = getStructuringElement(MORPH_RECT,Size(5, 5));
//    Mat elementClose_c = getStructuringElement(MORPH_RECT,Size(5, 5));
//    Mat elementBot = getStructuringElement(MORPH_RECT,Size(3, 3));
    Mat elementOpen = getStructuringElement(MORPH_ELLIPSE,Size(5, 5));
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

//    GammaTransform(originalImg,2);
    if(imageSavaFlag){
        imwrite("microp_2_Gamma.bmp",originalImg);
    }
    cvtColor(originalImg,TrainImg,COLOR_BGR2GRAY);
    if(imageSavaFlag){
        imwrite("microp_3_Gray.bmp",TrainImg);
    }

//        imshow("templateGet",TrainImg);
//        setMouseCallback("templateGet",on_mouse,&TrainImg);
//        waitKey();



    //微管定位-Adherent cells
//    threshold(TrainImg,TrainImg,0,255,CV_THRESH_OTSU);

    /**大津阈值法+三角去噪**/
    double lengthValue,max_H,thresholdTRI;
    int max_index,thref=255;
    Mat saveImg=TrainImg.clone();

    thresholdTRI=sourceTriangleCompare(saveImg,&max_H,&max_index,&lengthValue,thref);//基于大津阈值法的阈值判断左右
    if(thresholdTRI+10<=255){
        threshold(TrainImg,TrainImg,thresholdTRI+10,255,CV_THRESH_BINARY);//当阈值大于峰值，使用三角法滤波
    }else{
        threshold(TrainImg,TrainImg,thresholdTRI,255,CV_THRESH_BINARY);//当阈值大于峰值，使用三角法滤波
    }



    if(imageSavaFlag){
        imwrite("microp_4_OTSU.bmp",TrainImg);
    }


    morphologyEx(TrainImg,TrainImg,MORPH_OPEN,elementOpen);//去噪

    if(imageSavaFlag){
        imwrite("microp_5_morph.bmp",TrainImg);
    }

//    TrainImg=~TrainImg;

    if(imageSavaFlag){
        imwrite("microp_6_Neg.bmp",TrainImg);
    }




    findContours(TrainImg,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);//提取轮廓
    imageContours = Mat::zeros(TrainImg.size(), CV_8UC1);
    imageLines = Mat::zeros(TrainImg.size(), CV_8UC1);

    int pointSize=0,iNumber=0;
    for(int i=0;i<contours.size();i++){
        if(pointSize<contours[i].size())
        {
            pointSize=contours[i].size();
            iNumber=i;
        }
    }//找到点数最多的轮廓
    drawContours(imageContours, contours, iNumber, Scalar(255), -1);//绘制轮廓
    resultImg=imageContours.clone();
    if(imageSavaFlag){
        imwrite("microp_7_Contours.bmp",imageContours);
    }


    int maxNum=0;
    int averageCount_x=0;
    int averageCount_y=0;
    Point2f averageP;
    if(iNumber>=0)
    {
        if(1){
            tipRight.x=-1;
            for (int j=0;j<contours[iNumber].size();j++) {
                if(tipRight.x<contours[iNumber].at(j).x){
                    tipRight=contours[iNumber].at(j);
                    maxNum=j;
                }
            }
            if(maxNum>30&&(maxNum+30)<contours[iNumber].size()){
                for (int j=maxNum-30;j<maxNum+30;j++) {

                    if(abs(tipRight.x-contours[iNumber].at(j).x)<10){
                        averageP.y=averageP.y+contours[iNumber].at(j).y;
                        averageCount_y++;
                    }
                    if(abs(tipRight.x-contours[iNumber].at(j).x)<5){
//                        circle(resultImg, contours[iNumber].at(j),10, Scalar(255,255,255));
//                        circle(resultImg, contours[iNumber].at(j),3, Scalar(255,255,255));
                        averageP.x=averageP.x+contours[iNumber].at(j).x;
                        averageCount_x++;
                    }
                }
            }else{
                for (int j=0;j<contours[iNumber].size();j++) {
                    if(abs(tipRight.x-contours[iNumber].at(j).x)<10){
                        averageP.y=averageP.y+contours[iNumber].at(j).y;
                        averageCount_y++;
                    }
                    if(abs(tipRight.x-contours[iNumber].at(j).x)<5){
//                        circle(resultImg, contours[iNumber].at(j),10, Scalar(255,255,255));
//                        circle(resultImg, contours[iNumber].at(j),3, Scalar(255,255,255));
                        averageP.x=averageP.x+contours[iNumber].at(j).x;
                        averageCount_x++;
                    }
                }

            }
            tipRight.x=averageP.x/averageCount_x;
            tipRight.y=averageP.y/averageCount_y;


        }else{
            tipRight.x=100000;
            for (int j=0;j<contours[iNumber].size();j++) {
                if(tipRight.x>contours[iNumber].at(j).x){
                    tipRight=contours[iNumber].at(j);
                    maxNum=j;
                }
            }
        }
    }

    Time_Focus=((double)getTickCount()-Time_Focus)/getTickFrequency()*1000;

    /***将探针旋转至水平位置***/
//    float k=0,b=0;
//    Line_Fitting(&contoursInject,contoursInject.size(),k, b);
//    k=atan(-k);//[-90°,+90°]   取-k是考虑图像坐标系的Y轴的反的
//    angleK=0.5*k;
//    k=k*180/M_PI;//[-90°,+90°]   取-k是考虑图像坐标系的Y轴的反的

//    Mat rotationMatrix;
//    rotationMatrix=getRotationMatrix2D((Point2f)connerPoint,-k,1);
//    warpAffine(originalImg,TrainImg,rotationMatrix,originalImg.size());
////    imshow("ROTATION",TrainImg);
//    TrainImg=TrainImg(Rect(connerPoint.x-origin_offset_x,connerPoint.y-origin_offset_y,origin_offset_x+20,origin_offset_y*2));//选取模板

//    cout<<"connerPointX:"<<connerPoint.x<<",connerPointY:"<<connerPoint.y<<",Time:"<<Time_Focus<<endl;
//    imwrite(templateImageAddress,TrainImg);
//    imshow("templateGet",TrainImg);
//    setMouseCallback("templateGet",on_mouse,&TrainImg);
//    waitKey();
    return  resultImg;


}

void  Pose_Plane::GammaTransform(cv::Mat &image, double gamma)
{

    Mat imageGamma;	//灰度归一化
    Mat dist(image.size(),CV_64F);
    image.convertTo(imageGamma, CV_64F, 1.0 / 255, 0); 	//伽马变换
    pow(imageGamma, gamma, dist);//dist 要与imageGamma有相同的数据类型
    dist.convertTo(dist, CV_8U, 255, 0);
    image=dist;

}



int8_t Pose_Plane::nonOvershootPositioning()
{
    Mat imageRead,test_img,resultImg,roiImg,sampleImg;
    double Time_Focus=0;
    static int numFlag=0;
    Point targetP;
    static VideoCapture injectionVideo(videoAddress);
    static VideoWriter videoCreate("plane.avi", CV_FOURCC('D', 'I', 'V', 'X'), 25, Size(2048,542),0);//彩色图像

    char * filename=new char[100];

    /***摄像头方式***/
    if(cameraOpen)
    {
        while(Image_Control->GrabImage(imageRead,5)!=0)
        {
//            std::cout << "Grab image failed " << std::endl;
            /*break;*/
        }
    }
    else
    {
        /***视频方式***/
//        injectionVideo>>imageRead;
//        if (imageRead.empty())//视频结束
//        {
//            videoCreate.release();
//            return state_idle;
//        }
        /***图片方式***/
        if (numFlag*5>300)//视频结束
        {
            videoCreate.release();
            numFlag=0;
            return state_idle;
        }
        sprintf(filename,imgmicropipette,numFlag*5-150);
//        sprintf(filename,imgmicropipette,105);
        imageRead=imread(filename);
    }
    sampleImg= imread(sampleAddress);
    cvtColor(sampleImg,sampleImg,COLOR_BGR2GRAY);
//    imageRead=imageRead(Rect(imageRead.size().width/2-100,0,imageRead.size().width/2,imageRead.size().height));//去除视频的黑边,缩小路径规划区域
    resize(imageRead,test_img,Size(1024,542));
    imwrite("microp_1_original.bmp",test_img);

    if(actOpen)
    {
        if(numFlag==0){
            if(manipulationSelection!=manipulationSelection_last){
                line2DParams.dev=manipulationSelection;//设置控制对象
                Ump_Select_Dev(&line2DParams);
                manipulationSelection_last=manipulationSelection;
            }
            Ump_Read_Position(&line2DParams);
//            cout<<""<<",X:"<<line2DParams.home_x<<",Y:"<<line2DParams.home_y<<",Z:"<<line2DParams.home_z<<endl;
        }else {
            Ump_Read_Position(&line2DParams);
        }

    }

    showImgUI(test_img);
    resultImg=tipPositioning_otsu(Time_Focus,test_img);//获取尖端位置
    cvtColor(test_img,test_img,COLOR_BGR2GRAY);
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    char * text=new char[100];
    if(tipPoint.size()>0)
    {
        int i=0;
        if(tipPoint[i].x>50 && (test_img.size().width-tipPoint[i].x)>50 && tipPoint[i].y>50 && (test_img.size().height-tipPoint[i].y)>50)
        {
            roiImg=test_img(Rect(tipPoint[i].x-100,tipPoint[i].y-100,200,200));//考虑边界
            matchTemplate(roiImg,sampleImg,roiImg,TM_CCOEFF_NORMED);
            //寻找最佳匹配位置
            cv::minMaxLoc(roiImg, &minVal, &maxVal, &minLoc, &maxLoc);
            cout <<maxVal<< endl;
            sprintf(text,"%d : %f",numFlag,maxVal);
            if(maxVal>0.91)
            {
                cout<<"maxVal:"<<maxVal<<",x:"<<tipPoint[i].x<<",y:"<<tipPoint[i].y<<",X:"<<line2DParams.home_x<<",Y:"<<line2DParams.home_y<<",Z:"<<line2DParams.home_z<<endl;
               keepState=-1;
            }
            circle(test_img, tipPoint[i],10, Scalar(0,0,255));
            circle(test_img, tipPoint[i],3, Scalar(0,0,255));
            imwrite("microp_9_tip.bmp",test_img);
            putText(test_img,text,cvPoint(50,50),FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255),2);
        }
    }
    else
    {
        cout <<0<< endl;
        sprintf(text,"%d : %f",numFlag,0.0);
        putText(test_img,text,cvPoint(50,50),FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255),2);
    }
    showImgUI(test_img);
    if(actOpen&&keepState>0)
    {
    line2DParams.target_y=line2DParams.home_y;
    line2DParams.target_x=line2DParams.home_x;
    line2DParams.target_z=line2DParams.home_z+5000;//向下移动5μm
    line2DParams.target_d=line2DParams.home_d;
    line2DParams.speed=400;
    Ump_Goto_Position(&line2DParams);
    }
    vector <Mat> vImg_1;
    Mat outImg;
    vImg_1.push_back(test_img);
    vImg_1.push_back(resultImg);
    hconcat(vImg_1,outImg);
    videoCreate<<outImg;
    setMouseCallback("test_img",on_mouse,&imageRead);


    tipPoint.clear();
//            std::cout <<"Time_Focus:"<<Time_Focus<<",numFlag:"<<numFlag<< std::endl;
        numFlag++;
        if(keepState<0 || numFlag>1000)
        {

           numFlag=0;
           videoCreate.release();
           injectionVideo.release();
           return state_idle;
        }
        return state_nonOvershootPositioning;
}

Mat Pose_Plane::updateMotionHistory(Mat src,Mat lastSrc,int magnify,double diffnum, int duration )
{
    Mat originalImg,diffImg,lastImg;
    int imageCols,imageRows;
    static Mat lastDst=Mat::zeros(src.size().height,src.size().width,CV_8U);
//    cvtColor(src,originalImg,COLOR_BGR2GRAY);
//    cvtColor(lastSrc,lastImg,COLOR_BGR2GRAY);
    originalImg=src;
    lastImg=lastSrc;

//    diffImg=originalImg-lastImg;
    diffImg=lastImg-originalImg;
//    imwrite("diffImg.bmp",diffImg);
//    threshold(diffImg,diffImg,diffnum,255,CV_THRESH_BINARY);
//    threshold(diffImg,diffImg,diffnum,255,CV_THRESH_TRIANGLE);
//    threshold(diffImg,diffImg,diffnum,255,CV_THRESH_OTSU);
    /**大津阈值法+三角去噪**/
    double lengthValue,max_H,thresholdTRI;
    int max_index,thref=255;
    Mat saveImg=diffImg.clone();

    thresholdTRI=sourceTriangleCompare(saveImg,&max_H,&max_index,&lengthValue,thref);//基于大津阈值法的阈值判断左右
    if(thresholdTRI+15<=255){
        threshold(diffImg,diffImg,thresholdTRI+15,255,CV_THRESH_BINARY);//当阈值大于峰值，使用三角法滤波
    }else{
        threshold(diffImg,diffImg,thresholdTRI,255,CV_THRESH_BINARY);//当阈值大于峰值，使用三角法滤波
    }

//    imwrite("diffImg_th.bmp",diffImg);
//    diffImg=magnify*diffImg;
    imageCols=diffImg.cols-1;
    imageRows=diffImg.rows-1;
    for(int i=0;i<imageRows;i++)
    {
        for(int j=0;j<imageCols;j++)
        {
            if(diffImg.at<uchar>(i,j)==0)
            {
                if(lastDst.at<uchar>(i,j)>duration){
                    diffImg.at<uchar>(i,j)=lastDst.at<uchar>(i,j)-duration;
                }
            }
        }
    }
    lastDst=diffImg;
    return diffImg;
}



void Pose_Plane::touchCurrentMutation()
{
    if(planeSelection==state_touchDetection && touchFlag==2){
        currentMutationFlag=1;
    }
}

void Pose_Plane::sicmData(float current, float time)
{

    if(planeSelection==state_touchDetection){
        sicmParams=line2DParams;
        Ump_Read_Position(&sicmParams);
        cout<<"position:"<<sicmParams.home_z<<" current:"<<current<<" fluctuation:"<<time<<endl;
    }
}

int8_t Pose_Plane::touchDetection()
{
    Mat imageRead,test_img,roiImg,mhiImg,outImg,outImgSave,outImg_1;
    static Mat maskImg;
    double Time_Focus=0;
    float distance=0;
    static Point tipPose;
    static Mat lastImg;
    static int numFlag=0;
    char * filename1=new char[100];
    char * filename2=new char[100];
    char * filename3=new char[100];

    char * text=new char[100];
    int roiYLen=100;
    int roiXLen=100;
    vector <Mat> vImg_1;
    Scalar meanResult;

    static VideoCapture injectionVideo(videoAddress);
    static VideoWriter videoCreate("touch.avi", CV_FOURCC('D', 'I', 'V', 'X'), 20, Size(800,400),0);
//    static VideoWriter videoCreate("touch.avi", CV_FOURCC('D', 'I', 'V', 'X'), 25, Size(1024,542),1);//彩色图像



    cv::Mat_<double> srcMatrix(3,1);
    cv::Mat_<double> dstMatrix(3,1);

    if(actOpen)
    {
        if(numFlag==0)
        {
//            if(AutoOpen)
            if(0)
            {
                if(manipulationSelection!=manipulationSelection_last){
                    line2DParams.dev=manipulationSelection;//设置控制对象
                    Ump_Select_Dev(&line2DParams);
                    manipulationSelection_last=manipulationSelection;
                }
                Ump_Read_Position(&line2DParams);
                Sleep(500);

                float dx=0,dy=0,distance_xy=0,distance_xy_last=1000000;
                int touchNum;
                if(mulTouchSelection!=-1)
                {
                    if(!barycenterBFinal.empty()){
                        cout<<"size:"<<barycenterBFinal.size()<<endl;
                        for (int i=0;i<barycenterBFinal.size();i++) {
                            dx=abs((int)barycenterBFinal.at(i).x-1024/2);
                            dy=abs((int)barycenterBFinal.at(i).y-542/2);
                            distance_xy=0.4*dx+0.7*dy;
                            cout<<i<<": "<<barycenterBFinal.at(i).x<<"; "<<barycenterBFinal.at(i).y<<endl;
                            if(distance_xy<distance_xy_last){
                                touchNum=i;
                                distance_xy_last=distance_xy;
                            }//自动寻找比较中心的细胞
                        }
                        cout<<"cellX:"<<barycenterBFinal.at(touchNum).x<<"; cellY:"<<barycenterBFinal.at(touchNum).y<<endl;
                        srcMatrix(0,0)=(double)barycenterBFinal.at(touchNum).x;//统一单位为μm
                        srcMatrix(1,0)=(double)barycenterBFinal.at(touchNum).y;
                        srcMatrix(2,0)=(double)1;
                        srcMatrix.convertTo(srcMatrix, CV_64FC1);
                        dstMatrix.convertTo(dstMatrix, CV_64FC1);
                        transformMatrix.convertTo(transformMatrix, CV_64FC1);
                        Mat transMat=transformMatrix.clone();
                        dstMatrix=transMat*srcMatrix;//坐标转换

                        int targetX=dstMatrix(0,0)*1000;//单位由μm转换为nm
                        int targetY=dstMatrix(1,0)*1000;
                        cout<<"targetX:"<<targetX<<"; targetY:"<<targetY<<endl;

                        line2DParams.target_y=targetY;
                        line2DParams.target_x=targetX;
                        line2DParams.target_z=line2DParams.home_z;
                        line2DParams.target_d=line2DParams.home_d;
                        line2DParams.speed=1500;
                        line2DParams.acc=6000;
                        line2DParams.dev=manipulationSelection;//设置控制对象
                        Ump_Select_Dev(&line2DParams);
                        Ump_Goto_For_Injection(&line2DParams);
                        Sleep(3000);
                     }
                    }
            }
//            Ump_Read_Position(&line2DParams);
//            std::cout <<"Touch Begin!"<< std::endl;
//            sendState(QString("Touch Begin!"));
        }
    switch (touchFlag) {
        case 1:
            /*运动历史图像检测法*/
            /***摄像头方式***/
            if(cameraOpen)
            {
                Image_Control->GrabImage(imageRead,5);
                Sleep(50);
                while(Image_Control->GrabImage(imageRead,5)!=0)
                {
//                    std::cout << "Grab image failed " << std::endl;
                    /*break;*/
                }
            }
            else
            {
                /***视频方式***/
//                injectionVideo>>imageRead;
//                if (imageRead.empty())//视频结束
//                {
//                    videoCreate.release();
//                    return state_idle;
//                }
                /***图片方式***/
                backImg_original=imread("D:/qt_space/Microsystem/image/20231201/9/Touch_1.bmp");
                backImg=backImg_original;
//                resize(backImg_original,backImg,Size(1024,542));
                sprintf(filename1,"D:/qt_space/Microsystem/image/20231201/9/Touch_%d.bmp",numFlag+13);//17
                imageRead= imread(filename1);
                if(imageRead.empty()){
                    std::cout <<"Image Over!"<< std::endl;
                    return state_idle;
                }
            }
//            resize(imageRead,test_img,Size(1024,542));
            test_img=imageRead;

            if(numFlag==0)
            {
               test_img=imread("D:/qt_space/Microsystem/image/20231201/9/Touch_28.bmp");
               maskImg=~tipPositioningBK(Time_Focus,test_img,backImg,1);//获取尖端位置, and make the mask
               tipPose=tipRight;
               cout<<"get tip point"<<endl;
            }
            imageRead.copyTo(roiImg,maskImg);//去除针尖区域
            imwrite("roiImg.bmp",roiImg);
//            mhiImg=roiImg(Rect(tipPose.x-50,tipPose.y-25,100,100));//获取细胞区域
            mhiImg=roiImg(Rect(tipPose.x-roiXLen*2,tipPose.y-roiYLen*2,roiXLen*4,roiYLen*4));
            cvtColor(mhiImg,mhiImg,COLOR_BGR2GRAY);
            if(numFlag!=0){
                outImg=updateMotionHistory(mhiImg,lastImg,1,60,150);//duration越大延迟越短
//                sprintf(filename2,"mhiImg_%d.bmp",numFlag);
//                sprintf(filename3,"outImg_%d.bmp",numFlag);
//                imwrite(filename2,mhiImg);
//                imwrite(filename3,outImg);

                outImgSave=outImg.clone();
                meanResult=mean(outImgSave);
                sprintf(text,"numFlag: %d; meanResult: %f;",numFlag,meanResult[0]);
                putText(outImgSave,text,cvPoint(10,25),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255),1);
                vImg_1.push_back(mhiImg);
                vImg_1.push_back(outImgSave);
                hconcat(vImg_1,outImg_1);
                vImg_1.clear();
                videoCreate<<outImg_1;
            }
//            videoCreate<<outImg;

            cout <<"z_distance: "<<numFlag<<"; meanResult: "<<meanResult[0]<< endl;
//            cout <<"z_distance: "<<numFlag<<"; countNonZero: "<<countNonZero(outImg)<< endl;
            if(meanResult[0]>1)//检测到细胞运动
            {
                numFlag=0;
                Ump_stop(&line2DParams);
                Ump_Read_Position(&line2DParams);//获取当前微管位置
                touchPosition=line2DParams;//保存接触检测的精确位置
                std::cout <<"Touch Over!"<< std::endl;
                sendState(QString("Touch Over!"));
                videoCreate.release();
                return state_idle;
            }
            lastImg=mhiImg.clone();
            for(int i=0;i<tipPoint.size();i++){
               circle(test_img, tipPoint[i],10, Scalar(0,0,255));
               circle(test_img, tipPoint[i],3, Scalar(0,0,255));
            }
            tipPoint.clear();

            line2DParams.target_y=line2DParams.home_y;
            line2DParams.target_x=line2DParams.home_x;
            line2DParams.target_z=line2DParams.home_z+1000*numFlag;//以1μm速度下降
            line2DParams.target_d=line2DParams.home_d;
            line2DParams.speed=1;
            Ump_Goto_Position(&line2DParams);
            Sleep(500);
            break;
        case 2:
            /*离子电导检测法*/        
            if(currentMutationFlag!=1)// && numFlag==0
            {
                line2DParams.target_y=line2DParams.home_y;
                line2DParams.target_x=line2DParams.home_x;
                line2DParams.target_z=line2DParams.home_z+1000*numFlag;//以1μm/s速度下降
                line2DParams.target_d=line2DParams.home_d;
                line2DParams.speed=1;
                Ump_Goto_Position(&line2DParams);
                Sleep(50);
            }
            else if (currentMutationFlag==1)
            {
                Ump_stop(&line2DParams);
                Ump_Read_Position(&line2DParams);//获取当前微管位置
                touchPosition=line2DParams;//保存接触检测的精确位置
                currentMutationFlag=0;
                numFlag=0;//避免多次接触检测的过程中出现不读取当下位置的问题
                videoCreate.release();
                std::cout <<"Touch(Ion current)!!!"<< std::endl;
                sendState(QString("Touch  Over!"));
                if(mulTouchSelection==-1)
                {
                    return state_penetration;
                }else{
                    return state_idle;
                }

            }
            break;
        case 3:
            /*针尖移动检测法*/
            /***摄像头方式***/
            if(cameraOpen)
            {
                while(Image_Control->GrabImage(imageRead,50)!=0)
                {
//                    std::cout << "Grab image failed " << std::endl;
                    /*break;*/
                }
            }
        //    else
        //    {
        //        /***视频方式***/
        //        injectionVideo>>imageRead;
        //        if (imageRead.empty())//视频结束
        //        {
        //            videoCreate.release();
        //            return state_idle;
        //        }
        //        /***图片方式***/
        ////        imageRead= imread("D:/QT_space/line2D_test/image/line2d_probe/1.bmp");
        //    }
            resize(imageRead,test_img,Size(1024,542));
//            tipPositioning_otsu(Time_Focus,test_img);//获取尖端位置
            tipPositioningBK(Time_Focus,test_img,backImg,1);
            tipPoint.push_back(tipRight);
            if(numFlag==0)
            {
                Ump_Read_Position(&line2DParams);//获取当前微管位置
                tipPointFirst=tipPoint[0];
            }
            else
            {
                distance=abs(tipPointFirst.x-tipPoint[0].x)+abs(tipPointFirst.y-tipPoint[0].y);
                cout<<numFlag<<": "<<distance<<endl;
                sendState(QString("distance: %1").arg(distance));
                if(distance>=10)//偏移检测
                {
//                    overFlag=-1;
//                    std::cout <<"Needle Over!"<< std::endl;

//                }else if(overFlag==-1){
//                    numFlag=numFlag-2;
//                    if(distance<4){
                        Ump_Read_Position(&line2DParams);
                        touchPosition=line2DParams;//获取当前微管位置
                        touchPosition.home_z=touchPosition.home_z-8000;
    //                    line2DParams.dev=2;//设置控制对象
    //                    Ump_Select_Dev(&line2DParams);
    //                    Ump_Read_Position(&line2DParams);
                        videoCreate.release();
                        std::cout <<"Needle Touch!"<< std::endl;
                        sendState(QString("Needle Touch!"));
                        numFlag=0;
                        return state_idle;
//                        }
                }

                line2DParams.target_y=line2DParams.home_y;
                line2DParams.target_x=line2DParams.home_x;
                line2DParams.target_z=line2DParams.home_z+1000*numFlag;//以1μm速度下降
                line2DParams.target_d=line2DParams.home_d;
                line2DParams.speed=1;
                Ump_Goto_Position(&line2DParams);
                Sleep(500);
            }
            for(int i=0;i<tipPoint.size();i++){
               circle(test_img, tipPoint[i],10, Scalar(0,0,255));
               circle(test_img, tipPoint[i],3, Scalar(0,0,255));
            }
            tipPoint.clear();
            break;
    case 4:
        /***manual operation***/
        Ump_Read_Position(&line2DParams);//获取当前微管位置
        touchPosition=line2DParams;//保存接触检测的精确位置
        std::cout <<"Manual Touch!"<< std::endl;
        sendState(QString("Manual Touch!"));
        numFlag=0;
        return state_idle;
        default:
            break;
        }
    }

//    std::cout <<"touchFlag:"<<numFlag<<", distance:"<<distance<< std::endl;
    numFlag++;
    videoCreate<<test_img;
    return state_touchDetection;



}





int8_t Pose_Plane::coordinateTransformation()
{
    Mat imageRead,test_img,result_img,grayImg;    
    double Time_Focus=0;
//    double pi=3.1415926;
    char * filename= new char[100];
    static Point2f tipLast(0,0);
    static int8_t numFlag=-1,startFlag=0;
    static Point2f srcPoints[5],dstPoints[5];
    static VideoCapture injectionVideo(videoAddress);
    static VideoWriter videoCreate("coordinateTransformation.avi", CV_FOURCC('D', 'I', 'V', 'X'), 25, Size(2048,542),0);//彩色图像
    static int originalX,originalY,originalZ=0;

    if(numFlag==-1)//获取背景图像
    {
        backImg_original=getBackImg();
        resize(backImg_original,backImg,Size(1024,542));
        imwrite("microp_ct_back.bmp",backImg_original);
    }

    /***摄像头方式***/
    if(cameraOpen)
    {
//        while(Image_Control->GrabImage(imageRead,50)!=0)
//        {
//            std::cout << "Transfor: Grab failed " << std::endl;
//            /*break;*/
//        }
        Image_Control->GrabImage(imageRead,50);
        Sleep(100);//等待
        if(Image_Control->GrabImage(imageRead,50)!=0){
            std::cout << "Transfor: Grab failed " << std::endl;
            return state_coordinateTransformation;
        }
    }
    else
    {
        /***视频方式***/
        injectionVideo>>imageRead;
        if (imageRead.empty())//视频结束
        {
            videoCreate.release();
            return state_idle;
        }
        /***图片方式***/
//        imageRead= imread("D:/QT_space/line2D_test/image/line2d_probe/1.bmp");
    }
    resize(imageRead,test_img,Size(1024,542));

    showImgUI(test_img);
    sprintf(filename,"microp_ct_%d.bmp",startFlag);
//    imwrite(filename,test_img);

    if(auto_positioning_flag==0 && numFlag>=0)
    {
        if(mouseRatio.x>0&&mouseRatio.y>0)
        {
            tipRight.x=(int)1024.0*mouseRatio.x;//统一单位为μm
            tipRight.y=(int)542.0*mouseRatio.y;
            /*清零*/
            mouseRatio.x=0;
            mouseRatio.y=0;
            get_position_flag=1;
            tipPoint.push_back(tipRight);
        }
        cvtColor(test_img,result_img,COLOR_BGR2GRAY);

    }else{
        result_img=tipPositioningBK(Time_Focus,test_img,backImg,1);//获取尖端位置
        get_position_flag=1;
        tipPoint.push_back(tipRight);
    }


    cvtColor(test_img,grayImg,COLOR_BGR2GRAY);
    vector <Mat> vImg_1;
    Mat outImg;
    vImg_1.push_back(grayImg);
    vImg_1.push_back(result_img);
    hconcat(vImg_1,outImg);
    videoCreate<<outImg;

    if(actOpen)
    {
        if(get_position_flag==1){

            get_position_flag=0;//清除标志位
            if(manipulationSelection!=manipulationSelection_last){
                line2DParams.dev=manipulationSelection;//设置控制对象
                Ump_Select_Dev(&line2DParams);
                manipulationSelection_last=manipulationSelection;
            }
            if(ch_instrument_pose->poseSave.home_d<10&&ch_instrument_pose->poseSave.home_x<10&&ch_instrument_pose->poseSave.home_y<10){
                Ump_Read_Position(&line2DParams);
            }else{
                line2DParams=ch_instrument_pose->poseSave;//间接读取，避免报错
            }
//            Ump_Read_Position(&line2DParams);
//            line2DParams=ch_instrument_pose->poseSave;//间接读取，避免报错
            if(startFlag==0){
                originalZ=line2DParams.home_z;
            }

    //        if(tipPoint.size()!=0&&(!(tipPoint[0].x==tipLast.x&&tipPoint[0].y==tipLast.y)))
            if(tipPoint.size()!=0)
            {
                if(numFlag>=0)
                {
                    dstPoints[numFlag].x=line2DParams.home_x/1000;//坐标转换时单位统一为了μm
                    dstPoints[numFlag].y=line2DParams.home_y/1000;
                    srcPoints[numFlag].x=tipPoint[0].x;
                    srcPoints[numFlag].y=tipPoint[0].y;
                    cout<<"srcPoints:"<<srcPoints[numFlag].x<<","<<srcPoints[numFlag].y<<endl;
                    cout<<"dstPoints:"<<dstPoints[numFlag].x<<","<<dstPoints[numFlag].y<<endl;
                }

                tipLast=tipPoint[0];
                circle(test_img, tipPoint[0],10, Scalar(0,0,255));
                circle(test_img, tipPoint[0],3, Scalar(0,0,255));
                tipPoint.clear();

                if(numFlag>=2){
                    transformMatrix=getAffineTransform(srcPoints,dstPoints);
                    double v2[1][3] = { {0,0,1}};
                    Mat row(1, 3, CV_64F, &v2[0][0]);//生成一个1行3列的mat,用double数组初始化
                    transformMatrix.push_back(row);//添加一行至test
                    cout<<"transformMatrix:"<<transformMatrix<<endl;
                    videoCreate.release();
                    injectionVideo.release();
                    std::cout <<"Transform Over!"<< std::endl;
                    emit sendState(QString("Transform Over!"));
                    startFlag=0;
                    numFlag=-1;
                    return state_idle;
                }

                int liftHeight=20;

                /***提起***/
                line2DParams.target_y=line2DParams.home_y;
                line2DParams.target_x=line2DParams.home_x;
                line2DParams.target_z=line2DParams.home_z-liftHeight*1000;
                line2DParams.target_d=line2DParams.home_d;
                line2DParams.speed=100;
                line2DParams.acc=6000;
                Ump_Goto_For_Injection(&line2DParams);
                Sleep(800);//等待运动完成
                /***运动到下一点***/
                if(numFlag==-1)
                {
                    line2DParams.target_y=line2DParams.home_y+int(((tipLast.y-test_img.size().height/2)*0.65*1000-80000)*lensMagnify);//视野中央
                    line2DParams.target_x=line2DParams.home_x-int(((tipLast.x-test_img.size().width/2)*0.65*1000+100000)*lensMagnify);
                    originalX=line2DParams.target_x;
                    originalY=line2DParams.target_y;
                }else{
    //                line2DParams.target_y=line2DParams.home_y+int(cos(numFlag*3.14/2)*200000*(20/lensMagnify));
    //                line2DParams.target_x=line2DParams.home_x+int(sin(numFlag*3.14/2)*220000*(20/lensMagnify));
                    line2DParams.target_y=originalY+int(cos(numFlag*3.14/2)*200000*lensMagnify);
                    line2DParams.target_x=originalX+int(sin(numFlag*3.14/2)*270000*lensMagnify);
                }
                line2DParams.target_z=line2DParams.home_z-liftHeight*1000;
                line2DParams.target_d=line2DParams.home_d;
                line2DParams.speed=1000;
                line2DParams.acc=6000;
                Ump_Goto_For_Injection(&line2DParams);
                Sleep(800);//等待运动完成
                /***放下***/
                line2DParams.target_z=originalZ-2*1000;
                line2DParams.speed=100;
                line2DParams.acc=6000;
                Ump_Goto_For_Injection(&line2DParams);
                Sleep(800);//等待运动完成
            }
                numFlag++;
        }
    }else{
        videoCreate.release();
        injectionVideo.release();
        return state_idle;
    }

    startFlag++;
    return state_coordinateTransformation;
}


int8_t Pose_Plane::penetrationSleepChange(){

//    if(planeSelection==state_idle){
        planeSelection=state_penetration;
//    }
    return state_idle;
}


int8_t Pose_Plane::penetration()
{
    static int numFlag=0,peneFlag=0,startFlag=0,zUp=0,zDown=0;
    static int targetX,targetY,xBack=0,dDistance=0;

    Point2f penetrationSite;
    cv::Mat_<double> srcMatrix(3,1);
    cv::Mat_<double> dstMatrix(3,1);

    if(startFlag==0){
        cout <<"Penetration Start!!!"<< std::endl;
        emit sendState(QString("Penetration Start!!!"));
        if(servoSelection==1){
            backImg_original=getBackImg();
            resize(backImg_original,backImg,Size(1024,542));
        }        
        startFlag=1;
    }
    if(actOpen){
        if(AutoOpen)
        {
            if(numFlag<barycenterPFinal.size()){
                if(mulTouchSelection!=-1){
                    penetrationSite=barycenterPFinal.at(bestPathFinal.citys.at(numFlag)-1);
                    if(penetrationSleepFlag==0){
                        numFlag++;
                    }
                    peneFlag=1;

                    srcMatrix(0,0)=(double)penetrationSite.x;//统一单位为μm
                    srcMatrix(1,0)=(double)penetrationSite.y;
                    srcMatrix(2,0)=(double)1;
                    srcMatrix.convertTo(srcMatrix, CV_64FC1);
                    dstMatrix.convertTo(dstMatrix, CV_64FC1);
                    transformMatrix.convertTo(transformMatrix, CV_64FC1);
                    Mat transMat=transformMatrix.clone();
                    dstMatrix=transMat*srcMatrix;//坐标转换
                }
            } else {
                numFlag=0;
                startFlag=0;
                peneFlag=0;
                return state_idle;//结束本区域的检测
            }
        }else
        {
            /*手动指定目标位置*/
            if(mouseRatio.x>0&&mouseRatio.y>0)
            {
                penetrationSite.x=1024*mouseRatio.x;
                penetrationSite.y=542*mouseRatio.y;
                emit sendPosShow(penetrationSite.x,penetrationSite.y);

                tipViaPBVS.x=(double)1024*mouseRatio.x;
                tipViaPBVS.y=(double)542*mouseRatio.y;

                srcMatrix(0,0)=(double)1024*mouseRatio.x;//统一单位为μm
                srcMatrix(1,0)=(double)542*mouseRatio.y;
                srcMatrix(2,0)=(double)1;
                srcMatrix.convertTo(srcMatrix, CV_64FC1);
                dstMatrix.convertTo(dstMatrix, CV_64FC1);
                transformMatrix.convertTo(transformMatrix, CV_64FC1);
                cout<<"srcMatrix:"<<srcMatrix<<endl;
                Mat transMat=transformMatrix.clone();
                dstMatrix=transMat*srcMatrix;//坐标转换
                cout<<"dstMatrix:"<<dstMatrix<<endl;
                /*清零*/
                if(penetrationSleepFlag==0){
                    mouseRatio.x=0;
                    mouseRatio.y=0;
                }
                peneFlag=1;
            }
        }

        if(peneFlag){                        
            /*移动到细胞待扎入位置*/
            if(mulTouchSelection!=-1)
            {
            switch (touchFlag) {
            case 1:
                //运动历史图像
                zUp=13000+zUpAdd;//高于细胞表面2μm
                zDown=0+zDownAdd;//低于细胞表面5μm
//                xBack=zUp+zDown;//保持两轴距离一致
                xBack=(int)((zUp+zDown)*tan(angleD));//保持两轴距离一致
                dDistance=(int)((zUp+zDown)/cos(angleD));
                break;
            case 2:
                //离子电流
//                zUp=10000+zUpAdd;//高于细胞表面2μm
//                zDown=0+zDownAdd;//低于细胞表面0μm
////                xBack=zUp+zDown;//保持两轴距离一致
//                xBack=(int)((zUp+zDown)*tan(angleD));//保持两轴距离一致
//                dDistance=(int)((zUp+zDown)/cos(angleD));
                zUp=13000+zUpAdd;//高于细胞表面2μm
                zDown=0-zDownAdd;//低于细胞表面0μm
//                xBack=zUp+zDown;//保持两轴距离一致
                xBack=(int)((injectionDis)*tan(angleD));//保持两轴距离一致
                dDistance=(int)((injectionDis)/cos(angleD));
                break;
            case 3:
                zUp=13000+zUpAdd;//高于细胞表面2μm
                zDown=2000+zDownAdd;//低于细胞表面5μm
//                xBack=zUp+zDown;//保持两轴距离一致
                xBack=(int)((zUp+zDown)*tan(angleD));//保持两轴距离一致
                dDistance=(int)((zUp+zDown)/cos(angleD));
                //针尖接触基板
                break;
            case 4:
                //纯手动
//                zUp=10000+zUpAdd;//高于细胞表面2μm
//                zDown=2000+zDownAdd;//低于细胞表面5μm
////                xBack=zUp+zDown;//保持两轴距离一致
//                xBack=(int)((zUp+zDown)*tan(angleD));//保持两轴距离一致
//                dDistance=(int)((zUp+zDown)/cos(angleD));
                //针尖接触基板
//                zUp=20000+zUpAdd;//高于细胞表面13μm
//                zDown=0-zDownAdd;//低于细胞表面0μm
//                xBack=(int)((injectionDis)*tan(angleD));//保持两轴距离一致
//                dDistance=(int)((injectionDis)/cos(angleD));
                if(servoSelection==3){
                    zUp=20000+zUpAdd;//高于细胞表面13μm
                    zDown=0-zDownAdd;//低于细胞表面0μm
                    xBack=(int)((injectionDis2)*tan(angleD));//保持两轴距离一致
                    dDistance=(int)((injectionDis2)/cos(angleD));
                }else{
                    zUp=20000+zUpAdd;//高于细胞表面13μm
                    zDown=0-zDownAdd;//低于细胞表面0μm
                    xBack=(int)((injectionDis)*tan(angleD));//保持两轴距离一致
                    dDistance=(int)((injectionDis)/cos(angleD));
                }
                break;
            default:
                break;
            }
            cout <<"dDistance:"<<dDistance<< std::endl;
            targetX=dstMatrix(0,0)*1000;//单位由μm转换为nm
            targetY=dstMatrix(1,0)*1000;
            }

            Mat imageRead,testImage;
            double deltaX=100,deltaY=100;
            double Time=0;
            double lastDeltaX=0,lastDeltaY=0;
            double DDeltaX=0,DDeltaY=0;
            double sumDelX=0,sumDelY=0;
            int  controlBegin=100;
            double outX=1,outY=1;
            int iterationNum=0;


            switch (servoSelection) {
            case 1:
                if(penetrationSleepFlag==1){
                    while(controlBegin)
                    {
                        iterationNum++;
                        if(manipulationSelection!=manipulationSelection_last){
                            line2DParams.dev=manipulationSelection;//设置控制对象
                            Ump_Select_Dev(&line2DParams);
                            manipulationSelection_last=manipulationSelection;
                        }
                        if(ch_instrument_pose->poseSave.home_d<10&&ch_instrument_pose->poseSave.home_x<10&&ch_instrument_pose->poseSave.home_y<10){
                            Ump_Read_Position(&line2DParams);
                        }else{
                            line2DParams=ch_instrument_pose->poseSave;//间接读取，避免报错
                        }
//                        Ump_Read_Position(&line2DParams);
//                        line2DParams=ch_instrument_pose->poseSave;//间接读取，避免报错
                        /***摄像头方式***/
                        if(cameraOpen)
                        {
                            Image_Control->GrabImage(imageRead,50);
                            Sleep(50);//等待
                            while(Image_Control->GrabImage(imageRead,50)!=0){
                                std::cout << "Transfor: Grab failed " << std::endl;
                            }
                        }
                        resize(imageRead,testImage,Size(1024,542));

                        Mat resultImg1,resultImg2;
    //                    resultImg1=tipPositioning_mog2(Time,testImage,1,0);
                        resultImg1=tipPositioningBK(Time,testImage,backImg,1);
                        resultImg2=testImage;
                        circle(resultImg2, tipRight,10, Scalar(0,0,255));
                        circle(resultImg2, tipRight,3, Scalar(0,0,255));
                        imwrite("resultImg1.jpg",resultImg1);
                        imwrite("resultImg2.jpg",resultImg2);
                        deltaX=penetrationSite.x-(xBack-2500)/(0.6*1000)-tipRight.x;
                        deltaY=-(penetrationSite.y-tipRight.y);
                        if((abs(deltaX)+abs(deltaY))<=5 || iterationNum>6)
                        {
                            break;
                        }
                        if(controlBegin!=100)
                        {
                            DDeltaX=deltaX-lastDeltaX;
                            DDeltaY=deltaY-lastDeltaY;
                        }else{
                            DDeltaX=0;
                            DDeltaY=0;
                            controlBegin=1;
                        }
                        lastDeltaX=deltaX;
                        lastDeltaY=deltaY;
                        sumDelX+=deltaX;
                        sumDelY+=deltaY;
                        cout <<penetrationSite.x-(xBack-2500)/(0.6*1000)<<"  "<<tipRight.x<< std::endl;
                        cout <<penetrationSite.y<<"  "<<tipRight.y<< std::endl;
                        cout <<deltaX<<"  "<<deltaY<< std::endl;

                        if(abs(deltaX)<2.5){
                            outX=0;
                        }
                        if(abs(deltaY)<2.5){
                            outY=0;
                        }
                        //刺入前位置
                        line2DParams.target_y=line2DParams.home_y+outY*(deltaY*580*lensMagnify+sumDelY*0+DDeltaY*20);
                        line2DParams.target_x=line2DParams.home_x+outX*(deltaX*580*lensMagnify+sumDelX*0+DDeltaX*20);
                        line2DParams.target_z=touchPosition.home_z-zUp;
                        line2DParams.target_d=touchPosition.home_d;
                        line2DParams.speed=800;
                        line2DParams.acc=6000;
                        Ump_Goto_For_Injection(&line2DParams);
                        cout <<line2DParams.target_x<<"  "<<line2DParams.target_y<<endl;
                        Sleep(5);
                    }
                    if(manipulationSelection!=manipulationSelection_last){
                        line2DParams.dev=manipulationSelection;//设置控制对象
                        Ump_Select_Dev(&line2DParams);
                        manipulationSelection_last=manipulationSelection;
                    }
                    if(ch_instrument_pose->poseSave.home_d<10&&ch_instrument_pose->poseSave.home_x<10&&ch_instrument_pose->poseSave.home_y<10){
                        Ump_Read_Position(&line2DParams);
                    }else{
                        line2DParams=ch_instrument_pose->poseSave;//间接读取，避免报错
                    }
//                    Ump_Read_Position(&line2DParams);
//                    line2DParams=ch_instrument_pose->poseSave;//间接读取，避免报错
                    Sleep(50);
                    /*扎入细胞*/
                    line2DParams.target_y=line2DParams.home_y;
                    line2DParams.target_x=line2DParams.home_x;
                    line2DParams.target_z=touchPosition.home_z-zUp;//扎入细胞
                    line2DParams.target_d=touchPosition.home_d+dDistance;
                    line2DParams.speed=500;
                    line2DParams.acc=6000;
                    Ump_Goto_For_Injection(&line2DParams);
                    cout <<"Penetration 5s"<< std::endl;
                    sendState(QString("Penetration Waiting..."));
                    emit sendPenetrationSleep();
                    penetrationSleepFlag=0;
                    biopsyStartFlag=1;//每一次重新刺入，活检中都要重新获取针尖位置
                    return state_idle;
                }
//                for(int sleepNum=0; sleepNum<penetrationTime; sleepNum++)
//                {
//                    cout <<"Penetration Time:"<<sleepNum<< std::endl;
//                    sendState(QString("Time : %1").arg(sleepNum));
//                    Sleep(1000);//扎进细胞的时间
//                }
                /*退出细胞*/
                line2DParams.target_y=line2DParams.home_y;
                line2DParams.target_x=line2DParams.home_x;
                line2DParams.target_z=touchPosition.home_z-zUp;//高于细胞表面2μm
                line2DParams.target_d=touchPosition.home_d;
                line2DParams.speed=500;
                line2DParams.acc=6000;
                Ump_Goto_For_Injection(&line2DParams);
                penetrationSleepFlag=1;
                break;
            case 2:
                if(penetrationSleepFlag==1){
                    if(mulTouchSelection==1)
                    {
                        line2DParams.target_y=targetY;
                        line2DParams.target_x=targetX;
                        line2DParams.target_z=touchPosition.home_z-zUp;
                        line2DParams.target_d=touchPosition.home_d;
                        line2DParams.speed=500;
                        Ump_Goto_Position(&line2DParams);
                        Sleep(3000);
                        mulTouchSelection=-1;
                        return state_touchDetection;
                    }

                    //仅基于坐标变换的视觉伺服
//                    line2DParams.target_y=targetY;
//                    line2DParams.target_x=targetX-xBack;
//                    line2DParams.target_z=touchPosition.home_z-zUp;//安全位置
//                    line2DParams.target_d=touchPosition.home_d;
//                    line2DParams.speed=500;
//                    Ump_Goto_Position(&line2DParams);
//                    cout <<"Wait 5s"<< std::endl;
//                    emit sendState(QString("Wait 0.5s"));
//                    Sleep(500);

                    line2DParams.target_y=targetY;
                    line2DParams.target_x=targetX-xBack;
                    line2DParams.target_z=touchPosition.home_z-injectionDis-zDown;//扎入细胞前准备
                    line2DParams.target_d=touchPosition.home_d;
                    line2DParams.speed=500;
//                    Ump_Goto_Position(&line2DParams);
                    Ump_Goto_For_Injection(&line2DParams);
                    cout <<"Wait 5s"<< std::endl;
                    emit sendState(QString("Wait 0.5s"));
                    Sleep(500);
                    /*扎入细胞*/
                    line2DParams.target_y=targetY;
                    line2DParams.target_x=targetX-xBack;
                    line2DParams.target_z=touchPosition.home_z-injectionDis-zDown;//扎入细胞
    //                line2DParams.target_z=touchPosition.home_z;//扎入细胞
                    line2DParams.target_d=touchPosition.home_d+dDistance;
                    //调整速度
                    line2DParams.speed=200;
                    line2DParams.acc=6000;
                    Ump_Goto_For_Injection(&line2DParams);
                    cout <<"Penetration 5s"<< std::endl;
                    emit sendState(QString("Penetration Waiting..."));
                    emit sendPenetrationSleep();
                    penetrationSleepFlag=0;
                    biopsyStartFlag=1;//每一次重新刺入，活检中都要重新获取针尖位置
//                    if(!(voltageFromWaveGenerator||motorOpenFlagBase)){
//                        emit sendDigiVoltage(7, electroosmosisVoltage);
//                    }
                    return state_idle;
                }
                //时间判断转移到了positioning_cell里面，这样就可以跳出刺入循环，执行其他功能
//                for(int sleepNum=0; sleepNum<penetrationTime; sleepNum++)
//                {
//                    cout <<"Penetration Time:"<<sleepNum<< std::endl;
//                    sendState(QString("Time : %1").arg(sleepNum));
//                    Sleep(1000);//扎进细胞的时间
//                }
                /*退出细胞*/
                line2DParams.target_y=targetY;
                line2DParams.target_x=targetX-xBack;//退出细胞
                line2DParams.target_z=touchPosition.home_z-injectionDis-zDown;//高于细胞表面2μm
                line2DParams.target_d=touchPosition.home_d;
                line2DParams.speed=200;
                line2DParams.acc=6000;
                Ump_Goto_For_Injection(&line2DParams);
                Sleep(100);
//                if(!(voltageFromWaveGenerator||motorOpenFlagBase)){
//                    emit sendDigiVoltage(7, 0.1);
//                }
//                line2DParams.target_y=targetY;
//                line2DParams.target_x=targetX-xBack;//退出细胞
//                line2DParams.target_z=touchPosition.home_z-zUp;//高于细胞表面2μm
//                line2DParams.target_d=touchPosition.home_d;
//                line2DParams.speed=500;
//                line2DParams.acc=6000;
//                Ump_Goto_For_Injection(&line2DParams);
//                Sleep(100);
                penetrationSleepFlag=1;
                break;
            case 3:
                if(penetrationSleepFlag==1){
                    if(mulTouchSelection==1)
                    {
                        line2DParams.target_y=targetY;
                        line2DParams.target_x=targetX;
                        line2DParams.target_z=touchPosition.home_z-zUp;
                        line2DParams.target_d=touchPosition.home_d;
                        line2DParams.speed=500;
                        Ump_Goto_Position(&line2DParams);
                        Sleep(3000);
                        mulTouchSelection=-1;
                        return state_touchDetection;
                    }

                    //仅基于坐标变换的视觉伺服
                    line2DParams.target_y=targetY;
                    line2DParams.target_x=targetX-xBack;
                    line2DParams.target_z=touchPosition.home_z-zUp;//安全位置
                    line2DParams.target_d=touchPosition.home_d;
                    line2DParams.speed=500;
                    Ump_Goto_Position(&line2DParams);
                    cout <<"Wait 5s"<< std::endl;
                    emit sendState(QString("Wait 0.5s"));
                    Sleep(500);

                    line2DParams.target_y=targetY;
                    line2DParams.target_x=targetX-xBack;
                    line2DParams.target_z=touchPosition.home_z-zDown;//接触细胞
                    line2DParams.target_d=touchPosition.home_d;
                    line2DParams.speed=500;
//                    Ump_Goto_Position(&line2DParams);
                    Ump_Goto_For_Injection(&line2DParams);
                    cout <<"Wait 5s"<< std::endl;
                    emit sendState(QString("Wait 0.5s"));
                    Sleep(500);
                    /*扎入细胞*/
                    line2DParams.target_y=targetY;
                    line2DParams.target_x=targetX-xBack;
                    line2DParams.target_z=touchPosition.home_z-zDown;//扎入细胞
    //                line2DParams.target_z=touchPosition.home_z;//扎入细胞
                    line2DParams.target_d=touchPosition.home_d+dDistance;
                    //调整速度
                    line2DParams.speed=200;
                    line2DParams.acc=6000;
                    Ump_Goto_For_Injection(&line2DParams);
                    cout <<"Penetration 5s"<< std::endl;
                    emit sendState(QString("Penetration Waiting..."));
                    emit sendPenetrationSleep();
                    penetrationSleepFlag=0;
                    biopsyStartFlag=1;//每一次重新刺入，活检中都要重新获取针尖位置
//                    if(!(voltageFromWaveGenerator||motorOpenFlagBase)){
//                        emit sendDigiVoltage(7, electroosmosisVoltage);
//                    }
                    return state_idle;
                }
                //时间判断转移到了positioning_cell里面，这样就可以跳出刺入循环，执行其他功能
//                for(int sleepNum=0; sleepNum<penetrationTime; sleepNum++)
//                {
//                    cout <<"Penetration Time:"<<sleepNum<< std::endl;
//                    sendState(QString("Time : %1").arg(sleepNum));
//                    Sleep(1000);//扎进细胞的时间
//                }
                /*退出细胞*/
                line2DParams.target_y=targetY;
                line2DParams.target_x=targetX-xBack;//退出细胞
                line2DParams.target_z=touchPosition.home_z-zDown;//高于细胞表面2μm
                line2DParams.target_d=touchPosition.home_d-dDistance;
                line2DParams.speed=200;
                line2DParams.acc=6000;
                Ump_Goto_For_Injection(&line2DParams);
                Sleep(100);
//                if(!(voltageFromWaveGenerator||motorOpenFlagBase)){
//                    emit sendDigiVoltage(7, 0.1);
//                }
                line2DParams.target_y=targetY;
                line2DParams.target_x=targetX-xBack;
                line2DParams.target_z=touchPosition.home_z-zUp;//安全位置
                line2DParams.target_d=touchPosition.home_d-dDistance;
                line2DParams.speed=500;
                line2DParams.acc=6000;
                Ump_Goto_For_Injection(&line2DParams);
                Sleep(100);
                penetrationSleepFlag=1;
                break;
            case 4:
                //垂直刺入
                line2DParams.target_y=targetY;
                line2DParams.target_x=targetX;
                line2DParams.target_z=touchPosition.home_z-zUp;
                line2DParams.target_d=touchPosition.home_d;
                line2DParams.speed=500;
                Ump_Goto_Position(&line2DParams);
                cout <<"Wait 5s"<< std::endl;
                sendState(QString("Wait 0.5s"));
                Sleep(500);
                /*扎入细胞*/
                line2DParams.target_y=targetY;
                line2DParams.target_x=targetX;
                line2DParams.target_z=touchPosition.home_z+3000;//扎入细胞
                line2DParams.target_d=touchPosition.home_d;
                line2DParams.speed=250;
                line2DParams.acc=6000;
                Ump_Goto_For_Injection(&line2DParams);
                cout <<"Penetration 3s"<< std::endl;
                sendState(QString("Penetration 3s"));
                Sleep(3000);
                /*退出细胞*/
                line2DParams.target_y=targetY;
                line2DParams.target_x=targetX;//退出细胞
                line2DParams.target_z=touchPosition.home_z-zUp;//高于细胞表面2μm
                line2DParams.target_d=touchPosition.home_d;
                line2DParams.speed=250;
                line2DParams.acc=6000;
                Ump_Goto_For_Injection(&line2DParams);
                break;
            default:
                break;
            }

            cout <<"Next Please!"<< std::endl;
            sendState(QString("Next Please!"));
            peneFlag=0;//避免重复扎入
            if(mulTouchSelection==-1)
            {
                mulTouchSelection=1;
            }

        }


    }

    return state_penetration;

}


void Pose_Plane::zeroSet()
{
    line2DParams.dev=manipulationSelection;//设置控制对象
    Ump_Select_Dev(&line2DParams);
    Ump_Read_Position(&line2DParams);
    zeroParams=line2DParams;
}

void Pose_Plane::zeroReturn()
{
    zeroParams.target_y=zeroParams.home_y;
    zeroParams.target_x=zeroParams.home_x;
    zeroParams.target_z=zeroParams.home_z;
    zeroParams.target_d=zeroParams.home_d;
    zeroParams.speed=50;
    Ump_Select_Dev(&zeroParams);
    Ump_Goto_Position(&zeroParams);
}

int8_t Pose_Plane::manualmove()
{
    static int numFlag=0,peneFlag=0,startFlag=0;
    Point2f penetrationSite;
    cv::Mat_<double> srcMatrix(3,1);
    cv::Mat_<double> dstMatrix(3,1);

    double x_points=14.0,y_points=7.0;
    static double x_pose=0.0,y_pose=0.0;
    static int z_original=0,d_original=0;

    Mat imageRead;

    static Mat testImg,resultImg,backImg;
    double Time_Focus=0;
    char * filename= new char[100];
    static VideoCapture injectionVideo(videoAddress);
    static VideoWriter videoCreate("mog.avi", CV_FOURCC('D', 'I', 'V', 'X'), 25, Size(1024,542),1);

    sprintf(filename,"D:/HWKPROGRAMS/MicroSystem/image/0223_1/%d.bmp",numFlag);

    static QFile handle("manipulator_pose.txt");
    if(startFlag==0){handle.open(QIODevice::WriteOnly);}
    static QTextStream write_(&handle);

    /***摄像头方式***/
    if(cameraOpen)
    {
        while(Image_Control->GrabImage(imageRead,50)!=0)
        {
//            std::cout << "Transfor: Grab failed " << std::endl;
            /*break;*/
        }
    }
    else
    {
        /***视频方式***/
        injectionVideo>>imageRead;
        if (imageRead.empty())//视频结束
        {
            videoCreate.release();
            return state_idle;
        }
//          if(numFlag%10!=0)//跳帧读取
//          {
//              numFlag++;
//              return state_manualmove;
//          }
        /***图片方式***/
//        imageRead= imread("D:/QT_space/line2D_test/image/line2d_probe/1.bmp");
    }
//    imageRead=imageRead(Rect(0,0,imageRead.size().width-5,imageRead.size().height));
    resize(imageRead,testImg,Size(1024,542));
//   showImgUI(testImg);

    if(startFlag==0){
        cout <<"manualmove Start!!!"<< std::endl;
        sendState(QString("manualmove Start!!!"));
//        bg_model=createBackgroundSubtractorMOG2();
        if(manipulationSelection!=manipulationSelection_last){
            line2DParams.dev=manipulationSelection;//设置控制对象
            Ump_Select_Dev(&line2DParams);
            manipulationSelection_last=manipulationSelection;
        }
        if(ch_instrument_pose->poseSave.home_d<10&&ch_instrument_pose->poseSave.home_x<10&&ch_instrument_pose->poseSave.home_y<10){
            Ump_Read_Position(&line2DParams);
        }else{
            line2DParams=ch_instrument_pose->poseSave;//间接读取，避免报错
        }
        mappingParams=line2DParams;
        startFlag=1;
    }

//    Time_Focus=getTickCount();
//    emit sendDigiVoltage(5, 0);
//    Sleep(1);
//    emit sendDigiVoltage(6, 1);
//    Sleep(1);
//    emit sendDigiVoltage(7, 1);
//    Time_Focus=((double)getTickCount()-Time_Focus)/getTickFrequency(); //s时间
//    cout<<"switch time: "<<Time_Focus<<endl;



    switch (2) {
    case 1:
        //基于视觉的视觉伺服
//        backImg=imread("D:/QT_space/MicroSystem/image/background.bmp");
//        resultImg=tipPositioning_new(Time_Focus,testImg,backImg,1);
//        imwrite("snake.bmp",resultImg);
//        showImgUI(resultImg);

        if(numFlag<30){
            resultImg=tipPositioning_mog2(Time_Focus,testImg,1,1);
        }else{
            resultImg=tipPositioning_mog2(Time_Focus,testImg,1,0);
        }
        circle(testImg, tipRight,10, Scalar(0,0,255));
        circle(testImg, tipRight,3, Scalar(0,0,255));
        showImgUI(resultImg);

        videoCreate<<testImg;
        cout<<numFlag<<endl;
//        if(numFlag>100){
//            videoCreate.release();
//            return state_idle;
//        }
        break;
    case 2:
        //基于位置的视觉伺服
        /*手动指定目标位置*/
        if(mouseRatio.x>0&&mouseRatio.y>0)
        {
            tipViaPBVS.x=(double)1024*mouseRatio.x;
            tipViaPBVS.y=(double)542*mouseRatio.y;
            srcMatrix(0,0)=(double)1024*mouseRatio.x;//统一单位为μm
            srcMatrix(1,0)=(double)542*mouseRatio.y;
            srcMatrix(2,0)=(double)1;
            srcMatrix.convertTo(srcMatrix, CV_64FC1);
            dstMatrix.convertTo(dstMatrix, CV_64FC1);
            transformMatrix.convertTo(transformMatrix, CV_64FC1);
            cout<<"srcMatrix:"<<srcMatrix<<endl;
            Mat transMat=transformMatrix.clone();
            dstMatrix=transMat*srcMatrix;//坐标转换
            cout<<"dstMatrix:"<<dstMatrix<<endl;
            /*清零*/
            mouseRatio.x=0;
            mouseRatio.y=0;
            peneFlag=1;
        }

        if(peneFlag && actOpen){
            if(manipulationSelection!=manipulationSelection_last){
                line2DParams.dev=manipulationSelection;//设置控制对象
                Ump_Select_Dev(&line2DParams);
                manipulationSelection_last=manipulationSelection;
            }
            if(ch_instrument_pose->poseSave.home_d<10&&ch_instrument_pose->poseSave.home_x<10&&ch_instrument_pose->poseSave.home_y<10){
                Ump_Read_Position(&line2DParams);
            }else{
                line2DParams=ch_instrument_pose->poseSave;//间接读取，避免报错
            }
//            Ump_Read_Position(&line2DParams);
//            line2DParams=ch_instrument_pose->poseSave;//间接读取，避免报错
            int targetX=dstMatrix(0,0)*1000;//单位由μm转换为nm
            int targetY=dstMatrix(1,0)*1000;
            line2DParams.target_y=targetY;
            line2DParams.target_x=targetX;
            line2DParams.target_z=line2DParams.home_z;
            line2DParams.target_d=line2DParams.home_d;
            line2DParams.speed=1000;
//            line2DParams.speed=50;
            line2DParams.acc=6000;
            Ump_Goto_For_Injection(&line2DParams);
//            Ump_Goto_Position(&line2DParams);
            cout <<"Wait 5s"<< std::endl;
            sendState(QString("Wait 0.5s"));
            Sleep(500);
            sendState(QString("Next Please!"));
            peneFlag=0;//避免重复
        }
        break;
    case 3:

        if(Ump_Read_Position(&line2DParams)==1 && startFlag==2){
            imwrite(filename,testImg);
            write_<<"numFlag "<<numFlag<<"  x "<<line2DParams.home_x<<"  y "<<line2DParams.home_y<<"  z "<<line2DParams.home_z<<"  d "<<line2DParams.home_d<<endl;
            cout<<numFlag<<endl;
        }

        if(startFlag==1)
        {
            srcMatrix(0,0)=(double)1024*-0.1;//统一单位为μm
            srcMatrix(1,0)=(double)542*0.1;
            srcMatrix(2,0)=(double)1;
            srcMatrix.convertTo(srcMatrix, CV_64FC1);
            dstMatrix.convertTo(dstMatrix, CV_64FC1);
            transformMatrix.convertTo(transformMatrix, CV_64FC1);
            cout<<"srcMatrix:"<<srcMatrix<<endl;
            Mat transMat=transformMatrix.clone();
            dstMatrix=transMat*srcMatrix;//坐标转换
            cout<<"dstMatrix:"<<dstMatrix<<endl;
            if(manipulationSelection!=manipulationSelection_last){
                line2DParams.dev=manipulationSelection;//设置控制对象
                Ump_Select_Dev(&line2DParams);
                manipulationSelection_last=manipulationSelection;
            }
//            Ump_Read_Position(&line2DParams);
            if(ch_instrument_pose->poseSave.home_d<10&&ch_instrument_pose->poseSave.home_x<10&&ch_instrument_pose->poseSave.home_y<10){
                Ump_Read_Position(&line2DParams);
            }else{
                line2DParams=ch_instrument_pose->poseSave;//间接读取，避免报错
            }
//            line2DParams=ch_instrument_pose->poseSave;//间接读取，避免报错
            int targetX=dstMatrix(0,0)*1000;//单位由μm转换为nm
            int targetY=dstMatrix(1,0)*1000;
            line2DParams.target_y=targetY;
            line2DParams.target_x=targetX;
            line2DParams.target_z=line2DParams.home_z;
            line2DParams.target_d=line2DParams.home_d;
            line2DParams.speed=1000;
            line2DParams.acc=6000;
            Ump_Goto_For_Injection(&line2DParams);
            z_original=line2DParams.home_z;
            d_original=line2DParams.home_d;
            cout <<"Wait 5s"<< std::endl;
            sendState(QString("Wait 0.5s"));
            Sleep(1500);
            sendState(QString("Next Please!"));
            startFlag++;
            numFlag=-1;
        }else{
            srcMatrix(0,0)=(double)1024*(x_pose*0.8+0.1);//统一单位为μm
            srcMatrix(1,0)=(double)542*(y_pose*0.8+0.1);
            srcMatrix(2,0)=(double)1;
            srcMatrix.convertTo(srcMatrix, CV_64FC1);
            dstMatrix.convertTo(dstMatrix, CV_64FC1);
            transformMatrix.convertTo(transformMatrix, CV_64FC1);
            cout<<"srcMatrix:"<<srcMatrix<<endl;
            Mat transMat=transformMatrix.clone();
            dstMatrix=transMat*srcMatrix;//坐标转换
            cout<<"dstMatrix:"<<dstMatrix<<endl;
            if(manipulationSelection!=manipulationSelection_last){
                line2DParams.dev=manipulationSelection;//设置控制对象
                Ump_Select_Dev(&line2DParams);
                manipulationSelection_last=manipulationSelection;
            }
            if(ch_instrument_pose->poseSave.home_d<10&&ch_instrument_pose->poseSave.home_x<10&&ch_instrument_pose->poseSave.home_y<10){
                Ump_Read_Position(&line2DParams);
            }else{
                line2DParams=ch_instrument_pose->poseSave;//间接读取，避免报错
            }
//            Ump_Read_Position(&line2DParams);
//            line2DParams=ch_instrument_pose->poseSave;//间接读取，避免报错
            int targetX=dstMatrix(0,0)*1000;//单位由μm转换为nm
            int targetY=dstMatrix(1,0)*1000;
            line2DParams.target_y=targetY;
            line2DParams.target_x=targetX;
            line2DParams.target_z=z_original;
            line2DParams.target_d=d_original;
            line2DParams.speed=150;
            line2DParams.acc=6000;
            Ump_Goto_For_Injection(&line2DParams);
            cout <<"Wait 5s"<< std::endl;
            sendState(QString("Wait 3s"));
            Sleep(2500);
            sendState(QString("Next Please!"));

            //更新位置
            y_pose+=1/y_points;
            if (y_pose>1)
            {
                x_pose+=1/x_points;
                y_pose=0;
                if(x_pose>1){
                    startFlag=0;
                    handle.close();
                    planeSelection=state_idle;
                    cout<<"over!!!"<<endl;
                }
            }

        }

        break;
    default:
        break;
    }

    numFlag++;
    return state_manualmove;

}


int8_t Pose_Plane::angleMeasure()//测量D轴的倾角(5倍镜)
{
    static int numFlag=0;
    static Mat imageRead,test_img,result_img,grayImg;
    double Time_Focus=0;
    static Point2f tipLast(0,0);
    static VideoCapture injectionVideo(videoAddress);
    static VideoWriter videoCreate("angleMeasure.avi", CV_FOURCC('D', 'I', 'V', 'X'), 25, Size(2048,542),0);//彩色图像

    if(numFlag==0)//获取背景图像
    {
        backImg_original=getBackImg();
        resize(backImg_original,backImg,Size(1024,542));
    }

    /***摄像头方式***/
    if(cameraOpen)
    {
        while(Image_Control->GrabImage(imageRead,50)!=0)
        {
            std::cout << "Transfor: Grab failed " << std::endl;
            /*break;*/
        }
    }
    else
    {
        /***视频方式***/
        injectionVideo>>imageRead;
        if (imageRead.empty())//视频结束
        {
            videoCreate.release();
            return state_idle;
        }
        /***图片方式***/
//        imageRead= imread("D:/QT_space/line2D_test/image/line2d_probe/1.bmp");
    }
    resize(imageRead,test_img,Size(1024,542));

//    showImgUI(test_img);
    result_img=tipPositioningBK(Time_Focus,test_img,backImg,1);//获取尖端位置
    if(numFlag==0)
    {
        line2DParams.dev=manipulationSelection;//设置控制对象
        Ump_Select_Dev(&line2DParams);
        Ump_Read_Position(&line2DParams);
        Sleep(500);//等待运动完成
        line2DParams.target_y=line2DParams.home_y+int(((tipRight.y-test_img.size().height/2)*0.65*1000)*lensMagnify);//视野中央
        line2DParams.target_x=line2DParams.home_x-int(((tipRight.x-test_img.size().width/2)*0.65*1000)*lensMagnify);
        line2DParams.target_z=line2DParams.home_z;
        line2DParams.target_d=line2DParams.home_d;
        line2DParams.speed=1000;
        line2DParams.acc=6000;
        Ump_Goto_For_Injection(&line2DParams);
        Sleep(500);//等待运动完成
        numFlag++;
        return state_angleMeasure;
    }
    circle(test_img, tipRight,10, Scalar(0,0,255));
    circle(test_img, tipRight,3, Scalar(0,0,255));        
    cvtColor(test_img,grayImg,COLOR_BGR2GRAY);
    QImage showImg=MatToQImage(grayImg);
    emit sendImage(showImg);
    vector <Mat> vImg_1;
    Mat outImg;
    vImg_1.push_back(grayImg);
    vImg_1.push_back(result_img);
    hconcat(vImg_1,outImg);
    videoCreate<<outImg;


    double D_distance=100;
    if(actOpen){
        if(numFlag==1){
            tipLast=tipRight;
//            tipPoint.clear();
            line2DParams.dev=manipulationSelection;//设置控制对象
            Ump_Select_Dev(&line2DParams);
            Ump_Read_Position(&line2DParams);
            Sleep(100);
            line2DParams.target_y=line2DParams.home_y;
            line2DParams.target_x=line2DParams.home_x;
            line2DParams.target_z=line2DParams.home_z;
            line2DParams.target_d=line2DParams.home_d-D_distance*1000;
            line2DParams.speed=1000;
            line2DParams.acc=6000;
            Ump_Goto_For_Injection(&line2DParams);
            cout <<"Angle Measure Start!!!"<< std::endl;
            sendState(QString("Measure Start!!!"));
            numFlag++;
            Sleep(1000);
            return state_idle;
        }else if(numFlag==2){
            double X_distance=abs(tipRight.x-tipLast.x)+abs(tipRight.y-tipLast.y);
            X_distance*=0.68*lensMagnify;
            double angle=asin(X_distance/D_distance);
            cout <<"Angle_circle:"<<angle<< std::endl;
            angle=angle*180/3.14159;
            cout <<"Angle_degree:"<<angle<< std::endl;
            sendState(QString("Angle: %1").arg(angle));
            numFlag=0;
            videoCreate.release();
            injectionVideo.release();
            return state_idle;
        }

    }

    return state_angleMeasure;

}



Mat Pose_Plane::getBackImg()//获取背景图像
{
    Mat imageRead,test_img,result_img,grayImg,mogImage;
    if(actOpen)
    {
        if(manipulationSelection!=manipulationSelection_last){
            line2DParams.dev=manipulationSelection;//设置控制对象
            Ump_Select_Dev(&line2DParams);
            manipulationSelection_last=manipulationSelection;
        }
        if(ch_instrument_pose->poseSave.home_d<10&&ch_instrument_pose->poseSave.home_x<10&&ch_instrument_pose->poseSave.home_y<10){
            Ump_Read_Position(&line2DParams);
        }else{
            line2DParams=ch_instrument_pose->poseSave;//间接读取，避免报错
        }
//        Ump_Read_Position(&line2DParams);
//        line2DParams=ch_instrument_pose->poseSave;//间接读取，避免报错
        Sleep(500);
        line2DParams.target_y=line2DParams.home_y;
        line2DParams.target_x=line2DParams.home_x;
        line2DParams.target_z=line2DParams.home_z-20000;
        line2DParams.target_d=line2DParams.home_d;
        line2DParams.speed=1000;
        line2DParams.acc=6000;
        Ump_Goto_Position(&line2DParams);
        Sleep(500);
        line2DParams.target_y=line2DParams.home_y+400000*lensMagnify;
        line2DParams.target_x=line2DParams.home_x-400000*lensMagnify;
        line2DParams.target_z=line2DParams.home_z-20000;
        line2DParams.target_d=line2DParams.home_d;
        line2DParams.speed=2000;
        Ump_Goto_For_Injection(&line2DParams);
        Sleep(1000);//等待运动完成

        /***摄像头方式***/
        if(cameraOpen)
        {
            Image_Control->GrabImage(imageRead,50);
            Sleep(100);//等待
            while(Image_Control->GrabImage(imageRead,50)!=0)
            {
                std::cout << "Transfor: Grab failed " << std::endl;
                /*break;*/
            }
        }
        if(!imageRead.empty())
        {
            result_img=imageRead.clone();
//            resize(imageRead,result_img,Size(1024,542));
//            imwrite("back.bmp",result_img);
        }else{
            cout<<"back_img get error"<<endl;
        }

        line2DParams.target_y=line2DParams.home_y;
        line2DParams.target_x=line2DParams.home_x;
        line2DParams.target_z=line2DParams.home_z-20000;
        line2DParams.target_d=line2DParams.home_d;
        line2DParams.speed=2000;
        line2DParams.acc=6000;
        Ump_Goto_For_Injection(&line2DParams);
        Sleep(500);
        line2DParams.target_y=line2DParams.home_y;
        line2DParams.target_x=line2DParams.home_x;
        line2DParams.target_z=line2DParams.home_z;
        line2DParams.target_d=line2DParams.home_d;
        line2DParams.speed=1000;
        line2DParams.acc=6000;
        Ump_Goto_Position(&line2DParams);
        Sleep(300);//等待运动完成
    }
//    if(!imageRead.empty())
//    {
//        resize(imageRead,result_img,Size(1024,542));
//        imwrite("back.bmp",result_img);
//    }else{
//        cout<<"back_img get error"<<endl;
//    }

    return result_img;

}




int8_t Pose_Plane::tipFocusing(int range,int threshold,int fitNumber)//针尖聚焦
{
    return state_tipFocusing;

}


void Pose_Plane::Image_processing(Mat& Image_G, Mat& dst,int &subNum,int8_t fineTuning)
{

}


void Pose_Plane::Data_processing(Mat * Input_Image,uint32_t * sobel_cols,float * sobel_win,int *Sobel_Max_Num)
{
    Mat Image_D=*Input_Image;
    uint32_t * sobelColBegin=sobel_cols;
    float * sobelWinBegin=sobel_win;
    float sobel_max=0;
    int Image_Rows=Image_D.rows;
    int Image_Cols=Image_D.cols;
    uint8_t LENGTH_WIN=10;

    for(int i=0;i<Image_Cols;i++)
    {
        for(int j=0;j<Image_Rows;j++)
        {
            *sobel_cols+=Image_D.at<uchar>(j,i);//指针方法只能获得行向量指针，不能获得列向量指针
        }
        sobel_cols++;
    }//对每一列元素求和
    Image_Cols-=LENGTH_WIN;
    sobel_cols=sobelColBegin;
    for(int i=0;i<Image_Cols;)
    {
        for(int j=0;j<LENGTH_WIN;j++)
        {
          *sobel_win+=*(sobel_cols+i+j);
        }
        *sobel_win/=10;
        if(sobel_max<(*sobel_win))
        {
            sobel_max=*sobel_win;
            *Sobel_Max_Num=i/10;
        }
        sobel_win++;
        i=i+10;
    }//10列为窗口大小求平均
    sobel_win=sobelWinBegin;
    if(abs(sobel_max)>0)
    {
        for(int i=0;i<Image_Cols/10;i++)
        {
            *sobel_win/=sobel_max;
            sobel_win++;
        }//归一化
    }

}


/*********************
Function:斜轴特征定位
Abstract:利用倾斜探针局部清晰的特征，实现Z轴定位
 Author :胡伟康
*********************/
int8_t Pose_Plane::bevelPositioning()
{
    /***Test of single_slot***/
    int Sobel_Max_Num=0;
    static int i_count=0,sobelMaxNumLast=0,direction=1,turnFlag=0;
    int subNum=0;
    int stepSize=5;
    int8_t fineFlag=0;
    char * filename=new char[100];
    double Time_Focus;
    float Sobel_Win[500]={0}; //单张图片局部窗口清晰度值
    uint32_t Sobel_Cols[3000]={0};//每列灰度值之和
    float Array_Sharpness[Auto_Focus_Rows][Auto_Focus_Cols]={{0}};//多张图片清晰度值
    float Array_Sharpness_Save[Auto_Focus_Rows]={0};//多张图片清晰度值

    Mat Image_BGR,imageRead,test_img,maskImg,roiImg,roi_img;
    Mat sobel_x,sobel_y;
    FileStorage fs("Auto_Focus.yaml",FileStorage::WRITE);
    VideoWriter videoCreate("verticalP.avi", CV_FOURCC('D', 'I', 'V', 'X'), 3, Size(1024,542),0);

    if(i_count==0&&turnFlag==0){
        backImg_original=getBackImg();
        resize(backImg_original,backImg,Size(1024,542));
    }

    /***Test of sharpness***/
         /***Read image***/
        if(cameraOpen)
        {   /***摄像头方式***/
            Image_Control->GrabImage(imageRead,5);
            Sleep(50);
            while(Image_Control->GrabImage(imageRead,5)!=0)
            {
//                std::cout << "Grab image failed " << std::endl;
                /*break;*/
            }
//            sprintf(filename,"%d.bmp",i_count*50);
//            imwrite(filename,imageRead);
        }
        else
        {
            sprintf(filename,imgBevel,i_count*50);
            imageRead=imread(filename);
        }
        resize(imageRead,test_img,Size(1024,542));
        cvtColor(test_img,roi_img,CV_BGR2GRAY);
       /***Calculate sharpness***/
        Time_Focus=getTickCount();

        Mat elementOpen = getStructuringElement(MORPH_ELLIPSE,Size(10, 10));

        if(i_count==0){
            if(actOpen)
            {
                line2DParams.dev=manipulationSelection;//设置控制对象
                Ump_Select_Dev(&line2DParams);
                Ump_Read_Position(&line2DParams);
            }
        }
        tipPositioningBK(Time_Focus,test_img,backImg,1);//获取尖端位置
        roiImg=roi_img(Rect(0,tipRight.y-30,tipRight.x+35,60));
        imwrite("roiImg.png",roiImg);

        Image_processing(roiImg,sobel_y,subNum,fineFlag);
        /***清理原有数据***/
        memset(Sobel_Cols,0,sizeof(Sobel_Cols));
        memset(Sobel_Win,0,sizeof(Sobel_Win));
        if(subNum>40)//确保有充足信息的情况下，再进行图像处理
        {
            Data_processing(&sobel_y,&Sobel_Cols[0],&Sobel_Win[0],&Sobel_Max_Num);
        }
        else
        {
            Sobel_Max_Num=0;//否则线性特征置为0
        }
        //Register Liner Single
        Array_Sharpness[i_count][1]=i_count;//代表Z轴的上下移动
        Array_Sharpness[i_count][0]=Sobel_Max_Num;//代表X轴的左右运动
//        this->send(2);//观察线性关系
        //waitKey(10);
        Time_Focus=((double)getTickCount()-Time_Focus)/getTickFrequency()*1000;
        std::cout<<"i_count:"<<i_count<<",z:"<<i_count*50<<",x:"<<Sobel_Max_Num<<",Time_Focus:"<<Time_Focus<<"ms"<<std::endl;
        if(Sobel_Max_Num!=0)
        {
            cout<<"subNum:"<<subNum<<endl;
//            fineFlag=1;
            if(sobelMaxNumLast>Sobel_Max_Num)//越过了极大值，往回找一个，针尖在最右边
            {
//                direction=-1;
                turnFlag = 1;
            }
        }
        if(turnFlag==1 &&Sobel_Max_Num==0)
        {
            if(actOpen)
            {
                //保留清晰尖端位置
                line2DParams.dev=manipulationSelection;//设置控制对象
                Ump_Select_Dev(&line2DParams);
                Ump_Read_Position(&line2DParams);
                Sleep(100);
                tipClearPosition=line2DParams;
                tipClearPosition.home_z+=stepSize*1000;
                Ump_Goto_Position(&tipClearPosition);
            }
            //定位结束
            fs.release();
            videoCreate.release();
            cout << "Tip focusing over " <<endl;
            return state_idle;
        }
        if(actOpen)
        {
            /***自动移动Z位置***/
           line2DParams.target_d=line2DParams.home_d;
           line2DParams.target_y=line2DParams.home_y;
           line2DParams.target_z=line2DParams.home_z+direction*(i_count+1)*stepSize*1000;
           line2DParams.target_x=line2DParams.home_x;
           line2DParams.speed=250;
           Ump_Goto_Position(&line2DParams);
           Sleep(100);
        }
        if(turnFlag==0)
        {
           i_count++;
        }else{
           i_count--;
        }

        sobelMaxNumLast=Sobel_Max_Num;
        videoCreate.write(test_img);
        return state_bevelPositioning;



}

/*********************
Function:斜轴定位(无用，定位效果不好)
Abstract:基于斜轴的线性关系，先拟合直线，再一步到达尖端位置
Author :胡伟康
*********************/
void Pose_Plane::Go_Focus()
{
//    char * filename=new char[100];
//    int Now_X=0;
//    int subNum=0;
//    float Distance_Z=0,Target_X=70,Target_Z=0;
//    memset(Sobel_Cols,0,sizeof(Sobel_Cols));
//    memset(Sobel_Win,0,sizeof(Sobel_Win));
//    Mat Now_Image,Result_Image;
//    /***Read image***/
//   if(cameraOpen)
//   {   /***摄像头方式***/
//       while(Image_Control->GrabImage(Now_Image,5)!=0)
//       {
//           std::cout << "Grab image failed " << std::endl;
//           /*break;*/
//       }
//       sprintf(filename,"now.bmp");
//       imwrite(filename,Now_Image);
//   }
//   else
//   {
//       sprintf(filename,imgBevel,700);
//       Now_Image=imread(filename);
//   }
//   resize(Now_Image,Now_Image,Size(720,480));
//    /*** Image process ***/
//    Image_processing(Now_Image,Result_Image,subNum,0);
//    memset(Sobel_Cols,0,sizeof(Sobel_Cols));
//    memset(Sobel_Win,0,sizeof(Sobel_Win));
//    Data_processing(&Result_Image,&Sobel_Cols[0],&Sobel_Win[0],&Now_X);//获取当前清晰的左右位置
//    /*** calculate and curb ***/
////    Line_k=-0.305979;
//    Distance_Z=(Target_X-Now_X)*Line_k*stepSize;//Z轴相对运动距离
//    if(actOpen)
//    {
//        line2DParams.target_d=0;
//        line2DParams.target_y=0;
//        line2DParams.target_z=(int)(Distance_Z*1000);
//        line2DParams.target_x=0;
//        line2DParams.speed=500;
//        Ump_Take_Step(&line2DParams);//相对位移
//    }
//    std::cout<<"Now_X:"<<Now_X<<";Distance_Z="<<Distance_Z<<std::endl;
}

int8_t Pose_Plane::moveOut(int distance)//移出
{
    line2DParams.dev=manipulationSelection;//设置控制对象
    Ump_Select_Dev(&line2DParams);
    Ump_Read_Position(&line2DParams);
    Sleep(500);
    markParams=line2DParams;
    line2DParams.target_y=markParams.home_y;
    line2DParams.target_x=markParams.home_x;
    line2DParams.target_z=markParams.home_z-50000;
    line2DParams.target_d=markParams.home_d;
    line2DParams.speed=1000;
    Ump_Goto_Position(&line2DParams);

    line2DParams.target_y=markParams.home_y+distance;
    line2DParams.target_x=markParams.home_x;
    line2DParams.target_z=markParams.home_z-80000;
    line2DParams.target_d=markParams.home_d;
    line2DParams.speed=1000;
    Ump_Goto_Position(&line2DParams);
    cout <<"moveOut!!!"<< std::endl;
    sendState(QString("moveOut!!!"));

    return state_idle;

}

int8_t Pose_Plane::moveIn(int distance)//移入
{
    line2DParams.dev=manipulationSelection;//设置控制对象
    Ump_Select_Dev(&line2DParams);
    Ump_Read_Position(&line2DParams);
    line2DParams.target_y=markParams.home_y;
    line2DParams.target_x=markParams.home_x;
    line2DParams.target_z=markParams.home_z-80000;
    line2DParams.target_d=markParams.home_d;
    line2DParams.speed=1000;
    Ump_Goto_Position(&line2DParams);

    line2DParams.target_y=markParams.home_y;
    line2DParams.target_x=markParams.home_x;
    line2DParams.target_z=markParams.home_z-8000;
    line2DParams.target_d=markParams.home_d;
    line2DParams.speed=1000;
    Ump_Goto_Position(&line2DParams);
    cout <<"moveIn!!!"<< std::endl;
    sendState(QString("moveIn!!!"));

    return state_idle;

}


int8_t Pose_Plane::mogInit()
{
    static int numFlag=0,moveFlag=1,startFlag=0;

    Mat imageRead,testImg,resultImg,backImg;
    double Time_Focus=0;
    char * filename= new char[100];
    static VideoCapture injectionVideo(videoAddress);
    static VideoWriter videoCreate("mog.avi", CV_FOURCC('D', 'I', 'V', 'X'), 25, Size(1024,542),1);

    if(numFlag==0){
        backImg_original=getBackImg();
        resize(backImg_original,backImg,Size(1024,542));
    }

    /***摄像头方式***/
    if(cameraOpen)
    {
        while(Image_Control->GrabImage(imageRead,50)!=0)
        {
            std::cout << "Transfor: Grab failed " << std::endl;
            /*break;*/
        }
    }
    else
    {
        /***视频方式***/
        injectionVideo>>imageRead;
        if (imageRead.empty())//视频结束
        {
            videoCreate.release();
            return state_idle;
        }
//          if(numFlag%10!=0)//跳帧读取
//          {
//              numFlag++;
//              return state_manualmove;
//          }
        /***图片方式***/
//        imageRead= imread("D:/QT_space/line2D_test/image/line2d_probe/1.bmp");
    }
    resize(imageRead,testImg,Size(1024,542));


    if(numFlag==0){
        cout <<"mogInit Start!!!"<< std::endl;
        sendState(QString("mogInit Start!!!"));
        bg_model=createBackgroundSubtractorMOG2();
        tipPositioningBK(Time_Focus,testImg,backImg,1);//获取尖端位置
        if(actOpen && tipRight.x>0)
        {
            line2DParams.dev=manipulationSelection;//设置控制对象
            Ump_Select_Dev(&line2DParams);
            Ump_Read_Position(&line2DParams);
            line2DParams.target_y=line2DParams.home_y;
            line2DParams.target_x=line2DParams.home_x;
            line2DParams.target_z=line2DParams.home_z-10*1000;
            line2DParams.target_d=line2DParams.home_d;
            line2DParams.speed=500;
            Ump_Goto_Position(&line2DParams);
            Sleep(100);
            line2DParams.target_y=line2DParams.home_y+(tipRight.y-testImg.size().height/2)*0.68*1000;//视野中央
            line2DParams.target_x=line2DParams.home_x-(tipRight.x-testImg.size().width/2)*0.68*1000;
            line2DParams.target_z=line2DParams.home_z-10*1000;
            line2DParams.target_d=line2DParams.home_d;
            line2DParams.speed=500;
            Ump_Goto_Position(&line2DParams);
            Sleep(500);//等待运动完成
            line2DParams.dev=manipulationSelection;//设置控制对象
            Ump_Select_Dev(&line2DParams);
            Ump_Read_Position(&line2DParams);
            Sleep(100);//等待运动完成
        }
        numFlag=1;
        return state_mogInit;
    }

    resultImg=tipPositioning_mog2(Time_Focus,testImg,1,1);
    circle(testImg, tipRight,10, Scalar(0,0,255));
    circle(testImg, tipRight,3, Scalar(0,0,255));
    showImgUI(testImg);


    line2DParams.target_y=line2DParams.home_y+moveFlag*100*1000;
    line2DParams.target_x=line2DParams.home_x;
    line2DParams.target_z=line2DParams.home_z;
    line2DParams.target_d=line2DParams.home_d;
    line2DParams.speed=30;
    Ump_Goto_Position(&line2DParams);

    if(moveFlag==1){
        moveFlag=-1;
    }else{
        moveFlag=1;
    }

    videoCreate<<testImg;
    cout<<"mogFlag: "<<numFlag<<endl;
    if(numFlag++>25){
        videoCreate.release();
        startFlag=0;
        numFlag=0;
        Ump_stop(&line2DParams);
        return state_idle;
    }else{
        return state_mogInit;
    }


}




int8_t Pose_Plane::semiAutoBiopsy()
{

    return state_semiAutoBiopsy;

}


int8_t Pose_Plane::electrochemicalmapping()
{
    static int numFlag=0,startFlag=0,penetrationFlag=0;
    static int x_pose=0,y_pose=0;
    static int targetX=0,targetY=0;

    if(startFlag==0){
        line2DParams.dev=manipulationSelection;//设置控制对象
        Ump_Select_Dev(&line2DParams);
        Ump_Read_Position(&line2DParams);
        mappingParams=line2DParams;
        startFlag=1;
    }

    int x_distance=30,y_distance=30;
    int x_points=5,y_points=5;

//    double zUp=10000+zUpAdd;//高于细胞表面2μm
//    double zDown=0+zDownAdd;//低于细胞表面5μm
//    int xBack=(int)((zUp+zDown)*tan(angleD));//保持两轴距离一致
//    int dDistance=(int)((zUp+zDown)/cos(angleD));

    double zUp=10000+zUpAdd;//高于细胞表面2μm
    double zDown=0-zDownAdd;//低于细胞表面0μm
    int xBack=(int)((injectionDis)*tan(angleD));//保持两轴距离一致
    int dDistance=(int)((injectionDis)/cos(angleD));

    if(penetrationFlag==0)
    {
        line2DParams.target_y=mappingParams.home_y+y_pose*1000;
        targetY=line2DParams.target_y;
        line2DParams.target_x=mappingParams.home_x+x_pose*1000;
        targetX=line2DParams.target_x;
        line2DParams.target_z=mappingParams.home_z;
        line2DParams.target_d=mappingParams.home_d;
        line2DParams.speed=10;
        line2DParams.acc=6000;
        Ump_Goto_For_Injection(&line2DParams);
        Sleep(1000);
        penetrationFlag=1;

        if(mulTouchSelection==1)//更新touchPosition
        {
            mulTouchSelection=-1;
            return state_touchDetection;
        }

    }

    //更新位置
    if (y_pose>y_distance)
    {
        if(x_pose>x_distance){
            startFlag=0;
            return state_idle;
        }
        x_pose+=x_distance/x_points;
        y_pose=0;
    }
    else{
        y_pose+=y_distance/y_points;
    }



    //仅基于坐标变换的视觉伺服
    line2DParams.target_y=targetY;
    line2DParams.target_x=targetX-xBack;
    line2DParams.target_z=touchPosition.home_z-zUp;//扎入细胞
    line2DParams.target_d=touchPosition.home_d;
    line2DParams.speed=500;
    Ump_Goto_Position(&line2DParams);
    cout <<"Wait 5s"<< std::endl;
    sendState(QString("Wait 0.5s"));
    Sleep(100);
    line2DParams.target_y=targetY;
    line2DParams.target_x=targetX-xBack;
    line2DParams.target_z=touchPosition.home_z-injectionDis-zDown;//扎入细胞
    line2DParams.target_d=touchPosition.home_d;
    line2DParams.speed=500;
    Ump_Goto_Position(&line2DParams);
    cout <<"Wait 5s"<< std::endl;
    sendState(QString("Wait 0.5s"));
    Sleep(800);
    /*扎入细胞*/
    line2DParams.target_y=targetY;
    line2DParams.target_x=targetX-xBack;
    line2DParams.target_z=touchPosition.home_z-injectionDis-zDown;//扎入细胞
//                line2DParams.target_z=touchPosition.home_z;//扎入细胞
    line2DParams.target_d=touchPosition.home_d+dDistance;
    line2DParams.speed=500;
    line2DParams.acc=6000;
    Ump_Goto_For_Injection(&line2DParams);
    cout <<"Penetration 5s"<< std::endl;
    sendState(QString("Penetration Waiting..."));
    for(int sleepNum=0; sleepNum<penetrationTime; sleepNum++)
    {
        cout <<"Penetration Time:"<<sleepNum<< std::endl;
        sendState(QString("Time : %1").arg(sleepNum));
        Sleep(1000);//扎进细胞的时间
    }
    /*退出细胞*/
    line2DParams.target_y=targetY;
    line2DParams.target_x=targetX-xBack;//退出细胞
    line2DParams.target_z=touchPosition.home_z-injectionDis-zDown;//高于细胞表面2μm
    line2DParams.target_d=touchPosition.home_d;
    line2DParams.speed=500;
    line2DParams.acc=6000;
    Ump_Goto_For_Injection(&line2DParams);
    Sleep(100);
    line2DParams.target_y=targetY;
    line2DParams.target_x=targetX-xBack;//退出细胞
    line2DParams.target_z=touchPosition.home_z-zUp;//高于细胞表面2μm
    line2DParams.target_d=touchPosition.home_d;
    line2DParams.speed=500;
    line2DParams.acc=6000;
    Ump_Goto_For_Injection(&line2DParams);




//    //仅基于坐标变换的视觉伺服
//    line2DParams.target_y=targetY;
//    line2DParams.target_x=targetX-xBack;
//    line2DParams.target_z=touchPosition.home_z-zUp;//扎入细胞
//    line2DParams.target_d=touchPosition.home_d;
//    line2DParams.speed=500;
//    Ump_Goto_Position(&line2DParams);
//    cout <<"Wait 5s"<< std::endl;
//    sendState(QString("Wait 0.5s"));
//    Sleep(800);
//    /*扎入细胞*/
//    line2DParams.target_y=targetY;
//    line2DParams.target_x=targetX-xBack;
//    line2DParams.target_z=touchPosition.home_z-zUp;//扎入细胞
//    line2DParams.target_d=touchPosition.home_d+dDistance;
//    line2DParams.speed=500;
//    Ump_Goto_For_Injection(&line2DParams);
//    cout <<"Penetration 5s"<< std::endl;
//    sendState(QString("Penetration Waiting..."));
//    for(int sleepNum=0; sleepNum<penetrationTime; sleepNum++)
//    {
//        cout <<"Penetration Time:"<<sleepNum<< std::endl;
//        sendState(QString("Time : %1").arg(sleepNum));
//        Sleep(1000);//扎进细胞的时间
//    }
//    /*退出细胞*/
//    line2DParams.target_y=targetY;
//    line2DParams.target_x=targetX-xBack;//退出细胞
//    line2DParams.target_z=touchPosition.home_z-zUp;//高于细胞表面2μm
//    line2DParams.target_d=touchPosition.home_d;
//    line2DParams.speed=500;
//    Ump_Goto_For_Injection(&line2DParams);
    if(mulTouchSelection==-1)
    {
        mulTouchSelection=1;
    }



    numFlag++;
    penetrationFlag=0;
    return state_electrochemicalMapping;

}


// 生成三维S字形扫描轨迹的函数
void Pose_Plane::generateScanPath(int curveFlag){

    scanningPoints.clear();

    // 控制扫描方向
    bool forward = true;
    bool scanZ = true;
    double stepNum = scanning_numSteps[0];

//    Mat imgMonaTrans;
//    imageMonaLisa = imread("D:/0_MYM_ONLY/Microsystem/image/Mona_Lisa_resized.jpg");
//    resize(imageMonaLisa,imageMonaLisa,Size(75,111));

    imageMonaLisa = imread("D:/0_MYM_ONLY/Microsystem/image/XuBeihong_resized.jpg");
//    resize(imageMonaLisa,imageMonaLisa,Size(75,111));

    imageMonaLisa = imread("D:/0_MYM_ONLY/Microsystem/image/1_resized.jpg");
    cvtColor(imageMonaLisa,imageMonaLisa,CV_BGR2GRAY);

    paitingImg = imageMonaLisa.clone();
    for(int i = 0; i < imageMonaLisa.rows; i++)
    {
        for(int j = 0; j < imageMonaLisa.cols; j++)
        {
            paitingImg.at<uchar>(i, j) = 255 - imageMonaLisa.at<uchar>(i, j);
        }
    }

    imwrite("imageMonaSave.jpg",imageMonaLisa);

    switch (curveFlag) {
    case 0:
        // 在三维空间内扫描
        for (int i = 0; i < scanning_numSteps[scanning_axisOrder[0]]; ++i) {
            // 确定行的扫描方向
            bool scanForward = (i % 2 == 0) ? true : false;

            for (int j = 0; j < scanning_numSteps[scanning_axisOrder[1]]; ++j) {
                // 确定列的扫描方向
                bool scanUp = (scanForward) ? true : false;

                for (int k = 0; k < scanning_numSteps[scanning_axisOrder[2]]; ++k) {
                    // 确定当前点的坐标
                    double pose[3] = {};
    //                pose[scanning_axisOrder[0]] = (scanForward) ? i * scanning_stepLength[scanning_axisOrder[0]] : (scanning_numSteps[scanning_axisOrder[0]] - 1 - i) * scanning_stepLength[scanning_axisOrder[0]];
                    pose[scanning_axisOrder[0]] = i * scanning_stepLength[scanning_axisOrder[0]];
                    pose[scanning_axisOrder[1]] = (scanUp) ? j * scanning_stepLength[scanning_axisOrder[1]] : (scanning_numSteps[scanning_axisOrder[1]] - 1 - j) * scanning_stepLength[scanning_axisOrder[1]];
                    pose[scanning_axisOrder[2]] = (scanZ) ? k * scanning_stepLength[scanning_axisOrder[2]] : (scanning_numSteps[scanning_axisOrder[2]] - 1 - k) * scanning_stepLength[scanning_axisOrder[2]];

                    // 根据轴顺序调整坐标
    ////                double coord[3] = {currentX, currentY, currentZ};
    //                double temp[3];
    //                for (int idx = 0; idx < 3; ++idx) {
    ////                    temp[idx] = coord[scanning_axisOrder[idx]];
    //                    temp[idx] = pose[idx];
    //                }

                    // 添加顶点
                    scanningPoints.push_back(Point3f(pose[0], pose[1], pose[2]));
                    cout<<"x: "<<pose[0]<<" y: "<< pose[1]<<" z: "<<pose[2]<<endl;
                }

                scanZ = !scanZ;

                // 切换列的扫描方向
                scanUp = !scanUp;
            }

            // 切换行的扫描方向
            scanForward = !scanForward;
        }
        break;
    case 1:
        for (double k = 0; k < stepNum+1; ++k) {
            // 确定当前点的坐标
            double pose[3] = {};
            pose[0] = (1.0/2.75)*scanning_stepLength[0]*(sin(k*(2*3.1415926/stepNum)) + 2*sin(2*k*(2*3.1415926/stepNum)));
            pose[1] = (1.0/3.0)*(scanning_stepLength[1]*(cos(k*(2*3.1415926/stepNum)) - 2*cos(2*k*(2*3.1415926/stepNum))+1));
            pose[2] = -scanning_stepLength[2]*sin(3*k*(2*3.1415926/stepNum));

            // 添加顶点
            scanningPoints.push_back(Point3f(pose[0], pose[1], pose[2]));
            cout<<"k: "<<k<<" x: "<<pose[0]<<" y: "<< pose[1]<<" z: "<<pose[2]<<endl;
        }
        break;
    case 2:
        scanning_numSteps[scanning_axisOrder[2]] = paitingImg.size().width;
        scanning_numSteps[scanning_axisOrder[1]] = paitingImg.size().height;
        scanning_numSteps[scanning_axisOrder[0]] = 1;

        // 在三维空间内扫描
        for (int i = 0; i < scanning_numSteps[scanning_axisOrder[0]]; ++i) {
            // 确定行的扫描方向
            bool scanForward = (i % 2 == 0) ? true : false;

            for (int j = 0; j < scanning_numSteps[scanning_axisOrder[1]]; ++j) {
                // 确定列的扫描方向
                bool scanUp = (scanForward) ? true : false;

                for (int k = 0; k < scanning_numSteps[scanning_axisOrder[2]]; ++k) {
                    // 确定当前点的坐标
                    double pose[3] = {};
    //                pose[scanning_axisOrder[0]] = (scanForward) ? i * scanning_stepLength[scanning_axisOrder[0]] : (scanning_numSteps[scanning_axisOrder[0]] - 1 - i) * scanning_stepLength[scanning_axisOrder[0]];
                    pose[scanning_axisOrder[0]] = i * scanning_stepLength[scanning_axisOrder[0]];
                    pose[scanning_axisOrder[1]] = (scanUp) ? j * scanning_stepLength[scanning_axisOrder[1]] : (scanning_numSteps[scanning_axisOrder[1]] - 1 - j) * scanning_stepLength[scanning_axisOrder[1]];
                    pose[scanning_axisOrder[2]] = (scanZ) ? k * scanning_stepLength[scanning_axisOrder[2]] : (scanning_numSteps[scanning_axisOrder[2]] - 1 - k) * scanning_stepLength[scanning_axisOrder[2]];

                    // 根据轴顺序调整坐标
    ////                double coord[3] = {currentX, currentY, currentZ};
    //                double temp[3];
    //                for (int idx = 0; idx < 3; ++idx) {
    ////                    temp[idx] = coord[scanning_axisOrder[idx]];
    //                    temp[idx] = pose[idx];
    //                }

                    // 添加顶点
                    scanningPoints.push_back(Point3f(pose[0], pose[1], pose[2]));
                    cout<<"x: "<<pose[0]<<" y: "<< pose[1]<<" z: "<<pose[2]<<endl;
                }

                scanZ = !scanZ;

                // 切换列的扫描方向
                scanUp = !scanUp;
            }

            // 切换行的扫描方向
            scanForward = !scanForward;
        }
        break;
    default:
        break;
    }

    cout<<"Generated ScanPath Over"<<endl;
}

int8_t Pose_Plane::manualMoveControl(int axis, int direction)
{
    Mat imageRead;

    char * filename= new char[100];
    static int numFlag=0;
    static int reciprocatingDirection=1;

/*记录*/
    static char * filename1=new char[100];
    static int infoSaveInit=1;
    static QFile handle;
    static QTextStream write_;
    if(infoSaveInit==1){
        infoSaveInit=0;
        QDateTime timeMark= QDateTime::currentDateTime();
        sprintf(filename1,"%d_%d_%d_%d_reciprocating.txt",timeMark.date().dayOfYear(),timeMark.time().hour(),timeMark.time().minute(),timeMark.time().second());
        handle.setFileName(filename1);
        handle.open(QIODevice::Append);
        write_.setDevice(&handle);
    }
/*记录*/

    axisSelect=axis;

//    /***摄像头方式***/
//    if(cameraOpen)
//    {
//        Image_Control->GrabImage(imageRead,50);
//        Sleep(100);//等待
//        if(Image_Control->GrabImage(imageRead,50)!=0){
//            std::cout << "Transfor: Grab failed " << std::endl;
//        }
//    }
//    sprintf(filename,"Touch_%d.bmp",numFlag);//移动之前保留当下图片和位置
//    imwrite(filename,imageRead);



    if(manipulationSelection!=manipulationSelection_last){
        line2DParams.dev=manipulationSelection;//设置控制对象
        Ump_Select_Dev(&line2DParams);
        manipulationSelection_last=manipulationSelection;
    }


    if(ch_instrument_pose->poseSave.home_d<10&&ch_instrument_pose->poseSave.home_x<10&&ch_instrument_pose->poseSave.home_y<10){
        Ump_Read_Position(&line2DParams);
    }else{
        line2DParams=ch_instrument_pose->poseSave;//间接读取，避免报错
    }



/*记录*/
    static double cycleInit=getTickCount();
    double timeNow=(getTickCount()-cycleInit)/getTickFrequency();
    write_<<"timeNow "<<timeNow<<"  x "<<line2DParams.home_x<<"  y "<<line2DParams.home_y<<"  z "<<line2DParams.home_z<<"  d "<<line2DParams.home_d<<"  Speed "<<manualSpeed<<"\n";//移动之前保留当下图片和位置
    cout<<"timeNow "<<timeNow<<" scanningPointN "<<scanningPointN<<"  x "<<line2DParams.home_x<<"  y "<<line2DParams.home_y<<"  z "<<line2DParams.home_z<<"  d "<<line2DParams.home_d<<"  Speed "<<manualSpeed<<endl;//移动之前保留当下图片和位置
//    sendNowInfo(timeNow,0,line2DParams.home_x,line2DParams.home_y,line2DParams.home_z,line2DParams.home_d,0);
//    handle.close();
/*记录*/
    double manualDistaceSend=manualDistace*1000.0;
//    cout<<"manualDistaceSend"<<manualDistaceSend<<endl;

    switch (axisSelect) {
    case 1:
//        line2DParams.target_y=LIBUMP_ARG_UNDEF;
//        line2DParams.target_x=line2DParams.home_x+direction*manualDistaceSend;
//        line2DParams.target_z=LIBUMP_ARG_UNDEF;
//        line2DParams.target_d=LIBUMP_ARG_UNDEF;
        line2DParams.target_y=0;
        line2DParams.target_x=direction*manualDistaceSend;
        line2DParams.target_z=0;
        line2DParams.target_d=0;
        line2DParams.speed=manualSpeed;
        line2DParams.acc=manualAcc;

        sendTargetPose(line2DParams.target_x,0,0,0);
        Ump_Take_Step(&line2DParams);
//        Ump_Goto_For_Injection(&line2DParams);
        break;
    case 2:
//        line2DParams.target_y=line2DParams.home_y+direction*manualDistaceSend;
//        line2DParams.target_x=LIBUMP_ARG_UNDEF;
//        line2DParams.target_z=LIBUMP_ARG_UNDEF;
//        line2DParams.target_d=LIBUMP_ARG_UNDEF;

        line2DParams.target_y=direction*manualDistaceSend;
        line2DParams.target_x=0;
        line2DParams.target_z=0;
        line2DParams.target_d=0;

        line2DParams.speed=manualSpeed;
        line2DParams.acc=manualAcc;
        sendTargetPose(0,line2DParams.target_y,0,0);
        line2DParams.target_x=0;
        line2DParams.target_z=0;
        line2DParams.target_d=0;
        Ump_Take_Step(&line2DParams);
//        Ump_Goto_For_Injection(&line2DParams);
        break;
    case 3:
//        line2DParams.target_y=LIBUMP_ARG_UNDEF;
//        line2DParams.target_x=LIBUMP_ARG_UNDEF;
//        line2DParams.target_z=line2DParams.home_z-direction*manualDistaceSend;
//        line2DParams.target_d=LIBUMP_ARG_UNDEF;

        line2DParams.target_y=0;
        line2DParams.target_x=0;
        line2DParams.target_z=-direction*manualDistaceSend;
        line2DParams.target_d=0;

        line2DParams.speed=manualSpeed;
        line2DParams.acc=manualAcc;
        sendTargetPose(0,0,line2DParams.target_z,0);
        line2DParams.target_x=0;
        line2DParams.target_y=0;
        line2DParams.target_d=0;
        Ump_Take_Step(&line2DParams);
        emit manualZMoveCompleted(direction,manipulationSelection);
//        Ump_Goto_For_Injection(&line2DParams);
        break;
    case 4:
//        line2DParams.target_y=LIBUMP_ARG_UNDEF;
//        line2DParams.target_x=LIBUMP_ARG_UNDEF;
//        line2DParams.target_z=LIBUMP_ARG_UNDEF;
//        line2DParams.target_d=line2DParams.home_d-direction*manualDistaceSend;

        line2DParams.target_y=0;
        line2DParams.target_x=0;
        line2DParams.target_z=0;
        line2DParams.target_d=direction*manualDistaceSend;

        line2DParams.speed=manualSpeed;
        line2DParams.acc=manualAcc;
        sendTargetPose(0,0,0,line2DParams.target_d);
        Ump_Take_Step(&line2DParams);
//        Ump_Goto_For_Injection(&line2DParams);
        break;
    case 5:
        if(reciprocatingStrat==0){
            reciprocatingStrat++;
            poseReciprocating=line2DParams;
        }
        if(Ump_Busy(&line2DParams)==0)
//        if(numFlag%10==0)//10*10ms发送一次，最高频率10Hz
        {
            reciprocatingDirection=-reciprocatingDirection;
            line2DParams.target_y=poseReciprocating.home_y-reciprocatingDirection*manualDistaceSend;
            line2DParams.target_x=LIBUMP_ARG_UNDEF;
            line2DParams.target_z=LIBUMP_ARG_UNDEF;
            line2DParams.target_d=LIBUMP_ARG_UNDEF;
            line2DParams.speed=manualSpeed;
            line2DParams.acc=manualAcc;
            Ump_Goto_For_Injection(&line2DParams);
        }

        break;
    case 6:
        if(scanningStrat==0){
            scanningStrat++;
            poseScanning=line2DParams;
        }
        if(Ump_Busy(&line2DParams)==0)
        {
//            uint monaLisaTime = imageMonaLisa.at<uchar>((int)scanningPoints.at(scanningPointN).y,(int)scanningPoints.at(scanningPointN).x);
//            if(scanningSleepFlag<100){
//                scanningSleepFlag++;
//                cout<<"scanningSleepTIME: "<<scanningSleepFlag*10.0<<endl;
//                Sleep(monaLisaTime/8);
//            }else{
//                scanningSleepFlag = 0;
                if((!scanningPoints.empty())&&scanningPointN<scanningPoints.size()){

                    line2DParams.target_x=LIBUMP_ARG_UNDEF;
                    line2DParams.target_y=LIBUMP_ARG_UNDEF;
                    line2DParams.target_z=LIBUMP_ARG_UNDEF;
                    line2DParams.target_d=LIBUMP_ARG_UNDEF;

                    if(!(scanningPoints.at(scanningPointN).x>=(scanningPointLast.x-0.0001)&&scanningPoints.at(scanningPointN).x<=(scanningPointLast.x+0.0001))){

                        line2DParams.target_x=poseScanning.home_x+scanningPoints.at(scanningPointN).x*1000.0;
                        scanningPointLast.x=scanningPoints.at(scanningPointN).x;

                    }else if(!(scanningPoints.at(scanningPointN).y>=(scanningPointLast.y-0.0001)&&scanningPoints.at(scanningPointN).y<=(scanningPointLast.y+0.0001))) {

                        line2DParams.target_y=poseScanning.home_y+scanningPoints.at(scanningPointN).y*1000.0;
                        scanningPointLast.y=scanningPoints.at(scanningPointN).y;

                    }else if(!(scanningPoints.at(scanningPointN).z>=(scanningPointLast.z-0.0001)&&scanningPoints.at(scanningPointN).z<=(scanningPointLast.z+0.0001))){

                        line2DParams.target_z=poseScanning.home_z+scanningPoints.at(scanningPointN).z*1000.0;
                        scanningPointLast.z=scanningPoints.at(scanningPointN).z;
                    }
                    cout<<scanningPointN<<", x:"<<scanningPoints.at(scanningPointN).x<<", x2:"<<scanningPoints.at(scanningPointN).x<<"\n"<<endl;
                    scanningPointN++;

                    line2DParams.speed=manualSpeed;
                    line2DParams.acc=manualAcc;
                    sendTargetPose(line2DParams.target_x,line2DParams.target_y,line2DParams.target_z,line2DParams.target_d);
                    Ump_Goto_For_Injection(&line2DParams);


                }else{
                    scanningStrat=0;
                    planeSelection=state_idle;//结束运行
                    cout<<"scanningPointN over"<<endl;
                }
//            }

        }
        break;
    case 7:
        Ump_stop(&line2DParams);
        break;
    case 8:
        if(scanningStrat==0){
            scanningStrat++;
            poseScanning=line2DParams;
        }
        if(Ump_Busy(&line2DParams)==0)
        {

                if((!scanningPoints.empty())&&scanningPointN<scanningPoints.size()){

                    line2DParams.target_x=LIBUMP_ARG_UNDEF;
                    line2DParams.target_y=LIBUMP_ARG_UNDEF;
                    line2DParams.target_z=LIBUMP_ARG_UNDEF;
                    line2DParams.target_d=LIBUMP_ARG_UNDEF;


                    line2DParams.target_x=poseScanning.home_x+scanningPoints.at(scanningPointN).x*1000.0;
//                    scanningPointLast.x=scanningPoints.at(scanningPointN).x;
                    line2DParams.target_y=poseScanning.home_y+scanningPoints.at(scanningPointN).y*1000.0;
//                    scanningPointLast.y=scanningPoints.at(scanningPointN).y;
                    line2DParams.target_z=poseScanning.home_z+scanningPoints.at(scanningPointN).z*1000.0;
//                    scanningPointLast.z=scanningPoints.at(scanningPointN).z;

                    cout<<scanningPointN<<", x:"<<scanningPoints.at(scanningPointN).x<<endl;
                    scanningPointN++;

                    line2DParams.speed=manualSpeed;
                    line2DParams.acc=manualAcc;
                    sendTargetPose(line2DParams.target_x,line2DParams.target_y,line2DParams.target_z,line2DParams.target_d);
                    Ump_Goto_For_Injection(&line2DParams);


                }else{
                    scanningStrat=0;
                    planeSelection=state_idle;//结束运行
                    cout<<"scanningPointN over"<<endl;
                }

        }
        break;
    case 9:
        if(scanningStrat==0){
            scanningStrat++;
            poseScanning=line2DParams;
        }
        if(Ump_Busy(&line2DParams)==0)
        {
            uint monaLisaTime;
            //            if(scanningSleepFlag<100){
//                scanningSleepFlag++;
//                cout<<"scanningSleepTIME: "<<scanningSleepFlag*10.0<<endl;

//            }else{
//                scanningSleepFlag = 0;
                if((!scanningPoints.empty())&&scanningPointN<scanningPoints.size()){

                    monaLisaTime = paitingImg.at<uchar>((int)(scanningPoints.at(scanningPointN).y/scanning_stepLength[1]),(int)(scanningPoints.at(scanningPointN).x/scanning_stepLength[2]));
                    monaLisaTime=monaLisaTime*5;//等待时间
        //            monaLisaTime=1000;
                    cout<<"monaLisaTime: "<<monaLisaTime<<endl;

                    line2DParams.target_x=LIBUMP_ARG_UNDEF;
                    line2DParams.target_y=LIBUMP_ARG_UNDEF;
                    line2DParams.target_z=LIBUMP_ARG_UNDEF;
                    line2DParams.target_d=LIBUMP_ARG_UNDEF;

                    if(!(scanningPoints.at(scanningPointN).x>=(scanningPointLast.x-0.0001)&&scanningPoints.at(scanningPointN).x<=(scanningPointLast.x+0.0001))){

                        line2DParams.target_x=poseScanning.home_x+scanningPoints.at(scanningPointN).x*1000.0;
                        scanningPointLast.x=scanningPoints.at(scanningPointN).x;

                    }else if(!(scanningPoints.at(scanningPointN).y>=(scanningPointLast.y-0.0001)&&scanningPoints.at(scanningPointN).y<=(scanningPointLast.y+0.0001))) {

                        line2DParams.target_y=poseScanning.home_y+scanningPoints.at(scanningPointN).y*1000.0;
                        scanningPointLast.y=scanningPoints.at(scanningPointN).y;

                    }else if(!(scanningPoints.at(scanningPointN).z>=(scanningPointLast.z-0.0001)&&scanningPoints.at(scanningPointN).z<=(scanningPointLast.z+0.0001))){

                        line2DParams.target_z=poseScanning.home_z+scanningPoints.at(scanningPointN).z*1000.0;
                        scanningPointLast.z=scanningPoints.at(scanningPointN).z;
                    }
                    cout<<scanningPointN<<", x:"<<scanningPoints.at(scanningPointN).x<<", y:"<<scanningPoints.at(scanningPointN).y<<"\n"<<endl;
                    scanningPointN++;

                    line2DParams.speed=manualSpeed;
                    line2DParams.acc=manualAcc;
                    sendTargetPose(line2DParams.target_x,line2DParams.target_y,line2DParams.target_z,line2DParams.target_d);
                    Ump_Goto_For_Injection(&line2DParams);

                    Sleep(monaLisaTime);//抵达目标位置后延时


                }else{
                    scanningStrat=0;
                    planeSelection=state_idle;//结束运行
                    cout<<"scanningPointN over"<<endl;
                }
//            }

        }
        break;
    default:
        break;
    }

    numFlag++;
    return state_manualmoveStep;

}


void Pose_Plane::decision()
{
    static double cycleLast=getTickCount();
    double cycleNow,timeNow;

    switch (planeSelection) {
    case state_init:
        if(actOpen){
            line2DParamsFirst.dev=manipulationSelection;//设置控制对象
            Ump_Select_Dev(&line2DParamsFirst);
            line2DParams=line2DParamsFirst;
        }
        timePoseXY=new QTimer();
        timePoseXY->setInterval(10);
        timePoseXY->setTimerType(Qt::PreciseTimer);//增加这个语句，让计时更准确
        connect(timePoseXY,&QTimer::timeout,this,&Pose_Plane::decision);
        timePoseXY->start();
        planeSelection=state_idle;
        cout<<"waiting for XY Positioning "<<endl;
        break;
    case state_nonOvershootPositioning:
        planeSelection=nonOvershootPositioning();
        break;
    case state_coordinateTransformation:
        planeSelection=coordinateTransformation();
        break;
    case state_touchDetection:
        planeSelection=touchDetection();
        break;
    case state_penetration:
        planeSelection=penetration();
        break;
    case state_manualmove:
//        planeSelection=manualmove();
        manualmove();
        break;
    case state_angleMeasure:
        planeSelection=angleMeasure();
        break;
    case state_tipFocusing:
        planeSelection=tipFocusing(300,2,6);//范围、投票阈值、拟合点个数
        break;
    case state_bevelPositioning:
        planeSelection=bevelPositioning();
        break;
    case state_moveOut:
        planeSelection=moveOut(600000);
        break;
    case state_moveIn:
        planeSelection=moveIn(600000);
        break;
    case state_semiAutoBiopsy:
        planeSelection=semiAutoBiopsy();
        break;
    case state_mogInit:
        planeSelection=mogInit();
        break;
    case state_electrochemicalMapping:
        planeSelection=electrochemicalmapping();
        break;
    case state_manualmoveStep:
        manualMoveControl(axisSelect, 1);
        break;
    default:
//        cycleNow=getTickCount();
//        timeNow=(cycleNow-cycleLast)/getTickFrequency()*1000;
//        cycleLast=cycleNow;
//        cout<<"TIME:"<<timeNow<<endl;
        planeSelection=-1;
        break;
    }
}
