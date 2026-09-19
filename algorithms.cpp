#include "algorithms.h"

algorithms::algorithms()
{

}

/*********************
Function:快速开方
Abstract:牛顿法
Input   :
Author  :
*********************/
float algorithms::InvSqrt(float x)
{
    float xhalf = 0.5f*x;
    int i = *(int*)&x; // get bits for floating VALUE
    i = 0x5f375a86- (i>>1); // gives initial guess y0
    x = *(float*)&i; // convert bits BACK to float
    x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy
    return x;
}

uint8_t algorithms::Curve_Fitting(queue<QPointF>& points, int n, cv::Mat& A)
{
    //Number of key points
    queue<QPointF> pointsInput=points;
    vector<QPointF> key_point;
    QPointF pointTemporary;
    for(int i=0;i<points.size();i++){
        pointTemporary=pointsInput.front();
        pointsInput.pop();
        key_point.push_back(pointTemporary);
    }
    int N = key_point.size();

    //构造矩阵X
    cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);
    for (int i = 0; i < n + 1; i++)
    {
        for (int j = 0; j < n + 1; j++)
        {
            for (int k = 0; k < N; k++)
            {
                X.at<double>(i, j) = X.at<double>(i, j) +
                    std::pow(key_point.at(k).x(), i + j);

            }
        }
    }

    //构造矩阵Y
    cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
    for (int i = 0; i < n + 1; i++)
    {
        for (int k = 0; k < N; k++)
        {
            Y.at<double>(i, 0) = Y.at<double>(i, 0) +
                std::pow(key_point.at(k).x(), i) * key_point.at(k).y();
        }
    }

    A = cv::Mat::zeros(n + 1, 1, CV_64FC1);
    //求解矩阵A
    cv::solve(X, Y, A, cv::DECOMP_LU);
    return true;
}

//动态阈值的票箱
uint8_t algorithms::cvtAdjust(queue<QPointF>& curve,int threshold,int range,int& voteEnd,vector<Point3f>& voteMax)
{
    Mat curveOutcome;

    voteDynamic * pVoteDynamic;
    static voteDynamic * pVoteDynamicHead=nullptr;

    double paraCurve[3]={0};//拟合曲线参数
    double maxPose=0,maxPoseY=0;//预测极大值横坐标
    int voteLength=10;//单个票箱宽度（单位：μm）
    static int numBox=1;
    int i=0;
    Point3f vote3F;
    Curve_Fitting(curve,2,curveOutcome);//曲线拟合
    if(!curveOutcome.empty())
    {
        paraCurve[2]=curveOutcome.at<double>(0);//c
        paraCurve[1]=curveOutcome.at<double>(1);//b
        paraCurve[0]=curveOutcome.at<double>(2);//a
        emit drawCurve(&paraCurve[0]);

        maxPose=-paraCurve[1]*0.5/paraCurve[0]; //中轴线
        maxPoseY=paraCurve[0]*maxPose*maxPose+paraCurve[1]*maxPose+paraCurve[2];
        if(maxPose>=0&&maxPose<range&&paraCurve[0]<0)//超出范围不计票
        {
            pVoteDynamic=pVoteDynamicHead;

            if(pVoteDynamicHead==nullptr)//链表为空
            {
                pVoteDynamic= new voteDynamic;
                pVoteDynamic->votePoseHead=(int)(maxPose-voteLength/2);
                pVoteDynamic->votePoseTail=(int)(maxPose+voteLength/2);
                pVoteDynamic->voteNumber=1;
                pVoteDynamic->index=0;
                pVoteDynamicHead=pVoteDynamic;
            }
            else
            {
                for(;i<numBox;i++)//循环搜索匹配的票箱
                {
                    if(maxPose>=pVoteDynamic->votePoseHead&&maxPose<=pVoteDynamic->votePoseTail)
                    {
                        pVoteDynamic->voteNumber++;
                        break;
                    }else
                    {
                        if(i<(numBox-1))
                        {
                            pVoteDynamic=pVoteDynamic->pVote;
                        }
                    }
                }
                if(i==numBox)//没有匹配票箱，新建票箱
                {
                    pVoteDynamic->pVote= new voteDynamic;
                    pVoteDynamic->pVote->votePoseHead=(int)(maxPose-voteLength/2);
                    pVoteDynamic->pVote->votePoseTail=(int)(maxPose+voteLength/2);
                    pVoteDynamic->pVote->voteNumber=1;
                    pVoteDynamic->pVote->index=pVoteDynamic->index+1;
                    pVoteDynamic=pVoteDynamic->pVote;
                    numBox++;
                }
            }
          i=0;
          vote3F.x=pVoteDynamic->votePoseHead;
          vote3F.y=maxPose;
          vote3F.z=maxPoseY;
          voteMax.push_back(vote3F);
          //std::cout<<"maxPose: "<<maxPose<<" votePose: "<<pVoteDynamic->votePoseHead<<" a: "<<paraCurve[0]<<std::endl;
          if ( pVoteDynamic->voteNumber>=threshold)//设定投票成功阈值
          {
              emit voteSuccess(pVoteDynamic->votePoseHead);
              voteEnd=pVoteDynamic->votePoseHead;
              //std::cout<<"voteSuccess!!!"<<"climb"<<curve.at(3).x()<<std::endl;
              return 1;
          }


        }



    }

    return 0;


}



uint8_t algorithms::Line_Fitting(float * Point,uint8_t num,float * k,float * b)
{
    double x_mean = 0;
    double y_mean = 0;
    for(int i = 0; i < num; i++)
    {
        x_mean += *(Point+i*2);
        y_mean += *(Point+i*2+1);
    }
    x_mean /= num;
    y_mean /= num;

    double Dxx = 0, Dxy = 0, Dyy = 0;
    double A=0,B=0,C=0;

    for(int i = 0; i < num; i++)
    {
        Dxx += (*(Point+i*2) - x_mean) * (*(Point+i*2) - x_mean);
        Dxy += (*(Point+i*2) - x_mean) * (*(Point+i*2+1) - y_mean);
        Dyy += (*(Point+i*2+1) - y_mean) * (*(Point+i*2+1) - y_mean);
    }
    double lambda = ( (Dxx + Dyy) - sqrt( (Dxx - Dyy) * (Dxx - Dyy) + 4 * Dxy * Dxy) ) / 2.0;
    double den = sqrt( Dxy * Dxy + (lambda - Dxx) * (lambda - Dxx) );
    A = Dxy / den;
    B = (lambda - Dxx) / den;
    C = - A * x_mean - B * y_mean;
    *k=-A/B;
    *b=-C/B;
    return 0;

}


/*********************
Function:三角形阈值法源码
Abstract:
Input   :
Author  :
*********************/
double algorithms::sourceTriangleCompare( const Mat& _src ,double * max_num, int * max_index, double * lengthValue,double  threshold)
{
    Size size = _src.size();
    int step = (int) _src.step;
    if( _src.isContinuous() )
    {
        size.width *= size.height;
        size.height = 1;
        step = size.width;
    }

    const int N = 256;
    int i, j, h[N] = {0};
    #if CV_ENABLE_UNROLLED
    int h_unrolled[3][N] = {};
    #endif
    for( i = 0; i < size.height; i++ )
    {
        const uchar* src = _src.ptr() + step*i;
        j = 0;
        #if CV_ENABLE_UNROLLED
        for( ; j <= size.width - 4; j += 4 )
        {
            int v0 = src[j], v1 = src[j+1];
            h[v0]++; h_unrolled[0][v1]++;
            v0 = src[j+2]; v1 = src[j+3];
            h_unrolled[1][v0]++; h_unrolled[2][v1]++;
        }
        #endif
        for( ; j < size.width; j++ )
            h[src[j]]++;
    }

    int left_bound = 0, right_bound = 0, max_ind = 0, max = 0;
    int temp;
    bool isflipped = false;

    #if CV_ENABLE_UNROLLED
    for( i = 0; i < N; i++ )
    {
        h[i] += h_unrolled[0][i] + h_unrolled[1][i] + h_unrolled[2][i];
    }
    #endif

    for( i = 0; i < N; i++ )
    {
        if( h[i] > 0 )
        {
            left_bound = i;
            break;
        }
    }
    if( left_bound > 0 )
        left_bound--;

    for( i = N-1; i > 0; i-- )
    {
        if( h[i] > 0 )
        {
            right_bound = i;
            break;
        }
    }
    if( right_bound < N-1 )
        right_bound++;

    for( i = 0; i < N; i++ )
    {
        if( h[i] > max)
        {
            max = h[i];
            max_ind = i;
        }
    }

//     int right_sum=0;
//     int left_sum=0;
//    for( i = 0; i < N; i++ )
//    {
//        if( i<max_ind )
//        {
//           left_sum=left_sum+h[i];
//        }
//        else if(i>max_ind)
//        {
//           right_sum=right_sum+h[i];
//        }
//    }

//    if( max_ind-left_bound < right_bound-max_ind)
//        if( left_sum < right_sum)
    * max_index=max_ind;
     if( max_ind <= threshold)

    {
        isflipped = true;
        i = 0, j = N-1;
        while( i < j )
        {
            temp = h[i]; h[i] = h[j]; h[j] = temp;
            i++; j--;
        }
        left_bound = N-1-right_bound;
        max_ind = N-1-max_ind;
    }

    double thresh = left_bound;
    double a, b, dist = 0, tempdist;

    /*
     * We do not need to compute precise distance here. Distance is maximized, so some constants can
     * be omitted. This speeds up a computation a bit.
     */
    a = max; b = left_bound-max_ind;
    for( i = left_bound+1; i <= max_ind; i++ )
    {
        tempdist = a*i + b*h[i];
        if( tempdist > dist)
        {
            dist = tempdist;
            thresh = i;
        }
    }
    thresh--;

    if( isflipped )
        thresh = N-1-thresh;


    *lengthValue=dist;
    *max_num=max;
    return thresh;
}


void algorithms::unevenLightCompensate(Mat &image, int blockSize)
{
    if (image.channels() == 3) cvtColor(image, image, 7);
    double average = mean(image)[0];
    int rows_new = ceil(double(image.rows) / double(blockSize));
    int cols_new = ceil(double(image.cols) / double(blockSize));
    Mat blockImage;
    blockImage = Mat::zeros(rows_new, cols_new, CV_32FC1);
    for (int i = 0; i < rows_new; i++)
    {
        for (int j = 0; j < cols_new; j++)
        {
            int rowmin = i*blockSize;
            int rowmax = (i + 1)*blockSize;
            if (rowmax > image.rows) rowmax = image.rows;
            int colmin = j*blockSize;
            int colmax = (j + 1)*blockSize;
            if (colmax > image.cols) colmax = image.cols;
            Mat imageROI = image(Range(rowmin, rowmax), Range(colmin, colmax));
            double temaver = mean(imageROI)[0];
            blockImage.at<float>(i, j) = temaver;
        }
    }
    blockImage = blockImage - average;
    Mat blockImage2;
    resize(blockImage, blockImage2, image.size(), (0, 0), (0, 0), INTER_CUBIC);
    Mat image2;
    image.convertTo(image2, CV_32FC1);
    Mat dst = image2 - blockImage2;
    dst.convertTo(image, CV_8UC1);
}


void algorithms::sharpnessFunction(Mat fiboImg,Mat &outImg,int funSelect,double & y)
{
    Mat sobel_x,sobel_y;
    Mat OTSUImg,TRIAImg;
    Mat sharpImg;
    int Image_Rows=0;
    int Image_Cols=0;
    double Sharp_Row=0,Sharp_Col=0;
    Scalar tempVal;
    double fiboMean;
    double minValue, maxValue;
    y=0;

    /***细胞个数评价指标相关参数***/
    Mat elementClose=getStructuringElement(MORPH_ELLIPSE, Size(11,11));//对应20X，720x480图像
    Mat elementClose_test1=getStructuringElement(MORPH_ELLIPSE, Size(11,11));//对应40X图像
    Mat elementClose_test2=getStructuringElement(MORPH_ELLIPSE, Size(17,17));//对应40X图像
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    /**大津阈值法+三角去噪**/
    double lengthValue,max_H,thresholdTRI;
    int max_index,thref;

    switch (funSelect)
    {
    case 1:
        //Energy
        cvtColor(fiboImg,sharpImg,COLOR_BGR2GRAY);
        Image_Cols=sharpImg.cols-1;
        Image_Rows=sharpImg.rows-1;
        for(int i=0;i<Image_Rows;i++)
        {
            for(int j=0;j<Image_Cols;j++)
            {
                Sharp_Col=sharpImg.at<uchar>(i+1,j)-sharpImg.at<uchar>(i,j);
                Sharp_Row=sharpImg.at<uchar>(i,j+1)-sharpImg.at<uchar>(i,j);
                y+=Sharp_Col*Sharp_Col+Sharp_Row*Sharp_Row;
            }
        }
        break;
    case 2:
        //Tenengrad
        cvtColor(fiboImg,sharpImg,COLOR_BGR2GRAY);
        Image_Cols=sharpImg.cols;
        Image_Rows=sharpImg.rows;
        Sobel(sharpImg,sobel_x,CV_8U,1,0);
        Sobel(sharpImg,sobel_y,CV_8U,0,1);
        outImg=sobel_x;
        for(int i=0;i<Image_Rows;i++)
        {
            for(int j=0;j<Image_Cols;j++)
            {
                Sharp_Col=sobel_x.at<uchar>(i,j);
                Sharp_Row=sobel_y.at<uchar>(i,j);
                y+=Sharp_Col*Sharp_Col+Sharp_Row*Sharp_Row;
            }
        }
        break;
    case 3:
        //Brenner
        cvtColor(fiboImg,sharpImg,COLOR_BGR2GRAY);
        Image_Cols=sharpImg.cols-2;
        Image_Rows=sharpImg.rows;
        for(int i=0;i<Image_Rows;i++)
        {
            for(int j=0;j<Image_Cols;j++)
            {
                Sharp_Col=sharpImg.at<uchar>(i,j+2)-sharpImg.at<uchar>(i,j);
                y+=Sharp_Col*Sharp_Col;
            }
        }
        break;
    case 4:
        //variance
        cvtColor(fiboImg,sharpImg,COLOR_BGR2GRAY);
        tempVal=mean(sharpImg);
        fiboMean=tempVal.val[0];
        Image_Cols=sharpImg.cols;
        Image_Rows=sharpImg.rows;
        for(int i=0;i<Image_Rows;i++)
        {
            for(int j=0;j<Image_Cols;j++)
            {
                Sharp_Col=sharpImg.at<uchar>(i,j);
                y+=(Sharp_Col-fiboMean)*(Sharp_Col-fiboMean);
            }
        }
        y=y/(Image_Cols*Image_Rows);
        break;
    case 5:
        //normalized variance
        cvtColor(fiboImg,sharpImg,COLOR_BGR2GRAY);
        tempVal=mean(sharpImg);
        fiboMean=tempVal.val[0];
        Image_Cols=sharpImg.cols;
        Image_Rows=sharpImg.rows;
        for(int i=0;i<Image_Rows;i++)
        {
            for(int j=0;j<Image_Cols;j++)
            {
                Sharp_Col=sharpImg.at<uchar>(i,j);
                y+=(Sharp_Col-fiboMean)*(Sharp_Col-fiboMean);
            }
        }
        y=y/(Image_Cols*Image_Rows*fiboMean);
        break;
    case 6:
        //Number of cells
        cvtColor(fiboImg,sharpImg,COLOR_BGR2GRAY);
        GammaTransform(sharpImg,2);//γ变换
        GaussianBlur(sharpImg,sharpImg,Size(5,5),8);
        unevenLightCompensate(sharpImg, 200);//均匀光照
        threshold(sharpImg,sharpImg,0,255,CV_THRESH_TRIANGLE);//二值化
        morphologyEx(sharpImg,sharpImg,MORPH_CLOSE,elementClose);//连通
        sharpImg=~sharpImg;
        outImg=sharpImg.clone();
        findContours(sharpImg,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);//提取轮廓
        for(auto cIterator = contours.begin(); cIterator!=contours.end(); cIterator++)
        {
           if(cIterator->size()>0 && cIterator->size()<200)//过滤噪声，对应20X，720x480图像
            {
                y++;
            }
        }
        break;
    case 7:
        //Number of cells
        cvtColor(fiboImg,sharpImg,COLOR_BGR2GRAY);
//        GammaTransform(sharpImg,2);//γ变换
//        unevenLightCompensate(sharpImg, 125);//均匀光照
        GaussianBlur(sharpImg,sharpImg,Size(11,11),11);


        /**大津阈值法+三角去噪**/
        OTSUImg=sharpImg.clone();
        TRIAImg=sharpImg.clone();
        thref=threshold(OTSUImg,OTSUImg,0,255,CV_THRESH_OTSU)+2;//大津阈值法找阈值
        thresholdTRI=sourceTriangleCompare(TRIAImg,&max_H,&max_index,&lengthValue,thref);//基于大津阈值法的阈值判断左右
        if(thref>=max_index){
            threshold(sharpImg,sharpImg,thresholdTRI,255,CV_THRESH_BINARY);//当阈值大于峰值，使用三角法滤波
        }else{
            sharpImg=OTSUImg;
        }


        morphologyEx(sharpImg,sharpImg,MORPH_CLOSE,elementClose);//连通
        sharpImg=~sharpImg;
        outImg=sharpImg.clone();
        findContours(sharpImg,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);//提取轮廓
//        y=contours.size();
        for(auto cIterator = contours.begin(); cIterator!=contours.end(); cIterator++)
        {
           if(cIterator->size()>0 && cIterator->size()<200)//过滤噪声，对应20X，720x480图像
            {
                y++;
            }
        }
        break;
    case 8://只用三角阈值
        cvtColor(fiboImg,sharpImg,COLOR_BGR2GRAY);
//        unevenLightCompensate(sharpImg, 125);//均匀光照
        GaussianBlur(sharpImg,sharpImg,Size(11,11),11);
        threshold(sharpImg,sharpImg,0,255,CV_THRESH_TRIANGLE);//当阈值大于峰值，使用三角法滤波
        morphologyEx(sharpImg,sharpImg,MORPH_CLOSE,elementClose);//连通
        sharpImg=~sharpImg;
        outImg=sharpImg.clone();
        findContours(sharpImg,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);//提取轮廓
        for(auto cIterator = contours.begin(); cIterator!=contours.end(); cIterator++)
        {
           if(cIterator->size()>0 && cIterator->size()<200)//过滤噪声，对应20X，720x480图像
            {
                y++;
            }
        }
        break;
    case 9://只用大津阈值法
        cvtColor(fiboImg,sharpImg,COLOR_BGR2GRAY);
//        unevenLightCompensate(sharpImg, 125);//均匀光照
        GaussianBlur(sharpImg,sharpImg,Size(11,11),11);
        threshold(sharpImg,sharpImg,0,255,CV_THRESH_OTSU);//当阈值大于峰值，使用三角法滤波
        morphologyEx(sharpImg,sharpImg,MORPH_CLOSE,elementClose);//连通
        sharpImg=~sharpImg;        
        outImg=sharpImg.clone();
        findContours(sharpImg,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);//提取轮廓
        for(auto cIterator = contours.begin(); cIterator!=contours.end(); cIterator++)
        {
           if(cIterator->size()>0 && cIterator->size()<200)//过滤噪声，对应20X，720x480图像
            {
                y++;
            }
        }
        break;
//    case 10:
//        //测试版本
//        cvtColor(fiboImg,sharpImg,COLOR_BGR2GRAY);
////        GammaTransform(sharpImg,2);//γ变换
//        unevenLightCompensate(sharpImg, 125);//均匀光照
////        GaussianBlur(sharpImg,sharpImg,Size(27,27),17);
//        GaussianBlur(sharpImg,sharpImg,Size(11,11),11);
//        threshold(sharpImg,sharpImg,0,255,CV_THRESH_TRIANGLE);//当阈值大于峰值，使用三角法滤波
//        morphologyEx(sharpImg,sharpImg,MORPH_DILATE,elementClose_test1);//连通
//        morphologyEx(sharpImg,sharpImg,MORPH_DILATE,elementClose_test2);//连通
////        morphologyEx(sharpImg,sharpImg,MORPH_ERODE,elementClose_test);//连通
//        sharpImg=~sharpImg;
//        outImg=sharpImg.clone();
//        findContours(sharpImg,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);//提取轮廓
////        y=contours.size();
//        for(auto cIterator = contours.begin(); cIterator!=contours.end(); cIterator++)
//        {
//           if(cIterator->size()>0 && cIterator->size()<200)//过滤噪声，对应20X，720x480图像
//            {
//                y++;
//            }
//        }
//        break;
    case 10:
        //测试版本
        cvtColor(fiboImg,sharpImg,COLOR_BGR2GRAY);
//        GammaTransform(sharpImg,2);//γ变换
        unevenLightCompensate(sharpImg, 125);//均匀光照
//        GaussianBlur(sharpImg,sharpImg,Size(27,27),17);
        GaussianBlur(sharpImg,sharpImg,Size(11,11),11);
        threshold(sharpImg,sharpImg,0,255,CV_THRESH_OTSU);//当阈值大于峰值，使用三角法滤波c
        sharpImg=~sharpImg;

//        imwrite("1threshold.jpg",sharpImg);
//        distanceTransform(sharpImg,sharpImg,CV_DIST_L2,3);
//        minMaxLoc(sharpImg, &minValue, &maxValue);
//        threshold(sharpImg,sharpImg,maxValue*0.35,255,CV_THRESH_BINARY);
//        sharpImg.convertTo(sharpImg,CV_8UC1);
//        imwrite("2distanceTransform.jpg",sharpImg);
        morphologyEx(sharpImg,sharpImg,MORPH_ERODE,elementClose_test1);//连通
        morphologyEx(sharpImg,sharpImg,MORPH_ERODE,elementClose_test2);//连通
        outImg=sharpImg.clone();
        findContours(sharpImg,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);//提取轮廓
//        y=contours.size();
        for(auto cIterator = contours.begin(); cIterator!=contours.end(); cIterator++)
        {
           if(cIterator->size()>0 && cIterator->size()<200)//过滤噪声，对应20X，720x480图像
            {
                y++;
            }
        }
        break;
    case 11:
        //用于检测荧光图像
        cvtColor(fiboImg,sharpImg,COLOR_BGR2GRAY);
//        GammaTransform(sharpImg,2);//γ变换
//        unevenLightCompensate(sharpImg, 125);//均匀光照
//        GaussianBlur(sharpImg,sharpImg,Size(11,11),11);
        threshold(sharpImg,sharpImg,0,255,CV_THRESH_OTSU);//当阈值大于峰值，使用三角法滤波
        distanceTransform(sharpImg,sharpImg,CV_DIST_L2,3);
        minMaxLoc(sharpImg, &minValue, &maxValue);
        threshold(sharpImg,sharpImg,maxValue*0.5,255,CV_THRESH_BINARY);
        sharpImg.convertTo(sharpImg,CV_8UC1);
//        morphologyEx(sharpImg,sharpImg,MORPH_ERODE,elementClose_test1);//连通
//        morphologyEx(sharpImg,sharpImg,MORPH_ERODE,elementClose_test2);//连通
//        sharpImg=~sharpImg;
//        morphologyEx(sharpImg,sharpImg,MORPH_ERODE,elementClose_test);//连通
        outImg=sharpImg.clone();
        findContours(sharpImg,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);//提取轮廓
//        y=contours.size();
        for(auto cIterator = contours.begin(); cIterator!=contours.end(); cIterator++)
        {
           if(cIterator->size()>0 && cIterator->size()<200)//过滤噪声，对应20X，720x480图像
            {
                y++;
            }
        }
        break;
    default: break;
    }

}

void  algorithms::GammaTransform(cv::Mat &image, double gamma)
{

    Mat imageGamma;	//灰度归一化
    Mat dist(image.size(),CV_64F);
    image.convertTo(imageGamma, CV_64F, 1.0 / 255, 0); 	//伽马变换
    pow(imageGamma, gamma, dist);//dist 要与imageGamma有相同的数据类型
    dist.convertTo(dist, CV_8U, 255, 0);
    image=dist;

}
