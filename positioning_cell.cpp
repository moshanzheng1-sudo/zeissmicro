#if _MSC_VER >= 1600
#pragma execution_character_set("utf-8")

#endif
#include "positioning_cell.h"


Auto_Focus::Auto_Focus( SBaslerCameraControl *Image_Control, params_struct &params)
{
    ImgControl=Image_Control;
    cellParamsFirst=params;
}

Auto_Focus::~Auto_Focus()
{

}


void Auto_Focus::showImgUI(Mat imageInput)
{
    Mat image=imageInput.clone();
    cvtColor(image, image, CV_BGR2RGB);
    QImage img = QImage((const unsigned char*)(image.data),image.cols,image.rows, QImage::Format_RGB888);
    emit sendImage(img);
}




/*********************
Function:获取坐标旋转矩阵
Abstract:基于SVD，三个对应点获取旋转矩阵
Input   :需转换坐标系点，转换后坐标系点
Author  :胡伟康
*********************/
cv::Mat Get3DR_TransMatrix(const std::vector<Point3d>& srcPoints, const std::vector<Point3d>&  dstPoints)
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




//生成高斯噪声
double generateGaussianNoise(double mu, double sigma)
{
    //定义小值
    const double epsilon = numeric_limits<double>::min();
    static double z0, z1;
    static bool flag = false;
    flag = !flag;
    //flag为假构造高斯随机变量X
    if (!flag)
        return z1 * sigma + mu;
    double u1, u2;
    //构造随机变量
    do
    {
        u1 = rand() * (1.0 / RAND_MAX);
        u2 = rand() * (1.0 / RAND_MAX);
    } while (u1 <= epsilon);
    //flag为真构造高斯随机变量
    z0 = sqrt(-2.0*log(u1))*cos(2 * CV_PI*u2);
    z1 = sqrt(-2.0*log(u1))*sin(2 * CV_PI*u2);
    return z0*sigma + mu;
}

//为图像添加高斯噪声
Mat addGaussianNoise(Mat &srcImag)
{
    Mat dstImage = srcImag.clone();
    int channels = dstImage.channels();
    int rowsNumber = dstImage.rows;
    int colsNumber = dstImage.cols*channels;
    //判断图像的连续性
    if (dstImage.isContinuous())
    {
        colsNumber *= rowsNumber;
        rowsNumber = 1;
    }
    for (int i = 0; i < rowsNumber; i++)
    {
        for (int j = 0; j < colsNumber; j++)
        {
            //添加高斯噪声
            int val = dstImage.ptr<uchar>(i)[j] +
                generateGaussianNoise(0, 5)*5 ;
            if (val < 0)
                val = 0;
            if (val>255)
                val = 255;
            dstImage.ptr<uchar>(i)[j] = (uchar)val;
        }
    }
    return dstImage;
}


static int  sizeTable[] = { 0, 9, 99, 999, 9999, 99999, 999999, 9999999,99999999, 999999999};

// Requires positive x
static int intSize(int x) {
    for (int i=1; ; i++)
        if (x <= sizeTable[i])
            return sizeTable[i-1]+1;
}

/*清晰度评价函数选择、步距、起始位置、范围、投票阈值*/
int8_t Auto_Focus::coarseAdjust(int functionSlect,int stepS, int beginPose,int range,int threshold,int fitNumber)
{
    using namespace cv;

    /***Test of single_slot***/
    double y1,y2,y3,y4,y8,y9;
    static double x=0.0,y=0.0,yClear=0.0;
    static int i_count=0;
    static FileStorage fs("Auto_Focus.yaml",FileStorage::WRITE);
    static VideoCapture injectionVideo(autofocusVideo);
    static VideoWriter videoCreate("cellAutofocusing.avi", CV_FOURCC('D', 'I', 'V', 'X'), 5, Size(2048,542),false);

    int direction=-1;
    static int8_t clearFlag=0,positionFlag=0;
    static float yClearMax=0;
    queue<QPointF> curve;


    char * filename=new char[100];
    char * text=new char[100];
    Mat readImg,transitImg,calImg;
    Mat showImg;
    Mat outImg,grayImg;
    vector<Mat> vImg_1;


    double Time_Focus=getTickCount();
    /***摄像头方式***/    
      if(cameraOpen)
      {
        while(ImgControl->GrabImage(readImg,5)!=0)
        {
//            std::cout << "Grab image failed " << std::endl;
        }
        if(i_count*stepS>=range)//结束
        {
            videoCreate.release();
            injectionVideo.release();
            fs.release();
            this->send(1);//观察关系
            return state_idle;
        }
      }
      else
      {
          /***视频方式***/
//          injectionVideo>>readImg;
//          if (readImg.empty())//视频结束
//          {
//              videoCreate.release();
//              injectionVideo.release();
//              fs.release();
//              this->send(1);//观察关系
//              return state_idle;
//          }

//          if(i_count%10!=0)//跳帧读取
//          {
//              return state_coarseAdjust;
//          }
          /***图片方式***/
//         sprintf(filename,imgPath,i_count*stepSize+beginPose);
//          readImg=imread(filename);
          if (i_count*5>300)//视频结束
          {
              videoCreate.release();
              this->send(1);//观察关系
              return state_idle;
          }
          sprintf(filename,imgCell,i_count*5-150);
          readImg=imread(filename);

      }
//      int pathWidth=readImg.size().width*(1-shrinkRate);
//      int pathHeight=readImg.size().height*(1-shrinkRate);
//      transitImg=readImg(Rect(readImg.size().width/2-pathWidth/2,readImg.size().height/2-pathHeight/2,pathWidth,pathHeight));//去除视频的黑边,缩小路径规划区域

     resize(readImg,transitImg,Size(1024,542));
//      transitImg=transitImg(Rect(0.4*transitImg.size().width,0,0.6*transitImg.size().width,transitImg.size().height));//去掉黑斑

      cvtColor(transitImg,grayImg,COLOR_BGR2GRAY);

      if(actOpen&&i_count==0){
          cellParamsFirst.dev=2;//设置控制对象（细胞）
          Ump_Select_Dev(&cellParamsFirst);
          Ump_Read_Position(&cellParamsFirst);
      }


        /***评价获取***/
//      sharpnessFunction(transitImg,outImg,1,y1);//Energy
//      sharpnessFunction(transitImg,outImg,2,y2);//Tenengrad
//      sharpnessFunction(transitImg,outImg,3,y3);//Brenner
//      sharpnessFunction(transitImg,outImg,4,y4);//variance
//      sharpnessFunction(transitImg,outImg,8,y8);//triangle
//      sharpnessFunction(transitImg,outImg,9,y9);//OTSU
//      sharpnessFunction(transitImg,outImg,functionSlect,yClear);
      sharpnessFunction(transitImg,outImg,functionSlect,y);


      resize(grayImg,grayImg,Size(1024,542));
      resize(outImg,outImg,Size(1024,542));
        vImg_1.push_back(grayImg);
        vImg_1.push_back(outImg);
        hconcat(vImg_1,showImg);

        if(yClear>yClearMax){
            yClearMax=yClear;
        }else{
            clearFlag=1;
            cellPosition=cellParamsFirst;
            cellPosition.home_z=cellParamsFirst.home_z+direction*(i_count)*stepS*1000;//将清晰位置保留下来
        }


        Array_Sharpness[len_rows][0]=abs(x);//Z轴位置
        Array_Sharpness_Save[len_rows]=y;
        if(y>maxSimilarity.y)
        {
            maxSimilarity.y=y;
            maxSimilarity.x=x;
        }
        for (int i=0;i<len_rows;i++)
        {
            Array_Sharpness[i][1]=Array_Sharpness_Save[i]/maxSimilarity.y;//归一化
            if(i>=(len_rows-fitNumber))
            {
               curve.push(QPointF((double)Array_Sharpness[i][0],(double)Array_Sharpness[i][1]));
            }//保留末尾fitNumber个点
        }
        if(curve.size()>=fitNumber)
        {
            if(cvtAdjust(curve,threshold,range,voteEnd_,voteMax_))
            {
                curve.~queue();
                Time_Focus=((double)getTickCount()-Time_Focus)/getTickFrequency()*1000;
//                std::cout<<x<<","<<y<<",Time_Focus:"<<Time_Focus<<"ms"<<std::endl;
                int maxPoseAverage=0,sizePose=0;
                for(int i = 0;i<voteMax_.size();i++)
                {
                    if(voteEnd_==voteMax_[i].x)
                    {
                        maxPoseAverage+=voteMax_[i].y;
                        sizePose++;
                    }
                }
                maxPoseAverage/=sizePose;//求平均
                if(actOpen){
                    cellParamsFirst.target_d=cellParamsFirst.home_d;
                    cellParamsFirst.target_y=cellParamsFirst.home_y;
                    cellParamsFirst.target_z=cellParamsFirst.home_z-(int)maxPoseAverage*1000;
                    cellParamsFirst.target_x=cellParamsFirst.home_x;
                    cellParamsFirst.speed=1000;
                    Ump_Goto_Position(&cellParamsFirst);
                    vaguePosition=cellParamsFirst;//将模糊位置保留下来
                }
                maxP=maxPoseAverage;
                /***结束***/
    //            sprintf(text,"x:%f,y:%f,yClear:%f",x,y,yClear);
    //            putText(showImg,text,cvPoint(50,50),FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255),2);
    //            videoCreate<<showImg;
                cout<<"cell autofocused"<<endl;
                videoCreate.release();
                injectionVideo.release();
                fs.release();
    //            this->send(1);//观察关系
                return state_idle;
            }
        }//动态曲线拟合估计极大值位置
        //移动到下一点
        if(actOpen){
            cellParamsFirst.target_d=cellParamsFirst.home_d;
            cellParamsFirst.target_y=cellParamsFirst.home_y;
            cellParamsFirst.target_z=cellParamsFirst.home_z+direction*(i_count+1)*stepS*1000;
            cellParamsFirst.target_x=cellParamsFirst.home_x;
            cellParamsFirst.speed=500;
            Ump_Goto_Position(&cellParamsFirst);
        }
        sprintf(text,"x:%f,y:%f,yClear:%f",x,y,yClear);
        putText(showImg,text,cvPoint(50,50),FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255),2);
        videoCreate<<showImg;
        std::cout<<x<<","<<y<<","<<y1/1000000<<","<<y2/1000000<<","<<y3/1000000<<","<<y4<<","<<y8<<","<<y9<<std::endl;//绘图
        i_count++;
        len_rows++;
        x=x+direction*stepS;
        return state_coarseAdjust;
}



const int FIBOSIZE = 20;
void fibonacci(vector<int>& F)
{
    F.push_back(1);
    F.push_back(2);
    for(int i=2;i<FIBOSIZE;i++)
    {
        F.push_back(F[i-1]+F[i-2]);
    }
}







void Auto_Focus::fineGetSharpness(int funSelect,int xPose, double &y)
{
//    /***驱动微操作器移动到指定位置***/
    Mat fiboImg,outImg;
    char * filename=new char[100];
    static int xPose_last=0;
    static int distance=abs(-200-xPose);
    static int ySize;

////    emit goAndGetImg(xPose,fiboImg);

//    while(ImgControl->GrabImage(fiboImg,5)!=0)
//    {
//        std::cout << "Grab image failed " << std::endl;
//    }
//    /***获取图片并计算清晰度值***/
//    while(!fiboImg.empty());

    if(xPose_last!=0)
    {

      distance=distance+abs(xPose_last-xPose);//累计路程
    }
    else
    {
        sprintf(filename,imgPath,-180);
        fiboImg=imread(filename);
        resize(fiboImg,fiboImg,Size(720,480));
        sharpnessFunction(fiboImg,outImg,funSelect,y);
        ySize=intSize(y);
    }


    sprintf(filename,imgPath,xPose);
    fiboImg=imread(filename);
    resize(fiboImg,fiboImg,Size(720,480));
    sharpnessFunction(fiboImg,outImg,funSelect,y);

    xPose_last=xPose;
    y=y/ySize;
//    y=y/ySize+generateGaussianNoise(0,1);
    cout<<"pose:"<<xPose<<",distance:"<<distance<<endl;


}


int8_t Auto_Focus::fineAdjust(int searchSelect)
{
     /***方法一:直接求平均***/
    double maxPoseAverage=0;
    int maxPoseInt=0;
    int sizePose=0;
    /***方法二：斐波那契搜索***/
    double xPose_1=-180,xPose_2=220;
    double xPose_1_last=xPose_1,xPose_2_last=xPose_2;
    double aSharpness=0,bSharpness=0;
    double r_k=0;
    vector<int> fiboVector;
    double Time_Focus=0;

    switch (searchSelect)
    {
    case 1:
        /***方法一:直接求平均***/
        for(int i = 0;i<voteMax_.size();i++)
        {
            if(voteEnd_==voteMax_[i].x)
            {
                maxPoseAverage+=voteMax_[i].y;
                sizePose++;
            }
        }
        maxPoseAverage/=sizePose;//求平均
        if(actOpen){
            cellParamsFirst.target_d=cellParamsFirst.home_d;
            cellParamsFirst.target_y=cellParamsFirst.home_y;
            cellParamsFirst.target_z=cellParamsFirst.home_z-(int)maxPoseAverage*1000;
            cellParamsFirst.target_x=cellParamsFirst.home_x;
            cellParamsFirst.speed=1000;
            Ump_Goto_Position(&cellParamsFirst);
            vaguePosition=cellParamsFirst;//将模糊位置保留下来
        }
        maxP=maxPoseAverage;
//        maxP=maxSimilarity.x;
//        maxP=220;
        cout<<"maxPoseAverage:"<<maxP<<endl;
        break;
    case 2:
        /***方法二：斐波那契搜索***/
        Time_Focus=getTickCount();
        fibonacci(fiboVector);
        for(int i=11;i>1;i--)
        {
            r_k=(float)fiboVector[i-2]/fiboVector[i];
            if(i==11)
            {
                xPose_1=xPose_1_last+r_k*(xPose_2_last-xPose_1_last);
                xPose_2=xPose_2_last-r_k*(xPose_2_last-xPose_1_last);
                fineGetSharpness(3,(int)xPose_1,aSharpness);
                fineGetSharpness(3,(int)xPose_2,bSharpness);
            }
            if(aSharpness<bSharpness)
            {
               xPose_1_last=xPose_1;
               xPose_1=xPose_2;
               aSharpness=bSharpness;
               xPose_2=xPose_2_last-r_k*(xPose_2_last-xPose_1_last);
               fineGetSharpness(3,(int)xPose_2,bSharpness);
            }
            else
            {
                xPose_2_last=xPose_2;
                xPose_2=xPose_1;
                bSharpness=aSharpness;
                xPose_1=xPose_1_last+r_k*(xPose_2_last-xPose_1_last);
                fineGetSharpness(3,(int)xPose_1,aSharpness);
            }
        }
        maxPoseAverage=(xPose_1+xPose_2)/2;
        Time_Focus=((double)getTickCount()-Time_Focus)/getTickFrequency()*1000;
        cout<<"maxPoseAverage:"<<maxPoseAverage<<"Time_Focus:"<<Time_Focus<<"ms"<<endl;
        break;
    case 3:
        /***方法三：曲线拟合***/
    //    memset(voteBox,0,sizeof(voteBox));
    //    memset(Array_Sharpness,0,sizeof(Array_Sharpness_Save));
    //    memset(Array_Sharpness_Save,0,sizeof(Array_Sharpness_Save));
    //    voteMax.clear();
    //    sizePose=0;
    //    maxPoseInt=(int)maxPoseAverage-15-250;
    //    maxPoseInt=maxPoseInt-maxPoseInt%5;
    //    coarseAdjust(2,5,maxPoseInt,30,2);
    //    maxPoseAverage=0;
    //    for(int i = 0;i<voteMax.size();i++)
    //    {
    //        if(voteEnd==voteMax[i].x)
    //        {
    //            maxPoseAverage+=voteMax[i].y;
    //            sizePose++;
    //        }
    //    }
    //    maxPoseAverage/=sizePose;//求平均得最终得聚焦平面
        break;
    default:
        break;

    }
    return state_idle;


}


/*********************
Function:三角形阈值法源码
Abstract:
Input   :
Author  :
*********************/
double sourceTriangle( const Mat& _src ,int * left_num,double * max_num, int * max_index, double * lengthValue)
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

    for( i = 0; i < max_ind; i++ )
    {
        if( h[i] > max)
        {
            max = h[i];
            max_ind = i;
        }
    }

    if( max_ind-left_bound < right_bound-max_ind)
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

    * max_index=max_ind;
    *lengthValue=dist;
    *left_num=left_bound;
    *max_num=max;
    return thresh;
}


#define cvQueryHistValue_1D( hist, idx0 ) \
    ((float)cvGetReal1D( (hist)->bins, (idx0)))



int8_t Auto_Focus::cellAutofocus()
{
    static int i_count=0,poseCount=0;
    int hist_size = 256;    //直方图尺寸
    int histgramLen[200]={0};
    float range[] = {0,255};  //灰度级的范围
    float* ranges[]={range};
    char * text=new char[100];
    char * filename= new char[100];
    Mat imageRead,hisImg;
    Mat mImg,transferImg,test_img,roiImg1,roiImg2;
    Mat finalImg=Mat(cv::Size(1024,542),CV_8U,Scalar(0,0,0));
    Mat histImg=Mat(cv::Size(1024,542),CV_8U,Scalar(255,255,255));
    Mat criteriaImg=cv::Mat::zeros(cv::Size(1024,542),CV_8U);
    static VideoCapture injectionVideo(autofocusVideo);
    static VideoWriter videoCreate("test4.avi", CV_FOURCC('D', 'I', 'V', 'X'), 5, Size(2048,1084),false);
    double Time_Focus=getTickCount();
    /**需要匹配的图像输入**/
    /***摄像头方式***/
    if(cameraOpen)
    {
        while(ImgControl->GrabImage(imageRead,5)!=0)
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
//            this->send(1);//观察关系
//            return -1;
//        }
//        if(i_count%10!=0)//跳帧读取
//        {
//            i_count++;
//            return 4;
//        }
        /***图片方式***/
        if (i_count*5>300)//视频结束
        {
            videoCreate.release();
            this->send(1);//观察关系
            return state_pathPlaning;
        }
        sprintf(filename,imgCell,i_count*5-150);//60
//        sprintf(filename,imgCell,60);//60
        imageRead=imread(filename);

    }
    int pathWidth=imageRead.size().width*(1-shrinkRate);
    int pathHeight=imageRead.size().height*(1-shrinkRate);
    mImg=imageRead(Rect(imageRead.size().width/2-pathWidth/2,imageRead.size().height/2-pathHeight/2,pathWidth,pathHeight));//去除视频的黑边,缩小路径规划区域



    Mat tImg;
    resize(mImg,mImg,Size(1024,542));
    cvtColor(mImg,tImg,COLOR_BGR2GRAY);
//    test_img=tImg(Rect(0.4*tImg.size().width,0,0.6*tImg.size().width,tImg.size().height));//去掉黑斑
    /*离焦对比度-距离曲线-用20X1_2图集*/
//    int sumVal1=0,sumVal2=0;
//    roiImg1=test_img(Rect(0,480,15,15));//background
//    roiImg2=test_img(Rect(320,1000,15,15));//cell
//    for(int i=0;i<15;i++)
//    {
//        for(int j=0;j<15;j++)
//        {
//            sumVal1+=roiImg1.at<uchar>(i,j);
//            sumVal2+=roiImg2.at<uchar>(i,j);
//        }
//    }
//    float valDiff=(-sumVal1+sumVal2)/(15*15);
//    cout<<valDiff<<endl;
//    imwrite("1.bmp",roiImg1);
//    imwrite("2.bmp",roiImg2);



//    /**直方图计算**/
//    CvHistogram* gray_hist = cvCreateHist(1,&hist_size,CV_HIST_ARRAY,ranges,1);
//    //计算灰度图像的一维直方图
//    IplImage temp = (IplImage)test_img;
//    IplImage *src = &temp;
//    cvCalcHist(&src,gray_hist,0,0);
//    //归一化直方图
//    cvNormalizeHist(gray_hist,1.0);

//    //绘制直方图
//    double val =0;
//    float bin_w=histImg.size().width/256;
//    for (int i = 0; i <255; i++)
//    {
//        val = 5*(cvQueryHistValue_1D(gray_hist,i)*histImg.size().height);//获取矩阵元素值，并转换为对应高度
//        cv::rectangle(histImg, cvPoint(i*bin_w, histImg.size().height),cvPoint((i + 1)*bin_w, (int)(histImg.size().height - val)),Scalar(255,255,0), 1, 8, 0);
//    }
//     imwrite("cellA2.bmp",histImg);
//    imshow("cellA2.bmp",histImg);
//    waitKey();


     /**类间方差计算**/
//    int thre = 0;
//    double delta = 0;
//    double U_t = 0;
//    for(int m=0; m<256; m++)
//    {
//        U_t += cvQueryHistValue_1D(gray_hist,m)*m;
//    }
//    double u = 0, w = 0;
//    for(int k=0; k<256; k++)
//    {

//        u += cvQueryHistValue_1D(gray_hist,k)*k;   //
//        w += cvQueryHistValue_1D(gray_hist,k);      //灰度大于阈值k的像素的概率

//        double t = U_t * w - u;
//        double delta_tmp = t * t / (w * (1 - w) );

//        if(delta_tmp > delta)
//        {
//            delta = delta_tmp;
//            thre = k;
//        }
//    }




    /**图像分割效果观察**/
    int contoursNum=0;
    static int contoursMax=0;
    double thref=0;
    Mat elementClose=getStructuringElement(MORPH_ELLIPSE, Size(11,11));
    Mat elementBlack=getStructuringElement(MORPH_ELLIPSE, Size(40,40));
//    Mat resultImg = Mat::zeros(test_img.size(), CV_8UC1);
    Mat contoursImg;


    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    Mat imgGamma,imgEven;
    /*适用于NIH-3T3的参数uneven:50, Gamma:无*/
    //越大越均匀，特征越弱，越小模糊效果越强
    //NIH-3T3必须要有光照均匀处理，否则右上角有干扰
//    unevenLightCompensate(test_img, 125);
//    GammaTransform(test_img,2);
    imwrite("cell_1gray.bmp",test_img);
    /*适用于Huvec的参数Size(11,11),11*/
    /*适用于NIH-3T3的参数Size(23,23),13*/
    //尺寸越小则连通区域小，碎块明显，噪声明显；大则碎块少，太大则过分连通
    GaussianBlur(test_img,test_img,Size(11,11),11);
    imwrite("cell_2gaussian.bmp",test_img);
//    test_img.copyTo(imgGamma);
//    morphologyEx(test_img,test_img,MORPH_BLACKHAT,elementBlack);//光照和对比度差问题
//    unevenLightCompensate(test_img, 200);

    test_img.copyTo(imgEven);
    hisImg=test_img.clone();
    Mat OTSUImg=test_img.clone();
    Mat TRIAImg=test_img.clone();

    /**大津阈值法+三角去噪**/
    double lengthValue,max_H,thresholdTRI;
    int max_index;
    thref=threshold(OTSUImg,OTSUImg,0,255,CV_THRESH_OTSU)+2;//大津阈值法找阈值(Huvec:2,NIH-3T3:2)
    thresholdTRI=sourceTriangleCompare(TRIAImg,&max_H,&max_index,&lengthValue,thref);//基于大津阈值法的阈值判断左右
    if(thref>=max_index){
        threshold(test_img,test_img,thresholdTRI,255,CV_THRESH_BINARY);//当阈值大于峰值，使用三角法滤波
    }else{
        test_img=OTSUImg;
    }
    imwrite("cell_3threshold.bmp",test_img);
    /**三角阈值法+大津三角去噪**/
//    double lengthValue,max_H,thresholdTRI;
//    int max_index;
//    thref=threshold(OTSUImg,OTSUImg,0,255,CV_THRESH_OTSU );//大津阈值法找阈值
//    thresholdTRI=sourceTriangleCompare(TRIAImg,&max_H,&max_index,&lengthValue,thref+3);//基于大津阈值法的阈值判断左右
//    threshold(test_img,test_img,thresholdTRI,255,CV_THRESH_BINARY);//二值化


//    thref=threshold(test_img,test_img,0,255,CV_THRESH_TRIANGLE);
//    imwrite("1triangle.bmp",test_img);
    morphologyEx(test_img,test_img,MORPH_CLOSE,elementClose);//连通
    imwrite("cell_4morphology.bmp",test_img);
    test_img=~test_img;
    imwrite("cell_5_test_img.bmp",test_img);
    test_img.copyTo(contoursImg);
    findContours(contoursImg,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);//提取轮廓


    for(auto cIterator = contours.begin(); cIterator!=contours.end(); cIterator++)
    {

        int sizeL;
       if(cIterator->size()>0 && cIterator->size()<200)//过滤噪声
        {
           drawContours(finalImg, contours, contoursNum, Scalar(255), 1, 8, hierarchy);//绘制轮廓
           sizeL=cIterator->size()/10;
           histgramLen[sizeL]++;
//           contoursNum=contoursNum+(int)cIterator->size();
           contoursNum++;
        }
    }
    if(contoursMax<contoursNum)
    {
        contoursMax=contoursNum;
        maxP=i_count*5;
    }
    imwrite("cell_6finalImg.bmp",finalImg);

    /**直方图计算**/
    CvHistogram* gray_hist = cvCreateHist(1,&hist_size,CV_HIST_ARRAY,ranges,1);
    //计算灰度图像的一维直方图
    IplImage temp = (IplImage)hisImg;
    IplImage *src = &temp;
    cvCalcHist(&src,gray_hist,0,nullptr);
    //归一化直方图
    cvNormalizeHist(gray_hist,1.0);
    //绘制直方图
    double val =0;
    float bin_w=histImg.size().width/256;
    for (int i = 0; i <255; i++)
    {
        val = 3*(cvQueryHistValue_1D(gray_hist,i)*histImg.size().height);//获取矩阵元素值，并转换为对应高度
        if(i>thref){
            if(val!=0)
            {
                cv::rectangle(histImg, cvPoint(i*bin_w, histImg.size().height),cvPoint((i + 1)*bin_w, (int)(histImg.size().height - val)),Scalar(0,0,0), 2, 8, 0);
            }
        }else {
            if(val!=0)
            {
            cv::rectangle(histImg, cvPoint(i*bin_w, histImg.size().height),cvPoint((i + 1)*bin_w, (int)(histImg.size().height - val)),Scalar(200,200,125), 2, 8, 0);
            }
        }
    }

//    imwrite("cellA3.bmp",test_img);
//    imshow("2",test_img);
//    waitKey(10);


    /**曲线关系观察**/
    Array_Sharpness[poseCount][1]=contoursNum;//代表Z轴的上下移动
//        Array_Sharpness[poseCount][1]=valDiff;//灰度差
    Array_Sharpness[poseCount][0]=i_count*5;//类间方差
//    for (int i = 0; i <poseCount; i++)
//    {
//        val = Array_Sharpness[i][1]*criteriaImg.size().height/150;//获取矩阵元素值，并转换为对应高度
//        cv::rectangle(criteriaImg,
//                      cvPoint(i*criteriaImg.size().width/poseCount, criteriaImg.size().height),
//                      cvPoint((i + 1)*criteriaImg.size().width/poseCount,(int)(criteriaImg.size().height - val)),
//                      Scalar(255,255,0), 1, 8, 0);
//    }
    poseCount++;
    /**第4张图片标题**/
    sprintf(text,"The number of cells:%d",contoursNum);
    putText(criteriaImg,text,cvPoint(50,50),FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255),2);
    /**第3张图片标题**/
    sprintf(text,"Gray histogram; Frame:%d",i_count);
    putText(histImg,text,cvPoint(50,50),FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255),2);
    len_rows=poseCount;
    i_count++;
//    if(i_count>630){
//        i_count++;
//        i_count--;
//    }



   cvReleaseHist(&gray_hist);
   /**四图融合展示**/
   resize(imgEven,imgEven,Size(1024,542));
   resize(test_img,test_img,Size(1024,542));
   resize(histImg,histImg,Size(1024,542));
   resize(criteriaImg,criteriaImg,Size(1024,542));
   vector <Mat> vImg_1;
   vector <Mat> vImg_2;
   vImg_1.push_back(imgEven);
   vImg_1.push_back(test_img);
   vImg_2.push_back(histImg);
   vImg_2.push_back(criteriaImg);
   Mat outImg_1,outImg_2;
   hconcat(vImg_1,outImg_1);
   hconcat(vImg_2,outImg_2);
   vImg_1.clear();
   vImg_1.push_back(outImg_1);
   vImg_1.push_back(outImg_2);
   vconcat(vImg_1,outImg_1);
   videoCreate<<outImg_1;
   Time_Focus=((double)getTickCount()-Time_Focus)/getTickFrequency()*1000;
//   for(int i=0;i<20;i++)
//   {
//       cout<<histgramLen[i]<<" ";
//   }
//   cout<<endl;
   cout <<"Time_Focus:"<<Time_Focus<<" ,i_count:"<<i_count<< std::endl;
   return state_cellAutofocus;

}

int8_t Auto_Focus::cellDetection()
{
    static int i_count=0,poseCount=0;
    int hist_size = 256;    //直方图尺寸
    int histgramLen[200]={0};
    float range[] = {0,255};  //灰度级的范围
    float* ranges[]={range};
    char * text=new char[100];
    char * filename= new char[100];
    Mat imageRead,hisImg;
    Mat mImg,transferImg,test_img,roiImg1,roiImg2;
    Mat finalImg=Mat(cv::Size(1024,542),CV_8U,Scalar(0,0,0));
    Mat histImg=Mat(cv::Size(1024,542),CV_8U,Scalar(255,255,255));
    Mat criteriaImg=cv::Mat::zeros(cv::Size(1024,542),CV_8U);
    static VideoCapture injectionVideo(autofocusVideo);
    static VideoWriter videoCreate("test4.avi", CV_FOURCC('D', 'I', 'V', 'X'), 5, Size(2048,1084),false);
    double Time_Focus=getTickCount();
    /**需要匹配的图像输入**/
    /***摄像头方式***/
    if(cameraOpen)
    {
        while(ImgControl->GrabImage(imageRead,5)!=0)
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
//            this->send(1);//观察关系
//            return -1;
//        }
//        if(i_count%10!=0)//跳帧读取
//        {
//            i_count++;
//            return 4;
//        }
        /***图片方式***/
        if (i_count>6)//视频结束
        {
            videoCreate.release();
            this->send(1);//观察关系
            return state_pathPlaning;
        }
        sprintf(filename,imgCell,i_count);//60
//        sprintf(filename,imgCell,60);//60
        imageRead=imread(filename);
        imwrite("first111.bmp",imageRead);

    }
    int pathWidth=imageRead.size().width;
    int pathHeight=imageRead.size().height;
    mImg=imageRead(Rect(pathWidth/2,0,pathWidth/2,pathHeight));

    imwrite("first222.bmp",mImg);

    Mat tImg;
    resize(mImg,tImg,Size(512,542));
    cvtColor(tImg,test_img,COLOR_BGR2GRAY);



    /**图像分割效果观察**/
    int contoursNum=0;
    static int contoursMax=0;
    double thref=0;
    Mat elementClose=getStructuringElement(MORPH_ELLIPSE, Size(11,11));
    Mat elementBlack=getStructuringElement(MORPH_ELLIPSE, Size(40,40));
//    Mat resultImg = Mat::zeros(test_img.size(), CV_8UC1);
    Mat contoursImg;


    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    Mat imgGamma,imgEven;
    /*适用于NIH-3T3的参数uneven:50, Gamma:无*/
    //越大越均匀，特征越弱，越小模糊效果越强
    //NIH-3T3必须要有光照均匀处理，否则右上角有干扰
//    unevenLightCompensate(test_img, 125);
//    GammaTransform(test_img,2);
    imwrite("cell_1gray.bmp",test_img);
    /*适用于Huvec的参数Size(11,11),11*/
    /*适用于NIH-3T3的参数Size(23,23),13*/
    //尺寸越小则连通区域小，碎块明显，噪声明显；大则碎块少，太大则过分连通
    GaussianBlur(test_img,test_img,Size(11,11),11);
    imwrite("cell_2gaussian.bmp",test_img);
//    test_img.copyTo(imgGamma);
//    morphologyEx(test_img,test_img,MORPH_BLACKHAT,elementBlack);//光照和对比度差问题
//    unevenLightCompensate(test_img, 200);

    test_img.copyTo(imgEven);
    hisImg=test_img.clone();
    Mat OTSUImg=test_img.clone();
    Mat TRIAImg=test_img.clone();

    /**大津阈值法+三角去噪**/
    double lengthValue,max_H,thresholdTRI;
    int max_index;
    thref=threshold(OTSUImg,OTSUImg,0,255,CV_THRESH_OTSU)+2;//大津阈值法找阈值(Huvec:2,NIH-3T3:2)
    thresholdTRI=sourceTriangleCompare(TRIAImg,&max_H,&max_index,&lengthValue,thref);//基于大津阈值法的阈值判断左右
//    if(thref>=max_index){
        threshold(test_img,test_img,thresholdTRI,255,CV_THRESH_BINARY);//当阈值大于峰值，使用三角法滤波
//    }else{
//        test_img=OTSUImg;
//    }
    imwrite("cell_3threshold.bmp",test_img);
    /**三角阈值法+大津三角去噪**/
//    double lengthValue,max_H,thresholdTRI;
//    int max_index;
//    thref=threshold(OTSUImg,OTSUImg,0,255,CV_THRESH_OTSU );//大津阈值法找阈值
//    thresholdTRI=sourceTriangleCompare(TRIAImg,&max_H,&max_index,&lengthValue,thref+3);//基于大津阈值法的阈值判断左右
//    threshold(test_img,test_img,thresholdTRI,255,CV_THRESH_BINARY);//二值化


//    thref=threshold(test_img,test_img,0,255,CV_THRESH_TRIANGLE);
//    imwrite("1triangle.bmp",test_img);
    morphologyEx(test_img,test_img,MORPH_CLOSE,elementClose);//连通
    imwrite("cell_4morphology.bmp",test_img);
    test_img=~test_img;
    imwrite("cell_5_test_img.bmp",test_img);
    test_img.copyTo(contoursImg);
    findContours(contoursImg,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);//提取轮廓


    for(auto cIterator = contours.begin(); cIterator!=contours.end(); cIterator++)
    {

        int sizeL;
       if(cIterator->size()>0 && cIterator->size()<200)//过滤噪声
        {
           drawContours(finalImg, contours, contoursNum, Scalar(255), 1, 8, hierarchy);//绘制轮廓
           sizeL=cIterator->size()/10;
           histgramLen[sizeL]++;
//           contoursNum=contoursNum+(int)cIterator->size();
           contoursNum++;
        }
    }
    if(contoursMax<contoursNum)
    {
        contoursMax=contoursNum;
        maxP=i_count*5;
    }
    imwrite("cell_6finalImg.bmp",finalImg);

    /**直方图计算**/
    CvHistogram* gray_hist = cvCreateHist(1,&hist_size,CV_HIST_ARRAY,ranges,1);
    //计算灰度图像的一维直方图
    IplImage temp = (IplImage)hisImg;
    IplImage *src = &temp;
    cvCalcHist(&src,gray_hist,0,nullptr);
    //归一化直方图
    cvNormalizeHist(gray_hist,1.0);
    //绘制直方图
    double val =0;
    float bin_w=histImg.size().width/256;
    for (int i = 0; i <255; i++)
    {
        val = 3*(cvQueryHistValue_1D(gray_hist,i)*histImg.size().height);//获取矩阵元素值，并转换为对应高度
        if(i>thref){
            if(val!=0)
            {
                cv::rectangle(histImg, cvPoint(i*bin_w, histImg.size().height),cvPoint((i + 1)*bin_w, (int)(histImg.size().height - val)),Scalar(0,0,0), 2, 8, 0);
            }
        }else {
            if(val!=0)
            {
            cv::rectangle(histImg, cvPoint(i*bin_w, histImg.size().height),cvPoint((i + 1)*bin_w, (int)(histImg.size().height - val)),Scalar(200,200,125), 2, 8, 0);
            }
        }
    }

//    imwrite("cellA3.bmp",test_img);
//    imshow("2",test_img);
//    waitKey(10);


    /**曲线关系观察**/
    Array_Sharpness[poseCount][1]=contoursNum;//代表Z轴的上下移动
//        Array_Sharpness[poseCount][1]=valDiff;//灰度差
    Array_Sharpness[poseCount][0]=i_count*5;//类间方差
//    for (int i = 0; i <poseCount; i++)
//    {
//        val = Array_Sharpness[i][1]*criteriaImg.size().height/150;//获取矩阵元素值，并转换为对应高度
//        cv::rectangle(criteriaImg,
//                      cvPoint(i*criteriaImg.size().width/poseCount, criteriaImg.size().height),
//                      cvPoint((i + 1)*criteriaImg.size().width/poseCount,(int)(criteriaImg.size().height - val)),
//                      Scalar(255,255,0), 1, 8, 0);
//    }
    poseCount++;
    /**第4张图片标题**/
    sprintf(text,"The number of cells:%d",contoursNum);
    putText(criteriaImg,text,cvPoint(50,50),FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255),2);
    /**第3张图片标题**/
    sprintf(text,"Gray histogram; Frame:%d",i_count);
    putText(histImg,text,cvPoint(50,50),FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255),2);
    len_rows=poseCount;
    i_count++;
//    if(i_count>630){
//        i_count++;
//        i_count--;
//    }



   cvReleaseHist(&gray_hist);
   /**四图融合展示**/
   resize(imgEven,imgEven,Size(512,542));
   resize(test_img,test_img,Size(512,542));
   resize(histImg,histImg,Size(512,542));
   resize(criteriaImg,criteriaImg,Size(512,542));
   vector <Mat> vImg_1;
   vector <Mat> vImg_2;
   vImg_1.push_back(imgEven);
   vImg_1.push_back(test_img);
   vImg_2.push_back(histImg);
   vImg_2.push_back(criteriaImg);
   Mat outImg_1,outImg_2;
   hconcat(vImg_1,outImg_1);
   hconcat(vImg_2,outImg_2);
   vImg_1.clear();
   vImg_1.push_back(outImg_1);
   vImg_1.push_back(outImg_2);
   vconcat(vImg_1,outImg_1);
   videoCreate<<outImg_1;
   Time_Focus=((double)getTickCount()-Time_Focus)/getTickFrequency()*1000;
//   for(int i=0;i<20;i++)
//   {
//       cout<<histgramLen[i]<<" ";
//   }
//   cout<<endl;
   cout <<"Time_Focus:"<<Time_Focus<<" ,i_count:"<<i_count<< std::endl;
   return state_cellDetection;

}





float Auto_Focus::dist(Point2f A, Point2f B)
{
    return 1/InvSqrt((A.x - B.x) * (A.x - B.x) + (A.y - B.y) * (A.y - B.y));
}

void Auto_Focus::GetDist(vector<Point2f> p, int n)
{
    if(p.empty())
    {
        cout << "The vector is empty!" << endl;
        return;
    }
    vector<vector<float>> w(n,vector<float>(n));   //两两城市之间路径长度
    for(int i = 0; i < n; i++)
    {
        for(int j = i + 1; j < n; j++)
           { w[i][j] = w[j][i] = dist(p[i], p[j]);}
    }
    pathL=w;
}

void Auto_Focus::Init(int n)
{
    nCase = 0;
    bestPath.len = 0;
    for(int i = 0; i < n; i++)
    {
        bestPath.citys.push_back(i);
        if(i != n - 1)
        {
//            printf("%d--->", i);
            bestPath.len += pathL[i][i + 1];
        }
        else{
 //            printf("%d\n", i);
        }

    }
    cout<<"initial path: "<<bestPath.len<<endl;
}
injectPath Auto_Focus::GetNext(injectPath p, int n)
{
    injectPath ans = p;
    int x = (int)(n * (rand() / (RAND_MAX + 1.0)));
    int y = (int)(n * (rand() / (RAND_MAX + 1.0)));
    while(x == y)
    {
        x = (int)(n * (rand() / (RAND_MAX + 1.0)));
        y = (int)(n * (rand() / (RAND_MAX + 1.0)));
    }
    swap(ans.citys[x], ans.citys[y]);
    ans.len = 0;
    for(int i = 0; i < n - 1; i++)
        ans.len += pathL[ans.citys[i]][ans.citys[i + 1]];
//    cout << "nCase = " << nCase << endl;
    nCase++;
    return ans;
}


void Auto_Focus::barycentersGet(Mat & showImg, Mat imageRead)
{
    VideoCapture injectionVideo(autofocusVideo);
    Mat transitImg,calImg,tImg;
    char * filename=new char[100];
    /***摄像头方式***/

//      if(cameraOpen)
//      {
//        while(ImgControl->GrabImage(imageRead,5)!=0)
//        {
//            std::cout << "Grab image failed " << std::endl;
//        }
//      }
//      else
//      {
          /***视频方式***/
//          injectionVideo.set(CAP_PROP_POS_FRAMES,maxP*2);
//          injectionVideo>>readImg;
//          if (readImg.empty())//视频结束
//          {
//              return ;
//          }
          /***图片方式***/
//            sprintf(filename,imgCell,maxP-150);
//              sprintf(filename,imgCell,6);
//            imageRead=imread(filename);
//      }
//    int pathWidth=imageRead.size().width*(1-shrinkRate);
//    int pathHeight=imageRead.size().height*(1-shrinkRate);
//    transitImg=imageRead(Rect(imageRead.size().width/2-pathWidth/2,imageRead.size().height/2-pathHeight/2,pathWidth,pathHeight));//去除视频的黑边,缩小路径规划区域
//    resize(transitImg,transitImg,Size(1024,542));
//    resize(imageRead,transitImg,Size(1024,542));
    transitImg=imageRead;

//      Mat mImg;
//      int pathWidth=imageRead.size().width;
//      int pathHeight=imageRead.size().height;
//      mImg=imageRead(Rect(pathWidth/2,0,pathWidth/2,pathHeight));
//      resize(mImg,transitImg,Size(512,542));



    /***细胞个数评价指标相关参数***/
   vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    double y;
    Mat outImg;

    /**bare area指标相关参数***/
    vector<vector<Point>> contoursBare;
     vector<Vec4i> hierarchyBare;
    Mat bareImg,bareShow;
    Mat elementErode=getStructuringElement(MORPH_ELLIPSE, Size(71,71));


   Mat contrastImg=Mat(transitImg.size(),imageRead.type(),Scalar(0, 0, 0));
    //提取质心
   if(flourescence==0){
        sharpnessFunction(transitImg,outImg,10,y);
   }else{
        sharpnessFunction(transitImg,outImg,11,y);
   }


    bareImg=~outImg.clone();
    morphologyEx(bareImg,bareImg,MORPH_ERODE,elementErode);//erode
    bareShow=bareImg.clone();
    imwrite("bareImg.bmp",bareImg);//输出原始图像
    findContours(bareImg,contoursBare,hierarchyBare,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);//提取轮廓
    // get the moments
    vector<Moments> muBare(contoursBare.size());
    for( int i = 0; i<contoursBare.size(); i++ )
    { muBare[i] = moments( contoursBare[i], false ); }

    // get the centroid of figures.
    vector<Point2f> mcBare(contoursBare.size());
    for( int i = 0; i<contoursBare.size(); i++)
    {
        if(muBare[i].m00>0){
            mcBare[i] = Point2f( muBare[i].m10/muBare[i].m00 , muBare[i].m01/muBare[i].m00 );
        }
        else {
            mcBare[i] = Point2f(0.0,0.0);
        }
    }
    barycenterBare = mcBare;//保存质心数据
    for( int i = 0; i<barycenterBare.size(); i++ )
    {
        circle(bareShow, barycenterBare[i],5, Scalar(0,0,255), 1, 8, 0 );
    }
    imwrite("barycenterBare.bmp",bareShow);//输出原始图像




    Mat roiImg;
//    Rect r1((int)(outImg.size().width*0.1), (int)(outImg.size().height*0.1), (int)(outImg.size().width*0.8), (int)(outImg.size().height*0.8));
//    Mat maskImg = Mat::zeros(outImg.size(), CV_8UC1);
//    maskImg(r1).setTo(255);
    outImg.copyTo(roiImg);
    imwrite("cellRoiImg.bmp",roiImg);//输出原始图像

    cout<<"TP SIZE:"<<y<<endl;
    showImg=outImg.clone();
    findContours(roiImg,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);//提取轮廓
    int contoursNum=0;
    for(auto cIterator = contours.begin(); cIterator!=contours.end(); cIterator++)
    {
       drawContours(contrastImg, contours, contoursNum, Scalar(0,0,255), 1, 8, hierarchy);//绘制轮廓
       contoursNum++;
    }

    // get the moments
    vector<Moments> mu(contours.size());
    for( int i = 0; i<contours.size(); i++ )
    { mu[i] = moments( contours[i], false ); }

    // get the centroid of figures.
    vector<Point2f> mc(contours.size());
    for( int i = 0; i<contours.size(); i++)
    { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }
    barycenterAll = mc;//保存质心数据
    int widthMin=(int)(outImg.size().width*0.1);
    int widthMax=(int)(outImg.size().width*0.9);
    int heightMin=(int)(outImg.size().height*0.1);
    int heightMax=(int)(outImg.size().height*0.9);
    for( int i = 0; i<barycenterAll.size(); i++ )
    {
        if(barycenterAll[i].x>widthMin&&barycenterAll[i].x<widthMax&&barycenterAll[i].y>heightMin&&barycenterAll[i].y<heightMax){
            barycenterP.push_back(barycenterAll[i]);
            circle(contrastImg, barycenterAll[i],5, Scalar(0,0,255), 1, 8, 0 );
        }

    }
    imwrite("barycenter.bmp",showImg);
    resize(contrastImg,contrastImg,imageRead.size());
    contrastImg+=imageRead;
    imwrite("cell_7contrast.bmp",contrastImg);//输出原始图像




    // draw contours
//    Mat drawing(sharpImg.size(), CV_8UC3, Scalar(255,255,255));
//    for( int i = 0; i<contours.size(); i++ )
//    {
//    Scalar color = Scalar(167,151,0); // B G R values
//    drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
//    circle( drawing, mc[i], 4, color, -1, 8, 0 );
//    }

}


int8_t Auto_Focus::barycentersWrite()
{
    QFile handle("Cells.txt");
    handle.open(QIODevice::WriteOnly);
    QTextStream write_(&handle);

    write_<<"NAME : Cells\n";
    write_<<"COMMENT : "<<barycenterP.size()<<" cells\n";
    write_<<"TYPE : TSP\n";
    write_<<"DIMENSION : "<<barycenterP.size()<<"\n";
    write_<<"EDGE_WEIGHT_TYPE : EUC_2D\n";
    write_<<"NODE_COORD_SECTION\n";
    for (int i=1;i<barycenterP.size()+1;i++)
    {
       write_<<i<<" "<<(int)barycenterP[i-1].x<<" "<<(int)barycenterP[i-1].y<<"\n";
    }
    write_<<"EOF\n";
    handle.close();
    QFile::remove("Cells.tsp");
    rename("Cells.txt","Cells.tsp");

    return 1;

}

void Auto_Focus::goToDefocus()//移动到离焦平面
{
    cellParamsFirst.dev=2;//设置控制对象
    Ump_Select_Dev(&cellParamsFirst);
    Ump_Read_Position(&cellParamsFirst);
    cellParamsFirst.target_y=cellParamsFirst.home_y;
    cellParamsFirst.target_x=cellParamsFirst.home_x;
    cellParamsFirst.target_z=cellParamsFirst.home_z+20000;
    cellParamsFirst.target_d=cellParamsFirst.home_d;
    cellParamsFirst.speed=1000;
    Ump_Goto_Position(&cellParamsFirst);
}


void Auto_Focus::goOut()
{
    cellParamsFirst.dev=2;//设置控制对象
    Ump_Select_Dev(&cellParamsFirst);
    Ump_Read_Position(&cellParamsFirst);
    cellMark=cellParamsFirst;
    cellParamsFirst.target_y=cellMark.home_y+outDistance;
    cellParamsFirst.target_x=cellMark.home_x+outDistance;
    cellParamsFirst.target_z=cellMark.home_z;
    cellParamsFirst.target_d=cellMark.home_d;
    cellParamsFirst.speed=1000;
    Ump_Goto_Position(&cellParamsFirst);
}

void Auto_Focus::comeBack()
{
    cellParamsFirst.target_y=cellMark.home_y;
    cellParamsFirst.target_x=cellMark.home_x;
    cellParamsFirst.target_z=cellMark.home_z;
    cellParamsFirst.target_d=cellMark.home_d;
    cellParamsFirst.speed=1000;
    Ump_Goto_Position(&cellParamsFirst);
}


Mat Auto_Focus::getCellImg()//获取细胞图像
{

    Mat imageRead,whiteImg,blackImg,cellImg,result_img;
    if(flourescence==0)
    {
        if(actOpen)
        {
            cellParamsFirst.dev=2;//设置控制对象
            Ump_Select_Dev(&cellParamsFirst);
            Ump_Read_Position(&cellParamsFirst);
    //        cellParamsFirst.target_y=cellMark.home_y;
    //        cellParamsFirst.target_x=cellMark.home_x;
    //        cellParamsFirst.target_z=cellMark.home_z-20*1000;
    //        cellParamsFirst.target_d=cellMark.home_d;
    //        cellParamsFirst.speed=1000;
    //        Ump_Goto_Position(&cellParamsFirst);
    //        Sleep(1000);//等待运动完成

    //        /***摄像头方式***/
    //        if(cameraOpen)
    //        {
    //            ImgControl->GrabImage(imageRead,50);
    //            Sleep(100);//等待
    //            while(ImgControl->GrabImage(imageRead,50)!=0)
    //            {
    //                std::cout << "Transfor: Grab failed " << std::endl;
    //                /*break;*/
    //            }
    //        }

    //        whiteImg=imageRead.clone();

            cellParamsFirst.target_y=cellParamsFirst.home_y;
            cellParamsFirst.target_x=cellParamsFirst.home_x;
            cellParamsFirst.target_z=cellParamsFirst.home_z+120*1000;
            cellParamsFirst.target_d=cellParamsFirst.home_d;
            cellParamsFirst.speed=1000;
            Ump_Goto_Position(&cellParamsFirst);
            Sleep(1000);//等待运动完成

            if(cameraOpen)
            {
                ImgControl->GrabImage(imageRead,50);
                Sleep(100);//等待
                while(ImgControl->GrabImage(imageRead,50)!=0)
                {
                    std::cout << "Transfor: Grab failed " << std::endl;
                    /*break;*/
                }
            }

            blackImg=imageRead.clone();

            cellParamsFirst.target_y=cellParamsFirst.home_y;
            cellParamsFirst.target_x=cellParamsFirst.home_x;
            cellParamsFirst.target_z=cellParamsFirst.home_z;
            cellParamsFirst.target_d=cellParamsFirst.home_d;
            cellParamsFirst.speed=1000;
            Ump_Goto_Position(&cellParamsFirst);

            cellImg=blackImg;
        }
    }else{
        if(cameraOpen)
        {
            ImgControl->GrabImage(imageRead,50);
            Sleep(100);//等待
            while(ImgControl->GrabImage(imageRead,50)!=0)
            {
                std::cout << "Transfor: Grab failed " << std::endl;
                /*break;*/
            }
        }
        cellImg=imageRead.clone();

    }


    if(!cellImg.empty())
    {
        resize(cellImg,result_img,Size(1024,542));
        imwrite("back.bmp",result_img);
    }else{
        cout<<"cell_img get error"<<endl;
    }

    return result_img;

}

void Auto_Focus::mousePosition(float x,float y)
{
    mouseRatio.x=x*1024.0;
    mouseRatio.y=y*542.0;
    if(Auto_Focus_Slect==state_pathPlaning){
        barycenterP.push_back(mouseRatio);
        emit sendPositions(mouseRatio.x, mouseRatio.y);
    }

//    cout<<x<<"  "<<y<<endl;
}


/*********************
Function:路径规划
Abstract:基于质心位置，获取最优路径
Author :胡伟康
*********************/
int8_t Auto_Focus::pathPlaning()
{
    static int pCount=0;
    static Mat imageRead,drawing,cellImg;
    QImage showImg;
    Mat outImg=cv::Mat::zeros(cv::Size(1024,542),CV_8U);

    switch (2) {
    case 1://基于细胞识别选择刺入目标点
        if(pCount==0)
        {
           cellImg=getCellImg();
           barycentersGet(drawing,cellImg);
           pCount++;
        }
        break;
    case 2://手动选择刺入目标点
        if(penetrationCollectFlag==0){
            //初始清空刺入目标点
            if(cameraOpen)
            {
                ImgControl->GrabImage(imageRead,50);
                Sleep(100);//等待
                while(ImgControl->GrabImage(imageRead,50)!=0)
                {
                    std::cout << "Transfor: Grab failed " << std::endl;
                    /*break;*/
                }
            }
            imwrite("path_manual_save.bmp",imageRead);
            barycenterP.clear();
            penetrationCollectFlag++;
            cout<<"penetration targets collection is starting"<<endl;
            return state_pathPlaning;
        }else if (penetrationCollectFlag==1){
            //等待收集刺入位置

             return state_pathPlaning;
        }else{
            //收集结束，开始优化路径
            cout<<"penetration targets collection is over"<<endl;
        }
        break;
    default:
        break;
    }

    //LKH-algorithm init
    barycentersWrite();
    GetDist(barycenterP,barycenterP.size());
    Init(barycenterP.size());

    //LKH-based path optimization
    QProcess pro;
    pro.execute("../MicroSystem/LKH-3.exe",QStringList()<<"../MicroSystem/Cells.par");
    pro.kill();

    QFile fl("Best.txt");
    fl.open(QIODevice::ReadOnly);
    QTextStream read_(&fl);

    QByteArray array;

    bestPath.citys.clear();
    while(fl.atEnd() == false)
    {
        int cNum=0;
        array = fl.readLine();
        array =array.trimmed();
        for(int i=0;i<array.size();i++)
        {
            cNum = cNum+(array.at(i)-48)*pow(10,array.size()-i-1);
        }
        bestPath.citys.push_back(cNum);
        bestPathPoint.push_back(barycenterP[cNum-1]);
    }
    fl.close();
//    for( int i = 0; i<bestPath.citys.size()-1; i++ )
//    {
//        int iNum=bestPath.citys[i]-1;
//        int iNum_=bestPath.citys[i+1]-1;
//        circle(drawing, barycenterP[iNum],4, Scalar(0,0,0), 1, 8, 0 );
//        line( drawing, barycenterP[iNum],barycenterP[iNum_], Scalar(255,0,0), 1, 8, 0 );
////            arrowedLine( drawing, barycenterP[iNum],barycenterP[iNum_], Scalar(255,0,0), 1, 8, 0 );
//    }
//    imwrite("cell_8path.bmp",drawing);
    cout<<"cellPathOut"<<endl;
    return state_idle;
}


int8_t Auto_Focus::touchPreparation()
{
    if(actOpen){

        cellPosition.dev=2;//设置控制对象（细胞）
        Ump_Select_Dev(&cellPosition);
        cellPosition.target_d=cellPosition.home_d;
        cellPosition.target_y=cellPosition.home_y;
        cellPosition.target_z=cellPosition.home_z;
        cellPosition.target_x=cellPosition.home_x;
        cellPosition.speed=1000;
        Ump_Goto_Position(&cellPosition);//将细胞移回视野内

        verticalPosition.dev=1;//设置控制对象（针）
        Ump_Select_Dev(&verticalPosition);
        verticalPosition.target_d=verticalPosition.home_d;
        verticalPosition.target_y=verticalPosition.home_y;
        verticalPosition.target_z=verticalPosition.home_z+10000;//偏离10μm
        verticalPosition.target_x=verticalPosition.home_x;
        verticalPosition.speed=1000;
        Ump_Goto_Position(&verticalPosition);//将微管移回视野内
    }

    return state_idle;
}


int8_t Auto_Focus::segmentTransform()
{
    if(actOpen==2){
        cellPosition.dev=2;//设置控制对象（细胞）
        Ump_Select_Dev(&cellPosition);
        Ump_Read_Position(&cellPosition);

        cellPosition.target_d=cellPosition.home_d;
        cellPosition.target_y=cellPosition.home_y+500*1000;
        cellPosition.target_z=cellPosition.home_z;
        cellPosition.target_x=cellPosition.home_x;
        cellPosition.speed=1000;
        Ump_Goto_Position(&cellPosition);//将细胞换个视野
    }

    return state_idle;
}


int8_t Auto_Focus::imageStitching()
{
    /***Test of single_slot***/
    static int i_count=0;
    static FileStorage fs("Auto_Focus.yaml",FileStorage::WRITE);
    static VideoCapture injectionVideo(autofocusVideo);
    static VideoWriter videoCreate("imageStitching.avi", CV_FOURCC('D', 'I', 'V', 'X'), 5, Size(2048,542),false);



    char * filename=new char[100];
    char * text=new char[100];
    Mat readImg,transitImg,calImg;
    Mat showImg;
    Mat outImg,grayImg;
    vector<Mat> vImg_1;


    double Time_Focus=getTickCount();
    /***摄像头方式***/
      if(cameraOpen)
      {
        while(ImgControl->GrabImage(readImg,5)!=0)
        {
//            std::cout << "Grab image failed " << std::endl;
        }
      }
      else
      {
          /***视频方式***/
//          injectionVideo>>readImg;
//          if (readImg.empty())//视频结束
//          {
//              videoCreate.release();
//              injectionVideo.release();
//              fs.release();
//              this->send(1);//观察关系
//              return state_idle;
//          }

//          if(i_count%10!=0)//跳帧读取
//          {
//              return state_coarseAdjust;
//          }
          /***图片方式***/
          sprintf(filename,imgCell,13);
          readImg=imread(filename);
      }
     resize(readImg,transitImg,Size(1024,542));
     cvtColor(transitImg,grayImg,COLOR_BGR2GRAY);

     double minVal, maxVal;
     cv::Point minLoc, maxLoc;



     Mat outcomeImg;
     Mat leftImg=grayImg(Rect(0, 0, grayImg.cols*0.7, grayImg.rows));
//     imwrite("leftImg.bmp",leftImg);
     Mat rightImg=grayImg(Rect(grayImg.cols*0.3, 0, grayImg.cols*0.7-1, grayImg.rows));
//     imwrite("rightImg.bmp",rightImg);
     Mat templateImg=leftImg(Rect(leftImg.cols*0.7, 0.4*leftImg.rows, leftImg.cols*0.2, 0.2*leftImg.rows));
     matchTemplate(rightImg,templateImg,outcomeImg,TM_CCOEFF_NORMED);
     minMaxLoc(outcomeImg, &minVal, &maxVal, &minLoc, &maxLoc);





    return state_idle;
}



int8_t Auto_Focus::penetrationSleepChange(){

    if(Auto_Focus_Slect==state_idle){
        Auto_Focus_Slect=state_penetrationSleep;
    }
    return state_idle;
}


int8_t Auto_Focus::penetrationSleep(){

    for(int sleepNum=0; sleepNum<penetrationTime; sleepNum++)
    {
        cout <<"Penetration Time:"<<sleepNum<< std::endl;
        sendState(QString("Time : %1").arg(sleepNum));
        Sleep(1000);//扎进细胞的时间
    }
     emit sendPenetration();

    return state_idle;

}





void Auto_Focus::decision()
{
    switch (Auto_Focus_Slect) {
    case 100:
        if(actOpen==2){
            cellParamsFirst.dev=2;//设置控制对象（细胞）
            Ump_Select_Dev(&cellParamsFirst);
            Ump_Read_Position(&cellParamsFirst);
            cellPosition=cellParamsFirst;
        }
        timeAutoF=new QTimer();
        timeAutoF->setInterval(20);
        connect(timeAutoF,&QTimer::timeout,this,&Auto_Focus::decision);
        timeAutoF->start();
        Auto_Focus_Slect=-1;
        cout<<"waiting for autofocusing "<<endl;
        break;
    case state_coarseAdjust:
        /*清晰度评价函数选择、步距、起始位置、范围、投票阈值、拟合点个数*/
        Auto_Focus_Slect=coarseAdjust(2,5,-150,500,3,6);
        break;
    case state_fineAdjust:
        Auto_Focus_Slect=fineAdjust(1);
        break;
    case state_cellAutofocus:
        Auto_Focus_Slect=cellAutofocus();
        break;
    case state_pathPlaning:
        Auto_Focus_Slect=pathPlaning();
        break;
    case state_segmentTransform:
        Auto_Focus_Slect=segmentTransform();
        break;
    case state_imageStitching:
        Auto_Focus_Slect=imageStitching();
        break;
    case state_cellDetection:
        Auto_Focus_Slect=cellDetection();
        break;
    case state_penetrationSleep:
        Auto_Focus_Slect=penetrationSleep();
        break;
    default:
        Auto_Focus_Slect=-1;
        break;
    }
}

//void Auto_Focus::run()
//{
//    timeAutoF=new QTimer();
//    timeAutoF->setInterval(20);
//    connect(timeAutoF,&QTimer::timeout,this,&Auto_Focus::decision);
//    timeAutoF->start(1);
//    exec();

//}

