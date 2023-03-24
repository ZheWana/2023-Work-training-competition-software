#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <math.h>
#include <wiringPi.h>
#include <stdio.h>
#include <wiringSerial.h>
#include <unistd.h>
#include <string.h>

using namespace cv;
using namespace std;

//wiringpi IO
#define STATE_INT_PIN 9 
#define RESET_INT_PIN 8
#define INIT_OK_PIN 7
//#define X_PASS_PIN 8
//#define Y_PASS_PIN 9

//调试
#define DEBUG_QR_CODE 0

//状态
#define STATE_RESET 0xff                            //RESET
#define STATE_TEST 0

#define STATE_INIT 1                             //初始状态，发车前往二维码位置
#define STATE_QR_CODE_DETECT 2                      //到达二维码位置区域，开始识别二维码
#define STATE_QR_CODE_COLOR_WAY 3                      //   MOYU
#define STATE_CALIBRATION_FIRST 4
#define STATE_CAL_TO_COLOR_WAY 5
#define STATE_COLOR_DETECT 6                        //颜色识别，找到对应物块位置


#define STATE_GOOD_FETCH_OK_PUT_WAY 7

#define STATE_CIRCLE_DETECT   8                 //圆形坐标识别

#define STATE_FINISH 9                  //over

#define STATE_CALIBRATION_SECOND 10



#define STATE_YAW_DETECT 11                         //行走识别直线获取角度

#define CROSS_DETECT 12

#define STATE_CAL_TO_CIRCLE_WAY 13

#define COLOR_RED 1
#define COLOR_GREEN 2
#define COLOR_BLUE 3


void display(Mat& im, Mat& bbox)
{
    int n = bbox.rows;
    for (int i = 0; i < n; i++)
    {
        line(im, Point2d(bbox.at<float>(i, 0), bbox.at<float>(i, 1)), Point2d(bbox.at<float>(i, 2), bbox.at<float>(i,3)), Scalar(0, 0, 255), 2);
        line(im, Point2d(bbox.at<float>(i, 2), bbox.at<float>(i, 3)), Point2d(bbox.at<float>(i, 4), bbox.at<float>(i, 5)), Scalar(255, 0, 0), 2);
        line(im, Point2d(bbox.at<float>(i, 4), bbox.at<float>(i, 5)), Point2d(bbox.at<float>(i, 6), bbox.at<float>(i, 7)), Scalar(255, 0, 0), 2);
        line(im, Point2d(bbox.at<float>(i, 0), bbox.at<float>(i, 1)), Point2d(bbox.at<float>(i, 6), bbox.at<float>(i, 7)), Scalar(255, 0, 0), 2);
    }
    if(DEBUG_QR_CODE)
        imshow("QR_CODE_Result", im);
}

uint8_t qrcode_read(Mat image,char *buffer)
{
    QRCodeDetector qrDecoder = QRCodeDetector();
    Mat bbox, rectifiedImage;
    std::string data = qrDecoder.detectAndDecode(image, bbox, rectifiedImage);
    if (data.length() > 0)
    {
        if(DEBUG_QR_CODE)
            cout << "Decoded Data : " << data << endl;
        display(image, bbox);
        //rectifiedImage.convertTo(rectifiedImage, CV_8UC3);
        //imshow("Rectified QRCode", rectifiedImage);
        strcpy(buffer,data.c_str());
        return 0;
    }
    else
    {
        if (DEBUG_QR_CODE)
            cout << "QR Code not detected" << endl;
        return -1;                              //没有检测到二维码
    }
}

float cal_line_angle(Vec4i l)       //计算直线角度
{
    return atan2f((l[3] - l[1]) ,(l[2] - l[0]))*180/CV_PI;
}

float getDist_P2L( Point2d pointP, Point2d pointA, Point2d pointB)
{
    //求直线方程
    int A = 0, B = 0, C = 0;
    A = pointA.y - pointB.y;
    B = pointB.x - pointA.x;
    C = pointA.x * pointB.y - pointA.y * pointB.x;
    //代入点到直线距离公式
    float distance = 0;
    distance = ((float)abs(A * pointP.x + B * pointP.y + C)) / ((float)sqrtf(A * A + B * B));
    return distance;
}
void buffer_insert(uint8_t *buffer,uint16_t data1,uint16_t data2)
{
    *(uint16_t*)(buffer+2)=data1;
    *(uint16_t*)(buffer+4)=data2;
}
void buffer_insert_p(uint8_t *buffer,uint16_t* data1_p,uint16_t* data2_p)
{
    *(uint16_t*)(buffer+2)=*data1_p;
    *(uint16_t*)(buffer+4)=*data2_p;
}
uint8_t cross_detect(Mat * img,float * x,float *y)
{
    Mat image_gray,image_canny;
    Mat element;                    //算子
    float angle[20];                 //存放直线角度数据
    float dis[20];                  //存放直线距离数据
    //将图像转换为灰度图
      cvtColor(*img, image_gray, COLOR_BGR2GRAY);
     //阈值二值化
     threshold(image_gray, image_gray, 100, 255, 0);
            element = getStructuringElement(MORPH_RECT, Size(5, 5));
            //进行形态学操作
            //morphologyEx(image_gray, image_gray, MORPH_OPEN, element);
            //边缘检测
            Canny(image_gray, image_canny, 50, 150, 3, false);
            vector<Vec4i> lines;//定义一个矢量结构lines用于存放得到的线段矢量集合
            HoughLinesP(image_canny, lines, 1, CV_PI / 180, 80, 100, 100);              

            
            if (lines.size() <= 50|| lines.size()==0)
            {
                
                //【4】依次在图中绘制出每条线段
                for (size_t i = 0; i < lines.size(); i++)
                {
                    Vec4i l = lines[i];
                    //此句代码的OpenCV3版为：
                    line(*img, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1, LINE_AA);
                    angle[i] = cal_line_angle(l);
                    dis[i] = getDist_P2L(Point2d(0, 0), Point(l[0], l[1]), Point(l[2], l[3]));
                    //printf("angle=%f,dis=%f\n", angle[i], dis[i]);
                }
               // printf("\n\n");
                float dis_x[10]={0};
                float dis_y[10]={0};
                int num_x=0;
                int num_y=0;
                for(size_t i = 0; i < lines.size(); i++)
                {
                    if(fabs(angle[i])<10)
                    {
                        if(num_x>=10)
                            return 0;
                        dis_x[num_x++]=dis[i];
                    }
                    if(fabs(angle[i])>80)
                    {
                        if(num_y>=10)
                             return 0;
                        dis_y[num_y++]=dis[i];
                    }

                }
                if(num_x>=2&&num_y>=2)
                {
                    for(int i=0;i<num_x;i++)
                    {
                        for(int j=i;j<num_x;j++)
                        {
                            if(dis_x[j]<dis_x[i])
                            {
                                float temp=dis_x[j];
                                dis_x[j]=dis_x[i];
                                dis_x[i]=temp;
                            }
                        }
                        for(int j=i;j<num_y;j++)
                        {
                            if(dis_y[j]<dis_y[i])
                            {
                                float temp=dis_y[j];
                                dis_y[j]=dis_y[i];
                                dis_y[i]=temp;
                            }
                        }
                            
                    }
                    int jianduan_x[10]={0},jianduan_y[10]={0};
                    int jianduan_x_num=0,jianduan_y_num=0;
                    for(int i=0;i<num_x-1;i++)
                    {
                        //printf("dis_x:%f\n",dis_x[i]);
                        if(dis_x[i+1]-dis_x[i]>10)
                            jianduan_x[jianduan_x_num++]=i;

                    }
                    for(int i=0;i<num_y-1;i++)
                    {
                        //printf("dis_y:%f\n",dis_y[i]);
                        if(dis_y[i+1]-dis_y[i]>10)
                            jianduan_y[jianduan_y_num++]=i;
                    }
                    if(jianduan_x_num==1&&jianduan_y_num==1)
                    {
                        //printf("cross\n\n\n");
                        float dis_sum_x_left=0;
                        float dis_sum_x_right=0;
                        float dis_sum_y_up=0;
                        float dis_sum_y_down=0;

                        for(int i=0;i<=jianduan_x[0];i++)
                        {
                            dis_sum_x_left+=dis_x[i];
                        }
                        for(int i=0;i<=jianduan_y[0];i++)
                        {
                            dis_sum_y_up+=dis_y[i];
                        }
                        for(int i=jianduan_x[0]+1;i<num_x;i++)
                        {
                            dis_sum_x_right+=dis_x[i];
                        }
                        for(int i=jianduan_y[0]+1;i<num_y;i++)
                        {
                            dis_sum_y_down+=dis_y[i];
                        }
                        float x1=dis_sum_x_left/(jianduan_x[0]+1);
                        float x2=dis_sum_x_right/(num_x-jianduan_x[0]-1);
                        float y1=dis_sum_y_up/(jianduan_y[0]+1);
                        float y2=dis_sum_y_down/(num_y-jianduan_y[0]-1);
                        //printf("x1=%f,x2=%f,y1=%f,y2=%f\n\n",x1,x2,y1,y2);
                       // printf("cross:%f,%f\n",(y1+y2)/2-325,(x1+x2)/2-100);
                        *x=(y1+y2)/2-325;
                        *y=(x1+x2)/2-100;
                        return 1;
                    }
                    printf("hhh");
                }
            
            }
            else
            {
                if(lines.size()>50)
                    printf("error:too much line!!\n");
                else
                    printf("error:no line found!!\n");
            }    
             return 0;  
}

uint8_t state_int_flag=0;
uint8_t reset_int_flag=0;
void state_int_callback(void){
    printf("state_int\n\n");
    state_int_flag=1;
}
void reset_int_callback(void){
    printf("reset_int\n\n");
    reset_int_flag=1;
    digitalWrite(INIT_OK_PIN,0);
}


int main(int argc, char* argv[])
{
    char QR_CODE_Buffer[10];                    //二维码字符串
    uint8_t state = STATE_RESET;        //状态机初始状态
    uint8_t color_chose_num=0;
    uint8_t color_chose= COLOR_RED;             //hsv识别颜色
    Mat cam_image_1;                            //竖直朝下摄像头
    Mat cam_image_2;                            //水平朝前摄像头
    Mat image_hsv;                              //hsv转换后
    Mat image_gray;                             //灰度图
    Mat image_canny;                            //canny
    //三种物块颜色识别图
    Mat COLOR_R_IMAGE;                          
    Mat COLOR_G_IMAGE;
    Mat COLOR_B_IMAGE;

    Mat element;                    //算子
    Mat hsv_red1, hsv_red2;
    vector<Vec2f> mylines;          //定义一个矢量结构lines用于存放得到的线段矢量集合
    vector<Vec3f> circles;          //定义一个矢量结构circles用于存放得到的矢量集合
    float angle[20];                 //存放直线角度数据
    float dis[20];                  //存放直线距离数据

    uint8_t color_order[3]={0,0,0};

    int fd ;
    if((fd = serialOpen("/dev/serial0", 115200)) < 0)
    {
        printf("Unable to open serial device\n");
        return 1 ;
    }
    uint8_t buffer[4*2]={0x55,0x55,0x00,0x00,0x00,0x00,0xAA,0xAA};
    
    //init
    wiringPiSetup();
    //pinMode(X_PASS_PIN, OUTPUT); 
    //pinMode(Y_PASS_PIN, OUTPUT); 
    pinMode(STATE_INT_PIN, INPUT); 
    pinMode(RESET_INT_PIN, INPUT); 
    pinMode(INIT_OK_PIN, OUTPUT); 
    pullUpDnControl (STATE_INT_PIN, PUD_DOWN);
    pullUpDnControl (RESET_INT_PIN, PUD_DOWN);
    pullUpDnControl (INIT_OK_PIN, PUD_DOWN);
    wiringPiISR(STATE_INT_PIN,INT_EDGE_RISING,&state_int_callback);
    //wiringPiISR(RESET_INT_PIN,INT_EDGE_RISING,&reset_int_callback);

    VideoCapture capture_1;
    VideoCapture capture_2(0);
   // capture_1.release();
    //capture_1.set(CAP_PROP_FRAME_WIDTH, 1280);
   // capture_1.set(CAP_PROP_FRAME_HEIGHT, 720);
    while (1)
    {
         //buffer_insert(buffer,0x1234,0x4321);
        //write(fd,buffer,8);
        double fps;
        double t = 0;
        t = cv::getTickCount();
        //printf("state=%d\n\n\n",state);
        if(reset_int_flag)
        {
            reset_int_flag=0;
            state=STATE_RESET;
        }
        
        switch (state)
        {
        case STATE_TEST:
            capture_2 >> cam_image_2;
            capture_1 >> cam_image_1;
            imshow("img1",cam_image_1);
            imshow("img2",cam_image_2);
            break;
        case STATE_RESET:
            color_chose_num=0;
            state=STATE_INIT;
            digitalWrite(INIT_OK_PIN,1);
            break;
        case STATE_INIT:
            if(state_int_flag)
            {
                state_int_flag=0;
                state=STATE_QR_CODE_DETECT;
            }
            break;
        case STATE_QR_CODE_DETECT:                  //二维码识别阶段
            capture_2 >> cam_image_2;
            //capture_1 >> cam_image_1;
            if (!qrcode_read(cam_image_2, QR_CODE_Buffer))
            {
                printf("%s\n", QR_CODE_Buffer);
                if(strlen(QR_CODE_Buffer)==3)
                {
                    for(int i=0;i<3;i++)
                        color_order[i]=QR_CODE_Buffer[i]-'0';  
                    // QRCODE DETECT OVER
                    buffer_insert(buffer,0X5555,0xaaaa);
                    write(fd,buffer,8);
                    printf("QRCODE DETECT:");
                    for(int i=0;i<3;i++)
                        printf("%d ",color_order[i]);
                    printf("\n");
                    state=STATE_QR_CODE_COLOR_WAY;
                     capture_2.release();
                }
            }
            imshow("QRCODE",cam_image_2);
            break;
        case STATE_QR_CODE_COLOR_WAY:           
            if(state_int_flag)      //can CALIBRATION
            {

                printf("can CALIBRATION\n");
                capture_1.open(2);
                capture_1.set(CAP_PROP_FRAME_WIDTH, 1280);
                capture_1.set(CAP_PROP_FRAME_HEIGHT, 720);
                color_chose=QR_CODE_Buffer[0];
                state_int_flag=0;
                state=STATE_CALIBRATION_FIRST;
            }
        break;
        case STATE_CALIBRATION_FIRST:
        {
             if(state_int_flag)      //can detect color
            {
                printf("calibration ok\n");
                state_int_flag=0;
                color_chose=color_order[color_chose_num];
                state=STATE_CAL_TO_COLOR_WAY;
                break;
                
            }
            Mat src;
            capture_1 >> src;
            resize(src,src,Size(640,360));
            cam_image_1 = src(Range(0, 270-1), Range(0, 640-1));
            float x=0,y=0;
            if(cross_detect(&cam_image_1,&x,&y)==1)
            {
                printf("cross=%f,%f\n",x,y);
                int16_t cross_x=(int16_t)x-30;
                int16_t cross_y=(int16_t)y;
                buffer_insert_p(buffer,(uint16_t*)&cross_x,(uint16_t*)&cross_y);
                write(fd,buffer,8);
            }
                
        }
           
            
        break;
        case STATE_CAL_TO_COLOR_WAY:
            if(state_int_flag==1)
            {
                printf("can detect color\n");
                state_int_flag=0;
                state=STATE_COLOR_DETECT;
            }
        break;
        case STATE_COLOR_DETECT:                //物块识别
        {
            printf("color_detect\n");
            if(state_int_flag==1)
            { 
                state_int_flag=0;
                color_chose_num++;
                if(color_chose_num==5)      //over
                {
                    buffer_insert(buffer,0Xffff,0xffff);
                    write(fd,buffer,8);
                    state=STATE_GOOD_FETCH_OK_PUT_WAY;
                    break;
                }
                if(color_chose_num%2==0)
                {
                    color_chose=color_order[color_chose_num/2];
                }
                
            }
             if(color_chose_num%2==1)
            {
                printf("fetch_wait\n");
                break;
            }
            capture_1 >> cam_image_1;
            resize(cam_image_1,cam_image_1,Size(640,360));
            //cam_image_1 = imread("wukuai.jpeg");
            cvtColor(cam_image_1, image_hsv, COLOR_BGR2HSV, 3);        //HSV转换
            switch (color_chose)
            {
            case COLOR_RED:
                inRange(image_hsv, Scalar(0, 43, 46), Scalar(10, 255, 255), hsv_red1);
                inRange(image_hsv, Scalar(156, 43, 46), Scalar(180, 255, 255), hsv_red2);
                COLOR_R_IMAGE = hsv_red1 + hsv_red2;
                element = getStructuringElement(MORPH_RECT, Size(15, 15));
                morphologyEx(COLOR_R_IMAGE, COLOR_R_IMAGE, MORPH_OPEN, element);
                morphologyEx(COLOR_R_IMAGE, COLOR_R_IMAGE, MORPH_CLOSE, element);
                Canny(COLOR_R_IMAGE, image_canny, 50, 150, 3, false);
                break;
            case COLOR_GREEN:
                inRange(image_hsv, Scalar(68, 108, 46), Scalar(92, 255, 255), COLOR_G_IMAGE);
                element = getStructuringElement(MORPH_RECT, Size(15, 15));
                morphologyEx(COLOR_G_IMAGE, COLOR_G_IMAGE, MORPH_OPEN, element);
                morphologyEx(COLOR_G_IMAGE, COLOR_G_IMAGE, MORPH_CLOSE, element);
                Canny(COLOR_G_IMAGE, image_canny, 50, 150, 3, false);
                break;
            case COLOR_BLUE:
                inRange(image_hsv, Scalar(97, 112, 108), Scalar(112, 255, 255), COLOR_B_IMAGE);
                element = getStructuringElement(MORPH_RECT, Size(15, 15));
                morphologyEx(COLOR_B_IMAGE, COLOR_B_IMAGE, MORPH_OPEN, element);
                morphologyEx(COLOR_B_IMAGE, COLOR_B_IMAGE, MORPH_CLOSE, element);
                Canny(COLOR_B_IMAGE, image_canny, 50, 150, 3, false);
                break;
            }
            vector<vector<Point>> contours;
            findContours(image_canny, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
            int index = 0;
            float max_area = 0;
            Point2f vertices[4];
            float r_blue = 0;
            for (int i = 0; i < contours.size(); i++)
            {
                if (contourArea(contours[i]) > max_area)
                {
                    max_area = contourArea(contours[i]);
                    RotatedRect box = minAreaRect(contours[i]);//寻找最小包围矩形
                    box.points(vertices);
                    index = i;
                }
            }
            if (max_area > 10)
            {
                for (int i = 0; i < 4; ++i)
                {
                    line(cam_image_1, vertices[i % 4], vertices[(i + 1) % 4], Scalar(0, 0, 255), 2);
                }
                Point2f center_box;
                center_box.x = (vertices[0].x + vertices[2].x) / 2.0;
                center_box.y = (vertices[0].y + vertices[2].y) / 2.0;
                int16_t x=(int16_t)center_box.x-325;
                int16_t y=(int16_t)center_box.y-180;
                buffer_insert_p(buffer,(uint16_t*)&x,(uint16_t*)&y);
                write(fd,buffer,8);
                printf("%.0f,%.0f\n",center_box.x,center_box.y);
            }
            
            imshow("BLUE", image_canny);
            imshow("cam1", cam_image_1);
        }
            break;
        case STATE_GOOD_FETCH_OK_PUT_WAY:
            if(state_int_flag)
            {
                printf("can calibration second \n");
                state_int_flag=0;
                state=STATE_CALIBRATION_SECOND;
            }
            break;
         case STATE_CALIBRATION_SECOND:
         {
            if(state_int_flag)      //can detect color
            {
                printf("calibration second ok\n");
                state_int_flag=0;
                color_chose_num=0;
                state=STATE_CAL_TO_CIRCLE_WAY;
                break;
                
            }
            Mat src;
            capture_1 >> src;
            resize(src,src,Size(640,360));
            cam_image_1 = src(Range(0, 270-1), Range(0, 640-1));
            float x=0,y=0;
            if(cross_detect(&cam_image_1,&x,&y)==1)
            {
                printf("cross=%f,%f\n",x,y);
                int16_t cross_x=(int16_t)x-30;
                int16_t cross_y=(int16_t)y;
                buffer_insert_p(buffer,(uint16_t*)&cross_x,(uint16_t*)&cross_y);
                write(fd,buffer,8);
            }
         }
        break;
        case STATE_CAL_TO_CIRCLE_WAY:
        {
            if(state_int_flag==1)
            {
                printf("can detect CIRCLE\n");
                state_int_flag=0;
                state=STATE_CIRCLE_DETECT;
            }
        }
        break;
        case STATE_CIRCLE_DETECT:               //靶心识别
        {
            Mat src;
            if(state_int_flag==1)
            { 
                state_int_flag=0;
                color_chose_num++;
                if(color_chose_num==5)      //over
                {
                    buffer_insert(buffer,0Xffff,0xffff);
                    write(fd,buffer,8);
                    state=STATE_FINISH;
                    break;
                }
                if(color_chose_num%2==0)
                {
                    color_chose=color_order[2-color_chose_num/2];
                }
                
            }
             if(color_chose_num%2==1)
            {
                printf("put_wait\n");
                break;
            }
            capture_1 >> cam_image_1;
             resize(cam_image_1,cam_image_1,Size(640,360));
            //printf("%d,%d\n", src.rows, src.cols);
            //cam_image_1 = src(Range(0, 540 - 1), Range(0, 1280 - 1));
           // cam_image_1 = imread("ring4.jpg");
            cvtColor(cam_image_1, image_hsv, COLOR_BGR2HSV, 3);        //HSV转换
            switch (color_chose)
            {
            case COLOR_RED:
            {
                inRange(image_hsv, Scalar(0, 43, 46), Scalar(10, 255, 255), hsv_red1);
                inRange(image_hsv, Scalar(156, 43, 46), Scalar(180, 255, 255), hsv_red2);
                COLOR_R_IMAGE = hsv_red1 + hsv_red2;
                element = getStructuringElement(MORPH_RECT, Size(15, 15));
                //进行形态学操作
                morphologyEx(COLOR_R_IMAGE, COLOR_R_IMAGE, MORPH_CLOSE, element);
                morphologyEx(COLOR_R_IMAGE, COLOR_R_IMAGE, MORPH_OPEN, element);
                Canny(COLOR_R_IMAGE, image_canny, 50, 150, 3, false);
            }
            break;
            case COLOR_GREEN:
            {
                inRange(image_hsv,  Scalar(56, 32, 71), Scalar(104, 255, 255), COLOR_G_IMAGE);
                element = getStructuringElement(MORPH_RECT, Size(15, 15));
                //进行形态学操作
                morphologyEx(COLOR_G_IMAGE, COLOR_G_IMAGE, MORPH_CLOSE, element);
                morphologyEx(COLOR_G_IMAGE, COLOR_G_IMAGE, MORPH_OPEN, element);
                Canny(COLOR_G_IMAGE, image_canny, 50, 150, 3, false);
                break;
            }
            case COLOR_BLUE:
            {
                inRange(image_hsv,Scalar(106, 83, 79), Scalar(143, 255, 255), COLOR_B_IMAGE);
                element = getStructuringElement(MORPH_RECT, Size(15, 15));
                //进行形态学操作
                morphologyEx(COLOR_B_IMAGE, COLOR_B_IMAGE, MORPH_CLOSE, element);
                morphologyEx(COLOR_B_IMAGE, COLOR_B_IMAGE, MORPH_OPEN, element);
                Canny(COLOR_B_IMAGE, image_canny, 50, 150, 3, false);
                break;
            }
            }
            vector<vector<Point>> contours;
            findContours(image_canny, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
            int index=0;
            float max_area = 0;
            Point2f center;
            float r=0;
            for (int i = 0; i < contours.size(); i++)
            {
                if (contourArea(contours[i]) > max_area)
                {
                    max_area = contourArea(contours[i]);
                    minEnclosingCircle(contours[i], center, r);
                    index = i;
                }
            }
            if (max_area > 10)
            {
                circle(cam_image_1, center, r, Scalar(255, 255, 0), 3);  //圈出靶心
                circle(cam_image_1, center, 2, Scalar(0, 0, 255), 3);
                int16_t x=(int16_t)center.x-325;
                int16_t y=(int16_t)center.y-100;
                buffer_insert_p(buffer,(uint16_t*)&x,(uint16_t*)&y);
                write(fd,buffer,8);
                printf("%.0f,%.0f\n",center.x,center.y);
            }
            
		    imshow("cam1", cam_image_1);
            imshow("CIRCLE_DETECT", image_canny);
            
        }
            break;
        case STATE_FINISH:
            printf("over\n");
        break;
        case CROSS_DETECT:
        {
            Mat src;
            capture_1 >> src;
            resize(src,src,Size(640,360));
            cam_image_1 = src(Range(0, 270-1), Range(0, 640-1));
            //将图像转换为灰度图
            cvtColor(cam_image_1, image_gray, COLOR_BGR2GRAY);
            //阈值二值化
            threshold(image_gray, image_gray, 100, 255, 0);
            element = getStructuringElement(MORPH_RECT, Size(5, 5));
            //进行形态学操作
            //morphologyEx(image_gray, image_gray, MORPH_OPEN, element);
            //边缘检测
            Canny(image_gray, image_canny, 50, 150, 3, false);
            vector<Vec4i> lines;//定义一个矢量结构lines用于存放得到的线段矢量集合
            HoughLinesP(image_canny, lines, 1, CV_PI / 180, 80, 100, 100);

            
            if (lines.size() <= 50|| lines.size()==0)
            {
                
                //【4】依次在图中绘制出每条线段
                for (size_t i = 0; i < lines.size(); i++)
                {
                    Vec4i l = lines[i];
                    //此句代码的OpenCV3版为：
                    line(cam_image_1, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1, LINE_AA);
                    angle[i] = cal_line_angle(l);
                    dis[i] = getDist_P2L(Point2d(0, 0), Point(l[0], l[1]), Point(l[2], l[3]));
                    printf("angle=%f,dis=%f\n", angle[i], dis[i]);
                }
                printf("\n\n");
                float dis_x[10]={0};
                float dis_y[10]={0};
                int num_x=0;
                int num_y=0;
                for(size_t i = 0; i < lines.size(); i++)
                {
                    if(fabs(angle[i])<10)
                    {
                        if(num_x>=10)
                            break;
                        dis_x[num_x++]=dis[i];
                    }
                    if(fabs(angle[i])>80)
                    {
                        if(num_y>=10)
                            break;
                        dis_y[num_y++]=dis[i];
                    }

                }
                if(num_x>=2&&num_y>=2)
                {
                    for(int i=0;i<num_x;i++)
                    {
                        for(int j=i;j<num_x;j++)
                        {
                            if(dis_x[j]<dis_x[i])
                            {
                                float temp=dis_x[j];
                                dis_x[j]=dis_x[i];
                                dis_x[i]=temp;
                            }
                        }
                        for(int j=i;j<num_y;j++)
                        {
                            if(dis_y[j]<dis_y[i])
                            {
                                float temp=dis_y[j];
                                dis_y[j]=dis_y[i];
                                dis_y[i]=temp;
                            }
                        }
                            
                    }
                    int jianduan_x[10]={0},jianduan_y[10]={0};
                    int jianduan_x_num=0,jianduan_y_num=0;
                    for(int i=0;i<num_x-1;i++)
                    {
                        printf("dis_x:%f\n",dis_x[i]);
                        if(dis_x[i+1]-dis_x[i]>10)
                            jianduan_x[jianduan_x_num++]=i;

                    }
                    for(int i=0;i<num_y-1;i++)
                    {
                        printf("dis_y:%f\n",dis_y[i]);
                        if(dis_y[i+1]-dis_y[i]>10)
                            jianduan_y[jianduan_y_num++]=i;
                    }
                    if(jianduan_x_num==1&&jianduan_y_num==1)
                    {
                        printf("cross\n\n\n");
                        float dis_sum_x_left=0;
                        float dis_sum_x_right=0;
                        float dis_sum_y_up=0;
                        float dis_sum_y_down=0;

                        for(int i=0;i<=jianduan_x[0];i++)
                        {
                            dis_sum_x_left+=dis_x[i];
                        }
                        for(int i=0;i<=jianduan_y[0];i++)
                        {
                            dis_sum_y_up+=dis_y[i];
                        }
                        for(int i=jianduan_x[0]+1;i<num_x;i++)
                        {
                            dis_sum_x_right+=dis_x[i];
                        }
                        for(int i=jianduan_y[0]+1;i<num_y;i++)
                        {
                            dis_sum_y_down+=dis_y[i];
                        }
                        float x1=dis_sum_x_left/(jianduan_x[0]+1);
                        float x2=dis_sum_x_right/(num_x-jianduan_x[0]-1);
                        float y1=dis_sum_y_up/(jianduan_y[0]+1);
                        float y2=dis_sum_y_down/(num_y-jianduan_y[0]-1);
                        printf("x1=%f,x2=%f,y1=%f,y2=%f\n\n",x1,x2,y1,y2);
                        printf("cross:%f,%f\n",(y1+y2)/2-325,(x1+x2)/2-100);
                    }
                    printf("hhh");
                }
            
            }
            else
            {
                if(lines.size()>50)
                    printf("error:too much line!!\n");
                else
                    printf("error:no line found!!\n");
            }
        
            imshow("cross",cam_image_1);
        } 
        break;
        case STATE_YAW_DETECT:                  //直线识别，角度
        {
            Mat src;
            capture_1 >> src;
            //src = imread("saidao1.jpg");
            //printf("%d,%d\n", src.rows, src.cols);
            cam_image_1 = src(Range(0, 520-1), Range(0, 1280-1));
            
            //将图像转换为灰度图
            cvtColor(cam_image_1, image_gray, COLOR_BGR2GRAY);
            //阈值二值化
            threshold(image_gray, image_gray, 100, 255, 0);
            element = getStructuringElement(MORPH_RECT, Size(5, 5));
            //进行形态学操作
            //morphologyEx(image_gray, image_gray, MORPH_OPEN, element);
            //边缘检测
            Canny(image_gray, image_canny, 50, 150, 3, false);
            vector<Vec4i> lines;//定义一个矢量结构lines用于存放得到的线段矢量集合
            HoughLinesP(image_canny, lines, 1, CV_PI / 180, 80, 200, 100);

            if (lines.size() <= 50|| lines.size()==0)
            {
                
                //【4】依次在图中绘制出每条线段
                for (size_t i = 0; i < lines.size(); i++)
                {
                    Vec4i l = lines[i];
                    //此句代码的OpenCV3版为：
                    line(cam_image_1, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1, LINE_AA);
                    angle[i] = cal_line_angle(l);
                    dis[i] = getDist_P2L(Point2d(0, 0), Point(l[0], l[1]), Point(l[2], l[3]));
                   // printf("angle=%f,dis=%f\n", angle[i], dis[i]);
                }
                uint8_t x_pass=0,y_pass=0;
                for (size_t i = 0; i < lines.size(); i++)
                {
                    if (abs(angle[i]) < 15 && dis[i] < 540 && dis[i]>440)
                    {
                        x_pass=1;
                        printf("检测到横线\n");
                    }
                    if (abs(angle[i]) <= 90 && abs(angle[i]) >= 75 && dis[i] < 690 && dis[i]>590)
                    {
                        y_pass=1;
                        //printf("检测到竖线\n");
                    }
                }
                //digitalWrite(INIT_OK_PIN,x_pass);
            
            }
            else
            {
                if(lines.size()>50)
                    printf("error:too much line!!\n");
                else
                    printf("error:no line found!!\n");
            }
            
            imshow("YAW_DETECT", image_gray);
            printf("\n\n");
           
        }
            
            break;
        default:
            break;

        }
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        fps = 1.0 / t;
        char f[20];
        sprintf(f, "fps=%.2f", fps);
        //putText(cam_image_1, f, Point(20, 30), cv::FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255), 2);
        
        
        //imshow("cam1", cam_image_1);
        //imshow("cam2", cam_image_2);
        waitKey(1);

    }

}


