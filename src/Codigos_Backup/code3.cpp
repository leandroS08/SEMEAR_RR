#include <iostream>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <ros/ros.h>
#include "std_msgs/Float32.h"

using namespace cv;
using namespace std;

#define PI 3.1415926

char* source_window = "Original Video";
char* result_window = "Result Video";

void bird_Eyes(Mat&, Mat&); 

int iLowV = 0;
int iHighV = 255;
void select_Channel(Mat&, Mat&, int, int);

int right_x;
void histogram_Right(Mat&);

/* Variáveis das funções de sliding_Window */
int h_rectangle = 25;
int l_rectangle = 40;
int num_rectangle = 20;

vector<Point2f> right_Line;
vector<Point2i> iright_Line;
void sliding_Window_Right(Mat&, Mat&);

int left_x;
int previous_left_x = -1;
void histogram_Left(Mat&);

vector<Point2f> left_Line;
vector<Point2i> ileft_Line;
int control_left[20];
void sliding_Window_Left(Mat&, Mat&);

vector<Point2f> navegavel;
vector<Point2i> navegavel_int;
void navegation_Zone(Mat&, Mat&);

vector<Point2f> final_navegavel;
vector<Point2i> final_navegavel_int;
void inverse_bird_Eyes(Mat&, Mat&, Mat&);

vector <Point2i> central_line;
void central_Line(Mat&);

float shift_Central(Mat&);

int main( int argc, char** argv )
{
    ros::init(argc, argv, "shift_central");
    ros::NodeHandle nh;
    ros::Publisher vision_pub = nh.advertise<std_msgs::Float32>("shift_central_topic", 100);
    std_msgs::Float32 vision_msg;
    float shift_result;

    Mat src; // Frame original lido da câmera
    Mat bird_img; // Bird Eyes Transformation
    Mat color_img; // Manipulação dos canais de cor
    Mat sliding_img; //
    Mat navegation_img; 
    Mat result_img;

    /* Abertura do vídeo */ 
    //char* videoName = argv[1];
    VideoCapture cap("/home/leandro/catkin_ws/src/robocar_race/src/road2.mp4");

    if ( !cap.isOpened() )
    {
        cout << "Erro ao abrir o video" << endl;
        return -1;
    }

    /* Janelas */
    namedWindow(source_window, CV_WINDOW_NORMAL);
    namedWindow(result_window, CV_WINDOW_NORMAL);

    for(int j = 0; j<num_rectangle; j++)
        control_left[j] = -1;

    while ( (waitKey(33) != 27) && (cap.isOpened()) )
    {
        cap.read(src);

        bird_Eyes(src, bird_img);

        select_Channel(bird_img, color_img, iLowV, iHighV);

        histogram_Right(color_img);

        sliding_img = bird_img.clone();
        sliding_Window_Right(color_img, sliding_img);
        
        histogram_Left(color_img);

        sliding_Window_Left(color_img, sliding_img);

        navegation_Zone(sliding_img,navegation_img);

        inverse_bird_Eyes(src, bird_img, result_img);

        central_Line(result_img);

        shift_result = shift_Central(result_img);
        vision_msg.data = shift_result;
        ROS_INFO_STREAM(vision_msg.data);

        vision_pub.publish(vision_msg);

        imshow(source_window, src);
        imshow(result_window, result_img);

        right_Line.clear();
        iright_Line.clear();
        left_Line.clear();
        ileft_Line.clear();
        navegavel.clear();
        navegavel_int.clear();
        final_navegavel.clear();
        final_navegavel_int.clear();
        central_line.clear();

        //waitKey(0);
    }

    waitKey(0);
}

void bird_Eyes(Mat& in, Mat& out)
{
    Mat tr; // variável de manipulação interna da função

    int Rows = in.rows;
    int Cols = in.cols;

    Point2f src_vertices[4];
    src_vertices[0] = Point(        0, 0.70*Rows); // A
    src_vertices[1] = Point(0.10*Cols, 0.10*Rows); // B
    src_vertices[2] = Point(0.10*Cols, 0.90*Rows); // C
    src_vertices[3] = Point(     Cols, 0.70*Rows); // D

    Point2f dst_vertices[4];
    dst_vertices[0] = Point(  0, 480);
    dst_vertices[1] = Point(  0,   0);
    dst_vertices[2] = Point(640,   0);
    dst_vertices[3] = Point(640, 480);

    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    tr = Mat(480, 640, CV_8UC3);
    warpPerspective(in, tr, M, tr.size(), INTER_LINEAR, BORDER_CONSTANT);

    out = tr;
    
    /*char* bird_window = "Bird Eyes Transformation";
    namedWindow(bird_window, CV_WINDOW_NORMAL);
    imshow(bird_window, out);*/
}

void select_Channel(Mat& in, Mat& out, int low, int high)
{
    Mat tr;

    GaussianBlur(in,tr,Size(3,3),0,0,BORDER_DEFAULT);
    cvtColor(tr,tr,CV_RGB2HLS); // conversão para HLS

    vector<Mat> hls_planes; 
    split( tr, hls_planes ); // separação dos canais

    /*namedWindow("H channel", CV_WINDOW_NORMAL);
    namedWindow("S channel", CV_WINDOW_NORMAL);
    namedWindow("L channel", CV_WINDOW_NORMAL);
    imshow("H channel", hls_planes[0]);
    imshow("S channel", hls_planes[1]);
    imshow("L channel", hls_planes[2]);*/

    //inRange(hls_planes[1], Scalar(low), Scalar(high), out); // selecão do canal H
    inRange(hls_planes[1], 72, 194, tr); // selecão do canal H
    bitwise_not ( tr, tr );

    out = tr;

    /*char* color_window = "Selecao do Canal e das Intensidades de Cor";
    namedWindow(color_window, CV_WINDOW_NORMAL);
    imshow(color_window, out);*/
}

void histogram_Right(Mat& in)
{
    Mat histImage(in.rows, in.cols, CV_8UC3, Scalar(255,255,255) );

    int count[in.cols];
    int count_Num = 0;
    int count_Den = 0;
    //int normalized_count[in.cols];
    float inicial_Row = in.rows * 0.9;

    for (int col = (in.cols)/2; col < in.cols; ++col)
    {
        count[col]=0;
        for (int row = inicial_Row; row < in.rows; ++row)
        {
            if ( (int)(in.at<uchar>(row,col)) != 0)
            {
                ++count[col];
                count_Num+=(col*count[col]);
                count_Den+=count[col];
            }
                
        }
        //cout<<"A coluna  " << col << "  tem   " << count[col] << endl;
        //normalize(count[col], normalized_count[col], 0, in.rows, NORM_MINMAX, -1, Mat() );
        circle(histImage, Point(col, in.rows - count[col]),1,Scalar(0,255,255),1,8,0);
    }

    if (count_Den != 0)
        right_x = count_Num / count_Den;
    

    /*cout << " X da linha direita " << right_x << endl;
    waitKey(0);*/

    char* histogram_window = "Histogram";
    namedWindow(histogram_window, CV_WINDOW_NORMAL);
    imshow(histogram_window, histImage);
}

void sliding_Window_Right(Mat& in, Mat& out)
{
    // Desenha o quadrado inicial resultante do histograma
    int y_Reference = in.rows;
    Point2d initial_Center = Point(right_x, y_Reference);
    Point2d final_Center;

    Point P1, P2; // pontos auxiliares para desenho dos retângulos
    P1.x = right_x - (l_rectangle/2);
    P1.y = y_Reference - (h_rectangle/2);
    P2.x = right_x + (l_rectangle/2);
    P2.y = y_Reference;
    rectangle(out, P1, P2, Scalar(255,255,0), 2, 8, 0);

    Point2d previous_Center = initial_Center;
    Point2d new_Center;
    int count[l_rectangle];
    int count_Num;
    int count_Den;

    for (int i=0; i<num_rectangle; i++)
    {
        count_Num = 0;
        count_Den = 0;

        new_Center.x = previous_Center.x;
        new_Center.y = previous_Center.y - (h_rectangle/2);

        P1.x = new_Center.x - (l_rectangle/2);
        P1.y = new_Center.y - (h_rectangle/2);
        P2.x = new_Center.x + (l_rectangle/2);
        P2.y = new_Center.y;

        for (int col = P1.x ; col <= (P1.x + l_rectangle); col++)
        {
            count[col-P1.x] = 0;
            for (int row = P1.y ; row <= (P1.y + h_rectangle); row++)
            {
                if ( (int)(in.at<uchar>(row,col)) != 0)
                {
                    ++count[col-P1.x];
                    count_Num+=(col*count[col-P1.x]);
                    count_Den+=count[col-P1.x];
                } 
            }
            //cout << "A coluna " << col << "(indice " << col-P1.x << " ) tem  " << count[col-P1.x] << endl;
        }

        if (count_Den != 0)
            new_Center.x = count_Num / count_Den;

        P1.x = new_Center.x - (l_rectangle/2);
        P1.y = new_Center.y - (h_rectangle/2);
        P2.x = new_Center.x + (l_rectangle/2);
        P2.y = new_Center.y;

        right_Line.push_back(new_Center);
        iright_Line.push_back(new_Center);

        rectangle(out, P1, P2, Scalar(0,0,255), 2, 8, 0);

        previous_Center = new_Center;
    }
}

void histogram_Left(Mat& in)
{
    Mat histImage(in.rows, in.cols, CV_8UC3, Scalar(255,255,255) );

    int count[in.cols];
    int count_Num = 0;
    int count_Den = 0;
    //int normalized_count[in.cols];
    float inicial_Row = in.rows * 0.9;
    float shift_Row = in.rows * 0.1;

    for (int col = 0; col < (in.cols)/2; ++col)
    {
        count[col]=0;

        for (int i=0; (i<10) && (count_Den < 10); i++)
        {
            for (int row = inicial_Row - (i*shift_Row); row < in.rows - (i*shift_Row); ++row)
            {
                if ( (int)(in.at<uchar>(row,col)) != 0)
                {
                    ++count[col];
                    count_Num+=(col*count[col]);
                    count_Den+=count[col];
                }
            }
        }
        circle(histImage, Point(col, in.rows - count[col]),1,Scalar(0,255,255),1,8,0);
    }

    //normalize(count[col], normalized_count[col], 0, in.rows, NORM_MINMAX, -1, Mat() );

    if ( ((count_Den > 10) || (previous_left_x == -1)) && (count_Den != 0) )
    {
        left_x = count_Num / count_Den;
        previous_left_x = left_x;
    }
    else   
        left_x = previous_left_x;

    char* histogram_window = "Histogram 2";
    namedWindow(histogram_window, CV_WINDOW_NORMAL);
    imshow(histogram_window, histImage);
}

void sliding_Window_Left(Mat& in, Mat& out)
{
    // Desenha o quadrado inicial resultante do histograma
    int y_Reference = in.rows;
    Point2d initial_Center = Point(left_x, y_Reference);
    Point2d final_Center;

    Point P1, P2; // pontos auxiliares para desenho dos retângulos
    P1.x = left_x - (l_rectangle/2);
    P1.y = y_Reference - (h_rectangle/2);
    P2.x = left_x + (l_rectangle/2);
    P2.y = y_Reference;
    rectangle(out, P1, P2, Scalar(255,255,0), 2, 8, 0);

    Point2d previous_Center = initial_Center;
    Point2d new_Center;
    int count[l_rectangle];
    int count_Num;
    int count_Den;

    for (int i=0; i<num_rectangle; i++)
    {
        count_Num = 0;
        count_Den = 0;

        new_Center.x = previous_Center.x;
        new_Center.y = previous_Center.y - (h_rectangle/2);

        P1.x = new_Center.x - (l_rectangle/2);
        P1.y = new_Center.y - (h_rectangle/2);
        P2.x = new_Center.x + (l_rectangle/2);
        P2.y = new_Center.y;

        for (int col = P1.x ; col <= (P1.x + l_rectangle); col++)
        {
            count[col-P1.x] = 0;
            for (int row = P1.y ; row <= (P1.y + h_rectangle); row++)
            {
                if ( (int)(in.at<uchar>(row,col)) != 0)
                {
                    ++count[col-P1.x];
                    count_Num+=(col*count[col-P1.x]);
                    count_Den+=count[col-P1.x];
                } 
            }
            //cout << "A coluna " << col << "(indice " << col-P1.x << " ) tem  " << count[col-P1.x] << endl;
        }

        //cout << " O quadrado " << i << " tem " << count_Den << " pontos " << endl;
        if ( (count_Den > 10) || (control_left[i] == -1) )
        {
            if (count_Den != 0)
                new_Center.x = count_Num / count_Den;

            control_left[i] = new_Center.x;
        }
        else
        {
            new_Center.x = control_left[i];
            //cout << "Centro anterior [ " << i << " ]: ( " << previous_left_Line[i].x << " , " << previous_left_Line[i].y << " ) " << endl;
            //cout << "Centro anterior [ " << i << " ]: " << previous_left_Line[1].x << endl;
        }

        left_Line.push_back(new_Center);
        ileft_Line.push_back(new_Center); 

        P1.x = new_Center.x - (l_rectangle/2);
        P1.y = new_Center.y - (h_rectangle/2);
        P2.x = new_Center.x + (l_rectangle/2);
        P2.y = new_Center.y;

        rectangle(out, P1, P2, Scalar(0,100,255), 2, 8, 0);

        previous_Center = new_Center;
    }

    char* sliding_window = "Sliding Window";
    namedWindow(sliding_window, CV_WINDOW_NORMAL);
    imshow(sliding_window, out);
}

void navegation_Zone(Mat& in, Mat& out)
{
    out = in;

    for (int i=0; i<right_Line.size(); i++)
    {
        navegavel.push_back(right_Line[i]);
        navegavel_int.push_back(right_Line[i]);
    }
        
    for (int j=left_Line.size()-1; j>=0; j--)
    {
        navegavel.push_back(left_Line[j]);
        navegavel_int.push_back(left_Line[j]);
    }
        
    polylines(out,navegavel_int,1,Scalar(255,0,0),2,8,0);

    /*Mat draw(in.rows, in.cols, CV_8UC3, Scalar(0,0,0) );
    fillPoly(draw, right_Line, Scalar (255, 0, 0), 8);
    char* draw_window = "Regiao Navegavel";
    namedWindow(draw_window, CV_WINDOW_NORMAL);
    imshow(draw_window, draw);*/

    char* navegation_window = "Regiao Navegavel";
    namedWindow(navegation_window, CV_WINDOW_NORMAL);
    imshow(navegation_window, out);
}

void inverse_bird_Eyes(Mat& src, Mat& in, Mat& out)
{
    out = src.clone();

    int Rows = src.rows;
    int Cols = src.cols;

    Point2f src_vertices[4];
    src_vertices[0] = Point(        0,      Rows); // A
    src_vertices[1] = Point(0.30*Cols, 0.70*Rows); // B
    src_vertices[2] = Point(0.70*Cols, 0.70*Rows); // C
    src_vertices[3] = Point(     Cols,      Rows); // D

    Point2f dst_vertices[4];
    dst_vertices[0] = Point(  0, 480);
    dst_vertices[1] = Point(  0,   0);
    dst_vertices[2] = Point(640,   0);
    dst_vertices[3] = Point(640, 480);

    Mat M = getPerspectiveTransform(dst_vertices, src_vertices);

    perspectiveTransform(navegavel, final_navegavel, M);

    for(int i=0 ; i<navegavel.size() ; i++)
    {
        final_navegavel_int.push_back(Point(final_navegavel[i].x, final_navegavel[i].y));
        /*cout << "Ponto inicial: ( " << right_Line[i].x << " ; " << right_Line[i].y << " ) " << endl;
        cout << "Ponto navegavel:   ( " << navegavel_int[i].x << " ; " << navegavel_int[i].y << " ) " << endl;*/
    }

    polylines(out,final_navegavel_int,1,Scalar(255,0,0),3,8,0);
}

void central_Line(Mat& in)
{
    int x_add, y_add;

    for(int i=0; i<final_navegavel.size(); i++)
    {
        y_add = (int) final_navegavel[i].y;
        x_add = ( (int) final_navegavel[i].x + (int) final_navegavel[(num_rectangle*2)-i-1].x ) / 2;
        central_line.push_back(Point(x_add,y_add));
    }

    polylines(in,central_line,0,Scalar(0,255,255),2,8,0);
}

float shift_Central(Mat& in)
{
    float frame_center = in.cols / 2;
    float shift;
    float shift_num = 0;
    float shift_den = 0;

    line(in,Point(frame_center,0),Point(frame_center,in.rows),Scalar(0,255,0),3,8,0);

    for(int i=0; i < central_line.size(); i++)
    {
        shift_num  += frame_center - central_line[i].x;
        shift_den++;
    }

    shift = shift_num/shift_den;

    cout << "Deslocamento da linha central :" << shift << endl;

    return shift;
}


