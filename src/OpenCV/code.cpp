#include <iostream>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

#define PI 3.1415926

char* source_window = "Original Video";
char* result_window = "Result Video";

void bird_Eyes(Mat&, Mat&); 

Mat transformationMat;
Mat trans_mat;
void bird_Eyes_2(Mat&, int, int, int, int, int, Mat&);

int iLowV = 0;
int iHighV = 255;
void select_Channel(Mat&, Mat&, int, int);

int right_x;
void histogram_Right(Mat&);

/* Variáveis das funções de sliding_Window */
int h_rectangle = 50;
int l_rectangle = 100;
int num_rectangle = 20;

vector<Point2f> right_Line;
vector<Point2i> iright_Line;
void sliding_Window_Right(Mat&, Mat&);

Point2i button_left;
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
void inverse_bird_Eyes_2(Mat&, Mat&, Mat&);

vector <Point2i> central_line;
void central_Line(Mat&);

float shift_Central(Mat&);

int main( int argc, char** argv )
{
    Mat src; // Frame original lido da câmera
    Mat tr1;
    Mat bird_img; // Bird Eyes Transformation
    Mat color_img; // Manipulação dos canais de cor
    Mat sliding_img; //
    Mat navegation_img; 
    Mat result_img;

    /* Abertura do vídeo */ 
    char* videoName = argv[1];
    VideoCapture cap(videoName);
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

        tr1 = src;

        bird_Eyes_2(tr1, 40, 90, 90, 430, 600, bird_img);

        bird_img = src;

        select_Channel(bird_img, color_img, iLowV, iHighV);

        histogram_Right(color_img);

        sliding_img = bird_img.clone();
        sliding_Window_Right(color_img, sliding_img);
        
        histogram_Left(color_img);

        sliding_Window_Left(color_img, sliding_img);

        navegation_Zone(sliding_img,navegation_img);

        //inverse_bird_Eyes(src, bird_img, result_img);

        //inverse_bird_Eyes_2(src, bird_img, result_img);

        central_Line(result_img);

        shift_Central(result_img);

        imshow(source_window, src);
        imshow(result_window, sliding_img);

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
    src_vertices[0] = Point(        0, 0.90*Rows); // 
    src_vertices[1] = Point(0.30*Cols, 0.20*Rows); // 
    src_vertices[2] = Point(0.70*Cols, 0.20*Rows); // 
    src_vertices[3] = Point(     Cols, 0.90*Rows); // 

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

void bird_Eyes_2(Mat& in, int alpha_, int beta_, int gamma_, int f_, int dist_, Mat& out)
{
    //int frameWidth = ;
    //int frameHeight = ;

    int offsetx = 0;
    int offsety = -175;

    trans_mat = (Mat_<double>(2,3) << 1, 0, offsetx, 0, 1, offsety);
    warpAffine(in,out,trans_mat,in.size());

    double focalLength, dist, alpha, beta, gamma; 

    alpha =((double)alpha_ -90) * PI/180;
    beta =((double)beta_ -90) * PI/180;
    gamma =((double)gamma_ -90) * PI/180;
    focalLength = (double)f_;
    dist = (double)dist_;

    Size image_size = in.size();
    double w = (double)image_size.width, h = (double)image_size.height;

    // Projecion matrix 2D -> 3D
    Mat A1 = (Mat_<float>(4, 3)<< 
        1, 0, -w/2,
        0, 1, -h/2,
        0, 0, 0,
        0, 0, 1 );

    // Rotation matrices Rx, Ry, Rz
    Mat RX = (Mat_<float>(4, 4) << 
        1, 0, 0, 0,
        0, cos(alpha), -sin(alpha), 0,
        0, sin(alpha), cos(alpha), 0,
        0, 0, 0, 1 );

    Mat RY = (Mat_<float>(4, 4) << 
        cos(beta), 0, -sin(beta), 0,
        0, 1, 0, 0,
        sin(beta), 0, cos(beta), 0,
        0, 0, 0, 1  );

    Mat RZ = (Mat_<float>(4, 4) << 
        cos(gamma), -sin(gamma), 0, 0,
        sin(gamma), cos(gamma), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1  );

    // R - rotation matrix
    Mat R = RX * RY * RZ;

    // T - translation matrix
    Mat T = (Mat_<float>(4, 4) << 
        1, 0, 0, 0,  
        0, 1, 0, 0,  
        0, 0, 1, dist,  
        0, 0, 0, 1); 

    // K - intrinsic matrix 
    Mat K = (Mat_<float>(3, 4) << 
        focalLength, 0, w/2, 0,
        0, focalLength, h/2, 0,
        0, 0, 1, 0
        ); 

    transformationMat = K * (T * (R * A1));

    warpPerspective(in, out, transformationMat, image_size, INTER_CUBIC | WARP_INVERSE_MAP);

    Mat deslocamento = (Mat_<double>(2,3) << 1, 0, 0, 
	                                         0, 1, (0.05*out.cols));

    warpAffine(out,out,deslocamento,out.size());

    char* bird_window = "Bird Eyes Transformation";
    namedWindow(bird_window, CV_WINDOW_NORMAL);
    imshow(bird_window, out);
}

void select_Channel(Mat& in, Mat& out, int low, int high)
{
    Mat tr;
    Mat imgThresholded;

    GaussianBlur(in,tr,Size(3,3),0,0,BORDER_DEFAULT);
    cvtColor(tr,tr,CV_RGB2HSV); // conversão para HLS

    inRange(tr, Scalar(0, 75, 135), Scalar(145, 255, 208), imgThresholded); //Threshold the image
      
    //morphological opening (remove small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

    //morphological closing (fill small holes in the foreground)
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    /*namedWindow("H channel", CV_WINDOW_NORMAL);
    namedWindow("S channel", CV_WINDOW_NORMAL);
    namedWindow("L channel", CV_WINDOW_NORMAL);
    imshow("H channel", hls_planes[0]);
    imshow("S channel", hls_planes[1]);
    imshow("L channel", hls_planes[2]);*/

    //inRange(hls_planes[1], Scalar(low), Scalar(high), out); // selecão do canal H
    //inRange(hls_planes[1], 30, 180, tr); // selecão do canal L
    //bitwise_not ( imgThresholded, imgThresholded );

    out = imgThresholded;

    char* color_window = "Selecao do Canal e das Intensidades de Cor";
    namedWindow(color_window, CV_WINDOW_NORMAL);
    imshow(color_window, out);
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

    /*char* histogram_window = "Histogram";
    namedWindow(histogram_window, CV_WINDOW_NORMAL);
    imshow(histogram_window, histImage);*/
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

    button_left.y = inicial_Row + shift_Row;

    //normalize(count[col], normalized_count[col], 0, in.rows, NORM_MINMAX, -1, Mat() );

    if ( ( (count_Den > 10) || (previous_left_x == -1))  && (count_Den != 0) )
    {
        button_left.x = count_Num / count_Den;
        previous_left_x = button_left.x;
    }
    else   
        button_left.x = previous_left_x;

    /*char* histogram_window = "Histogram 2";
    namedWindow(histogram_window, CV_WINDOW_NORMAL);
    imshow(histogram_window, histImage);*/
}

void sliding_Window_Left(Mat& in, Mat& out)
{
    // Desenha o quadrado inicial resultante do histograma
    Point2d initial_Center = Point(button_left.x, button_left.y);
    Point2d final_Center;

    Point P1, P2; // pontos auxiliares para desenho dos retângulos
    P1.x = button_left.x - (l_rectangle/2);
    P1.y = button_left.y - (h_rectangle/2);
    P2.x = button_left.x + (l_rectangle/2);
    P2.y = button_left.y;
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

void inverse_bird_Eyes_2(Mat& src, Mat& in, Mat& out)
{
    out = src.clone();

    perspectiveTransform(navegavel, final_navegavel, transformationMat.inv());

    for(int i=0 ; i<navegavel.size() ; i++)
    {
        final_navegavel_int.push_back(Point(final_navegavel[i].x, final_navegavel[i].y));
        /*cout << "Ponto inicial: ( " << right_Line[i].x << " ; " << right_Line[i].y << " ) " << endl;
        cout << "Ponto navegavel:   ( " << navegavel_int[i].x << " ; " << navegavel_int[i].y << " ) " << endl;*/
    }

    //warpAffine(in,out,trans_mat.inv(),in.size());

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

    //cout << "Deslocamento da linha central :" << shift << endl;
}


