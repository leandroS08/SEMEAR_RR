/* Próximos passos:
        - (+-) Fazer o sliding window para a linha pontilhada central
        - (X) Armazenar os pontos centrais dos retangulos do sliding window em um vetor
        - (X) Formar um polígono com todos os pontos adquiridos
        - (X) Achar a linha central desse polígono
        - (X) Retornar para a base original (inverso do bird eyes)
        - (X) Calcular o deslocamento em relação ao centro da câmera
*/

#include <iostream>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

#define PI 3.1415926

char* source_window = "Original Video";
char* result_window = "Result Video";

int h_rectangle = 25;
int l_rectangle = 40;
int num_rectangle = 20;
int loop_counter = 0;
int teste;
int left_x, right_x;
Point aux;
vector<Point2f> right_Line;
vector<Point2i> right_Line_int;
vector<Point2f> navegavel;
vector<Point2i> navegavel_int;
vector <Point2i> central_line;
//vector <float> pts_metros;
float pixel_metrs;

void bird_Eyes(Mat&, Mat&);

void select_Channel(Mat&, Mat&, int, int);

void histogram(Mat&);

void sliding_Window(Mat&, int, Mat&);

void sliding_Recursive(Mat&, Point, Mat&);

void navegation_Zone(Mat&, Mat&);

void inverse_bird_Eyes(Mat&, Mat&, Mat&);

void central_Line(Mat&);

void calc_pixel_metrs(Mat&);

float shift_Central(Mat&);


int main( int argc, char** argv )
{
    Mat src; // Frame original lido da câmera
    Mat bird_img; // Bird Eyes Transformation
    Mat color_img; // Canal isolado e valores de cores selecionados
    Mat sliding_img;
    Mat navegation_img;
    Mat result_img;

    VideoCapture cap("road.mp4");
    if ( !cap.isOpened() )
    {
        cout << "Erro ao abrir o video" << endl;
        return -1;
    }

    /* Janelas */
    namedWindow(source_window, CV_WINDOW_NORMAL);
    //namedWindow(result_window, CV_WINDOW_NORMAL);

    int iLowV = 0;
    int iHighV = 255;

    /*cvCreateTrackbar("LowV", color_window, &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", color_window, &iHighV, 255);*/

    while ( (waitKey(33) != 27) && (cap.isOpened()) )
    {
        cap.read(src);
        
        bird_Eyes(src, bird_img);

        select_Channel(bird_img, color_img, iLowV, iHighV);

        histogram(color_img);

        sliding_img = bird_img.clone();
        sliding_Window(color_img, left_x, sliding_img);
        sliding_Window(color_img, right_x, sliding_img);

        for(int i=0;i<num_rectangle/2;i++){
            aux = right_Line[i];
            right_Line[i] = right_Line[num_rectangle-1-i];
            right_Line[num_rectangle-1-i] = aux;
        }
        /*for(int i=0;i<right_Line.size();i++){
            cout << right_Line[i];
        }
        cout << "--------------------";*/

        navegation_Zone(bird_img, navegation_img);

        inverse_bird_Eyes(src, bird_img, result_img);

        central_Line(result_img);

        calc_pixel_metrs(result_img);

        shift_Central(result_img);

        imshow(source_window, src);
        //imshow(result_window, tr);

        char* navegation_window = "Resultado";
        namedWindow(result_window, CV_WINDOW_NORMAL);
        imshow(result_window, result_img);

        right_Line.clear();
        navegavel.clear();
        right_Line_int.clear();
        navegavel_int.clear();
        central_line.clear();

        waitKey(0);
    }

    waitKey(0);

    destroyAllWindows();

    return 0;
 }

void bird_Eyes(Mat& in, Mat& out)
{
    Mat tr;

    int Rows = in.rows;
    int Cols = in.cols;

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

    char* color_window = "Selecao do Canal e das Intensidades de Cor";
    namedWindow(color_window, CV_WINDOW_NORMAL);
    imshow(color_window, out);
}

void histogram(Mat& in)
{
    Mat histImage(in.rows, in.cols, CV_8UC3, Scalar(255,255,255) );

    int count[in.cols];
    int count_Num = 0;
    int count_Den = 0;
    //int normalized_count[in.cols];
    float inicial_Row = in.rows * 0.9;

    for (int col = 0; col < in.cols; ++col)
    {
        count[col]=0;
        for (int row = inicial_Row; row < in.rows; ++row)
        {
            if ( (int)(in.at<uchar>(row,col)) != 0)
                ++count[col];
        }
        //cout<<"A coluna  " << col << "  tem   " << count[col] << endl;
        //normalize(count[col], normalized_count[col], 0, in.rows, NORM_MINMAX, -1, Mat() );
        circle(histImage, Point(col, in.rows - count[col]),1,Scalar(0,0,255),1,8,0);
    }

    for (int col = 0; col < (in.cols)/2; ++col)
    {
        count_Num+=(col*count[col]);
        count_Den+=count[col];
    }
    if (count_Den != 0){
        left_x = count_Num / count_Den;
    }else{
        teste = 0;
    }

    count_Num = 0;
    count_Den = 0;
    for (int col = (in.cols)/2; col < in.cols; ++col)
    {
        count_Num+=(col*count[col]);
        count_Den+=count[col];
    }
    if (count_Den != 0){
        right_x = count_Num / count_Den;
    }

    /*cout << " X da linha esquerda " << left_x << endl;
    cout << " X da linha direita " << right_x << endl;
    waitKey(0);*/

    /*char* histogram_window = "Histogram";
    namedWindow(histogram_window, CV_WINDOW_NORMAL);
    imshow(histogram_window, histImage);*/
}

void sliding_Window(Mat& in, int x_Reference, Mat& out)
{  
    loop_counter = 0;

    int y_Reference = in.rows;

    Point P1, P2;
    P1.x = x_Reference - (l_rectangle/2);
    P1.y = y_Reference - (h_rectangle/2);
    P2.x = x_Reference + (l_rectangle/2);
    P2.y = y_Reference;

    rectangle(out, P1, P2, Scalar(255,0,0), 2, 8, 0);

    sliding_Recursive(in, Point(x_Reference, y_Reference), out);

    char* sliding_window = "Sliding Window";
    namedWindow(sliding_window, CV_WINDOW_NORMAL);
    imshow(sliding_window, out);
}

void sliding_Recursive(Mat& in, Point previous_Center, Mat& out)
{
    Point2d new_Center;
    int count[l_rectangle];
    int count_Num = 0;
    int count_Den = 0;

    new_Center.x = previous_Center.x;
    new_Center.y = previous_Center.y - (h_rectangle/2);

    Point P1, P2;
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
                ++count[col-P1.x];
        }

        //cout << "A coluna " << col << "(indice " << col-P1.x << " ) tem  " << count[col-P1.x] << endl;
    }


    for (int col = P1.x; col < (P1.x + l_rectangle); col++)
    {
        count_Num+=(col*count[col-P1.x]);
        count_Den+=count[col-P1.x];
    }
    if (count_Den != 0)
        new_Center.x = count_Num / count_Den;

    P1.x = new_Center.x - (l_rectangle/2);
    P1.y = new_Center.y - (h_rectangle/2);
    P2.x = new_Center.x + (l_rectangle/2);
    P2.y = new_Center.y;

    right_Line.push_back(new_Center);
    right_Line_int.push_back(new_Center);

    rectangle(out, P1, P2, Scalar(0,0,255), 2, 8, 0);

    loop_counter++;
    if (loop_counter<num_rectangle)  
        sliding_Recursive(in, new_Center, out);
}

void navegation_Zone(Mat& in, Mat& out)
{
    out = in;

    /*right_Line.push_back(Point(120,243));
    right_Line.push_back(Point(87,476));
    right_Line_int.push_back(Point(120,243));
    right_Line_int.push_back(Point(87,476));*/

    polylines(out,right_Line_int,1,Scalar(0,0,255),2,8,0);

    /*Mat draw(in.rows, in.cols, CV_8UC3, Scalar(0,0,0) );
    fillPoly(draw, right_Line, Scalar (255, 0, 0), 8);
    char* draw_window = "Regiao Navegavel";
    namedWindow(draw_window, CV_WINDOW_NORMAL);
    imshow(draw_window, draw);*/

    /*char* navegation_window = "Regiao Navegavel";
    namedWindow(navegation_window, CV_WINDOW_NORMAL);
    imshow(navegation_window, out);*/
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

    perspectiveTransform(right_Line, navegavel, M);

    for(int i=0 ; i<right_Line.size() ; i++)
    {
        navegavel_int.push_back(Point(navegavel[i].x, navegavel[i].y));
        /*cout << "Ponto inicial: ( " << right_Line[i].x << " ; " << right_Line[i].y << " ) " << endl;
        cout << "Ponto navegavel:   ( " << navegavel_int[i].x << " ; " << navegavel_int[i].y << " ) " << endl;*/
        //cout << " -> " << i << " " << navegavel[i];
    }
    //cout << " ----------------" << endl;
    polylines(out,navegavel_int,1,Scalar(255,0,0),2,8,0);
}

/*void inverse_bird_Eyes(Mat& src, Mat& in, Mat& out)
{
    Mat tr;

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

    M = getPerspectiveTransform(dst_vertices, src_vertices);
    tr = Mat(src.rows, src.cols, CV_8UC3);
    warpPerspective(in, tr, M, tr.size(), INTER_LINEAR, BORDER_CONSTANT);

    out = tr;

    char* navegation_window = "Resultado";
    namedWindow(result_window, CV_WINDOW_NORMAL);
    imshow(result_window, out);
}*/

void central_Line(Mat& in)
{
    int x_add, y_add;

    for(int i= 0; i < num_rectangle; i++)
    {   
        y_add = (int) navegavel[num_rectangle-1-i].y;
        x_add = ( (int) navegavel[num_rectangle-1 - i].x + (int) navegavel[num_rectangle+i].x ) / 2;
        central_line.push_back(Point(x_add,y_add));
    }

    polylines(in,central_line,0,Scalar(0,255,255),2,8,0);
}

void calc_pixel_metrs(Mat& in){
    float aux;
    float val_pista = 2.5/2;
    float dist;
    
    dist = navegavel[num_rectangle].x - ((navegavel[num_rectangle].x + navegavel[num_rectangle-1].x)/2);
    pixel_metrs = (val_pista)/dist;
    cout<< "Um pixel = " << pixel_metrs << " metros" << endl;


    /*for(int i = 0; i < num_rectangle; i++ ){
        
        aux = (navegavel[num_rectangle].x - (in.cols/2) )*pixel_metrs;
        cout<< "Quadrado " << i <<  " = " << aux << " metros" << endl;
        pts_metros.push_back(aux);
    }*/
}

float shift_Central(Mat& in)
{
    float frame_center = in.cols / 2;
    float shift;
    float shift_num = 0;
    float shift_den = 0;

    line(in,Point(frame_center,0),Point(frame_center,in.rows),Scalar(0,255,0),2,8,0);

    for(int i=0; i < central_line.size(); i++)
    {
        shift_num  += (frame_center - central_line[i].x)*pixel_metrs;
        shift_den++;
    }

    shift = shift_num/shift_den;

    cout << "Deslocamento da linha central :" << shift << endl;
}
