#include <iostream>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

#define PI 3.1415926

Mat src; // Frame original lido da câmera
Mat tr;  // Mat que receberá todas as transformações no frame

char* source_window = "Original Video";
char* result_window = "Result Video";

void select_Region(Mat&);

void sobel_Transform(Mat&);

void bird_Eyes(Mat&);

void select_Region2(Mat&);

void select_Channel(Mat&);

void histogram(Mat&);

void sliding_Window();

int main( int argc, char** argv )
{
    VideoCapture cap("road.mp4");

    if ( !cap.isOpened() )
    {
        cout << "Erro ao abrir o video" << endl;
        return -1;
    }

    /* Janelas */
    namedWindow(source_window, CV_WINDOW_NORMAL);
    namedWindow(result_window, CV_WINDOW_NORMAL);


    while ( (waitKey(33) != 27) && (cap.isOpened()) )
    {
        cap.read(src);

        tr = src;

        //select_Region(tr);

        //sobel_Transform(tr);

        bird_Eyes(tr);

        //select_Region2(tr);

        select_Channel(tr);

        //histogram(tr);

        sliding_Window();

        imshow(source_window, src);
        imshow(result_window, tr);

        waitKey(0);
    }

    waitKey(0);

    destroyWindow(source_window);
    destroyWindow(result_window);

    return 0;
 }


 void select_Region(Mat& roi)
 {
    Mat imageDest;

    int Rows, Cols;

    Rows = roi.rows;
    Cols = roi.cols;
    
    Mat mask (roi.rows,roi.cols, CV_8UC3, Scalar(0,0,0));

    for(int i=0; i<mask.cols; i++)
        for(int j=0; j<mask.rows; j++)
            mask.at<uchar>(Point(i,j)) = 0;
    
    // Create Polygon from vertices
    vector<Point> ROI_Poly, ROI_Output;
    ROI_Poly.push_back(Point(        0,      Rows)); //A
    ROI_Poly.push_back(Point(0.40*Cols, 0.60*Rows)); //B
    ROI_Poly.push_back(Point(0.60*Cols, 0.60*Rows)); //C
    ROI_Poly.push_back(Point(     Cols,      Rows)); //E

    approxPolyDP(ROI_Poly,ROI_Output, 1.0, true);
        
    // Fill polygon white
    fillConvexPoly(mask, &ROI_Output[0], ROI_Output.size(), Scalar(255,255,255)); 
        
    // Cut out ROI and store it in imageDest
    roi.copyTo(imageDest, mask);

    roi = imageDest;             
 }


void sobel_Transform(Mat& sb)
{
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;

    GaussianBlur(sb,sb,Size(3,3),0,0,BORDER_DEFAULT);

    cvtColor(sb,sb,CV_RGB2HLS);

    Sobel(sb,grad_x,ddepth,1,0,3,scale,delta,BORDER_DEFAULT);
    convertScaleAbs(grad_x,abs_grad_x);

    Sobel(sb,grad_y,ddepth,0,1,3,scale,delta,BORDER_DEFAULT);
    convertScaleAbs(grad_y,abs_grad_y);

    addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, sb);
}

/*
void bird_Eyes(Mat& in)
{
    int offsetx = 0;
    int offsety = -150;

    Mat trans_mat = (Mat_<double>(2,3) << 1, 0, offsetx, 0, 1, offsety);
    warpAffine(in,in,trans_mat,in.size());

    double alpha =((double)14 - 90) * PI/180;
    double beta = 0;
    double gamma = 0;
    double focalLength = 500.0;
    double dist = 500.0;

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

    Mat transformationMat = K * (T * (R * A1));

    warpPerspective(in, in, transformationMat, image_size, INTER_CUBIC | WARP_INVERSE_MAP);
}
*/

void transform(Point2f* src_vertices, Point2f* dst_vertices, Mat& src, Mat &dst)
{
    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
}

void bird_Eyes(Mat& in)
{
    int Rows = in.rows;
    int Cols = in.cols;

    Point2f src_vertices[4];
    src_vertices[0] = Point(        0,      Rows);
    src_vertices[1] = Point(0.30*Cols, 0.70*Rows);
    src_vertices[2] = Point(0.70*Cols, 0.70*Rows);
    src_vertices[3] = Point(     Cols,      Rows);

    Point2f dst_vertices[4];
    dst_vertices[0] = Point(  0, 480);
    dst_vertices[1] = Point(  0,   0);
    dst_vertices[2] = Point(640,   0);
    dst_vertices[3] = Point(640, 480);

    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    Mat dst(480, 640, CV_8UC3);
    warpPerspective(in, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
    //namedWindow("dst", CV_WINDOW_NORMAL);
    //imshow("dst", dst);
    tr = dst;

    /*Mat dst2(480, 640, CV_8UC3);
    transform(src_vertices, dst_vertices, src, dst2);
    namedWindow("dst2", CV_WINDOW_NORMAL);
    imshow("dst2", dst2);*/
}

void select_Region2(Mat& roi)
{
    Mat imageDest;

    int Rows, Cols;

    Rows = roi.rows;
    Cols = roi.cols;
    
    Mat mask (roi.rows,roi.cols, CV_8UC3, Scalar(0,0,0));

    for(int i=0; i<mask.cols; i++)
        for(int j=0; j<mask.rows; j++)
            mask.at<uchar>(Point(i,j)) = 0;
    
    // Create Polygon from vertices
    vector<Point> ROI_Poly, ROI_Output;
    ROI_Poly.push_back(Point(0.30*Cols, 0.50*Rows)); //A
    ROI_Poly.push_back(Point(0.30*Cols,      Rows)); //B
    ROI_Poly.push_back(Point(0.70*Cols,      Rows)); //C
    ROI_Poly.push_back(Point(0.70*Cols, 0.50*Rows)); //D 

    approxPolyDP(ROI_Poly,ROI_Output, 1.0, true);
        
    // Fill polygon white
    fillConvexPoly(mask, &ROI_Output[0], ROI_Output.size(), Scalar(255,255,255)); 
        
    // Cut out ROI and store it in imageDest
    roi.copyTo(imageDest, mask);

    roi = imageDest;   
}

void select_Channel(Mat& in)
{
    cvtColor(in,tr,CV_RGB2HLS);

    vector<Mat> hsl_planes;
    split( tr, hsl_planes );

    /*namedWindow("H channel", CV_WINDOW_NORMAL);
    namedWindow("S channel", CV_WINDOW_NORMAL);
    namedWindow("L channel", CV_WINDOW_NORMAL);
    imshow("H channel", hsl_planes[0]);
    imshow("S channel", hsl_planes[1]);
    imshow("L channel", hsl_planes[2]);*/

    inRange(hsl_planes[1], 50, 110, tr); 
}

/*
void histogram(Mat& in)
{
    int histSize = 256;
    float range[] = { 0, 255 } ; //the upper boundary is exclusive
    const float* histRange = { range };
    bool uniform = true; 
    bool accumulate = false;

    Mat hist;
    calcHist( &in, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );
    
    // Draw the histogram for Saturation
    int hist_w = in.cols; int hist_h = in.rows;
    int bin_w = cvRound( (double) hist_w/histSize );
    Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0,0,0) );
    normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

    /// Draw for each channel
    for( int i = 1; i < histSize; i++ )
    {
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
                        Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
                        Scalar( 0, 255, 0), 2, 8, 0  );
    }

    char* histogram_window = "Histogram";
    namedWindow(histogram_window, CV_WINDOW_NORMAL);
    imshow(histogram_window, histImage);
}*/


void sliding_Window()
{
    int h_rectangle = 30;
    int l_rectangle = 50;

    Point P1, P2;

    P1.x = 569 - (l_rectangle/2);
    P1.y = 468 - (h_rectangle/2);
    P2.x = 569 + (l_rectangle/2);
    P2.y = tr.cols;

    rectangle(tr, P1, P2, 255, 2, 8, 0);


    
}
