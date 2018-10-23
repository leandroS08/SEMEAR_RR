#include <iostream>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

#define PI 3.1415926

int frameWidth = 640;
int frameHeight = 480;

void InversePerspectiveMapping(const Mat& src, Mat& tr1, int alpha_, int beta_, int gamma_, int f_, int dist_);

 int main( int argc, char** argv )
 {
    VideoCapture cap(argv[1]);

    if ( !cap.isOpened() )
    {
         cout << "Erro ao abrir o video" << endl;
         return -1;
    }

    /* Janelas */
    namedWindow("Source Video", CV_WINDOW_NORMAL);
    namedWindow("Inverse Perspective Mapping", CV_WINDOW_NORMAL);

    /* Variáveis Mat */
    Mat src, tr1, tr2, tr3, tr4;
    
    int alpha_ = 90, beta_ = 90, gamma_ = 90;
    int f_ = 500, dist_ = 500;

    createTrackbar("Alpha", "Inverse Perspective Mapping", &alpha_, 180);
    createTrackbar("Beta", "Inverse Perspective Mapping", &beta_, 180);
    createTrackbar("Gamma", "Inverse Perspective Mapping", &gamma_, 180);
    createTrackbar("f", "Inverse Perspective Mapping", &f_, 2000);
    createTrackbar("Distance", "Inverse Perspective Mapping", &dist_, 2000);

    while (waitKey(33) != 27 && cap.isOpened())
    {
        cap.read(src);

        InversePerspectiveMapping(src,tr1,alpha_,beta_,gamma_,f_,dist_);

        imshow("Source Video",src);      
        imshow("Inverse Perspective Mapping", tr1);
    }

    return 0;
}


void InversePerspectiveMapping(const Mat& src, Mat& tr1, int alpha_, int beta_, int gamma_, int f_, int dist_)
{
    //int frameWidth = ;
    //int frameHeight = ;

    int offsetx = 0;
    int offsety = -175;

    Mat trans_mat = (Mat_<double>(2,3) << 1, 0, offsetx, 0, 1, offsety);
    warpAffine(src,tr1,trans_mat,src.size());

    double focalLength, dist, alpha, beta, gamma; 

    alpha =((double)alpha_ -90) * PI/180;
    beta =((double)beta_ -90) * PI/180;
    gamma =((double)gamma_ -90) * PI/180;
    focalLength = (double)f_;
    dist = (double)dist_;

    Size image_size = src.size();
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

    warpPerspective(tr1, tr1, transformationMat, image_size, INTER_CUBIC | WARP_INVERSE_MAP);
}