#include <iostream>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;


 int main( int argc, char** argv )
 {
    VideoCapture cap("road.mp4");

    if ( !cap.isOpened() )
    {
         cout << "Erro ao abrir o video" << endl;
         return -1;
    }

    /* Janelas */
    namedWindow("Source Video", CV_WINDOW_NORMAL);
    namedWindow("Result Video", CV_WINDOW_NORMAL);

    /* Variáveis Mat */
    Mat src, tr;
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;

    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;


    while (waitKey(33) != 27 && cap.isOpened())
    {
        cap.read(src);

        GaussianBlur(src,tr,Size(3,3),0,0,BORDER_DEFAULT);

        cvtColor(tr,tr,CV_BGR2GRAY);

        Sobel(tr,grad_x,ddepth,1,0,3,scale,delta,BORDER_DEFAULT);
        convertScaleAbs(grad_x,abs_grad_x);

        Sobel(tr,grad_y,ddepth,0,1,3,scale,delta,BORDER_DEFAULT);
        convertScaleAbs(grad_y,abs_grad_y);

        addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, tr);

        imshow("Source Video", src);   
        imshow("Result Video", tr);
    }

    return 0;
}
