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
    namedWindow("H Channel", CV_WINDOW_NORMAL);
    namedWindow("S Channel", CV_WINDOW_NORMAL);
    namedWindow("V Channel", CV_WINDOW_NORMAL);

    /* Variáveis Mat */
    Mat src, tr;
    Mat channel[3];

    while (waitKey(33) != 27 && cap.isOpened())
    {
        cap.read(src);

        cvtColor(src,tr,CV_BGR2HSV);

        split(tr,channel);

        imshow("Source Video", src);   
        imshow("Result Video", tr);
        imshow("H Channel", channel[0]);
        imshow("S Channel", channel[1]);
        imshow("V Channel", channel[2]);
    }

    return 0;
}
