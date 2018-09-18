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
    Mat channel[3];
    Mat imgThresholded;

    int iLowV = 0;
    int iHighV = 255;

    cvCreateTrackbar("LowV", "Result Video", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Result Video", &iHighV, 255);

    while (waitKey(33) != 27 && cap.isOpened())
    {
        cap.read(src);

        cvtColor(src,tr,CV_BGR2HSV);

        split(tr,channel);

        inRange(channel[2], Scalar(iLowV), Scalar(iHighV), imgThresholded); 

        imshow("Source Video", src);   
        imshow("Result Video", imgThresholded);
    }

    return 0;
}
