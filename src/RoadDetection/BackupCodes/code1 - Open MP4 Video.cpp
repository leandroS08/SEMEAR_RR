#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

 int main( int argc, char** argv )
 {
    VideoCapture cap("road.mp4");

    if ( !cap.isOpened() )
    {
         cout << "Erro ao abrir ao webcam" << endl;
         return -1;
    }

    namedWindow("Source", CV_WINDOW_NORMAL);

    Mat src;

    while (waitKey(33) != 27 && cap.isOpened())
    {
        cap.read(src);

        imshow("Source Video",src);      
    }

    return 0;
}