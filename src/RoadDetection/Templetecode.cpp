#include <iostream>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

char* source_window = "Original Video";
char* result_window = "Result Video";

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

    Mat src, dst;

    while (waitKey(33) != 27 && cap.isOpened())
    {
        cap.read(src);

        imshow(source_window, src);
        imshow(result_window, dst);
    }

    waitKey(0);

    destroyWindow(source_window);
    destroyWindow(result_window);

    return 0;
 }