#include <iostream>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

Mat src, src_gray;
Mat dst, detected_edges;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
char* source_window = "Original Video";
char* result_window = "Result Video";

void CannyThreshold(int, void*)
{
    /// Reduce noise with a kernel 3x3
    blur( src_gray, detected_edges, Size(3,3) );

    /// Canny detector
    Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

    /// Using Canny's output as a mask, we display our result
    dst = Scalar::all(0);

    src.copyTo( dst, detected_edges);
 }

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

    while (waitKey(33) != 27 && cap.isOpened())
    {
        cap.read(src);

        cvtColor( src, src_gray, CV_BGR2GRAY );

        /// Create a Trackbar for user to enter threshold
        createTrackbar( "Min Threshold:", source_window, &lowThreshold, max_lowThreshold, CannyThreshold );

        /// Show the image
        CannyThreshold(0, 0);

        imshow(source_window, src);
        imshow(result_window, dst);
    }

    waitKey(0);

    destroyWindow(source_window);
    destroyWindow(result_window);

    return 0;
 }




