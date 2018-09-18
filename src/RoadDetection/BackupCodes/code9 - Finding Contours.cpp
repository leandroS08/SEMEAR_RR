#include <iostream>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

Mat src; Mat src_gray;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

char* source_window = "Original Video";
char* result_window = "Result Video";

void thresh_callback(int, void* )
{
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  Canny( src_gray, canny_output, thresh, thresh*2, 3 );
  findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );

  for( size_t i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, (int)i, color, 2, 8, hierarchy, 0, Point() );
     }

  imshow(result_window, drawing );
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

        cvtColor( src, src_gray, COLOR_BGR2GRAY );
        blur( src_gray, src_gray, Size(3,3) );

        createTrackbar( " Canny thresh:", source_window, &thresh, max_thresh, thresh_callback );
        thresh_callback( 0, 0 );

        imshow(source_window, src);

        waitKey(0);
    }

    waitKey(0);

    destroyWindow(source_window);
    destroyWindow(result_window);

    return 0;
 }