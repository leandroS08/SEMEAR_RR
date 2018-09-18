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
    Mat src;
    int Rows, Cols;

    while (waitKey(33) != 27 && cap.isOpened())
    {
        cap.read(src);

        Rows = src.rows;
        Cols = src.cols;
        
        Mat mask (src.rows,src.cols, CV_8UC3, Scalar(0,0,0));

        for(int i=0; i<mask.cols; i++)
            for(int j=0; j<mask.rows; j++)
                mask.at<uchar>(Point(i,j)) = 0;
        
        // Create Polygon from vertices
        vector<Point> ROI_Poly, ROI_Output;
        ROI_Poly.push_back(Point(        0, 0.75*Rows)); //A
        ROI_Poly.push_back(Point(0.30*Cols, 0.60*Rows)); //B
        ROI_Poly.push_back(Point(0.60*Cols, 0.60*Rows)); //C
        ROI_Poly.push_back(Point(0.98*Cols,      Rows)); //E
        ROI_Poly.push_back(Point(        0,      Rows)); //F

        /* Geometricamente regular
        ROI_Poly.push_back(Point(        0, 0.75*Rows)); //A
        ROI_Poly.push_back(Point(0.25*Cols, 0.50*Rows)); //B
        ROI_Poly.push_back(Point(0.75*Cols, 0.50*Rows)); //C
        ROI_Poly.push_back(Point(     Cols, 0.75*Rows)); //D
        ROI_Poly.push_back(Point(     Cols,      Rows)); //E
        ROI_Poly.push_back(Point(        0,      Rows)); //F */


        approxPolyDP(ROI_Poly,ROI_Output, 1.0, true);
        
        // Fill polygon white
        fillConvexPoly(mask, &ROI_Output[0], ROI_Output.size(), Scalar(255,255,255));                 
        
        // Create new image for result storage
        Mat imageDest;
        
        // Cut out ROI and store it in imageDest
        src.copyTo(imageDest, mask);  

        imshow("Source Video", src);   
        imshow("Result Video", imageDest);
    }

    destroyWindow("Source Video");
    destroyWindow("Result Video");

    return 0;
}
