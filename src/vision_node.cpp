//#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

extern "C"
{
#include "darknet/darknet.h"
}

using namespace std;
using namespace cv;

image asd;

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        cerr << "usage: " << argv[0] << " videofeed" << endl;
        return 1;
    }

    VideoCapture cap(argv[1]);
    if (!cap.isOpened())
    {
        cerr << "cannot open video." << endl;
        return 1;
    }

    Mat frame;

    cap >> frame;

    cout << frame.rows << " x " << frame.cols << " x " << frame.channels() << endl;

    asd.h = 2;
    asd.w = 3;
    asd.c = 5;

    cout << asd.h << endl;
    //asd();

    return 0;
}

