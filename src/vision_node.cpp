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

const Scalar green_colour(0, 255, 0);

network net;

// code adapted from darknet
image Mat_to_image(const Mat &mat)
{
    image im;
    int h, w, c, step;
    unsigned char *data;

    h = mat.rows;
    w = mat.cols;
    c = mat.channels();
    step = mat.step;
    data = (unsigned char *)mat.data;

    im = make_image(w, h, c);

    for(int k = 0; k < c; ++k)
        for(int i = 0; i < h; ++i)
            for(int j = 0; j < w; ++j)
                im.data[(c - k - 1)*w*h + i*w + j] = data[i*step + j*c + k]/255.;

    return im;
}

// code adapted from darknet
int predict_classifier(const Mat& mat)
{
    int top = 2;

    int *indexes = (int *)calloc(top, sizeof(int));
    int size = net.w;

    image im = Mat_to_image(mat);
    image r = resize_min(im, size);
    resize_network(&net, r.w, r.h);

    float *predictions = network_predict(net, r.data);
    if (net.hierarchy)
        hierarchy_predictions(predictions, net.outputs, net.hierarchy, 1, 1);
    top_k(predictions, net.outputs, top, indexes);

    if(r.data != im.data)
        free_image(r);

    int result = indexes[0];
    
    free(indexes);
    free_image(im);

    return result;
}

int main(int argc, char **argv)
{
    if (argc < 4)
    {
        cerr << "usage: " << argv[0] << " cfg" << " weights" << " video" << endl;
        return 1;
    }

    VideoCapture cap(argv[3]);
    if (!cap.isOpened())
    {
        cerr << "cannot open video." << endl;
        return 1;
    }

    net = parse_network_cfg(argv[1]);
    load_weights(&net, argv[2]);
    set_batch_network(&net, 1);
    srand(2222222);

    namedWindow("predictions",1);
    while (1)
    {
        Mat frame, original;

        if (!cap.read(frame))
            exit(0);

        original = frame.clone();

        for (int x = 0; x < 19; ++x)
        {
            for (int y = 0; y < 19; ++y)
            {
                Rect grid_rect(x*14, y*14, 28, 28);
                Mat grid = original(grid_rect);
                //int p = predict_classifier(argv[1], argv[2], grid);
                int p = predict_classifier(grid);
                if (p == 0) // debris found
                {
                    rectangle(frame, grid_rect, green_colour, 3);
                }
            }
        }

        imshow("predictions", frame);
        if(waitKey(30) == 27)
            exit(0);
    }

    return 0;
}

