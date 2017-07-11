#include <opencv2/opencv.hpp>
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
const int im_w = 256;
const int im_h = 144;
const int sw_wnd_size = 28;
const int sw_step_size = 14;
const int sw_xsteps = 1 + (im_w - sw_wnd_size) / sw_step_size;
const int sw_ysteps = 1 + (im_h - sw_wnd_size) / sw_step_size;

const int num_of_class = 4;
const int debris_class = 1;

network net;

void merge_bounding_boxes(vector<Rect>& out, const vector<Rect>& in)
{
    Rect merged;
    if (in.size() == 0)
        return;

    merged = in[0];
    for (auto bbox : in)
    {
        Rect intersection = bbox & merged;
        if (intersection.area() > 0)
        {
            merged = bbox | merged;
        }
        else
        {
            out.push_back(merged);
            merged = bbox;
        }
    }

    out.push_back(merged);
}

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
    int top = num_of_class;

    int *indexes = (int *)calloc(top, sizeof(int));
    int size = net.w;

    image im = Mat_to_image(mat);
    image r = resize_min(im, size);
    resize_network(&net, r.w, r.h);

    float *predictions = network_predict(net, im.data);
    if (net.hierarchy)
        hierarchy_predictions(predictions, net.outputs, net.hierarchy, 1, 1);
    top_k(predictions, net.outputs, top, indexes);

    int result = indexes[0];
    
    free(indexes);
    free_image(im);

    return result;
}

int main(int argc, char **argv)
{
    VideoCapture cap;

    if (argc < 3)
    {
        cerr << "usage: " << argv[0] << " cfg" << " weights" << " [video]" << endl;
        return 1;
    }
    else if (argc == 3)
    {
        cap.open("rtsp://192.168.1.10:554/user=admin&password=&channel=1&stream=0.sdp?");
    }
    else
    {
        cap.open(argv[3]);
    }

    if (!cap.isOpened())
    {
        cerr << "cannot open video." << endl;
        return 1;
    }

    //// XXX
    //VideoWriter output_video;
    //output_video.open("./hehe.mp4",
                      //cap.get(CV_CAP_PROP_FOURCC),
                      //cap.get(CV_CAP_PROP_FPS),
                      //cv::Size(cap.get(CV_CAP_PROP_FRAME_WIDTH), 
                               //cap.get(CV_CAP_PROP_FRAME_HEIGHT)));
    //if (!output_video.isOpened())
    //{
        //cerr << "Cannot open video for writing." << endl;
        //exit(-1);
    //}
    //else
    //{
        //cout << "Writing video ..." << endl;
    //}

    // set up CNN
    net = parse_network_cfg(argv[1]);
    load_weights(&net, argv[2]);
    set_batch_network(&net, 1);
    srand(2222222);

    vector<Rect> raw_bbox, merged_bbox;

    namedWindow("predictions", WINDOW_NORMAL);

    while (1)
    {
        Mat capture, original, frame;

        if (!cap.read(capture))
            break;

        resize(capture, frame, Size(im_w, im_h), 0, 0, CV_INTER_AREA);

        original = frame.clone();
        raw_bbox.clear();
        merged_bbox.clear();

        for (int x = 0; x < sw_xsteps; ++x)
        {
            for (int y = 0; y < sw_ysteps; ++y)
            {
                Rect grid_rect(x*sw_step_size, 
                               y*sw_step_size,
                               sw_wnd_size,
                               sw_wnd_size);
                Mat grid = original(grid_rect);
                int p = predict_classifier(grid);
                if (p == debris_class) // debris found
                {
                    raw_bbox.push_back(grid_rect);
                }
            }
        }

        merge_bounding_boxes(merged_bbox, raw_bbox);
        for (auto bbox : merged_bbox)
            rectangle(frame, bbox, green_colour, 3);

        imshow("predictions", frame);
        if(waitKey(1) == 27)
            break;
    }
    cap.release();
    return 0;
}

