#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "boost/program_options.hpp"

#include <ctime>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <unordered_set>

#include "debris_thresholds.h"

extern "C"
{
#include "darknet/darknet.h"
}

using namespace std;
using namespace cv;
namespace po = boost::program_options;

const string video_url = 
"rtsp://192.168.1.10:554/user=admin&password=&channel=1&stream=0.sdp?";

// output video/image size
const int im_w = 256;
const int im_h = 144;

// target position for debris to go to
const int target_x = im_w / 2;
const int target_y = im_h;

// thresholds for controlling servos
const Scalar open_door_colour(50, 50, 255);
const Rect open_door_rect(open_door_box_x1 * im_w,
                          open_door_box_y1 * im_h,
                          open_door_box_w * im_w,
                          open_door_box_h * im_h);

const Scalar close_door_colour(0, 0, 155);
const Rect close_door_rect(close_door_box_x1 * im_w,
                           close_door_box_y1 * im_h,
                           close_door_box_w * im_w,
                           close_door_box_h * im_h);

const Scalar stay_opened_colour(150, 150, 255);
const Rect stay_opened_rect(stay_opened_box_x1 * im_w,
                            stay_opened_box_y1 * im_h,
                            stay_opened_box_w * im_w,
                            stay_opened_box_h * im_h);

// properties of sliding window
const int sw_wnd_size = 28;
const int sw_step_size = 14;
const int sw_xsteps = 1 + (im_w - sw_wnd_size) / sw_step_size;
const int sw_ysteps = 1 + (im_h - sw_wnd_size) / sw_step_size;
const Scalar green_colour(0, 255, 0);
const Scalar blue_colour(255, 0, 0);

const int num_of_class = 4;
const int debris_class = 1;

// EMA parameter
const float ema_param = .7;
const float threshold_after_ema = .37;

string cfg_file;
string weights_file;
string video_file;
string save_video_file;
string save_raw_video_file;
bool save_video = false;
bool save_raw_video = false;
bool visualise = false;

network net;

void parse_options(int argc, char **argv)
{
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "vision node")
        ("cfg,c", po::value<string>(), "darknet configuration file")
        ("weights,w", po::value<string>(), "darknet weights file")
        ("file,f", po::value<string>(), "source video file (leave empty to use camera)")
        ("visualise,v", "display video with opencv")
        ("save,s", "save output video with detection")
        ("saveraw,r", "save raw output video");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);    

    if (vm.count("help"))
    {
        cout << desc << endl;
        exit(0);
    }

    if (vm.count("cfg"))
    {
        cfg_file = vm["cfg"].as<string>();
    }
    else
    {
        cerr << "cfg not set\n";
        exit(-1);
    }

    if (vm.count("weights"))
    {
        weights_file = vm["weights"].as<string>();
    }
    else
    {
        cerr << "weights not set\n";
        exit(-1);
    }

    if (vm.count("file"))
    {
        video_file = vm["file"].as<string>();
    }
    else
    {
        video_file = video_url;
    }

    if (vm.count("visualise"))
    {
        visualise = true;
    }

    if (vm.count("save"))
    {
        save_video = true;

        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        stringstream ss;
        ss << std::put_time(&tm, "obj_det_%d-%m-%Y_%H-%M-%S.mp4");
        save_video_file = ss.str();
    }

    if (vm.count("saveraw"))
    {
        save_raw_video = true;

        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        stringstream ss;
        ss << std::put_time(&tm, "raw_%d-%m-%Y_%H-%M-%S.mp4");
        save_raw_video_file = ss.str();
    }
}

/**
 * Merge overlapping Rects.
 * \param out   output Rect
 * \param in    input Rect
 */
void merge_bounding_boxes(vector<Rect>& out, vector<Rect>& in, bool correct_order)
{
    Rect merged;
    if (in.size() == 0)
        return;

    unordered_set<int> unmerged;
    while (1)
    {
        bool unchanged = true;

        unmerged.clear();
        for (int i = 0; i < in.size(); ++i)
            unmerged.insert(i);

        for (int i = 0; i < in.size(); ++i)
        {
            merged = in[i];
            bool was_merged = false;

            for (int j = i + 1; j < in.size(); ++j)
            {
                Rect intersection = in[j] & merged;
                if (intersection.area() > 0)
                {
                    merged = in[j] | merged;
                    unmerged.erase(i);
                    unmerged.erase(j);
                    was_merged = true;
                }
            }

            if (was_merged)
            {
                unchanged = false;
                out.push_back(merged);
            }
        }

        for (auto &i : unmerged)
            out.push_back(in[i]);

        if (unchanged)
            break;

        in.clear();
        for (auto &bbox : out)
            in.push_back(bbox);
        out.clear();
    }
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
int predict_classifier(const Mat& mat, int x, int y)
{
    static float old_predictions[sw_xsteps][sw_ysteps][num_of_class] {0};
    int top = num_of_class;

    int indexes[num_of_class];
    int size = net.w;

    image im = Mat_to_image(mat);
    image r = resize_min(im, size);
    resize_network(&net, r.w, r.h);

    float *predictions = network_predict(net, im.data);
    if (net.hierarchy)
        hierarchy_predictions(predictions, net.outputs, net.hierarchy, 1, 1);

    // ema
    for (int i = 0; i < top; ++i)
    {
        predictions[i] = ema_param*(1 - ema_param)*old_predictions[x][y][i] + 
                         (1 - ema_param)*predictions[i];
        old_predictions[x][y][i] = predictions[i];
    }

    top_k(predictions, net.outputs, top, indexes);

    int result = indexes[0];

    if (result == debris_class && 
        predictions[debris_class] > threshold_after_ema)
    {
        result = debris_class;
    }
    else
    {
        result = !debris_class;
    }
    
    free_image(im);

    return result;
}

Rect& get_nearest_debris(vector<Rect>& bboxes,
                          geometry_msgs::Point &point)
{
    // x/y coord of center of bbox
    int bb_x, bb_y;
    int dist;
    Rect* nearest_bbox = nullptr;

    point.x = -1;
    point.y = -1;

    int nearest_dist = 9999999;

    for (auto &bbox : bboxes)
    {
        bb_x = bbox.x + bbox.width / 2;
        bb_y = bbox.y + bbox.height / 2;
        dist = (bb_x - target_x) * (bb_x - target_x) + 
                  (bb_y - target_y) * (bb_y - target_y);

        if (dist < nearest_dist)
        {
            point.x = (float) bb_x / im_w;
            point.y = (float) bb_y / im_h;
            nearest_dist = dist;
            nearest_bbox = &bbox;
        }
    }

    return *nearest_bbox;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle nh;

    VideoCapture cap;
    VideoWriter output_video, output_raw_video;

    parse_options(argc, argv);

    cap.open(video_file);

    if (!cap.isOpened())
    {
        cerr << "cannot open video." << endl;
        return 1;
    }

    // set up ros image publisher
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
    sensor_msgs::ImagePtr msg;

    // set up debris coordinates publisher
    ros::Publisher coord_pub;
    coord_pub = nh.advertise<geometry_msgs::Point>("debris_coord", 1);
    geometry_msgs::Point coord_msg;

    if (save_video)
    {
        output_video.open(save_video_file,
                          VideoWriter::fourcc('X', '2', '6', '4'),
                          cap.get(CV_CAP_PROP_FPS),
                          cv::Size(im_w, im_h));
        if (!output_video.isOpened())
        {
            cerr << "Cannot open video for writing." << endl;
            exit(-1);
        }
    }

    if (save_raw_video)
    {
        output_raw_video.open(save_raw_video_file,
                          VideoWriter::fourcc('X', '2', '6', '4'),
                          cap.get(CV_CAP_PROP_FPS),
                          cv::Size(cap.get(CV_CAP_PROP_FRAME_WIDTH),
                                   cap.get(CV_CAP_PROP_FRAME_HEIGHT)));
        if (!output_raw_video.isOpened())
        {
            cerr << "Cannot open raw video for writing." << endl;
            exit(-1);
        }
    }

    // set up CNN
    net = parse_network_cfg((char *) cfg_file.c_str());
    load_weights(&net, (char *) weights_file.c_str());
    set_batch_network(&net, 1);
    srand(2222222);

    vector<Rect> raw_bbox, merged_bbox;

    if (visualise)
        namedWindow("predictions", WINDOW_NORMAL);

    int capture_buf = 0;
    while (ros::ok())
    {
        Mat capture, original, frame;

        if (!cap.read(capture))
        {
            // XXX
            // in case we aren't getting the frame due to lag
            if (capture_buf++ > 1000)
                break;
            continue;
        }
        capture_buf = 0;

        resize(capture, frame, Size(im_w, im_h), 0, 0, CV_INTER_AREA);

        original = frame.clone();
        raw_bbox.clear();
        merged_bbox.clear();

        // perform prediction on sliding window
        for (int x = 0; x < sw_xsteps; ++x)
        {
            for (int y = 0; y < sw_ysteps; ++y)
            {
                Rect grid_rect(x*sw_step_size, 
                               y*sw_step_size,
                               sw_wnd_size,
                               sw_wnd_size);
                Mat grid = original(grid_rect);
                int p = predict_classifier(grid, x, y);
                if (p == debris_class) // debris found
                {
                    raw_bbox.push_back(grid_rect);
                }
            }
        }

        // merge overlapping sliding windows
        merge_bounding_boxes(merged_bbox, raw_bbox, true);
        for (auto bbox : merged_bbox)
            rectangle(frame, bbox, green_colour, 2);

        // visualise thresholds
        rectangle(frame, open_door_rect, open_door_colour, 1);
        rectangle(frame, close_door_rect, close_door_colour, 1);
        rectangle(frame, stay_opened_rect, stay_opened_colour, 1);

        // publish debris coord if detected
        Rect& nearest_bbox = get_nearest_debris(merged_bbox, coord_msg);
        if (merged_bbox.size() > 0)
        {
            rectangle(frame, nearest_bbox, blue_colour, 3);
        }
        coord_pub.publish(coord_msg);

        // publish image as ros msg
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);

        // show and save frame
        if (visualise)
            imshow("predictions", frame);
        if (save_video)
            output_video << frame;
        if (save_raw_video)
            output_raw_video << capture;
        if(waitKey(1) == 27)
            break;
    }
    cap.release();
    output_video.release();
    return 0;
}

