#ifndef DECOMPRESSER_H
#define DECOMPRESSER_H

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Int8MultiArray.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <condition_variable>
#include <pressure_pad/pressure_read.h>
#include <sensor_msgs/Image.h>

#include "pressure_pad/force_generator.h"
#include "pressure_pad/slip_detect.h"

#define DEFINED_FORCE   1300        //what counts as a safe force
#define DEFINED_MOMENT  300         //what counts as a safe moment

#define X_RATIO         6.5/1000    //Convert from pixels to m in the x-direction
#define Y_RATIO         5.75/1000   //Convert from pixels to m in the y-direction

#define HIGH_THRESH     130         //Pixel count higher than this counts as a flat surface
#define LOW_THRESH      6           //Pixel count lower than this counts as a flat surface

#define BUF_COUNT       40          //How many readings are stored before a slip detection is performed. The slower the expected slip the higher
                                    //this number needs to be

class DECOMPRESSER
{
private:
    //Buffer Definitions
    struct image_buffer
    {
        std::vector<cv::Mat> left_images;
        std::vector<cv::Mat> right_images;
    };

    struct buffer
    {
        std::vector < std::vector<uchar> > left_readings;
        std::vector < std::vector<uchar> > right_readings;
    };

    std::vector< std::vector<double> > buf_per_cell;

    //Mutex's

    std::mutex left_mx;
    std::mutex right_mx;

    std::mutex mx_left_image;
    std::mutex mx_right_image;

    //Conditional Variables
    std::condition_variable con; //!< Wakes the left forces calculating thread
    std::condition_variable con_r; //!< Wakes the right force calculating thread
    std::condition_variable con_left_image;
    std::condition_variable con_right_image;

    //Node Handle
    ros::NodeHandle n_;

    //Publishers
    ros::Publisher pub_left_;
    ros::Publisher pub_right_;

    image_transport::Publisher pub;

    image_transport::ImageTransport it_;

    //Subscribers
    ros::Subscriber sub_left_;
    ros::Subscriber sub_right_;

    //Vectors
    std::vector<int16_t> input_scan_left_;
    std::vector<int16_t> input_scan_right_;

    //Booleans
    bool left_aDone = false;
    bool right_aDone = false;

    bool new_data_left = false;
    bool new_data_right = false;
    bool new_image_left = false;
    bool new_image_right = false;

    //Vectors
    std::vector<double> forces_left;
    std::vector<double> forces_right;

    //Buffers
    image_buffer img_buf;
    buffer buf;

    cv::SimpleBlobDetector::Params params;

private:
    void left_scan(const std_msgs::Int8MultiArrayConstPtr &input);
    void right_scan(const std_msgs::Int8MultiArrayConstPtr &input);

    bool is_safe(cv::Mat &image, bool is_left, pressure_pad::pressure_read &message, slip_detect &slip_read);

private:
    DECOMPRESSER();

public:
    DECOMPRESSER(ros::NodeHandle &n);

    void wake_con();
    void left_reader();
    void right_reader();

    void left_rivet_detector();
    void right_rivet_detector();
};

#endif // DECOMPRESSER_H
