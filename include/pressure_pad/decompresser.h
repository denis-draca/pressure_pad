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

#include <vector>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <eigen3/Eigen/Dense>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <pressure_pad/pressure_read.h>

#define DEFINED_MOMENT 500
#define X_RATIO 6.5/1000
#define Y_RATIO 5.75/1000

class DECOMPRESSER
{
private:
    std::mutex mx_left; //!< Locks the resistor map in resistor_Convert method when the left thread is accessing it
    std::mutex mx_right; //!< Locks the resistor map in resistor_Convert method when the right thread is accessing it

    std::mutex left_mx;
    std::mutex right_mx;

    std::mutex mx_left_image;
    std::mutex mx_right_image;

    std::condition_variable con; //!< Wakes the left forces calculating thread
    std::condition_variable con_r; //!< Wakes the right force calculating thread
    std::condition_variable con_left_image;
    std::condition_variable con_right_image;

    bool new_data_left = false;
    bool new_data_right = false;
    bool new_image_left = false;
    bool new_image_right = false;

    struct buffer
    {
        std::vector < std::vector<uchar> > left_readings;
        std::vector < std::vector<uchar> > right_readings;
    };

    buffer buf;

    //Node Handle
    ros::NodeHandle n_;

    //Publishers
    ros::Publisher pub_left_;
    ros::Publisher pub_right_;

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

    double constant_R;

    struct image_buffer
    {
        std::vector<cv::Mat> left_images;
        std::vector<cv::Mat> right_images;
    };

    std::vector<double> forces_left;
    std::vector<double> forces_right;

    image_buffer img_buf;

private:
    double pad_force(std::vector<uchar> &reading, bool left);

    void resistor_convert(Eigen::MatrixXd &all_ones,
                          std::vector<double> &aligned,
                          Eigen::MatrixXd &R,
                          std::vector< std::vector<double> > &resistor_map,
                          bool left);

    void left_scan(const std_msgs::Int8MultiArrayConstPtr &input);
    void right_scan(const std_msgs::Int8MultiArrayConstPtr &input);

    bool is_safe(cv::Mat &image, bool is_left, pressure_pad::pressure_read &message);

    double calc_force(std::vector<std::vector<double> > map);

    Eigen::MatrixXd constant_16by16(int value);
    Eigen::MatrixXd k_creator(Eigen::MatrixXd &ones, std::vector<double> &row);

    std::vector<std::vector<double> > make_aligned(std::vector<uchar> &reading);
    std::vector<double> resistor_row(Eigen::MatrixXd &result);

private:
    DECOMPRESSER();

public:
    void wake_con();
    DECOMPRESSER(ros::NodeHandle &n);

    void left_reader();
    void right_reader();

    void left_rivet_detector();
    void right_rivet_detector();
};

#endif // DECOMPRESSER_H
