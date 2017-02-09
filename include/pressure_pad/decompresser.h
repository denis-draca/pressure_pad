#ifndef DECOMPRESSER_H
#define DECOMPRESSER_H

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Float32.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

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

class DECOMPRESSER
{
private:
    std::mutex mx_left;
    std::mutex mx_right;

    std::mutex left_mx;
    std::mutex right_mx;

    std::condition_variable con;
    std::condition_variable con_r;

    bool new_data_left = false;
    bool new_data_right = false;

    struct buffer{
        std::vector < std::vector<uchar> > left_readings;
        std::vector < std::vector<uchar> > right_readings;
    };

    buffer buf;

    //Node Handle
    ros::NodeHandle n_;

    //Publishers
    image_transport::Publisher pub_left_;
    image_transport::Publisher pub_right_;

    ros::Publisher left_force_;
    ros::Publisher right_force_;

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

private:
    double pad_force(std::vector<uchar> &reading, bool left);
    void resistor_convert(Eigen::MatrixXd &all_ones, std::vector<double> &aligned, Eigen::MatrixXd &R, std::vector< std::vector<double> > &resistor_map, bool left);
    void left_scan(const std_msgs::Int8MultiArrayConstPtr &input);
    void right_scan(const std_msgs::Int8MultiArrayConstPtr &input);

    int find_max(std::vector<uchar> &v);

    std::vector<std::vector<double> > make_aligned(std::vector<uchar> &reading);
    std::vector<double> resistor_row(Eigen::MatrixXd &result);

    double calc_force(std::vector<std::vector<double> > map);

    Eigen::MatrixXd constant_16by16(int value);
    Eigen::MatrixXd k_creator(Eigen::MatrixXd &ones, std::vector<double> &row);




private:
    DECOMPRESSER();

public:
    void wake_con();
    DECOMPRESSER(ros::NodeHandle &n);
    void left_reader();
    void right_reader();
};

#endif // DECOMPRESSER_H