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

#define X_RATIO         (6.5/20)/1000    //Convert from pixels to m in the x-direction
#define Y_RATIO         (5.75/20)/1000   //Convert from pixels to m in the y-direction

#define HIGH_THRESH     130         //Pixel count higher than this counts as a flat surface
#define LOW_THRESH      6           //Pixel count lower than this counts as a flat surface

#define BUF_COUNT       40          //How many readings are stored before a slip detection is performed. The slower the expected slip the higher
                                    //this number needs to be

///
/// \brief The DECOMPRESSER class - Performs all the communication with the mbeds connected to the pressure pads. It does the necessary decompressions
///                                 ,custom message building and generates all the forces and rivet positons.
///
class DECOMPRESSER
{
private:
    //Buffer Definitions
    struct image_buffer
    {
        std::vector<cv::Mat> left_images; //!< left image reading
        std::vector<cv::Mat> right_images; //!< Right image reading
    };

    struct buffer
    {
        std::vector < std::vector<uchar> > left_readings; //!< left raw reading reading
        std::vector < std::vector<uchar> > right_readings;//!< Right raw reading
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
    ros::Publisher pub_left_; //!< Publishers the results relating to the left pressure pad
    ros::Publisher pub_right_; //!< Publishers the results relating to the Right pressure pad

    image_transport::Publisher pub;

    //Subscribers
    ros::Subscriber sub_left_; //!< Reads the left pressure pad
    ros::Subscriber sub_right_;//!< Reads the right pressure pad

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
    ///
    /// \brief left_scan - Gets the reading from the left Mbed and decompresses the reading
    /// \param input - Pushed through rosserial
    ///
    void left_scan(const std_msgs::Int8MultiArrayConstPtr &input);

    ///
    /// \brief right_scan - Gets the reading from the right Mbed and decompresses the reading
    /// \param input - Pushed through rosserial
    ///
    void right_scan(const std_msgs::Int8MultiArrayConstPtr &input);

    ///
    /// \brief is_safe - Performs all the necessary checks to detemine if the step is safe or not. This involves looking at the total amount of
    ///                  rivets and the moments created by them. A flat surface is detected as safe given a certain force
    /// \param image - image generated from the force values
    /// \param is_left - true if left image
    /// \param message - ROS custom message. Certain members will be completed here
    /// \param slip_read - Object containing the previous readings
    /// \return (bool) - True if the step is safe, will go false during a slip
    ///
    bool is_safe(cv::Mat &image, bool is_left, pressure_pad::pressure_read &message, slip_detect &slip_read);

    ///
    /// \brief img_upscale - Increases the resolution of the scan image
    /// \param small_img - initial image
    /// \param scale_factor - How much are we scaling by
    /// \return (Mat) - larger image
    ///
    cv::Mat img_upscale(cv::Mat &small_img, unsigned int scale_factor);

    ///
    /// \brief weighted_average - Takes the output from the blob detection and uses that as a starting point to refine the position of the rivet
    /// \param resize - Larger image
    /// \param centre - Blob detected centre of the rivet
    /// \param diameter - Circle containing the rivet
    /// \return - New centre point, containing the refined position of the rivet
    ///
    cv::Point2f weighted_average(cv::Mat &resize, cv::Point2f &centre, double diameter);

private:
    DECOMPRESSER();

public:
    DECOMPRESSER(ros::NodeHandle &n);

    // The following each run in a seperate thread

    ///
    /// \brief wake_con - forcefully wakes up all the conditional variables
    ///
    void wake_con();

    ///
    /// \brief left_reader - performs the force calculations for the left pad
    ///
    void left_reader();

    ///
    /// \brief right_reader - performs the force calculations for the right pad
    ///
    void right_reader();

    ///
    /// \brief left_rivet_detector - Performs the rivet detection for the left pad
    ///
    void left_rivet_detector();

    ///
    /// \brief right_rivet_detector - Performs the rivet detection for the right pad
    ///
    void right_rivet_detector();
};

#endif // DECOMPRESSER_H
