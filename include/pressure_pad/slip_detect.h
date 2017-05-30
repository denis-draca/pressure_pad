#ifndef SLIP_DETECT_H
#define SLIP_DETECT_H

#include <vector>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"

class slip_detect
{
private:
    std::vector<std::vector<cv::Point2f> >pt_buf; //!< Contains the detected rivet positions
public:
    slip_detect();

    ///
    /// \brief push_points - pushes in the detected rivet positions
    /// \param points - the xy points
    ///
    void push_points(std::vector<cv::Point2f> points);

    ///
    /// \brief has_slipped - checks if the position of the rivets has moved.
    /// \return (bool) - True if a slip has occured
    ///
    bool has_slipped();

    ///
    /// \brief saved_points - returns the total amount of readings in pt_buf
    /// \return (int)
    ///
    int saved_points();

    ///
    /// \brief force_clear - forcefully clears pt_buf
    ///
    void force_clear();
};

#endif // SLIP_DETECT_H
