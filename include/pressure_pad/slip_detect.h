#ifndef SLIP_DETECT_H
#define SLIP_DETECT_H

#include <vector>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"

class slip_detect
{
private:
    std::vector<std::vector<cv::Point2f> >pt_buf;
public:
    slip_detect();

    void push_points(std::vector<cv::Point2f> points);

    bool has_slipped();

    int saved_points();

    void force_clear();
};

#endif // SLIP_DETECT_H
