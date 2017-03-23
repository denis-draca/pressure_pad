#include "pressure_pad/slip_detect.h"

slip_detect::slip_detect()
{
}

void slip_detect::push_points(std::vector<cv::Point2f> &points)
{
    pt_buf.push_back(points);
}

bool slip_detect::has_slipped()
{
    if(pt_buf.empty())
    {
        ROS_INFO("NO SAVED READINGS");
        return false;
    }
    for(int i = 1; i < pt_buf.size(); i++)
    {
        if(pt_buf.at(i).size() == pt_buf.at(i+1).size())
        {
            for(int j = 0; j < pt_buf.at(i).size(); j++)
            {
                if(std::abs(pt_buf.at(0).at(j).x - pt_buf.at(i).at(j).x) > 1 || std::abs(pt_buf.at(0).at(j).y - pt_buf.at(i).at(j).y) > 1)
                {
                    return true;
                }
            }
        }
    }

    pt_buf.clear();

    return false;
}

int slip_detect::saved_points()
{
    return pt_buf.size();
}
