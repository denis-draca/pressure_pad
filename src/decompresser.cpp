#include "pressure_pad/decompresser.h"


/***********************************************************************
 ********************             ***************************************
 ******************** CONSTRUCTOR ***************************************
 ********************             ***************************************
 ***********************************************************************
 */

DECOMPRESSER::DECOMPRESSER(ros::NodeHandle &n):
    n_(n)
{
    pub_left_ = n_.advertise<pressure_pad::pressure_read>("/wallpusher/reading/left",1);
    pub_right_= n_.advertise<pressure_pad::pressure_read>("/wallpusher/reading/right",1);

    sub_left_ = n_.subscribe("/wallpusher/raw_scan/left", 1, &DECOMPRESSER::left_scan, this);
    sub_right_= n_.subscribe("/wallpusher/raw_scan/right", 1, &DECOMPRESSER::right_scan, this);

    params.filterByArea = false;
//    params.minArea = 1;
//    params.maxArea = 1000;
    params.filterByColor = true;
    params.blobColor = 255;
    params.filterByConvexity = false;
//    params.minConvexity = 0.1;
    params.filterByInertia = false;
    params.filterByCircularity = false;

}

/***********************************************************************
 ********************            ***************************************
 ******************** LEFT SCAN  ***************************************
 ********************            ***************************************
 ***********************************************************************
 */

void DECOMPRESSER::left_scan(const std_msgs::Int8MultiArrayConstPtr &input)
{
    if(input->data.size() == 0)
    {
        ROS_ERROR("No Data Transmitted");
        return;
    }

    if (input->layout.dim[0].label == "a" && !left_aDone)
    {
        for (int i = 0; i < input->layout.dim[0].size; i++)
        {
            input_scan_left_.push_back(input->data[i] + 128);
        }

        left_aDone = true;
        return;
    }

    if (left_aDone && input->layout.dim[0].label != "a")
    {
        for (int i = 0; i < input->data.size(); i++)
        {
           input_scan_left_.push_back(input->data.at(i) + 128);
        }

        left_aDone = false;
    }


    if(input_scan_left_.size() > 512)
    {
        ROS_ERROR("TOO MANY DATA POINTS SENT -> %d", (int)input_scan_left_.size());
        return;
    }

    cv::Mat image(ROWS,COLUMNS,CV_8U);

    std::vector <uchar> v;
    bool next = false;

    for (int i = 0; i < input_scan_left_.size(); i++)
    {
        if(next)
        {
            next = false;
            continue;
        }

        if (input_scan_left_.at(i) == 0)
        {
            if (i == input_scan_left_.size() - 1)
            {
                v.push_back(0);
            }
            else
            {
                for(int y = 0 ; y < input_scan_left_.at(i + 1); y++)
                {
                    v.push_back(0);
                }
            }
            next = true;
        }
        else
        {
            v.push_back(input_scan_left_.at(i));
        }

    }

    input_scan_left_.clear();


    if(v.size() == 0)
    {
        ROS_ERROR("DECOMPRESSED DATA is EMPTY -> %d", (int)v.size());
        return;
    }

    if (v.size() > 512)
    {
        ROS_ERROR("DECOMPRESSED DATA HAS TOO MANY DATA POINTS -> %d", (int)v.size());
        return;
    }

    left_mx.lock();

    buf.left_readings.push_back(v);
    new_data_left = true;

    left_mx.unlock();

    con.notify_all();

    int pos = 0;
    for (int y = 0; y < image.rows ; y++)
    {
        for (int x = image.cols - 1; x >= 0; x--)
        {
            if (pos >= v.size())
            {
                ROS_INFO("ACCESSING BEYOND THE DECOMPRESSED VECTOR pos -> %d  v.size() -> %d", (int)pos, (int)v.size());
                return;
            }
            image.at<uchar>(y,x) = v.at(pos);

            pos++;
        }
    }
}

/***********************************************************************
 ********************            ***************************************
 ******************** RIGHT SCAN ***************************************
 ********************            ***************************************
 ***********************************************************************
 */

void DECOMPRESSER::right_scan(const std_msgs::Int8MultiArrayConstPtr &input)
{
    if (input->layout.dim[0].label == "a" && !right_aDone)
    {
        for (int i = 0; i < input->layout.dim[0].size; i++)
        {
            input_scan_right_.push_back(input->data[i] + 128);
        }

        right_aDone = true;
        return;
    }

    if (right_aDone && input->layout.dim[0].label != "a")
    {
        for (int i = 0; i < input->data.size(); i++)
        {
           input_scan_right_.push_back(input->data.at(i) + 128);
        }

        right_aDone = false;
    }

    if(input_scan_right_.size() > 512)
    {
        ROS_INFO("TOO MANY DATA POINTS SENT -> %d", (int)input_scan_right_.size());
        return;
    }

    cv::Mat image(ROWS,COLUMNS,CV_8U);

    std::vector <uchar> v;
    bool next = false;

    for (int i = 0; i < input_scan_right_.size(); i++)
    {
        if(next)
        {
            next = false;
            continue;
        }

        if (input_scan_right_.at(i) == 0)
        {
            if (i == input_scan_right_.size() - 1)
            {
                v.push_back(0);
            }
            else
            {
                for(int y = 0 ; y < input_scan_right_.at(i + 1); y++)
                {
                    v.push_back(0);
                }
            }

            next = true;
        }
        else
        {
            v.push_back(input_scan_right_.at(i));
        }

    }

    input_scan_right_.clear();

    if(v.size() == 0)
    {
        ROS_INFO("DECOMPRESSED DATA is EMPTY -> %d", (int)v.size());
        return;
    }

    if (v.size() > 512)
    {
        ROS_INFO("DECOMPRESSED DATA HAS TOO MANY DATA POINTS -> %d", (int)v.size());
        return;
    }

    right_mx.lock();

    buf.left_readings.push_back(v);
    new_data_right = true;

    right_mx.unlock();
    con_r.notify_all();

    int pos = 0;
    for (int y = 0; y < image.rows ; y++)
    {
        for (int x = 0; x < image.cols; x++)
        {
            if (pos >= v.size())
            {
                ROS_INFO("ACCESSING BEYOND THE DECOMPRESSED VECTOR pos -> %d  v.size() -> %d", (int)pos, (int)v.size());
                return;
            }

            image.at<uchar>(y,x) = v.at(pos);

            pos++;
        }
    }
}

void DECOMPRESSER::wake_con()
{
    con.notify_all();
    con_r.notify_all();
    con_left_image.notify_all();
    con_right_image.notify_all();
}

void DECOMPRESSER::left_reader()
{
    std::vector<uchar> data;
    force_generator left_generator;

    while(ros::ok())
    {
        std::vector<double> force_per_cell;
        cv::Mat image(ROWS,COLUMNS,CV_8U, cv::Scalar(0,0,0));

        std::unique_lock<std::mutex> lk(left_mx);

        while(!new_data_left && ros::ok())
        {
            con.wait(lk);
        }

        if(!ros::ok())
        {
            break;
        }

        new_data_left = false;

        if(buf.left_readings.size() != 0)
        {
            data = buf.left_readings.back();
            buf.left_readings.clear();
        }

        lk.unlock();

        double force = left_generator.pad_force(data, true, image, force_per_cell);

        mx_left_image.lock();

        img_buf.left_images.push_back(image);
        forces_left.push_back(force);
        new_image_left = true;
//        buf_per_cell.push_back(force_per_cell);

        mx_left_image.unlock();

        con_left_image.notify_all();


        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
        pub.publish(msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void DECOMPRESSER::right_reader()
{
    std::vector<uchar> data;
    force_generator right_generator;
    std::vector<double> force_per_cell;

    while(ros::ok())
    {
        cv::Mat image(ROWS,COLUMNS,CV_8U);
        std::unique_lock<std::mutex> lk(right_mx);

        while(!new_data_right && ros::ok())
        {
            con_r.wait(lk);
        }

        if(!ros::ok())
        {
            break;
        }

        new_data_right = false;

        if(buf.right_readings.size() != 0)
        {
            data = buf.right_readings.back();
            buf.right_readings.clear();
        }

        lk.unlock();

        double force = right_generator.pad_force(data, false, image, force_per_cell);

        mx_right_image.lock();

        img_buf.right_images.push_back(image);
        new_image_right = true;
        forces_right.push_back(force);

        mx_right_image.unlock();

        con_right_image.notify_all();


        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void DECOMPRESSER::left_rivet_detector()
{
    slip_detect slip_read;
    pressure_pad::pressure_read left_read;
    while(ros::ok())
    {
        cv::Mat image;

        std::unique_lock<std::mutex> lk(mx_left_image);

        while(!new_image_left && ros::ok())
        {
            con_left_image.wait(lk);
        }

        if(!ros::ok())
        {
            break;
        }

        new_image_left = false;

        if(img_buf.left_images.size() != 0)
        {
            image = img_buf.left_images.back();
            img_buf.left_images.clear();
        }
        else
        {
            continue;
        }
        buf_per_cell.clear();

        lk.unlock();

        if(is_safe(image, true, left_read, slip_read))
        {
            left_read.safe = true;
        }
        else
        {
            left_read.safe = false;
        }

        left_read.header.stamp = ros::Time::now();
        pub_left_.publish(left_read);


        left_read.rivet_pos.clear();
//        left_read.force_per_cell.clear();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void DECOMPRESSER::right_rivet_detector()
{
    slip_detect slip_read;
    pressure_pad::pressure_read right_read;
    while(ros::ok())
    {
        cv::Mat image;

        std::unique_lock<std::mutex> lk(mx_right_image);

        while(!new_image_right && ros::ok())
        {
            con_right_image.wait(lk);
        }

        if(!ros::ok())
        {
            break;
        }

        new_image_right = false;

        if(img_buf.right_images.size() != 0)
        {
            image = img_buf.right_images.back();
            img_buf.right_images.clear();
        }
        else
        {
            continue;
        }

        lk.unlock();

        if(is_safe(image, false, right_read, slip_read))
        {
            right_read.safe = true;
        }
        else
        {
            right_read.safe = false;
        }
        right_read.header.stamp = ros::Time::now();
        pub_right_.publish(right_read);

        right_read.rivet_pos.clear();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool DECOMPRESSER::is_safe(cv::Mat &image, bool is_left, pressure_pad::pressure_read &message, slip_detect &slip_read)
{
    cv::Mat blur;
    cv::GaussianBlur( image, blur, cv::Size(3, 3), 0 , 0 );

    cv::Mat resize = img_upscale(image, 20);

    cv::SimpleBlobDetector detector(params);
    cv::Mat img_thresh(resize.size(), CV_8UC3, cv::Scalar(0));
    cv::threshold(resize, img_thresh, 10, 255, CV_THRESH_BINARY);

    std::vector<cv::KeyPoint> keypoints;

    detector.detect(img_thresh, keypoints);

    cv::Mat img_rgb(resize.size(), CV_8UC3);

    cv::cvtColor(resize, img_rgb, CV_GRAY2RGB);

//    for(int i = 0; i < keypoints.size(); i++)
//    {
//        cv::KeyPoint pt = keypoints.at(i);

//        if(pt.size < 30)
//            cv::circle(img_rgb, weighted_average(resize, pt.pt, pt.size), 10, cv::Scalar(0,0,255));
//        else
//            cv::circle(img_rgb, weighted_average(resize, pt.pt, pt.size), 10, cv::Scalar(0,255,0));
//    }

//    cv::namedWindow("resize", cv::WINDOW_NORMAL);
//    cv::imshow("resize", img_rgb);

//    cv::namedWindow("thresh", cv::WINDOW_NORMAL);
//    cv::imshow("thresh", img_thresh);

//    cv::waitKey(3);

    int z = 0;

    for(int y = 0; y < image.rows; y++)
    {
        for(int x = 0; x < image.cols; x++)
        {
            if(blur.at<uchar>(y,x) != 0)
            {
                z++;
            }
        }
    }

    double force;

    while(forces_left.size() == 0 && is_left)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if(is_left)
    {
        mx_left_image.lock();
        force = forces_left.back();

        forces_left.clear();

        mx_left_image.unlock();

    }

    while(forces_right.size() == 0 && !is_left)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if(!is_left)
    {
        mx_right_image.lock();
        force = forces_right.back();

        forces_right.clear();

        mx_right_image.unlock();
    }

    if(z >= HIGH_THRESH || z <= LOW_THRESH)
    {
        if(force <= 1)
        {
            message.has_slipped = false;
        }

        message.pad_force = force;
        message.is_flat = true;
        message.rivet_count = 0;

        if(force >= DEFINED_FORCE)
            return true;
        else
            return false;
    }
    else
    {
        message.pad_force = force;
        message.is_flat = false;
        message.rivet_count = keypoints.size();

        if(keypoints.size() != 0)
        {
            std::vector<cv::Point2f> temp_pts;
            force = force/keypoints.size();

            for(int i = 0; i < keypoints.size(); i++)
            {
                geometry_msgs::Point pt;
                pt.x = keypoints[i].pt.x / 20 /** X_RATIO*/;
                pt.y = keypoints[i].pt.y / 20 /** Y_RATIO*/;
                pt.z = 0.0;

                message.rivet_pos.push_back(pt);

                temp_pts.push_back(keypoints.at(i).pt);
            }

            slip_read.push_points(temp_pts);

            if(slip_read.saved_points() >= BUF_COUNT)
            {
                if(slip_read.has_slipped())
                {
                    ROS_ERROR("SLIPPED");
                    message.has_slipped = true;
                }

                if(slip_read.saved_points() >= BUF_COUNT)
                {
                    slip_read.force_clear();
                }
            }

            std::vector<double> moments;

            for(int i = 0; i < keypoints.size(); i++)
            {
                double moment_run = 0;
                for(int x = 0; x < keypoints.size(); x++)
                {
                    if(x != i)
                    moment_run += force*sqrt(pow(((keypoints[x].pt.x - keypoints[i].pt.x))*X_RATIO, 2) +
                                             pow(((keypoints[x].pt.y - keypoints[i].pt.y))*Y_RATIO, 2));
                }

                moments.push_back(moment_run);
//                cv::circle(im_with_keypoints,keypoints[i].pt,1, cv::Scalar(0,0,255));
            }

            double average_moment = 0;
            for(int i = 0; i < moments.size(); i++)
            {
                average_moment += moments.at(i);
            }

            average_moment = average_moment/moments.size();

            std::cout << "AVG_MOMENT: " << average_moment << std::endl;

            if(average_moment >= DEFINED_MOMENT)
                return true;
            else
                return false;
        }
        else
            return false;
    }

}

cv::Mat DECOMPRESSER::img_upscale(cv::Mat &small_img, unsigned int scale_factor)
{
    cv::Mat larger_img(small_img.rows * scale_factor, small_img.cols*scale_factor, CV_8U, cv::Scalar(0));

    for(int y = 0; y < small_img.rows; y++)
    {
        for(int x = 0; x < small_img.cols; x++)
        {
            uchar cell = small_img.at<uchar>(y,x);

            for(int z = (y*scale_factor); z < (y*scale_factor) + (scale_factor); z++)
            {
                for(int t = (x*scale_factor) ; t < (x*scale_factor) + (scale_factor); t++)
                {
                    if(z >= 0 && z < larger_img.rows && t >=0 && t < larger_img.cols)
                    {
                        larger_img.at<uchar>(z,t) = cell;
                    }
                }
            }
        }
    }

    return larger_img;
}

cv::Point2f DECOMPRESSER::weighted_average(cv::Mat &resize, cv::Point2f &centre, double diameter)
{
    int avg = 0;
    double x_avg = 0;
    double y_avg = 0;

    for(int y = centre.y - (diameter/2); y < centre.y + (diameter/2); y++)
    {
        for(int x = centre.x - (diameter/2); x < centre.x + (diameter/2); x++)
        {
            if(x >= 0 && x < resize.cols && y >= 0 && y < resize.rows)
            {
                uchar cell = resize.at<uchar>(y,x);
                x_avg += x * cell;
                y_avg += y * cell;

                avg += cell;
            }
        }
    }

    cv::Point2f pt;
    pt.x = x_avg/avg;
    pt.y = y_avg/avg;

    return pt;
}
