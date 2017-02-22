#include "pressure_pad/decompresser.h"


/***********************************************************************
 ********************             ***************************************
 ******************** CONSTRUCTOR ***************************************
 ********************             ***************************************
 ***********************************************************************
 */

DECOMPRESSER::DECOMPRESSER(ros::NodeHandle &n):
    n_(n), it_(n)
{
    constant_R = 1.0/5000.0;

    pub_left_ = n_.advertise<pressure_pad::pressure_read>("/wallpusher/reading/left",1);
    pub_right_= n_.advertise<pressure_pad::pressure_read>("/wallpusher/reading/right",1);

    sub_left_ = n_.subscribe("/wallpusher/raw_scan/left", 1, &DECOMPRESSER::left_scan, this);
    sub_right_= n_.subscribe("/wallpusher/raw_scan/right", 1, &DECOMPRESSER::right_scan, this);
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
        ROS_INFO("No Data Transmitted");
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
        ROS_INFO("TOO MANY DATA POINTS SENT -> %d", (int)input_scan_left_.size());
        return;
    }

    cv::Mat image(32,16,CV_8U);

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
        ROS_INFO("DECOMPRESSED DATA is EMPTY -> %d", (int)v.size());
        return;
    }

    if (v.size() > 512)
    {
        ROS_INFO("DECOMPRESSED DATA HAS TOO MANY DATA POINTS -> %d", (int)v.size());
        return;
    }

    buf.left_readings.push_back(v);
    new_data_left = true;
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

    mx_left_image.lock();

    img_buf.left_images.push_back(image);
    new_image_left = true;

    mx_left_image.unlock();

    con_left_image.notify_all();

//    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
//    pub_left_.publish(msg);

    cv::namedWindow("FOUND",cv::WINDOW_NORMAL);
    cv::imshow("FOUND",image);
    cv::waitKey(3);
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

    cv::Mat image(32,16,CV_8U);

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

    buf.left_readings.push_back(v);
    new_data_right = true;
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

//    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
//    pub_right_.publish(msg);

    cv::namedWindow("FOUND_RIGHT",cv::WINDOW_NORMAL);
    cv::imshow("FOUND_RIGHT",image);
    cv::waitKey(3);
}

void DECOMPRESSER::wake_con()
{
    con.notify_all();
    con_r.notify_all();
    con_left_image.notify_all();
    con_right_image.notify_all();
}

double DECOMPRESSER::pad_force(std::vector<uchar> &reading, bool left)
{
    std::vector<std::thread> thread_container;

    Eigen::MatrixXd R(16,1);

    for(int i = 0; i < 16; i++)
    {
        R(i,0) = (1.0/5000.0) * -1;
    }

    Eigen::MatrixXd all_ones = constant_16by16(1);

    bool broke = false;

    std::vector < std::vector<double> > aligned = make_aligned(reading);
    std::vector < std::vector<double> > resistor_map;

    for(int i = 0; i < aligned.size(); i++)
    {
        std::thread t(&DECOMPRESSER::resistor_convert, this,
                      std::ref(all_ones), //list of ones
                      std::ref(aligned[i]), //row from aligned reading vector of vectors
                      std::ref(R), //Constant
                      std::ref(resistor_map), //changed by reference, new multiarray containing the resistor values instead of ADC
                      left); //true if the reading comes from the left pad

        thread_container.push_back(std::move(t));
    }

    for(int i = 0; i < thread_container.size(); i++)
    {
        thread_container[i].join();
    }

    if(left)
    {
        double total_force = calc_force(resistor_map);
        std::cout << "\033[1;31m\nFront left Force: \033[0m" << total_force/9.81 <<std::endl;
        return total_force;
    }
    else
    {
        double total_force = calc_force(resistor_map);
        std::cout << "\033[1;31m\nFront Right Force: \033[0m" << total_force/9.81 << std::endl;
        return total_force;
    }
}

void DECOMPRESSER::resistor_convert(Eigen::MatrixXd &all_ones, std::vector<double> &aligned, Eigen::MatrixXd &R,
                                    std::vector< std::vector<double> > &resistor_map, bool left)
{
    Eigen::MatrixXd k_values = k_creator(std::ref(all_ones), std::ref(aligned));

    Eigen::MatrixXd resistors =  k_values.fullPivLu().solve(R);

    if(left)
    {
        mx_left.lock();
        resistor_map.push_back(resistor_row(resistors));
        mx_left.unlock();
    }
    else
    {
        mx_right.lock();
        resistor_map.push_back(resistor_row(resistors));
        mx_right.unlock();
    }
}


std::vector<std::vector<double> > DECOMPRESSER::make_aligned(std::vector<uchar> &reading)
{
    std::vector< std::vector<double> > aligned;

    for(int y = 0; y < 32; y++)
    {
        std::vector<double> temp;
        for(int x = 16*y; x < y*16 + 16; x++)
        {
            temp.push_back((double)reading.at(x));
        }
        aligned.push_back(temp);
    }
    return aligned;
}

double DECOMPRESSER::calc_force(std::vector<std::vector<double> > map)
{
    double force = 0;
    for(int x = 0; x < map.size(); x++)
    {
        for(int y = 0; y < map[x].size(); y++)
        {
            force += exp(-0.964637491289655*log(1/map[x][y])+12.2205540906434);
        }
    }

    return force;
}

std::vector<double> DECOMPRESSER::resistor_row(Eigen::MatrixXd &result)
{
    std::vector<double> res_row;

    for(int i = 0; i < 16; i++)
    {
        res_row.push_back(result(i,0));
    }

    return res_row;
}

Eigen::MatrixXd DECOMPRESSER::k_creator(Eigen::MatrixXd &ones, std::vector<double> &row)
{
    Eigen::MatrixXd k_value = ones;
    for(int j = 0; j < row.size(); j++)
    {
        double voltage = (3.3*row.at(j))/255;
        double k = (3.3/voltage) - 1;

        k_value(j,j) = -1*k;
    }

    return k_value;
}

void DECOMPRESSER::left_reader()
{
    std::vector<uchar> data;

    while(ros::ok())
    {
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

        double force = pad_force(data, true);

        forces_left.push_back(force);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void DECOMPRESSER::right_reader()
{
    std::vector<uchar> data;

    while(ros::ok())
    {
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

        double force = pad_force(data, false);

        forces_right.push_back(force);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void DECOMPRESSER::left_rivet_detector()
{
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

        lk.unlock();

        if(is_safe(image, true, left_read))
        {
            left_read.header.stamp = ros::Time::now();
            left_read.safe = true;
            pub_left_.publish(left_read);
        }
        else
        {
            left_read.header.stamp = ros::Time::now();
            left_read.safe = false;
            pub_left_.publish(left_read);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void DECOMPRESSER::right_rivet_detector()
{
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

        if(is_safe(image, false, right_read))
        {
            right_read.header.stamp = ros::Time::now();
            right_read.safe = true;
            pub_right_.publish(right_read);
        }
        else
        {
            right_read.header.stamp = ros::Time::now();
            right_read.safe = false;
            pub_right_.publish(right_read);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool DECOMPRESSER::is_safe(cv::Mat &image, bool is_left, pressure_pad::pressure_read &message)
{
    cv::Mat binary_image;
    cv::Mat binary_image_test;
    cv::Mat blur;
    cv::Mat im_with_keypoints(32,16,CV_8UC3,cv::Scalar(0,0,0));

    cv::GaussianBlur( image, blur, cv::Size(3, 3), 0 , 0 );

    cv::threshold(blur, binary_image, 10, 255, cv::THRESH_BINARY);
    cv::threshold(blur, binary_image_test, 1, 255, cv::THRESH_BINARY);

    cv::SimpleBlobDetector::Params params;

    params.filterByArea = true;
    params.minArea = 1;
    params.maxArea = 1000;
    params.filterByColor = true;
    params.blobColor = 255;
    params.filterByConvexity = false;
    params.filterByInertia = false;
    params.filterByCircularity = false;

    cv::SimpleBlobDetector detector(params);

    std::vector<cv::KeyPoint> keypoints;
    detector.detect( binary_image, keypoints);

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

//    cv::drawKeypoints( im_with_keypoints, keypoints, im_with_keypoints,
//                       cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

//    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", im_with_keypoints).toImageMsg();

//    if(is_left)
//    {
//        pub_left_.publish(msg);
//    }
//    else
//    {
//        pub_right_.publish(msg);
//    }

    if(z >= 130 || z <= 6)
    {
        std::cout << "FLAT SURFACE \n";

        double force;

        while(forces_left.size() == 0 && is_left)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        while(forces_right.size() == 0 && !is_left)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        if(is_left)
        {
            force = forces_left.back()/3.0;
        }
        else
        {
            force = forces_right.back()/3.0;
        }

        message.pad_force = force;
        message.is_flat = true;
        message.rivet_count = 0;

        if(force >= 1300)
        {
            return true;
        }
        else
            return false;
    }
    else
    {
        std::cout << "SURFACE WITH RIVETS: " << z << std::endl;

        if(keypoints.size() != 0)
        {

            for(int i = 0; i < keypoints.size(); i++)
            {
                geometry_msgs::Point pt;
                pt.x = keypoints[i].pt.x;
                pt.y = keypoints[i].pt.y;
                pt.z = 0;

                message.rivet_pos.push_back(pt);
            }

            double force;
            while(forces_left.size() == 0 && is_left)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            while(forces_right.size() == 0 && !is_left)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            if(is_left)
            {
                force = forces_left.back()/keypoints.size();
            }
            else
            {
                force = forces_right.back()/keypoints.size();
            }

            message.pad_force = force;

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
            }

            double average_moment = 0;
            for(int i = 0; i < moments.size(); i++)
            {
                average_moment += moments.at(i);
            }

            average_moment = average_moment/moments.size();

            std::cout << "AVG_MOMENT: " << average_moment << std::endl;

            message.is_flat = false;
            message.rivet_count = keypoints.size();

            if(average_moment >= DEFINED_MOMENT)
                return true;
            else
                return false;
        }
    }
}

Eigen::MatrixXd DECOMPRESSER::constant_16by16(int value)
{
    Eigen::MatrixXd cells(16,16);
    for(int i = 0; i < 16; i++)
    {
        for(int j = 0; j < 16; j++)
        {
            cells(i,j) = value;
        }
    }

    return cells;
}
