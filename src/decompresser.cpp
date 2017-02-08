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

    pub_left_ = it_.advertise("/wallpusher/decompressed/left",1);
    pub_right_= it_.advertise("/wallpusher/decompressed/right",1);

    left_force_ = n_.advertise<std_msgs::Float32>("/wallpusher/force/left", 1);
    right_force_ = n_.advertise<std_msgs::Float32>("/wallpusher/force/right", 1);

    sub_left_ = n_.subscribe("/wallpusher/raw_scan/left", 1, &DECOMPRESSER::left_scan, this);
    sub_right_= n_.subscribe("/wallpusher/raw_scan/right", 1, &DECOMPRESSER::right_scan, this);
}

void DECOMPRESSER::wake_con()
{
    con.notify_all();
    con_r.notify_all();
}

/***********************************************************************
 ********************            ***************************************
 ******************** LEFT SCAN  ***************************************
 ********************            ***************************************
 ***********************************************************************
 */

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
                      std::ref(aligned[i]), //aligned reading vector of vectors
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
        std::cout << "\033[1;31m\nFront left Force: \033[0m" << total_force/9.81;
        return total_force;
    }
    else
    {
        double total_force = calc_force(resistor_map);
        std::cout << "\033[1;31m\nFront Right Force: \033[0m" << total_force/9.81;
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

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
    pub_left_.publish(msg);

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

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
    pub_right_.publish(msg);

    cv::namedWindow("FOUND_RIGHT",cv::WINDOW_NORMAL);
    cv::imshow("FOUND_RIGHT",image);
    cv::waitKey(3);
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
    std_msgs::Float32 force_send;
    std::vector<uchar> data;
    while(ros::ok())
    {
        std::unique_lock<std::mutex> lk(left_mx);

        while(!new_data_left && ros::ok())
        {
            con.wait(lk);
        }

        new_data_left = false;

        if(buf.left_readings.size() != 0)
        {
            data = buf.left_readings.back();
            buf.left_readings.resize(buf.left_readings.size() - 1);
        }

        lk.unlock();

        double force = pad_force(data, true);

        force_send.data = force;

        left_force_.publish(force_send);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void DECOMPRESSER::right_reader()
{
    std_msgs::Float32 force_send;
    std::vector<uchar> data;
    while(ros::ok())
    {
        std::unique_lock<std::mutex> lk(right_mx);

        while(!new_data_right && ros::ok())
        {
            con_r.wait(lk);
        }

        new_data_right = false;

        if(buf.right_readings.size() != 0)
        {
            data = buf.right_readings.back();
            buf.right_readings.resize(buf.right_readings.size() - 1);
        }

        lk.unlock();

        double force = pad_force(data, false);

        force_send.data = force;

        right_force_.publish(force_send);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
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

int DECOMPRESSER::find_max(std::vector<uchar> &v)
{
    int max_value = v.front();

    for(int i = 0; i < v.size(); i++)
    {

        if (v.at(i) > max_value)
        {
            max_value = v.at(i);
        }
    }

    return max_value;
}
