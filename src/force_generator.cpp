#include "pressure_pad/force_generator.h"

force_generator::force_generator()
{
    Eigen::MatrixXd R_temp(COLUMNS,1);

    for(int i = 0; i < COLUMNS; i++)
    {
        R_temp(i,0) = (constant_R) * -1;
    }

    R = R_temp;
}

double force_generator::pad_force(std::vector<uchar> &reading, bool left, cv::Mat &image, std::vector<double> &force_per_cell)
{
    std::vector<std::thread> thread_container;

    Eigen::MatrixXd all_ones = constant_16by16(1);

    std::vector < std::vector<double> > aligned = make_aligned(reading);
    std::vector < std::vector<double> > resistor_map;

    for(int i = 0; i < aligned.size(); i++)
    {
        std::thread t(&force_generator::resistor_convert, this,
                      std::ref(all_ones), //list of ones
                      std::ref(aligned[i]), //row from aligned reading vector of vectors
                      std::ref(resistor_map), //changed by reference, new multiarray containing the resistor values instead of ADC
                      left, i); //true if the reading comes from the left pad

        thread_container.push_back(std::move(t));
    }

    for(int i = 0; i < thread_container.size(); i++)
    {
        thread_container[i].join();
    }

    return calc_force(resistor_map, image, force_per_cell);
}

void force_generator::resistor_convert(Eigen::MatrixXd &all_ones, std::vector<double> &aligned,
                                       std::vector< std::vector<double> > &resistor_map, bool left, int i)
{
    Eigen::MatrixXd k_values = k_creator(std::ref(all_ones), std::ref(aligned));

    Eigen::MatrixXd resistors =  k_values.fullPivLu().solve(R);

    while(resistor_map.size() != i)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

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


std::vector<std::vector<double> > force_generator::make_aligned(std::vector<uchar> &reading)
{
    std::vector< std::vector<double> > aligned;

    for(int y = 0; y < ROWS; y++)
    {
        std::vector<double> temp;
        for(int x = COLUMNS*y; x < y*COLUMNS + COLUMNS; x++)
        {
            temp.push_back((double)reading.at(x));
        }
        aligned.push_back(temp);
    }
    return aligned;
}

double force_generator::calc_force(std::vector<std::vector<double> > map, cv::Mat &image, std::vector<double> &force_per_cell)
{
    double force = 0;

    for(int x = 0; x < map.size(); x++)
    {
        for(int y = 0; y < map[x].size(); y++)
        {
            double temp_force = exp(-0.964637491289655*log(1/map[x][y])+12.2205540906434);
            force += temp_force;
//            force_per_cell.push_back(temp_force);

            image.at<uchar>(x,map[x].size() - 1 - y) = (uchar)temp_force;

        }
    }

    for(int x = map.size() - 1; x >= 0; x--)
    {
        for(int y = map[x].size() - 1; y >= 0; y--)
        {
            double temp_force = exp(-0.964637491289655*log(1/map[x][y])+12.2205540906434);
            force_per_cell.push_back(temp_force);
        }
    }

    return force;
}

Eigen::MatrixXd force_generator::constant_16by16(int value)
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

std::vector<double> force_generator::resistor_row(Eigen::MatrixXd &result)
{
    std::vector<double> res_row;

    for(int i = 0; i < COLUMNS; i++)
    {
        res_row.push_back(result(i,0));
    }

    return res_row;
}

Eigen::MatrixXd force_generator::k_creator(Eigen::MatrixXd &ones, std::vector<double> &row)
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
