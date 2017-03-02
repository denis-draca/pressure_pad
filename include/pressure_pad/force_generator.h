#ifndef FORCE_GENERATOR_H
#define FORCE_GENERATOR_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <mutex>
#include <thread>
#include <iostream>

#define COLUMNS         16
#define ROWS            32

typedef unsigned char uchar;
class force_generator
{
private:
    double constant_R = 1.0/5000.0;

    std::mutex mx_left; //!< Locks the resistor map in resistor_Convert method when the left thread is accessing it
    std::mutex mx_right; //!< Locks the resistor map in resistor_Convert method when the right thread is accessing it

private:

    double calc_force(std::vector<std::vector<double> > map);

    Eigen::MatrixXd constant_16by16(int value);
    Eigen::MatrixXd k_creator(Eigen::MatrixXd &ones, std::vector<double> &row);

    std::vector<std::vector<double> > make_aligned(std::vector<uchar> &reading);
    std::vector<double> resistor_row(Eigen::MatrixXd &result);

    void resistor_convert(Eigen::MatrixXd &all_ones,
                          std::vector<double> &aligned,
                          Eigen::MatrixXd &R,
                          std::vector< std::vector<double> > &resistor_map,
                          bool left);

public:
    force_generator();

    double pad_force(std::vector<uchar> &reading, bool left);
};

#endif // FORCE_GENERATOR_H
