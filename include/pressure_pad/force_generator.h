#ifndef FORCE_GENERATOR_H
#define FORCE_GENERATOR_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <mutex>
#include <thread>
#include <opencv2/opencv.hpp>
#include <map>
//#include <iostream>
#include <condition_variable>

#define COLUMNS         16
#define ROWS            32

typedef unsigned char uchar;

///
/// \brief The force_generator class
///
///        Performs all the force calculations and conversion required to properly use the pressure pad sensor
///


class force_generator
{
private:
    double constant_R = 1.0/5000.0; //!< Constant value used during the conversion from ADC to resistance

    std::mutex mx_left; //!< Locks the resistor map in resistor_Convert method when the left thread is accessing it
    std::mutex mx_right; //!< Locks the resistor map in resistor_Convert method when the right thread is accessing it

    Eigen::MatrixXd R;

    std::vector<std::thread> _thread_container_left;
    std::vector<std::thread> _thread_container_right;


    std::vector < std::vector<double> > aligned_left;
    std::vector < std::vector<double> > aligned_right;
    std::vector < std::vector<double> > resistor_map_left;
    std::vector < std::vector<double> > resistor_map_right;

    int _position_left = 0;
    int _position_right = 0;

    std::mutex _process_lock_left;
    std::mutex _process_lock_right;

    bool _left;
    bool data_left;
    bool data_right;
    bool _alive;

    std::condition_variable _con_left;
    std::condition_variable _con_right;

    std::vector< std::thread::id> _left_id;
    std::vector< std::thread::id> _right_id;

    Eigen::MatrixXd all_ones;


//    std::vector < std::vector<double> > resistor_map;

//    std::vector< std::map<std::thread::id, unsigned int> > _left_ids;
//    std::vector< std::map<std::thread::id, unsigned int> > _right_ids;


private:
    ///
    /// \brief calc_force - This method will generate the force at each cell point. It will also draw an image based on the calcualted forces
    /// \param map - the list of resistors at each cell point
    /// \param image - reference of the image that will be modified, this is the image that will be built based on the new forces
    /// \param force_per_cell - vector containing each of the individual forces. This is done as a single long line of forces
    /// \return (double) - the total force on the footpad
    ///
    double calc_force(cv::Mat &image, std::vector<double> &force_per_cell);

    ///
    /// \brief constant_16by16 - generates a 16x16 matrix containing all the same values
    /// \param value - the contained value
    /// \return (Eigen::MatrixXd) - object containing the matrix
    ///
    Eigen::MatrixXd constant_16by16(int value);

    ///
    /// \brief k_creator - generates the diagonal values in a 16x16 matrix
    /// \param ones - a 16x16 matrix containing all 1's
    /// \param row - vector containing the ADC values for a particular row
    /// \return (Eigen::MatrixXd) - 16x16 matrix with all the K values calculated
    ///
    Eigen::MatrixXd k_creator(Eigen::MatrixXd &ones, std::vector<double> &row);

    ///
    /// \brief make_aligned - Takes an unorganised, one dimensional array containing all the ADC readings and aligns them into a 2-dimensinal array
    /// \param reading - the unorganised reading
    /// \return ( std::vector<std::vector<double> >) - organised vector
    ///
    std::vector<std::vector<double> > make_aligned(std::vector<uchar> &reading);

    ///
    /// \brief resistor_row - converts eignen matrix to vector
    /// \param result - post resistor calculation result
    /// \return (std::vector<double>) - vector containing the resistors in certain row
    ///
    std::vector<double> resistor_row(Eigen::MatrixXd &result);

    ///
    /// \brief resistor_convert - Takes a row of readings containing the raw ADC values and converts them to a resistance reading
    /// \param all_ones - 16x16 vector containing only 1's
    /// \param aligned - a single row from the vector containing the aligned readings
    /// \param resistor_map - a vector passed in by reference, this will contain all the results from the multiple rows
    /// \param left - boolean thats true when its the left footpad
    /// \param i - row position
    ///
    void resistor_convert(Eigen::MatrixXd &all_ones,
                          std::vector<double> &aligned,
                          std::vector< std::vector<double> > &resistor_map, int i);

    void resistor_convert_v2_left();
    void resistor_convert_v2_right();

public:
    force_generator(bool left);
    ~force_generator();

    ///
    /// \brief pad_force - performs all the neccessary force calculations and conversions
    /// \param reading - raw adc reading
    /// \param left - true if its the left pad
    /// \param image - image that will be modified using the force values
    /// \param force_per_cell - vector that will be modified with all the raw force readings
    /// \return (double) - total force on the pad
    ///
    double pad_force(std::vector<uchar> &reading, cv::Mat &image, std::vector<double> &force_per_cell);
};

#endif // FORCE_GENERATOR_H
