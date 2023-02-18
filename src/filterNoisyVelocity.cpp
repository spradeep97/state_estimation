#include <cstddef>
#include <ios>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "KalmanFilter.hpp"
#include "LinearGaussianSystem.hpp"
#include "LinearMeasurementModel.hpp"



std::tuple<std::vector<double>, std::vector<double>> parse1DTimeseries(std::string filename)
{
    std::fstream fin;

    fin.open(filename, std::ios::in);

    std::cout << "Opened " << filename << std::endl;

    std::vector<std::string> row;

    std::string line, word, temp;

    std::vector<double> data, timestamp;

    int row_count = 1;

    while (fin.peek() != EOF)
    {
        row.clear();

        std::getline(fin, line);

        std::stringstream s(line);

        while(std::getline(s, word, ','))
        {
            row.push_back(word);
        }

        data.push_back(std::stod(row[0]));
        timestamp.push_back(std::stod(row[1]));

        ++row_count;
    }

    return std::make_tuple(data, timestamp);
}


int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        std::cout << "Incorrect number of arguments! Usage : " ;
        std::cout << argv[0] << " <path to 1d timeseries csv>";
        std::cout << " <output path>" << std::endl;
        return 1;
    }

    std::string filename = argv[1];
    std::string output_path = argv[2];

    std::tuple<std::vector<double>, std::vector<double>> timeseries = parse1DTimeseries(filename);
    std::cout << "Size of data : " << std::get<0>(timeseries).size() << std::endl;

    // Assuming constant dt
    std::vector<double> time_vec = std::get<1>(timeseries);
    // double dt = (time_vec.back() - time_vec.front()) / time_vec.size();
    double dt = 0.05;

    std::cout << "Avg dt : " << dt << std::endl;

    // Build a linear gaussian system : x_(t+1) = Ax(t) + R ; no control input to the system. So no use of B matrix
    // State vector = [pos, vel, acc]
    // pos(t+1) = pos(t) + vel(t)*dt
    // vel(t+1) = vel(t) + acc(t)*dt
    // acc(t+1) = acc(t)    => Constant acceleration assumption
    // Thus, A(t) = [[1, dt, 0],[0, 1, dt],[0, 0, 1]]
    
    Eigen::Matrix<double, 3, 3> A;
    A << 1.0, dt, 0.0, 0.0, 1.0, dt, 0.0, 0.0, 1.0;

    Eigen::Matrix<double, 3, 3> R;
    double cov_pred = 1E-4;
    R << cov_pred, 0.0, 0.0, 0.0, cov_pred, 0.0, 0.0, 0.0, cov_pred;

    se::LinearGaussianSystem system(A,R);


    // Next build a linear measurement model.
    Eigen::Matrix<double, 1, 3> C;
    C << 1.0, 0.0, 0.0;
    
    double cov_meas = 1.0;
    Eigen::Matrix<double, 1, 1> Q;
    Q << cov_meas;
    se::LinearMeasurementModel measurement_model(C, Q);

    Eigen::Matrix<double, 3, 1> init_mu;
    Eigen::Matrix<double, 3, 3> init_sigma;

    init_mu << 0.0, 0.0, 0.01;
    init_sigma << 1E-4, 0.0, 0.0, 0.0, 1E-1, 0.0, 0.0, 0.0, 1E-1;

    se::KalmanFilter filter(system, measurement_model, std::make_pair(init_mu, init_sigma));


    // Now go through the data
    std::vector<double> data_vec = std::get<0>(timeseries);

    std::vector<double> pos_estimate, vel_estimate, acc_estimate;

    std::ofstream out_file(output_path + "/filtered_data.csv");

    std::size_t index = 0;

    for (double s : data_vec)
    {
        filter.predict();
        Eigen::Matrix<double, 1, 1> z(s);
        filter.correct(z);

        Eigen::Matrix<double, 3, 1> state_estimate = filter.getMu();
        pos_estimate.push_back(state_estimate(0));
        vel_estimate.push_back(state_estimate(1));
        acc_estimate.push_back(state_estimate(2));

        out_file << state_estimate(0) << "," << state_estimate(1) << "," << state_estimate(2) << "," << time_vec[index] << "\n";

        ++index;
    }

    std::cout << "Final pos estimate : " << pos_estimate.back() << std::endl;
    std::cout << "Final vel estimate : " << vel_estimate.back() << std::endl;

    out_file.close();
    return 0;
}