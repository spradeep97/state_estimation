#include <iostream>
#include "LinearGaussianSystem.hpp"
#include "LinearMeasurementModel.hpp"
#include "KalmanFilter.cpp"

using Eigen::MatrixXd;

int main()
{
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd B = Eigen::MatrixXd::Identity(3,2);
    Eigen::MatrixXd R = 0.1 * Eigen::MatrixXd::Identity(3,3);
    se::LinearGaussianSystem system(A, B, R);

    Eigen::Matrix<double, 2, 3> C;
    C << 1, 0, 0, 0, 1, 0;
    Eigen::MatrixXd Q = 0.1 * Eigen::MatrixXd::Identity(2,2);
    se::LinearMeasurementModel measurement(C, Q);

    Eigen::Matrix<double, 3, 1> init_mu;
    Eigen::Matrix<double, 3, 3> init_sigma;

    init_mu << 0, 0, 0;
    init_sigma << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;

    

    se::KalmanFilter filter(system, measurement, std::make_pair(init_mu, init_sigma));

    Eigen::Vector2d u {1.0, 1.0};
    Eigen::Vector2d z {1.1, 0.9};
    filter.march(u, z);

    std::cout << filter.getMu() << std::endl;
    std::cout << filter.getSigma() << std::endl;
}