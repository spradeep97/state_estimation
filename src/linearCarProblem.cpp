#include <iostream>
#include "LinearGaussianSystem.hpp"
#include "LinearMeasurementModel.hpp"
#include "KalmanFilter.cpp"

using Eigen::MatrixXd;

int main()
{
    double dt = 1.0;
    Eigen::Matrix2d A; A << 1, dt, 0, 1;
    Eigen::Matrix2d R; R << std::pow(dt,4) / 4, std::pow(dt,3) / 2, std::pow(dt,3) / 2, std::pow(dt,2);
    se::LinearGaussianSystem system(A, R);

    Eigen::Matrix<double, 1, 2> C;
    C << 1, 0;
    Eigen::Matrix<double, 1, 1> Q; Q << 10;
    se::LinearMeasurementModel measurement(C, Q);

    Eigen::Matrix<double, 2, 1> init_mu;
    Eigen::Matrix<double, 2, 2> init_sigma;

    init_mu << 0, 0;
    init_sigma << 0, 0, 0, 0;

    Eigen::Matrix<double, 1, 1> z;

    se::KalmanFilter filter(system, measurement, std::make_pair(init_mu, init_sigma));

    for (int i = 0; i < 5; i++)
    {
        filter.predict();
        filter.setMu(filter.getMuPred());
        filter.setSigma(filter.getSigmaPred());
    }

    z = Eigen::Matrix<double, 1, 1>(5);

    filter.correct(z);
    
    std::cout << "mu:\n" << filter.getMu() << "\n---\n";
    std::cout << "sigma:\n" << filter.getSigma() << "\n---" << std::endl; 
}