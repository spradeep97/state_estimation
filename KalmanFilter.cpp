#include "KalmanFilter.hpp"

namespace se
{

void KalmanFilter::march(const Eigen::VectorXd& u, const Eigen::VectorXd& z)
{
    predict(u);
    correct(z);
}

void KalmanFilter::predict(const Eigen::VectorXd& u)
{
    // Prediction step
    m_mu_pred = m_system.getA() * m_mu + m_system.getB() * u;
    m_sigma_pred = m_system.getA() * m_sigma * m_system.getA().transpose() + m_system.getR();
}

void KalmanFilter::predict()
{
    // Prediction step
    m_mu_pred = m_system.getA() * m_mu;
    m_sigma_pred = m_system.getA() * m_sigma * m_system.getA().transpose() + m_system.getR();
}

void KalmanFilter::correct(const Eigen::VectorXd& z)
{
    // Calculate kalman gain
    Eigen::MatrixXd term1 = m_sigma_pred * m_measurement.getC().transpose();
    Eigen::MatrixXd K = term1 * (m_measurement.getC() * term1 + m_measurement.getQ()).inverse();

    // Correction step
    m_mu = m_mu_pred + K * (z - m_measurement.getC() * m_mu_pred);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m_system.getStateSize(), m_system.getStateSize());
    m_sigma = (I - K * m_measurement.getC()) * m_sigma_pred;
}

}   // namespace