#include "KalmanFilter.hpp"

namespace se
{

void KalmanFilter::march(const Eigen::VectorXd& u, const Eigen::VectorXd& z)
{
    // Prediction step
    Eigen::VectorXd mu_pred = m_system.getA() * m_mu + m_system.getB() * u;
    Eigen::MatrixXd sigma_pred = m_system.getA() * m_sigma * m_system.getA().transpose() + m_system.getR();

    // Calculate kalman gain
    Eigen::MatrixXd term1 = sigma_pred * m_measurement.getC().transpose();
    Eigen::MatrixXd K = term1 * (m_measurement.getC() * term1 + m_measurement.getQ()).inverse();

    // Correction step
    m_mu = mu_pred + K * (z - m_measurement.getC() * mu_pred);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m_system.getStateSize(), m_system.getStateSize());
    m_sigma = (I - K * m_measurement.getC()) * sigma_pred;
}

}   // namespace