#ifndef KALMAN_FILTER
#define KALMAN_FILTER

#include <iostream>
#include <memory>
#include <eigen3/Eigen/Dense>

#include "LinearGaussianSystem.hpp"
#include "LinearMeasurementModel.hpp"

namespace se
{
class KalmanFilter
{
public:
    /// @brief Constructor
    /// @param system           LinearGaussianSystem object
    /// @param measurement      LinearMeasurementModel object
    /// @param init             Initial state. Pair of mu and sigma
    explicit KalmanFilter(const se::LinearGaussianSystem& system, const se::LinearMeasurementModel& measurement,
                          const std::pair<Eigen::VectorXd, Eigen::MatrixXd>& init):
                          m_system(system),
                          m_measurement(measurement),
                          m_initial_state(init)
    {
        m_mu = m_initial_state.first;
        m_sigma = m_initial_state.second;
    }

    /// @brief Kalman filter algorithm single step
    /// @param u    Control input
    /// @param z    Sensor Measurement
    void march(const Eigen::VectorXd& u, const Eigen::VectorXd& z);

    /// @brief Mutator
    /// @param val
    /// @{
    void setSystem(const se::LinearGaussianSystem& val)                             { m_system = val; }
    void setMeasurement(const se::LinearMeasurementModel& val)                      { m_measurement = val; }
    void setInitialState(const std::pair<Eigen::VectorXd, Eigen::MatrixXd>& val)    { m_initial_state = val; }
    /// @}

    /// @brief Accessor
    /// @return Value
    /// @{
    const se::LinearGaussianSystem& getSystem()                             const { return m_system; }
    const se::LinearMeasurementModel& getMeasurement()                      const { return m_measurement; }
    const std::pair<Eigen::VectorXd, Eigen::MatrixXd> getInitialState()     const { return m_initial_state; }
    const Eigen::VectorXd& getMu()                                          const { return m_mu; }
    const Eigen::MatrixXd& getSigma()                                       const { return m_sigma; }
    /// @}

private:
    se::LinearGaussianSystem m_system;                              // The linear system dynamics
    se::LinearMeasurementModel m_measurement;                       // The linear measurement model
    std::pair<Eigen::VectorXd, Eigen::MatrixXd> m_initial_state;    // Initial state of the system
    Eigen::VectorXd m_mu;                                           // Currently maintained best guess of the state
    Eigen::MatrixXd m_sigma;                                        // Currently maintained covariance of the state
};
}   // namespace

#endif // KALMAN_FILTER