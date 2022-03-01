#ifndef LINEAR_MEASUREMENT_MODEL
#define LINEAR_MEASUREMENT_MODEL

#include <cstddef>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>

namespace se
{
class LinearMeasurementModel
{
public:
    /// @brief default ctor
    LinearMeasurementModel():
        m_state_size(1U), m_measurement_size(1U)
    {
        m_C = Eigen::MatrixXd::Identity(m_measurement_size, m_state_size);
        m_Q = Eigen::MatrixXd::Identity(m_measurement_size, m_measurement_size);
    }

    /// @brief Ctor
    /// @param state_size       size of the state vector
    /// @param measurement_size size of the measurement vector
    LinearMeasurementModel(std::size_t state_size, std::size_t measurement_size):
        m_state_size(state_size),
        m_measurement_size(measurement_size)
    {
        m_C = Eigen::MatrixXd::Identity(m_measurement_size, m_state_size);
        m_Q = Eigen::MatrixXd::Identity(m_measurement_size, m_measurement_size);
    }

    /// @brief Ctor
    /// @param C    C matrix
    /// @param Q    Q matrix
    LinearMeasurementModel(const Eigen::MatrixXd& C, const Eigen::MatrixXd& Q)
    {
        std::size_t state_size = C.cols();
        std::size_t measurement_size = C.rows();

        if (Q.rows() != Q.cols() || Q.rows() != measurement_size)
        {
            std::cerr << "Error! Attempting to initialize a non compatible Q matrix!\n";
            return;
        }

        m_state_size = state_size;
        m_measurement_size = measurement_size;
        m_C = C;
        m_Q = Q;
    }

    /// @brief Mutator
    /// @param val
    /// @{
    void setC(const Eigen::MatrixXd& val)
    {
        if (val.rows() != m_measurement_size || val.cols() != m_state_size)
        {
            std::cerr << "Attempting to set invalid shape for C matrix!\n";
            return;
        }

        m_C = val;
    }

    void setQ(const Eigen::MatrixXd& val)
    {
        if (val.rows() != m_measurement_size || val.cols() != m_measurement_size)
        {
            std::cerr << "Attempting to set invalid shape for Q matrix!\n";
            return;
        }

        m_Q = val;
    }
    /// @}

    /// @brief Accessor
    /// @return Value
    /// @{
    std::size_t getStateSize()          const { return m_state_size; }
    std::size_t getMeasurementSize()    const { return m_measurement_size; }
    const Eigen::MatrixXd& getC()       const { return m_C; }
    const Eigen::MatrixXd& getQ()       const { return m_Q; }
    /// @}

    /// @brief Function to print the measurement model matrices
    void print() const
    {
        std::cout << "C matrix:\n" << m_C << std::endl;
        std::cout << "Q matrix:\n" << m_Q << std::endl;
    }

private:
    std::size_t m_state_size;           // Size of the state vector
    std::size_t m_measurement_size;     // Size of the measurement vector
    Eigen::MatrixXd m_C;                // Measurement matrix
    Eigen::MatrixXd m_Q;                // Measurement covariance matrix
};
}   // namespace

#endif // LINEAR_MEASUREMENT_MODEL