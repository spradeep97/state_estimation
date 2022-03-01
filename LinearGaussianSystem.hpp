#ifndef LINEAR_GAUSSIAN_SYSTEM_HPP
#define LINEAR_GAUSSIAN_SYSTEM_HPP

#include <cstddef>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>

namespace se
{
class LinearGaussianSystem
{
public:
    /// @brief default ctor
    LinearGaussianSystem():
        m_state_size(1U), m_ctrl_size(0U)
    {
        m_A = Eigen::MatrixXd::Identity(1U, 1U);
        m_B = Eigen::MatrixXd::Zero(1U, 1U);
        m_R = Eigen::MatrixXd::Zero(1U, 1U);
    }

    /// @brief Ctor
    /// @param state_size   size of state vector
    /// @param ctrl_size    size of control vector
    LinearGaussianSystem(std::size_t state_size, std::size_t ctrl_size):
        m_state_size(state_size),
        m_ctrl_size(ctrl_size)
    {
        m_A = Eigen::MatrixXd::Identity(m_state_size, m_state_size);
        m_B = Eigen::MatrixXd::Zero(m_state_size, m_ctrl_size);
        m_R = Eigen::MatrixXd::Zero(m_state_size, m_state_size);
    }

    /// @brief Ctor
    /// @param A    A matrix
    /// @param B    B matrix
    /// @param R    R matrix
    LinearGaussianSystem(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& R)
    {
        if (A.rows() != A.cols())
        {
            std::cerr << "Error! Attempting to initialize a non-square A matrix!\n";
            return;
        }

        std::size_t state_size = A.rows();

        if (B.rows() != state_size)
        {
            std::cerr << "Error! Attempting to initialize a non compatible control matrix!\n";
            return;
        }

        std::size_t ctrl_size = B.cols();

        if (R.rows()!= R.cols() || R.rows()!= state_size)
        {
            std::cerr << "Error! Attempting to initialize a non compatible R matrix!\n";
            return;
        }

        m_A = A;
        m_B = B;
        m_R = R;
        m_state_size = state_size;
        m_ctrl_size = ctrl_size;
    }

    /// @brief Mutator
    /// @param val
    /// @{
    void setA(const Eigen::MatrixXd& val)
    {
        if (val.rows() != m_state_size || val.cols() != m_state_size)
        {
            std::cerr << "Attempting to set invalid shape for A matrix!\n";
            return;
        }

        m_A = val;
    }
    void setB(const Eigen::MatrixXd& val)
    {
        if (val.rows() != m_state_size || val.cols() != m_ctrl_size)
        {
            std::cerr << "Attempting to set invalid shape for B matrix!\n";
            return;
        }

        m_B = val;
    }
    void setR(const Eigen::MatrixXd& val)
    {
        if (val.rows() != m_state_size || val.cols() != m_state_size)
        {
            std::cerr << "Attempting to set invalid shape for R matrix!\n";
            return;
        }

        m_R = val;
    }
    /// @}

    /// @brief Accessor
    /// @return Value
    /// @{
    std::size_t getStateSize()         const { return m_state_size; }
    std::size_t getCtrlSize()          const { return m_ctrl_size; }
    const Eigen::MatrixXd& getA()      const { return m_A; }
    const Eigen::MatrixXd& getB()      const { return m_B; }
    const Eigen::MatrixXd& getR()      const { return m_R; }
    /// @}

    /// @brief Function to print the system matrices
    void print() const
    {
        std::cout << "A matrix:\n" << m_A << std::endl;
        std::cout << "B matrix:\n" << m_B << std::endl;
        std::cout << "R matrix:\n" << m_R << std::endl; 
    }

private:
    std::size_t m_state_size;      // Size of the state vector
    std::size_t m_ctrl_size;       // Size of the control vector
    Eigen::MatrixXd m_A;    // State transition matrix
    Eigen::MatrixXd m_B;    // Control matrix
    Eigen::MatrixXd m_R;    // State transition covariance
};

} // namespace

#endif  // LINEAR_GAUSSIAN_SYSTEM_HPP