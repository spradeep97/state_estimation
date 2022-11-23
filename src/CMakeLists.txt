cmake_minimum_required (VERSION 3.0)
project (state_estimation)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
 
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_subdirectory(kalman_filter)

# --------Building the linearCarProblem executable-------- #
add_executable (linearCarProblem linearCarProblem.cpp)

target_link_libraries (linearCarProblem Eigen3::Eigen KalmanFilter)

target_include_directories(linearCarProblem PUBLIC
                          "${PROJECT_BINARY_DIR}"
                          "${PROJECT_SOURCE_DIR}/kalman_filter"
                          )

# --------Building the eigen_tutorial executable-------- #
add_executable (eigen_tutorial eigen_tutorial.cpp)

target_link_libraries (eigen_tutorial Eigen3::Eigen KalmanFilter)

target_include_directories(eigen_tutorial PUBLIC
                          "${PROJECT_BINARY_DIR}"
                          "${PROJECT_SOURCE_DIR}/kalman_filter"
                          )

# --------Building the filterNoisyVelocity executable-------- #
add_executable(filterNoisyVelocity filterNoisyVelocity.cpp)

target_link_libraries (filterNoisyVelocity Eigen3::Eigen KalmanFilter)

target_include_directories(filterNoisyVelocity PUBLIC
                           "${PROJECT_BINARY_DIR}"
                           "${PROJECT_SOURCE_DIR}/kalman_filter"                          
)
