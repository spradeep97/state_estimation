cmake_minimum_required (VERSION 3.0)
project (state_estimation)
 
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
 
add_executable (linearCarProblem linearCarProblem.cpp)
target_link_libraries (linearCarProblem Eigen3::Eigen)