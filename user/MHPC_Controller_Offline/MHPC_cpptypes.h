
#ifndef MHPC_CPPTYPES_H
#define MHPC_CPPTYPES_H

#include<eigen3/Eigen/Dense>

template<typename T>
using Mat4_14 = Eigen::Matrix<T, 4, 14>;

template<typename T>
using Mat4_3 = Eigen::Matrix<T, 4, 3>;

// template<typename T>
// using Mat4_4 = Eigen::Matrix<T, 4, 4>;

template<typename T>
using Map_Mat4_14 = Eigen::Map<Eigen::Matrix<T, 4, 14>>;

template<typename T>
using Map_Mat4_3 = Eigen::Map<Eigen::Matrix<T, 4, 3>>;

template<typename T>
using Map_Mat4_4 = Eigen::Map<Eigen::Matrix<T, 4, 4>>;


#endif //MHPC_CPPTYPES_H
