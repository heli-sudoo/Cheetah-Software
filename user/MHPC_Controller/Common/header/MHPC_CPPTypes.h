#ifndef MHPC_CPPTYPES_H
#define MHPC_CPPTYPES_H

// #define EIGEN_NO_DEBUG 1 // Prevent Eigen from asserting dimension mismatch for speedup

#define CALC_DYN_AND_PAR 0
#define CALC_PARTIALS_ONLY 1
#define CALC_DYNAMICS_ONLY 2
#define N_TIMESTEPS_MAX 110


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Cholesky> // Cholesky decomposition for positive definitness analysis

#define PI 3.141592653589793238

#define DEBUG 1

//  #define TIME_BENCHMARK

#define int_T long long int

template<typename T, size_t m, size_t n>
using MatMN = Eigen::Matrix<T, m, n>;

template<typename T, size_t m>
using VecM = Eigen::Matrix<T, m, 1>;

template<typename T>
using DMat = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

template<typename T>
using DVec = Eigen::Matrix<T, Eigen::Dynamic, 1>;

template<typename T>
using Chol =  Eigen::LDLT<DMat<T>>;

using std::vector;


#endif //MHPC_CPPTYPES_H
