#ifndef MHPCUTILS_H
#define MHPCUTILS_H

#include "cppTypes.h"
#include "MHPC_CPPTypes.h"
#include "MHPC_CompoundTypes.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Controllers/LegController.h"
#include <vector>
#include <string>
template<typename T>
Vec14<T> Convert3DEstimateTo2D(const StateEstimate<T> *stateEstimate, const LegControllerData<T> *legdata);

template<typename T>
void read_data4(std::string FILENAME, std::vector<Vec4<T>>&V);

template<typename T>
void read_data3(std::string FILENAME, std::vector<Vec3<T>>&V);

template<typename T>
void read_fb_mat_data(std::string FILENAME, MatMN<T,4,14>* K);

template<typename T>
void Convert2DStateTo3D(FBModelState<T> &state, Vec14<T> & x2D) {}


#endif // MHPCUTILS_H