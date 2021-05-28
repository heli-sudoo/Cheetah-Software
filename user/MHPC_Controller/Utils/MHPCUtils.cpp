#include "MHPCUtils.h"
#include <iostream>
#include <fstream>

template<typename T>
Vec14<T>  Convert3DEstimateTo2D(const StateEstimate<T>* stateEstimate, const LegControllerData<T> *legdatas)
{   
    Vec14<T>  x2D;
    /* body state */
    x2D[0] = stateEstimate->position[0];
    x2D[1] = stateEstimate->position[2];
    x2D[2] = stateEstimate->rpy[1]; // pitch
    x2D[7] = stateEstimate->vWorld[0];
    x2D[8] = stateEstimate->vWorld[2];
    x2D[9] = stateEstimate->omegaBody[1]; 

    /* leg state*/
    T q_fh = (legdatas[0].q[1] + legdatas[1].q[1])/2;   // Front hip
    T q_fk = (legdatas[0].q[2] + legdatas[1].q[2])/2;   // Front knee
    T q_bh = (legdatas[2].q[1] + legdatas[3].q[1])/2;   // Back hip
    T q_bk = (legdatas[2].q[2] + legdatas[3].q[2])/2;   // Back knee
    T qd_fh = (legdatas[0].qd[1] + legdatas[1].qd[1])/2;   // Front hip
    T qd_fk = (legdatas[0].qd[2] + legdatas[1].qd[2])/2;   // Front knee
    T qd_bh = (legdatas[2].qd[1] + legdatas[3].qd[1])/2;   // Back hip
    T qd_bk = (legdatas[2].qd[2] + legdatas[3].qd[2])/2;   // Back knee
    x2D[3] = q_fh;
    x2D[4] = q_fk;
    x2D[5] = q_bh;
    x2D[6] = q_bk;
    x2D[10] = qd_fh;
    x2D[11] = qd_fk;
    x2D[12] = qd_bh;
    x2D[13] = qd_bk;
    return x2D;
}

template<typename T>
void read_data4(const char* FILENAME, std::vector<Vec4<T>>&V)
{
  V.clear();
  std::ifstream data_in(FILENAME);
  if (not data_in.is_open())
  {
    std::cout << "Fail to open " << FILENAME << std::endl;
    return;
  }
  Vec4<T> v;
  T v1, v2, v3, v4;

  V.clear(); // clear the content of V if not empty
  while(data_in >> v1>>v2>>v3>>v4){
    v << v1, v2, v3, v4;
    V.push_back(v);
  }
  data_in.close();
}

template<typename T>
void read_data3(const char* FILENAME, std::vector<Vec3<T>>&V)
{
  V.clear();
  std::ifstream data_in(FILENAME);
  if (not data_in.is_open())
  {
    std::cout << "Fail to open " << FILENAME << std::endl;
    return;
  }
  Vec3<T> v;
  T v1, v2, v3;

  V.clear(); // clear the content of V if not empty
  while(data_in >> v1>>v2>>v3){
    v << v1, v2, v3;
    V.push_back(v);
  }
  data_in.close();
}

template<typename T>
void read_fb_mat_data(const char* FILENAME, MatMN<T,4,14>* K)
{
  std::ifstream data_in(FILENAME);
  if (not data_in.is_open())
  {
    std::cout << "Fail to open " << FILENAME << std::endl;
    return;
  }

  T a;
  MatMN<T,4,14> K_temp;
  int ridx(0), cidx(0), step(0);
  while(data_in >> a){
    K_temp(ridx, cidx) = a; 
    if (cidx < 13){cidx++;}
    else
    {
        cidx = 0;
        ridx++;
    }

    if(ridx>3){
        ridx = 0;
        K[step] =K_temp;
        step++;
    }
  }
}

// Instantiation with casadi_real type
template Vec14<casadi_real> Convert3DEstimateTo2D<casadi_real>(const StateEstimate<casadi_real> *stateEstimate, 
                                                   const LegControllerData<casadi_real> *legdata);
template void read_data4<casadi_real> (const char* FILENAME, std::vector<Vec4<casadi_real>>&V);
template void read_data3<casadi_real>(const char* FILENAME, std::vector<Vec3<casadi_real>>&V);
template void read_fb_mat_data<casadi_real>(const char* FILENAME, MatMN<casadi_real,4,14>* K);
template void Convert2DStateTo3D<casadi_real>(FBModelState<casadi_real> &state, Vec14<casadi_real> & x2D);

// // Instantiation with double type
// template Vec14<double> Convert3DEstimateTo2D<double>(const StateEstimate<double> *stateEstimate, const LegControllerData<double> *legdata);
// template void read_data4<double> (const char* FILENAME, std::vector<Vec4<double>>&V);
// template void read_data3<double>(const char* FILENAME, std::vector<Vec3<double>>&V);
// template void read_fb_mat_data<double>(const char* FILENAME, MatMN<double,4,14>* K);
// template void Convert2DStateTo3D<double>(FBModelState<double> &state, Vec14<double> & x2D);


