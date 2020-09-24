#include "MHPC_Controller.hpp"
#include <fstream>
#include <iostream>
#include <vector>


void MHPC_Controller::runController(){

  Mat3<float> kpMat;
  Mat3<float> kdMat;

  kpMat <<  50*userParameters.kp, 0, 0, 0, userParameters.kp, 0, 0, 0, userParameters.kp;
  kdMat <<  5*userParameters.kd, 0, 0, 0, userParameters.kd, 0, 0, 0, userParameters.kd;
  int control_mode = _controlParameters->control_mode;


  static int iter(0);
  ++iter;

  if (goHome_check() and _controlParameters->control_mode >=2)
  {
    bounding = true;
    bounding_step = 0;
  }

  if(bounding_step >= _tau_ff_data.size()) 
  {
    bounding = false;
    control_mode = 0;
  }

  if (bounding)
  { 
    // set feedforward control
    _tau_ff = -_tau_ff_data[bounding_step]; // sign opposite as in MATLB, magnitude takes half

    if (_controlParameters->control_mode == 3)
    {
      // feedback control associated with x, z and pitch
      _K = _K_DDP_data[bounding_step];
      _Kp_cart = _K.block<4,3>(0,0);
      _Kd_cart = _K.block<4,3>(0,7); 
      _pos_act << _stateEstimate->position[0], _stateEstimate->position[2], _stateEstimate->rpy[1];
      _pos_des = _pos_des_data[bounding_step];
      _vel_act << _stateEstimate->vWorld[0], _stateEstimate->vWorld[2], _stateEstimate->omegaWorld[1];
      _vel_des = _vel_des_data[bounding_step];
      _tau_ff += -0.05*(_Kp_cart*(_pos_act - _pos_des) + _Kd_cart * (_vel_act - _vel_des));

      // feedback control associated with q1, q2, q3, q4
      _Kp_hip_knee = _K.block<4,4>(0,3);
      _Kd_hip_knee = _K.block<4,4>(0,10);
      float q1 = (_legController->datas[0].q[1] + _legController->datas[1].q[1])/2;   // Front hip
      float q2 = (_legController->datas[0].q[2] + _legController->datas[1].q[2])/2;   // Front knee
      float q3 = (_legController->datas[2].q[1] + _legController->datas[3].q[1])/2;   // Back hip
      float q4 = (_legController->datas[2].q[2] + _legController->datas[3].q[2])/2;   // Back knee

      float qd1 = (_legController->datas[0].qd[1] + _legController->datas[1].qd[1])/2;   // Front hip
      float qd2 = (_legController->datas[0].qd[2] + _legController->datas[1].qd[2])/2;   // Front knee
      float qd3 = (_legController->datas[2].qd[1] + _legController->datas[3].qd[1])/2;   // Back hip
      float qd4 = (_legController->datas[2].qd[2] + _legController->datas[3].qd[2])/2;   // Back knee
      _q_act << q1, q2, q3, q4;
      _qd_act << qd1, qd2, qd3, qd4;
      _q_des = -_qdes_data[bounding_step];     // hip and knee angles are opposite as in MATALAB
      _qd_des =  -_qddes_data[bounding_step];  
      _tau_ff += _Kp_hip_knee*(_q_act - _q_des) + _Kd_hip_knee*(_qd_act - _qd_des);
      // kpMat.diagonal() << 50, 0, 0;
      // kdMat.diagonal() << 2, 0, 0;
    }
    _tau_ff = _tau_ff/2;


    for(int leg(0); leg<2; ++leg){
        for(int jidx(0); jidx<3; ++jidx){
          // float pos = 0.0;
          _legController->commands[leg].qDes[0] =  0;
          _legController->commands[leg].qDes[1] =  -_qdes_data[bounding_step][0];
          _legController->commands[leg].qDes[2] =  -_qdes_data[bounding_step][1];
          _legController->commands[leg].qdDes[0] = 0;
          _legController->commands[leg].qdDes[1] =  -_qddes_data[bounding_step][0];   
          _legController->commands[leg].qdDes[2] =  -_qddes_data[bounding_step][1];
          _legController->commands[leg].tauFeedForward[0] = 0;
          _legController->commands[leg].tauFeedForward[1] = _tau_ff[0];
          _legController->commands[leg].tauFeedForward[2] = _tau_ff[1];
        }
        _legController->commands[leg].kpJoint = kpMat;
        _legController->commands[leg].kdJoint = kdMat;
      }

      for(int leg(2); leg<4; ++leg){
        for(int jidx(0); jidx<3; ++jidx){
          // float pos = 0.0;
          _legController->commands[leg].qDes[0] =  0;
          _legController->commands[leg].qDes[1] =  -_qdes_data[bounding_step][2];
          _legController->commands[leg].qDes[2] =  -_qdes_data[bounding_step][3];
          _legController->commands[leg].qdDes[0] = 0;
          _legController->commands[leg].qdDes[1] =  -_qddes_data[bounding_step][2];   
          _legController->commands[leg].qdDes[2] =  -_qddes_data[bounding_step][3];         
          _legController->commands[leg].tauFeedForward[0] = 0;
          _legController->commands[leg].tauFeedForward[1] = _tau_ff[2];
          _legController->commands[leg].tauFeedForward[2] = _tau_ff[3];;                
        }
        _legController->commands[leg].kpJoint = kpMat;
        _legController->commands[leg].kdJoint = kdMat;
      }
      // std::cout << "bounding step" << bounding_step << std::endl;
      // std::cout << _tau_ff_data[bounding_step] << std::endl;
      bounding_step ++;
      // std::cout << _tau_ff_data[bounding_step] << std::endl;
      // std::cout<<"bounding_step" << bounding_step << std::endl;
  }
  

  _legController->_maxTorque = 150;
  _legController->_legsEnabled = true;

  if (control_mode <= 1)
  {
    for(int leg(0); leg<2; ++leg){
          for(int jidx(0); jidx<3; ++jidx){
            // float pos = 0.0;
            _legController->commands[leg].qDes[jidx] =  userParameters.home_fleg[jidx];
            _legController->commands[leg].qdDes[jidx] = 0.;
            _legController->commands[leg].tauFeedForward[jidx] = 0;
          }
          _legController->commands[leg].kpJoint = kpMat;
          _legController->commands[leg].kdJoint = kdMat;
        }

        for(int leg(2); leg<4; ++leg){
          for(int jidx(0); jidx<3; ++jidx){
            // float pos = 0.0;
            _legController->commands[leg].qDes[jidx] = userParameters.home_bleg[jidx];
            _legController->commands[leg].qdDes[jidx] = 0.;
            _legController->commands[leg].tauFeedForward[jidx] = 0;
          }
          _legController->commands[leg].kpJoint = kpMat;
          _legController->commands[leg].kdJoint = kdMat;
        }

  }
}


void MHPC_Controller::initializeController()
{
  homeState.bodyPosition =  userParameters.home_pos.cast<float>();
  homeState.bodyOrientation = rpyToQuat(userParameters.home_rpy.cast<float>());
  homeState.bodyVelocity = SVec<float>::Zero();
  homeState.q = DVec<float>(12);
  homeState.q << userParameters.home_fleg.cast<float>() ,userParameters.home_fleg.cast<float>(), 
                 userParameters.home_bleg.cast<float>(), userParameters.home_bleg.cast<float>();
  homeState.qd  = DVec<float>(12);
  homeState.qd.setZero();
  std::cout << "initialize Controller" << std::endl;
  read_tau_data();
  read_qdes_data();
  read_qddes_data();
  read_fb_mat_data();
  read_pos_des_data();
  read_vel_des_data();

  /*
  // debug reading data
  write_tau_data();
  write_qdes_data();
  write_qddes_data();
  write_fb_mat_data();
  std::cout << _K_DDP_data[3] << std::endl;
  */

  bounding = false;
  bounding_step = 0;
}



bool MHPC_Controller::goHome_check()
{
  float thresh = 1e-04;
  
  Vec3<float> eR = _stateEstimate->rpy - userParameters.home_rpy.cast<float>();
  Vec3<float> p = _stateEstimate->position.cast<float>();
  Vec3<float> ep = homeState.bodyPosition - p;
  DVec<float> eq(12);
  for(int leg(0); leg<4; ++leg){
    for(int jidx(0); jidx<3; ++jidx){
      // float pos = 0.0;
      eq[3*leg+jidx] =  _legController->datas[leg].q[jidx];      
    }
  }
  eq -= homeState.q;
  float error = ep.norm() + eq.norm() + eR.norm();
  if (error <= thresh){
    return true;
  }else {
    return false;
  } 
}

void MHPC_Controller::read_tau_data()
{
  const char* FILENAME = "../user/MHPC_Controller/torque_HSDDP.txt";
  read_4f_data(FILENAME, _tau_ff_data);
}

void MHPC_Controller::read_qdes_data()
{
  const char* FILENAME = "../user/MHPC_Controller/qdes_HSDDP.txt";
  read_4f_data(FILENAME, _qdes_data);
}

void MHPC_Controller::read_qddes_data()
{
  const char* FILENAME = "../user/MHPC_Controller/qddes_HSDDP.txt";
  read_4f_data(FILENAME, _qddes_data);
}

void MHPC_Controller::read_pos_des_data(){
  const char* FILENAME = "../user/MHPC_Controller/pos_des_HSDDP.txt";
  read_3f_data(FILENAME, _pos_des_data);
}

void MHPC_Controller::read_vel_des_data(){
  const char* FILENAME = "../user/MHPC_Controller/vel_des_HSDDP.txt";
  read_3f_data(FILENAME, _vel_des_data);
}


void MHPC_Controller::read_4f_data(const char* FILENAME, std::vector<Vec4<float>>&V){
  std::ifstream data_in(FILENAME);
  if (not data_in.is_open())
  {
    std::cout << "Fail to open " << FILENAME << std::endl;
    return;
  }
  Vec4<float> v;
  float v1, v2, v3, v4;

  while(data_in >> v1>>v2>>v3>>v4){
    v << v1, v2, v3, v4;
    V.push_back(v);
  }
  data_in.close();
}

void MHPC_Controller::read_3f_data(const char* FILENAME, std::vector<Vec3<float>>&V){
  std::ifstream data_in(FILENAME);
  if (not data_in.is_open())
  {
    std::cout << "Fail to open " << FILENAME << std::endl;
    return;
  }
  Vec3<float> v;
  float v1, v2, v3;

  while(data_in >> v1>>v2>>v3){
    v << v1, v2, v3;
    V.push_back(v);
  }
  data_in.close();
}

void MHPC_Controller::read_fb_mat_data(){
  const char* FILENAME = "../user/MHPC_Controller/K_HSDDP.txt";
  std::ifstream data_in(FILENAME);
  if (not data_in.is_open())
  {
    std::cout << "Fail to open K_HSDDP.txt" << std::endl;
    return;
  }

  float a;
  Mat4_14<float> K_temp;
  int ridx(0), cidx(0);
  int step(0);
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
        _K_DDP_data[step] =K_temp;
        step++;
    }
  }
  _sim_len = step;
}

void MHPC_Controller::write_tau_data()
{
  const char* FILENAME = "../user/MHPC_Controller/torque_HSDDP_w.txt";  
  write_4f_data(FILENAME, _tau_ff_data);
}

void MHPC_Controller::write_qdes_data()
{
  const char* FILENAME = "../user/MHPC_Controller/qdes_HSDDP_w.txt";  
  write_4f_data(FILENAME, _qdes_data);
}

void MHPC_Controller::write_qddes_data()
{
  const char* FILENAME = "../user/MHPC_Controller/qddes_HSDDP_w.txt";  
  write_4f_data(FILENAME, _qddes_data);
}


void MHPC_Controller::write_4f_data(const char*FILENAME, std::vector<Vec4<float>>&V){
  std::ofstream data_out(FILENAME);

  if (not data_out.is_open())
  {
    std::cout << "Fail to open " << FILENAME << std::endl;
    return;
  }

  for (auto &v: V){
    for (int i = 0; i < v.size(); ++i)
    {
      data_out << v[i] << " ";
    }
    data_out << std::endl;
  }

  data_out.close();
}

void MHPC_Controller::write_fb_mat_data(){
  const char* FILENAME = "../user/MHPC_Controller/K_HSDDP_w.txt";
  write_mat_data(FILENAME, _K_DDP_data);
}

void MHPC_Controller::write_mat_data(const char*FILENAME, Mat4_14<float>* M){
  std::ofstream data_out(FILENAME);

  if (not data_out.is_open())
  {
    std::cout << "Fail to open " << FILENAME << std::endl;
    return;
  }
  Mat4_14<float> m;
  for (int step(0); step<_sim_len; ++step)
  {
    m = M[step];
    for (int ridx(0); ridx < m.innerSize(); ++ridx)
    {
      for (int cidx(0); cidx < m.outerSize(); ++cidx)
      {
        data_out << m(ridx, cidx) << " ";
      }
      data_out << std::endl;
    }
    
  }
  data_out.close();
}