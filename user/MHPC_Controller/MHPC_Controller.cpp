#include "MHPC_Controller.hpp"
#include <fstream>
#include <iostream>
#include <vector>


void MHPC_Controller::runController(){

  _legController->_maxTorque = 150;
  _legController->_legsEnabled = true;


  static int iter(0);
  ++iter;

  if (_controlParameters->control_mode == 1){
    standUp_control();
    bounding = false;}

  if (_controlParameters->control_mode == 3){
    bounding_control();
    _stand_up = false;
  }

  if (_controlParameters->control_mode == 4){
    jointPD_control();
    _stand_up = false;
    bounding = false;
  }

  if (_controlParameters->control_mode == 0)
  {
    _stand_up = false;
    bounding = false;
  }

  // std::cout<< "Contact " << _stateEstimate->contactEstimate << std::endl;
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

  bounding = false;
  _bounding_step = 0;
  _stand_up = false;
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
void MHPC_Controller::bounding_control(){
  if (!bounding){
    bounding = true;
    bounding_control_enter();
  }else {bounding_control_run();}
}

void MHPC_Controller::bounding_control_enter(){
  _bounding_step = 0;
}

void MHPC_Controller::bounding_control_run(){
  if (_bounding_step>=_tau_ff_data.size()){
    jointPD_control();
    return;
  }

  Mat3<float> kpMat;
  Mat3<float> kdMat;

  kpMat <<  50*userParameters.kp, 0, 0, 0, userParameters.kp, 0, 0, 0, userParameters.kp;
  kdMat <<  5*userParameters.kd, 0, 0, 0, userParameters.kd, 0, 0, 0, userParameters.kd;

  // set feedforward control
    _tau_ff = -_tau_ff_data[_bounding_step]; // sign opposite as in MATLB, magnitude takes half

    if (_controlParameters->control_mode == 3)
    {
      // feedback control associated with x, z and pitch
      _K = _K_DDP_data[_bounding_step];
      _Kp_cart = _K.block<4,3>(0,0);
      _Kd_cart.block<4,1>(0,0) = Vec4<float>::Zero();
      _Kd_cart = _K.block<4,3>(0,7); 
      _pos_act << _stateEstimate->position[0], _stateEstimate->position[2], _stateEstimate->rpy[1];
      _pos_des = _pos_des_data[_bounding_step];
      _vel_act << _stateEstimate->vWorld[0], _stateEstimate->vWorld[2], _stateEstimate->omegaBody[1];
      _vel_des = _vel_des_data[_bounding_step];
      _tau_ff += -0.5*(_Kp_cart*(_pos_act - _pos_des) + _Kd_cart * (_vel_act - _vel_des));

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
      _q_des = -_qdes_data[_bounding_step];     // hip and knee angles are opposite as in MATALAB
      _qd_des =  -_qddes_data[_bounding_step];  
      _tau_ff += _Kp_hip_knee*(_q_act - _q_des) + _Kd_hip_knee*(_qd_act - _qd_des);    
    }
    _tau_ff = _tau_ff/2;   

    // regulate roll motion
    float tau_roll_fb, kp_roll = 30, kd_roll=5, dist;
    _F_ff[0].setZero();
    _F_ff[1].setZero();
    _F_ff[2].setZero();
    _F_ff[3].setZero();
    
    tau_roll_fb = -kp_roll*_stateEstimate->rpy[0] - kd_roll*_stateEstimate->omegaBody[0];
    dist = -_legController->datas[0].p[2];

    if (_stateEstimate->contactEstimate[0] and _stateEstimate->contactEstimate[1])
    {
      _F_ff[0][0] = -0.5*tau_roll_fb/dist;
      _F_ff[1][0] = 0.5*tau_roll_fb/dist;
    }
    else if (_stateEstimate->contactEstimate[2] and _stateEstimate->contactEstimate[3])
    {
      _F_ff[2][0] = -0.5*tau_roll_fb/dist;
      _F_ff[3][0] = 0.5*tau_roll_fb/dist;
    }
        

    for(int leg(0); leg<2; ++leg){
        _legController->commands[leg].qDes[0] =  0;
        _legController->commands[leg].qDes[1] =  -_qdes_data[_bounding_step][0];
        _legController->commands[leg].qDes[2] =  -_qdes_data[_bounding_step][1];
        _legController->commands[leg].qdDes[0] = 0;
        _legController->commands[leg].qdDes[1] =  -_qddes_data[_bounding_step][0];   
        _legController->commands[leg].qdDes[2] =  -_qddes_data[_bounding_step][1];
        _legController->commands[leg].tauFeedForward[0] = 0;
        _legController->commands[leg].tauFeedForward[1] = _tau_ff[0];
        _legController->commands[leg].tauFeedForward[2] = _tau_ff[1];
        _legController->commands[leg].kpJoint = kpMat;
        _legController->commands[leg].kdJoint = kdMat;
        _legController->commands[leg].forceFeedForward = _F_ff[leg];
      }

      for(int leg(2); leg<4; ++leg){
        _legController->commands[leg].qDes[0] =  0;
        _legController->commands[leg].qDes[1] =  -_qdes_data[_bounding_step][2];
        _legController->commands[leg].qDes[2] =  -_qdes_data[_bounding_step][3];
        _legController->commands[leg].qdDes[0] = 0;
        _legController->commands[leg].qdDes[1] =  -_qddes_data[_bounding_step][2];   
        _legController->commands[leg].qdDes[2] =  -_qddes_data[_bounding_step][3];         
        _legController->commands[leg].tauFeedForward[0] = 0;
        _legController->commands[leg].tauFeedForward[1] = _tau_ff[2];
        _legController->commands[leg].tauFeedForward[2] = _tau_ff[3];  
        _legController->commands[leg].kpJoint = kpMat;
        _legController->commands[leg].kdJoint = kdMat;
        _legController->commands[leg].forceFeedForward = _F_ff[leg];
      }
                 

      _bounding_step ++;
}

void MHPC_Controller::jointPD_control(){
  Mat3<float> kpMat;
  Mat3<float> kdMat;

  kpMat <<  userParameters.kp, 0, 0, 0, userParameters.kp, 0, 0, 0, userParameters.kp;
  kdMat <<  userParameters.kd, 0, 0, 0, userParameters.kd, 0, 0, 0, userParameters.kd;

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

void MHPC_Controller::standUp_control_enter(){
  for(size_t leg(0); leg<4; ++leg){
    _init_foot_pos[leg] = _legController->datas[leg].p;
  }
  iter_stand = 0;
}

void MHPC_Controller::standUp_control_run(){
  iter_stand ++;
  float hMax_back = 0.25;
  // float hMax_front = 0.25;
  Mat3<float>kpJoint = Vec3<float>(100, 50, 50).asDiagonal();
  Mat3<float>kdJoint = Vec3<float>(5, 5, 5).asDiagonal();
  Mat3<float>kpCartesian = Vec3<float>(500, 500, 500).asDiagonal();
  Mat3<float>kdCartesian = Vec3<float>(8, 8, 8).asDiagonal();

  float progress = 2 * iter_stand * _controlParameters->controller_dt;

  if (progress > 1.){ progress = 1.; }

  for(int leg(0); leg<2; ++leg){
          for(int jidx(0); jidx<3; ++jidx){
            // float pos = 0.0;
            _legController->commands[leg].qDes[jidx] =  userParameters.home_fleg[jidx];
            _legController->commands[leg].qdDes[jidx] = 0.;
            _legController->commands[leg].tauFeedForward[jidx] = 0;

          }
          _legController->commands[leg].pDes = _init_foot_pos[leg];
          _legController->commands[leg].pDes[2] = 
              progress*(-hMax_back) + (1. - progress) * _init_foot_pos[leg][2];

          _legController->commands[leg].kpJoint = kpJoint;
          _legController->commands[leg].kdJoint = kdJoint;
          _legController->commands[leg].kpCartesian = kpCartesian;
          _legController->commands[leg].kdCartesian = kdCartesian;
        }

        for(int leg(2); leg<4; ++leg){
          for(int jidx(0); jidx<3; ++jidx){
            // float pos = 0.0;
            _legController->commands[leg].qDes[jidx] = userParameters.home_bleg[jidx];
            _legController->commands[leg].qdDes[jidx] = 0.;
            _legController->commands[leg].tauFeedForward[jidx] = 0;
          }
          _legController->commands[leg].pDes = _init_foot_pos[leg];
          _legController->commands[leg].pDes[2] = 
              progress*(-hMax_back) + (1. - progress) * _init_foot_pos[leg][2];

          _legController->commands[leg].kpJoint = kpJoint;
          _legController->commands[leg].kdJoint = kdJoint;
          _legController->commands[leg].kpCartesian = kpCartesian;
          _legController->commands[leg].kdCartesian = kdCartesian;
        }

}

void MHPC_Controller::standUp_control(){
  if (!_stand_up){
    standUp_control_enter();
    _stand_up = true;
  }
  else{standUp_control_run();}
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