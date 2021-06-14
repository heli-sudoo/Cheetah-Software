#include "FSM_State_Locomotion.h"
#include "MHPC_CompoundTypes.h"
#include "MHPCUtils.h"
template <typename T>
FSM_State_Locomotion<T>::FSM_State_Locomotion(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::LOCOMOTION, "LOCOMOTION")
{
	K_DDP_buf = new MatMN<T,4,14> [200]; // allocate sufficient memory to store pre-computed feedback gain
	read_tau_data();
	read_qdes_data();
	read_qddes_data();
	read_pos_des_data();
	read_vel_des_data();
	read_fb_gain_data();
	offline_steps = tau_ff_buf.size();

  HSDDP_OPTION<T> option;
  userParams = _controlFSMData->userParameters;
  mhpc = new MHPCLocomotion<T>(_controlFSMData->userParameters, option);
  mhpc->initialization(_controlFSMData);
}

template <typename T>
void FSM_State_Locomotion<T>::onEnter()
{
  // Default is to not transition
  this->nextStateName = this->stateName;
	iter = 0;
}

template <typename T>
void FSM_State_Locomotion<T>::run()
{
  // get state estimate in 2D case
  state2D = Convert3DEstimateTo2D(this->_data->_stateEstimator->getResultHandle(),
                            this->_data->_legController->datas);

  // execute offline MPC (full stance to back stance)
  if (iter < offline_steps)
  {
    pos_des = pos_des_buf[iter];
    vel_des = pos_des_buf[iter];
    qdes = qdes_buf[iter];
    qddes = qddes_buf[iter];
    Kddp = K_DDP_buf[iter];
    offline_controller();
  }
  else // execute online MPC (entering normal bounding)
    mhpc->run();
	iter++;
}

template <typename T>
FSM_StateName FSM_State_Locomotion<T>::checkTransition()
{
	this->nextStateName = this->stateName;

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode) {
    case K_LOCOMOTION:
      break;
    case K_PASSIVE:  // normal c
      this->nextStateName = FSM_StateName::PASSIVE;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_PASSIVE << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }

  // Get the next state
  return this->nextStateName;
}

template <typename T>
void FSM_State_Locomotion<T>::onExit()
{
		iter = 0;
}

template <typename T>
void FSM_State_Locomotion<T>::offline_controller()
{   
    Vec4<T> q  = state2D.segment(3,4);
    Vec4<T> qd = state2D.tail(4); 
    MatMN<T,4,3> Kp_body_ddp = Kddp.block(0,0,4,3);
    MatMN<T,4,3> Kd_body_ddp = Kddp.block(0,7,4,3);
    MatMN<T,4,4> Kp_joint_ddp = Kddp.block(0,3,4,4);
    MatMN<T,4,4> Kd_joint_ddp = Kddp.block(0,10,4,4);

    Vec3<double> Kp_temp = this->_data->userParameters->kp_joint; 
    Vec3<double> Kd_temp = this->_data->userParameters->kd_joint; 
    Mat3<T> Kp_pd = Kp_temp.cast<T>().asDiagonal();
    Mat3<T> Kd_pd = Kd_temp.cast<T>().asDiagonal();
    
    /* feedback from body state */
    T sp = 0.2, sd = 1; // scale the body state feeback if necessary
    udes += sp*Kp_body_ddp*(state2D.head(3)-pos_des)+ 
            sd*Kd_body_ddp*(state2D.segment(7,3)-vel_des);
    /* feedback from leg state*/
    udes += Kp_joint_ddp*(q -qdes)+ Kd_joint_ddp*(qd - qddes);
    
    for(int leg(0); leg<4; ++leg)
    {
      int leg2D = leg/2; // 0 or 1  
      this->_data->_legController->commands[leg].qDes[0] =  0;
      this->_data->_legController->commands[leg].qDes[1] =  qdes[2*leg2D];
      this->_data->_legController->commands[leg].qDes[2] =  qdes[2*leg2D+1];
      this->_data->_legController->commands[leg].qdDes[0] = 0;
      this->_data->_legController->commands[leg].qdDes[1] = qddes[2*leg2D];
      this->_data->_legController->commands[leg].qdDes[2] = qddes[2*leg2D+1];
      this->_data->_legController->commands[leg].tauFeedForward[0] = 0;
      this->_data->_legController->commands[leg].tauFeedForward[1] = udes[2*leg2D]/2;    // divided by 2 from 2D to 3D
      this->_data->_legController->commands[leg].tauFeedForward[2] = udes[2*leg2D+1]/2;
      this->_data->_legController->commands[leg].kpJoint = Kp_pd;
      this->_data->_legController->commands[leg].kdJoint = Kd_pd;
    }
}

template <typename T>
void FSM_State_Locomotion<T>::read_tau_data()
{
  const char* FILENAME = "../user/MHPC_Controller/PreComputeData/torque_HSDDP.txt";
  read_data4(FILENAME, tau_ff_buf);
}

template <typename T>
void FSM_State_Locomotion<T>::read_qdes_data()
{
  const char* FILENAME = "../user/MHPC_Controller/PreComputeData/qdes_HSDDP.txt";
  read_data4(FILENAME, qdes_buf);
}

template <typename T>
void FSM_State_Locomotion<T>::read_qddes_data()
{
  const char* FILENAME = "../user/MHPC_Controller/PreComputeData/qddes_HSDDP.txt";
  read_data4(FILENAME, qddes_buf);
}

template <typename T>
void FSM_State_Locomotion<T>::read_pos_des_data()
{
  const char* FILENAME = "../user/MHPC_Controller/PreComputeData/pos_des_HSDDP.txt";
  read_data3(FILENAME, pos_des_buf);
}

template <typename T>
void FSM_State_Locomotion<T>::read_vel_des_data()
{
  const char* FILENAME = "../user/MHPC_Controller/PreComputeData/vel_des_HSDDP.txt";
  read_data3(FILENAME, vel_des_buf);
}

template <typename T>
void FSM_State_Locomotion<T>::read_fb_gain_data()
{
  const char* FILENAME = "../user/MHPC_Controller/PreComputeData/K_HSDDP.txt";
  read_fb_mat_data(FILENAME, K_DDP_buf);
}

// template class FSM_State_Locomotion<double>;
template class FSM_State_Locomotion<casadi_real>;

