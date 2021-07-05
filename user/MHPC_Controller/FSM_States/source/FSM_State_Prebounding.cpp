#include "FSM_State_Prebounding.h"

template <typename T>
FSM_State_Prebounding<T>::FSM_State_Prebounding(ControlFSMData<T>* controlFSMData)
    : FSM_State<T>(controlFSMData, FSM_StateName::PREBOUNDING, "PREBOUNDING"),
    joint_pos_des(4, Vec3<T>::Zero()),
    joint_pos_init(4, Vec3<T>::Zero()),
    joint_pos_cmd(4, Vec3<T>::Zero())
{
	joint_pos_des[0] << 0, -0.65,1.569;
	joint_pos_des[1] << 0, -0.65,1.569;
	joint_pos_des[2] << 0, -0.6, 1.8;
	joint_pos_des[3] << 0, -0.6, 1.8;
}

template <typename T>
void FSM_State_Prebounding<T>::onEnter()
{
  // Default is to not transition
  this->nextStateName = this->stateName;

	iter = 0;
	for (int leg(0); leg < 4; ++leg)
  	 {
    	 joint_pos_init[leg] = this->_data->_legController->datas[leg].q;
  	 }

}

template <typename T>
void FSM_State_Prebounding<T>::run()
{
  T progress = T (iter * this->_data->controlParameters->controller_dt);
  if (progress > 1.){ progress = 1.; }   

  for (int leg(0); leg < 4; ++leg)
  {
  	joint_pos_cmd[leg] = progress*joint_pos_des[leg] + (1- progress)*joint_pos_init[leg];
  	this->jointPDControl(leg, joint_pos_cmd[leg], Vec3<T>::Zero());
  }
  ++iter;
}

template <typename T>
FSM_StateName FSM_State_Prebounding<T>::checkTransition()
{
	this->nextStateName = this->stateName;

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode) {
    case K_PREBOUNDING:
      break;
    case K_STAND_UP:
      this->nextStateName = FSM_StateName::STAND_UP;
      break;
    case K_PASSIVE:  // normal c
      this->nextStateName = FSM_StateName::PASSIVE;
      break;
    case K_LOCOMOTION:
      this->nextStateName = FSM_StateName::LOCOMOTION;
      break;
    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_PREBOUNDING << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }

  // Get the next state
  return this->nextStateName;

}

template <typename T>
void FSM_State_Prebounding<T>::onExit()
{
	iter = 0; // reset iter once exit
}

// explicit instantiation 
// template class FSM_State_Prebounding<double>;
template class FSM_State_Prebounding<float>;





