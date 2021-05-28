#ifndef FSM_STATE_LOCOMOTION_H
#define FSM_STATE_LOCOMOTION_H

#include "cppTypes.h"
#include "Math/orientation_tools.h"
#include "MHPC_CPPTypes.h"
#include "FSM_State.h"
#include "MHPCLocomotion.h"
#include "MHPCUserParameters.h"
#include "ControlFSMData.h"

template<typename T>
class FSM_State_Locomotion:public FSM_State<T>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // overload new operator to avoid alignment issue
	FSM_State_Locomotion(ControlFSMData<T>* _controlFSMData);
  ~FSM_State_Locomotion()
  {
    delete [] K_DDP_buf;
    delete mhpc;
  }
  
	// Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Behavior to be carried out when exiting a state
  void onExit();

  // Execute offline data for transitioning from prebounding to bounding (in 2D plane)
  void offline_controller();



public:
  void read_tau_data();
  void read_qdes_data();
  void read_qddes_data();
  void read_pos_des_data();
  void read_vel_des_data();
  void read_fb_gain_data();


 private:
  int iter = 0;   // track execution progress
  int offline_steps;   // length of offline command execution
  std::vector<Vec4<T>> tau_ff_buf;
  std::vector<Vec4<T>> qdes_buf;
  std::vector<Vec4<T>> qddes_buf;
  std::vector<Vec3<T>> pos_des_buf;
  std::vector<Vec3<T>> vel_des_buf;

  VecM<T, 4> qdes;
  VecM<T, 4> qddes;
  VecM<T, 3> pos_des;
  VecM<T, 3> vel_des;
  VecM<T, 4> udes;    // mhpc solution of control for one step
  MatMN<T, 4, 14> Kddp; // mppc solition of feedback gain
  VecM<T, 14> state2D; // temporaly holds 2D state

  MatMN<T,4,14>* K_DDP_buf = nullptr;
  MHPCLocomotion<T> *mhpc = nullptr;
  MHPCUserParameters * userParams = nullptr;

};
#endif // FSM_STATE_LOCOMOTION_H