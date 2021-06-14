#ifndef FSM_STATE_PREBOUNDING_H
#define FSM_STATE_PREBOUNDING_H

#include "FSM_State.h"
#include "MHPC_CPPTypes.h"
#include "MHPC_CPPTypes.h"
#include "MHPCUtils.h"
/**
 *
 */
template <typename T>
class FSM_State_Prebounding: public FSM_State<T> {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // overload new operator to avoid alignment issue

 public:
  FSM_State_Prebounding(ControlFSMData<T>* controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Behavior to be carried out when exiting a state
  void onExit();

private:
  int iter = 0; // tracking progress of prebounding
  std::vector<Vec3<T>> joint_pos_init; 
  std::vector<Vec3<T>> joint_pos_des; 
  std::vector<Vec3<T>> joint_pos_cmd; // commanded desired joint position 
};


#endif // FSM_STATE_PREBOUNDING_H