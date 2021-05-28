#ifndef FSM_State_H
#define FSM_State_H

#include <stdio.h>

#include "ControlFSMData.h"
#include "cppTypes.h"

// Normal robot states
#define K_PASSIVE 0
#define K_STAND_UP 1
#define K_PREBOUNDING 2
#define K_LOCOMOTION 3
#define K_RECOVERY_STAND 6
// Specific control states
#define K_JOINT_PD 51

#define K_INVALID 100

/**
 * Enumerate all of the FSM states so we can keep track of them.
 */
enum class FSM_StateName {
  INVALID,
  PASSIVE,
  JOINT_PD,
  STAND_UP,
  PREBOUNDING,
  LOCOMOTION,
};

/**
 *
 */
template <typename T>
class FSM_State {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Generic constructor for all states
  FSM_State(ControlFSMData<T>* _controlFSMData, FSM_StateName stateNameIn,
            std::string stateStringIn);

  // Behavior to be carried out when entering a state
  virtual void onEnter() = 0;// {}

  // Run the normal behavior for the state
  virtual void run() = 0; //{}

  // Behavior to be carried out when exiting a state
  virtual void onExit() = 0; // {}

  // Manages state specific transitions
  virtual FSM_StateName checkTransition() { return FSM_StateName::INVALID; }

  //
  virtual void transition() { /* do nothing*/ }

  //
  void jointPDControl(int leg, Vec3<T> qDes, Vec3<T> qdDes);
  void cartesianImpedanceControl(int leg, Vec3<T> pDes, Vec3<T> vDes,
                                 Vec3<T> kp_cartesian,
                                 Vec3<T> kd_cartesian);

  //
  void turnOnAllSafetyChecks();
  void turnOffAllSafetyChecks();

  // Holds all of the relevant control data
  ControlFSMData<T>* _data;

  // FSM State info
  FSM_StateName stateName;      // enumerated name of the current state
  FSM_StateName nextStateName;  // enumerated name of the next state
  std::string stateString;      // state name string

  // Pre controls safety checks
  bool checkSafeOrientation = false;  // check roll and pitch

  // Post control safety checks
  bool checkPDesFoot = false;          // do not command footsetps too far
  bool checkForceFeedForward = false;  // do not command huge forces
  bool checkLegSingularity = false;    // do not let leg

  // Leg controller command placeholders for the whole robot (3x4 matrices)
  Mat34<T> jointFeedForwardTorques;  // feed forward joint torques
  Mat34<T> jointPositions;           // joint angle positions
  Mat34<T> jointVelocities;          // joint angular velocities
  Mat34<T> footFeedForwardForces;    // feedforward forces at the feet
  Mat34<T> footPositions;            // cartesian foot positions
  Mat34<T> footVelocities;           // cartesian foot velocities

 private:
  // Create the cartesian P gain matrix
  Mat3<T> kpMat;

  // Create the cartesian D gain matrix
  Mat3<T> kdMat;
};

#endif  // FSM_State_H
