#ifndef CONTROLFSM_H
#define CONTROLFSM_H

#include <iostream>

// Contains all of the control related data
#include "ControlFSMData.h"

// Checks the robot state and commands for safety
#include "SafetyChecker.h"

// FSM States
#include "FSM_State.h"
#include "FSM_State_Locomotion.h"
#include "FSM_State_Passive.h"
#include "FSM_State_StandUp.h"
#include "FSM_State_Prebounding.h"

/**
 * Enumerate all of the operating modes
 */
enum class FSM_OperatingMode { 
  NORMAL, TRANSITIONING, ESTOP, EDAMP };

/**
 *
 */
template <typename T>
struct FSM_StatesList {
  FSM_State<T>* invalid;
  FSM_State_Passive<T>* passive;
  FSM_State_StandUp<T>* standUp;
  FSM_State_Prebounding<T>* prebounding; 
  FSM_State_Locomotion<T>* locomotion;
};


/**
 *
 */
template <typename T>
struct FSM_ControllerList {
};


/**
 * Control FSM handles the FSM states from a higher level
 */
template <typename T>
class ControlFSM {
 public:
  ControlFSM(Quadruped<T>* _quadruped,
             StateEstimatorContainer<T>* _stateEstimator,
             LegController<T>* _legController,
             Gait<T> *_gait,
             DesiredStateCommand<T>* _desiredStateCommand,
             USRCMD<T> *_usrcmd,
             ContactEstimator<T> *_contactEstimator,
             RobotControlParameters* controlParameters,
             MHPCUserParameters* userParameters,
             VisualizationData* visualizationData);

  // Initializes the Control FSM instance
  void initialize();

  // Runs the FSM logic and handles the state transitions and normal runs
  void runFSM();

  // This will be removed and put into the SafetyChecker class
  FSM_OperatingMode safetyPreCheck();

  //
  FSM_OperatingMode safetyPostCheck();

  // Gets the next FSM_State from the list of created states when requested
  FSM_State<T>* getNextState(FSM_StateName stateName);

  // Prints the current FSM status
  void printInfo(int opt);

  // Contains all of the control related data
  ControlFSMData<T> data;

  // FSM state information
  FSM_StatesList<T> statesList;  // holds all of the FSM States
  FSM_State<T>* currentState = nullptr;    // current FSM state
  FSM_State<T>* nextState = nullptr;       // next FSM state
  FSM_StateName nextStateName;   // next FSM state name
  SafetyChecker<T>* safetyChecker = nullptr;


 private:  
  // Operating mode of the FSM
  FSM_OperatingMode operatingMode;

  // Choose how often to print info, every N iterations
  int printNum = 10000;  // N*(0.001s) in simulation time

  // Track the number of iterations since last info print
  int printIter = 0;  // make larger than printNum to not print

  int iter = 0;

};

#endif  // CONTROLFSM_H
