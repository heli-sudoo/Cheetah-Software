#ifndef MHPC_CONTROLLER
#define MHPC_CONTROLLER

#include <RobotController.h>
#include "MHPC_CPPTypes.h"
#include "MHPCUserParameters.h"
#include "MHPC_CompoundTypes.h"
#include "Gait.h"
#include "ControlFSM.h"
#include "Controllers/ContactEstimator.h"

class MHPC_Controller:public RobotController{
  
  public:
    MHPC_Controller():RobotController(){
      usrcmd.vel = 1.5;
      usrcmd.height = -0.13;
      usrcmd.roll = 0;
      usrcmd.pitch = 0;
      usrcmd.yaw = 0;
    }

    virtual void initializeController();
    virtual void runController();
    virtual void updateVisualization(){}
    virtual ControlParameters* getUserControlParameters() {
      return &userParameters;
    }

  public:
    USRCMD<casadi_real> usrcmd; // user command

  protected:
    MHPCUserParameters userParameters;
    Gait<casadi_real> *_gait = nullptr;
    ContactEstimator<casadi_real> *_contactEstimator; // contact estimator by He Li
    ControlFSM<casadi_real> *_controlFSM;

};

#endif
