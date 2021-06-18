#include "MHPC_Controller.hpp"

void MHPC_Controller::initializeController()
{
  printf("Initialize MHPC_Controller\n");
  _gait = new Gait<casadi_real>;
  _contactEstimator = new ContactEstimator<casadi_real>(_model, _quadruped,
                                                        _legController, _stateEstimator,
                                                        _stateEstimate, _controlParameters);

  _controlFSM = new ControlFSM<casadi_real>(_quadruped,
                                             _stateEstimator,
                                             _legController,
                                             _gait,
                                             _desiredStateCommand,
                                             &usrcmd,
                                             _contactEstimator,
                                             _controlParameters,
                                             &userParameters,
                                             _visualizationData);
}

void MHPC_Controller::runController()
{
  // Find the desired state trajectory
  _desiredStateCommand->convertToStateCommands();

  // Run the Control FSM code
  _controlFSM->runFSM();
}  



