#include "MHPC_Controller.hpp"

void MHPC_Controller::initializeController()
{
  printf("Initialize MHPC_Controller\n");
  _gait = new Gait;
  _contactEstimator = new ContactEstimator<float>(_model, _quadruped,
                                                  _legController, _stateEstimator,
                                                  _stateEstimate, _controlParameters);

  _controlFSM = new ControlFSM<float>(_quadruped,  _stateEstimator, _legController,                                             
                                      _desiredStateCommand, &usrcmd, _contactEstimator,
                                      _gait, _controlParameters, &userParameters,
                                      _visualizationData);
}

void MHPC_Controller::runController()
{
  // Find the desired state trajectory
  _desiredStateCommand->convertToStateCommands();

  // Run the Control FSM code
  _controlFSM->runFSM();
}  



