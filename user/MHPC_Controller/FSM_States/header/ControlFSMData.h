#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include <ControlParameters/RobotParameters.h>
#include "MHPCUserParameters.h"
#include "Controllers/DesiredStateCommand.h"
#include "MHPC_CompoundTypes.h"
#include "Gait.h"
#include "Controllers/LegController.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Dynamics/Quadruped.h"
#include "Controllers/ContactEstimator.h"
/**
 *
 */
template <typename T>
struct ControlFSMData {
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Quadruped<T>* _quadruped;
  StateEstimatorContainer<T>* _stateEstimator;
  LegController<T>* _legController;
  DesiredStateCommand<T>* _desiredStateCommand;
  USRCMD<double> *_usrcmd;
  ContactEstimator<T> *_contactEstimator;
  Gait *_gait;
  RobotControlParameters* controlParameters;
  MHPCUserParameters* userParameters;
  VisualizationData* visualizationData;
};

template struct ControlFSMData<float>;

#endif  // CONTROLFSM_H