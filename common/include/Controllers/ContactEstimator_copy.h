/*! @file ContactEstimator.h
 *  @brief All Contact Estimation Algorithms
 *
 *  This file will contain all contact detection algorithms. For now, it just
 * has a pass-through algorithm which passes the phase estimation to the state
 * estimator.  This will need to change once we move contact detection to C++
 *
 *  We also still need to establish conventions for "phase" and "contact".
 */

#ifndef PROJECT_CONTACTESTIMATOR_H
#define PROJECT_CONTACTESTIMATOR_H

#include "Controllers/StateEstimatorContainer.h"
#include "Controllers/LegController.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Dynamics/Quadruped.h"


/*!
 * A "passthrough" contact estimator which returns the expected contact state
 */
template <typename T>
class ContactEstimator : public GenericEstimator<T>{
 public:
  ContactEstimator():GenericEstimator<T>(){
  
  }
 

  /*!
   * Set the estimated contact by copying the exptected contact state into the
   * estimated contact state
   */
  // virtual void run(){}

  virtual void run() {

    this->_stateEstimatorData.result->contactEstimate =
        *this->_stateEstimatorData.contactPhase;
  }

  /*!
   * Set up the contact estimator
   */
  virtual void setup(){}

  private:
 

};

#endif  // PROJECT_CONTACTESTIMATOR_H
