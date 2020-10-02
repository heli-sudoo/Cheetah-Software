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
    _state.bodyVelocity = SVec<T>::Zero();
    _state.bodyPosition = Vec3<T>::Zero();
    _state.bodyOrientation = Quat<T>::Zero();
    _state.q = DVec<T>::Zero(12);
    _state.qd = DVec<T>::Zero(12);
  }
 

  /*!
   * Set the estimated contact by copying the exptected contact state into the
   * estimated contact state
   */
  virtual void run();

  // virtual void run() {

  //   this->_stateEstimatorData.result->contactEstimate =
  //       *this->_stateEstimatorData.contactPhase;
  // }

  /*!
   * Set up the contact estimator
   */
  virtual void setup();

  private:
  FBModelState<T> _state; 
  Quadruped<T> *_robot;
  LegControllerData<T> *_legDatas;
  std::vector<Vec3<T>> world_foot_loc_old;
  std::vector<Vec3<T>> world_foot_loc_new;
  std::vector<Vec3<T>> world_foot_vel;
  size_t _foot_ID[4]; // FR FL HR HL
  T _diff[4];
  Vec3<T> _loc_foot;

};

#endif  // PROJECT_CONTACTESTIMATOR_H
