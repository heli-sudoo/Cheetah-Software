/*! @file ContactEstimator.cpp
 *  @brief All Contact Estimation Algorithms
 *
 *  This file will contain all contact detection algorithms. For now, it just
 * has a pass-through algorithm which passes the phase estimation to the state
 * estimator.  This will need to change once we move contact detection to C++
 */

#include "Controllers/ContactEstimator.h"
#include <cmath>

template<typename T>
void ContactEstimator<T>::run(){
	static int iter(0);
	iter++;

	_state.bodyPosition = this->_stateEstimatorData.result->position;
	_state.bodyOrientation = this->_stateEstimatorData.result->orientation;
	_state.bodyVelocity.head(3) = this->_stateEstimatorData.result->omegaBody;
	_state.bodyVelocity.tail(3) = this->_stateEstimatorData.result->vBody;
	_legDatas = this->_stateEstimatorData.legControllerData;
	_state.q << _legDatas[0].q,_legDatas[1].q,_legDatas[2].q,_legDatas[3].q; 
	_state.qd << _legDatas[0].qd,_legDatas[1].qd,_legDatas[2].qd,_legDatas[3].qd; 	

	this->_model->setState(_state);

	for (size_t leg(0); leg < 4; ++leg)
	{
		world_foot_loc_new[leg] = this->_model->getPosition(_foot_ID[leg],_loc_foot);
		world_foot_vel[leg] = this->_model->getLinearVelocity(_foot_ID[leg], _loc_foot);
		this->_stateEstimatorData.result->contactEstimate[leg] = 	
							(abs(world_foot_vel[leg][2])<0.2) ? 1:0;
	}


}

template<typename T>
void ContactEstimator<T>::setup(){
	_robot = this->_stateEstimatorData.legControllerData->quadruped;
	_loc_foot << 0, 0, -_robot->_kneeLinkLength;
	_foot_ID[0] = 8;   // link idx is wrong in quadruped.h 
	_foot_ID[1] = 11;
	_foot_ID[2] = 14;
	_foot_ID[3] = 17;
	
  	
	world_foot_loc_old = std::vector<Vec3<T>>(4);
	world_foot_loc_new = std::vector<Vec3<T>>(4);	
	world_foot_vel = std::vector<Vec3<T>>(4);

}

template class ContactEstimator<float>;
template class ContactEstimator<double>;