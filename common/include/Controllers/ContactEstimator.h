# ifndef CONTACT_ESTIMATOR_H
# define CONTACT_ESTIMATOR_H

#include "Controllers/LegController.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Dynamics/Quadruped.h"
#include "Controllers/StateEstimatorContainer.h"
#include "cppTypes.h"
#include <vector>

template <typename T>
class ContactEstimator
{
public:
	ContactEstimator(FloatingBaseModel<T>* model,
            Quadruped<T>* quadruped,
            LegController<T>* legController,
            StateEstimatorContainer<T>* stateEstimator,
            StateEstimate<T>* stateEstimate,
            RobotControlParameters* controlParameters);


	void init();
	void run_one_step();
  Vec2<int> getTakeoffState2D();
  Vec2<int> getTouchdownState2D();     
private:
	bool touchDown_Check(int leg);
	bool takeOff_Check(int leg);
	Vec4<T> getContactState();


public:
	Vec4<int> _first_takeoff;
	Vec4<int> _stance_flag;
	Vec4<int> _TD_flag;
	Vec4<int> _TF_flag;
	Vec4<T>   _contactState;
	Vec4<int> _stancetime_exp; 	// expected stance time (# time steps) for each foot
	Vec4<int> _stancetime; 		// track actual stance time (# time steps) for each foot
	size_t 	  _foot_ID[4] = {8,11,14,17}; // foot ID according to mini cheetah link ID convention
 


private:
  FloatingBaseModel<T>* _model = nullptr;
  Quadruped<T>* _quadruped = nullptr;
  LegController<T>* _legController = nullptr;
  StateEstimatorContainer<T>* _stateEstimator = nullptr;
  StateEstimate<T>* _stateEstimate = nullptr;
  FBModelState<T> _state;
  LegControllerData<T> *_legDatas = nullptr;
  RobotControlParameters* _controlParameters = nullptr;

  std::vector<Vec3<T>> _foot_vel_curr;
  std::vector<Vec3<T>> _foot_vel_prev;
  Vec3<T> _loc_foot;


};

# endif // CONTACT_ESTIMATOR_H