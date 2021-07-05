#include "Controllers/ContactEstimator.h"

template <typename T>
ContactEstimator<T>::ContactEstimator(FloatingBaseModel<T>* model,
                                      Quadruped<T>* quadurped,
                                      LegController<T>* legController,
                                      StateEstimatorContainer<T>* stateEstimator,
                                      StateEstimate<T>* stateEstimate,
                                      RobotControlParameters* controlParameters):
                                      _foot_vel_curr(4, Vec3<T>::Zero()),
                                      _foot_vel_prev(4, Vec3<T>::Zero())
{
  this->_model = model;
  this->_quadruped = quadurped;
  this->_legController = legController;
  this->_stateEstimator = stateEstimator;
  this->_stateEstimate = stateEstimate;
  this->_controlParameters = controlParameters;  
  this->_state.q.setZero(12);
  this->_state.qd.setZero(12);
  _loc_foot << 0,0,-_quadruped->_kneeLinkLength;
  init();
}

template <typename T>
void ContactEstimator<T>::init()
{
  _first_takeoff.setZero();
  _stance_flag.setOnes();
  _TD_flag.setZero();
  _TF_flag.setZero();
  _stancetime.setZero();
  _stancetime_exp.head(2).setConstant(round(0.08/_controlParameters->controller_dt)); // front stance
  _stancetime_exp.tail(2).setConstant(round(0.08/_controlParameters->controller_dt)); // back stance
  _contactState.setConstant(0.5);
}

template <typename T>
void ContactEstimator<T>::run_one_step()
{
  int leg = 0;
  _legDatas = _legController->datas;
  // get foot vel for the current and previous steps
  _foot_vel_prev = _foot_vel_curr;
  for (leg=0; leg < 4; ++leg)
   {
     _foot_vel_curr[leg] = _legDatas[leg].v; 
   } 
  for (leg = 0; leg < 4; ++leg)
   {
     _TD_flag[leg] = touchDown_Check(leg);
     _TF_flag[leg] = takeOff_Check(leg);
   } 

   _contactState = getContactState();

}

template <typename T>
Vec4<T> ContactEstimator<T>::getContactState()
{
  Vec4<T> progress(0.5, 0.5, 0.5, 0.5);
  if (!_first_takeoff.isZero())
  {
    for (int leg(0); leg < 4; ++leg)
    {
      if (_first_takeoff[leg]){ // if hasn't made the first takeoff      
        
        if (takeOff_Check(leg)) // if takeoff
        {
          progress[leg] = 0;
          _first_takeoff[leg] = 0;
          _stance_flag[leg] = 0;
        }
      }
      else{
        progress[leg] = 0;
      }
    }
  }else {
    for (int leg(0); leg < 4; ++leg)
    {
      if (!_stance_flag[leg]) {// if in flight phase, check touchdown
        // _stancetime[leg] = 0;
        if (touchDown_Check(leg)){
          _stance_flag[leg] = 1; 
        }
        progress[leg] = 0;
      }else{
        ++_stancetime[leg];
        if (_stancetime[leg]>_stancetime_exp[leg])
        {
          _stance_flag[leg] = 0;
          _stancetime[leg] = 0;
        }       
      }
      progress[leg] = _stancetime[leg]/_stancetime_exp[leg]; // works but  estimation goes below ground
    }
  }
  return progress;
}

template <typename T>
bool ContactEstimator<T>::touchDown_Check(int leg){
  return (_foot_vel_curr[leg]-_foot_vel_prev[leg]).norm() > 0.3;
}

template <typename T>
bool ContactEstimator<T>::takeOff_Check(int leg){
  _state.bodyPosition = _stateEstimate->position;
  _state.bodyOrientation = _stateEstimate->orientation;
  _state.bodyVelocity.head(3) = _stateEstimate->omegaBody;
  _state.bodyVelocity.tail(3) = _stateEstimate->vBody;
  _legDatas = _legController->datas;
  _state.q << _legDatas[0].q,_legDatas[1].q,_legDatas[2].q,_legDatas[3].q; 
  _state.qd << _legDatas[0].qd,_legDatas[1].qd,_legDatas[2].qd,_legDatas[3].qd; 
  _model->setState(_state);  

  return _model->getLinearVelocity(_foot_ID[leg], _loc_foot)[2]>0.7;
}

/* 
    Touchdown detection of the virtual 2D model
*/
template <typename T>
Vec2<int> ContactEstimator<T>::getTouchdownState2D()
{
  // if any one of the front (back) feet touches down, set TD to one
  Vec2<int> TD_flag_2D;
  for (int leg = 0; leg < 4; ++leg)
  {
    _TD_flag[leg] = touchDown_Check(leg);
  }

  TD_flag_2D[0] = !_TD_flag.head(2).isZero();
  TD_flag_2D[1] = !_TD_flag.tail(2).isZero();
  return TD_flag_2D;
}

/* 
    Takeoff detection of the virtual 2D model
*/
template <typename T>
Vec2<int> ContactEstimator<T>::getTakeoffState2D()
{
    // if any one of the front (back) feet takes off, set TF to one
  Vec2<int> TF_flag_2D;
  for (int leg = 0; leg < 4; ++leg)
  {
    _TF_flag[leg] = takeOff_Check(leg);
  }
  TF_flag_2D[0] = !_TF_flag.head(2).isZero();
  TF_flag_2D[1] = !_TF_flag.tail(2).isZero();
  return TF_flag_2D;
}

// explicit instantiation of template ContactEstimator since 
// template declaration and definition are in separate files
template class ContactEstimator<float>;
template class ContactEstimator<double>;
