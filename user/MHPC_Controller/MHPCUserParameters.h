#ifndef PROJECT_MHPCUSERPARAMETERS_H
#define PROJECT_MHPCUSERPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

class MHPCUserParameters : public ControlParameters {
public:
  MHPCUserParameters()
      : ControlParameters("user-parameters"),
        INIT_PARAMETER(tau_ff),
        INIT_PARAMETER(kp),
        INIT_PARAMETER(kd),
        INIT_PARAMETER(zero),
        INIT_PARAMETER(calibrate),
        INIT_PARAMETER(home_fleg),
        INIT_PARAMETER(home_bleg),
        INIT_PARAMETER(home_pos),
        INIT_PARAMETER(home_rpy)
      {}

  DECLARE_PARAMETER(double, tau_ff);
  DECLARE_PARAMETER(double, kp);
  DECLARE_PARAMETER(double, kd);
  DECLARE_PARAMETER(double, zero);
  DECLARE_PARAMETER(double, calibrate);
  DECLARE_PARAMETER(Vec3<double>, home_fleg);
  DECLARE_PARAMETER(Vec3<double>, home_bleg);
  DECLARE_PARAMETER(Vec3<double>, home_pos);
  DECLARE_PARAMETER(Vec3<double>, home_rpy);
};

#endif //PROJECT_MHPCUSERPARAMETERS_H
