#ifndef PROJECT_MHPCUSERPARAMETERS_H
#define PROJECT_MHPCUSERPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

class MHPCUserParameters : public ControlParameters {
public:
  MHPCUserParameters()
      : ControlParameters("user-parameters"),
        INIT_PARAMETER(kp_joint),
        INIT_PARAMETER(kd_joint),
        INIT_PARAMETER(home_fleg),
        INIT_PARAMETER(home_bleg),
        INIT_PARAMETER(home_pos),
        INIT_PARAMETER(home_rpy),
        INIT_PARAMETER(n_wbphase),
        INIT_PARAMETER(n_fbphase),
        INIT_PARAMETER(dt_wb),
        INIT_PARAMETER(dt_fb),
        INIT_PARAMETER(cmode),
        INIT_PARAMETER(groundH)
      {}
  DECLARE_PARAMETER(Vec3<double>, kp_joint);
  DECLARE_PARAMETER(Vec3<double>, kd_joint);
  DECLARE_PARAMETER(Vec3<double>, home_fleg);
  DECLARE_PARAMETER(Vec3<double>, home_bleg);
  DECLARE_PARAMETER(Vec3<double>, home_pos);
  DECLARE_PARAMETER(Vec3<double>, home_rpy);
  DECLARE_PARAMETER(double, n_wbphase);
  DECLARE_PARAMETER(double, n_fbphase);
  DECLARE_PARAMETER(double, dt_wb);
  DECLARE_PARAMETER(double, dt_fb);
  DECLARE_PARAMETER(double, cmode);
  DECLARE_PARAMETER(double, groundH);
};

#endif //PROJECT_MHPCUSERPARAMETERS_H
