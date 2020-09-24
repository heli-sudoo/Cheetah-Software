#ifndef MHPC_CONTROLLER
#define MHPC_CONTROLLER

#include <RobotController.h>
#include "MHPCUserParameters.h"
#include "MHPC_cpptypes.h"


class MHPC_Controller:public RobotController{
  public:
    MHPC_Controller():RobotController(),_mhpc_ini(cheetah::num_act_joint){
    _mhpc_ini.setZero();
    _K_DDP_data = new Mat4_14<float> [4500];
    }
    virtual ~MHPC_Controller(){delete [] _K_DDP_data;}

    virtual void initializeController();
    virtual void runController();
    virtual void updateVisualization(){}
    bool goHome_check();
    void read_tau_data();
    void read_qdes_data();
    void read_qddes_data();
    void read_pos_des_data();
    void read_vel_des_data();
    void read_fb_mat_data();
    void read_4f_data(const char*, std::vector<Vec4<float>>&);
    void read_3f_data(const char*, std::vector<Vec3<float>>&);

    void write_tau_data();
    void write_qdes_data();
    void write_qddes_data();
    void write_fb_mat_data();
    void write_4f_data(const char*, std::vector<Vec4<float>>&);
    void write_mat_data(const char*, Mat4_14<float>*);


    virtual ControlParameters* getUserControlParameters() {
      return &userParameters;
    }
    
    int _sim_len;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // overload new operator to avoid alignment issue
    std::vector<Vec4<float>> _tau_ff_data;
    std::vector<Vec4<float>> _qdes_data;
    std::vector<Vec4<float>> _qddes_data;
    std::vector<Vec3<float>> _pos_des_data;
    std::vector<Vec3<float>> _vel_des_data;
    // std::vector<Mat4_14<float>> _K_DDP_data;
    Mat4_14<float>* _K_DDP_data;

    Vec4<float> _tau_ff;  
    Mat4_14<float> _K;
    Mat4_3<float> _Kp_cart;
    Mat4_3<float> _Kd_cart; 
    Vec3<float> _pos_des;
    Vec3<float> _pos_act;
    Vec3<float> _vel_act;
    Vec3<float> _vel_des;

    Mat4<float> _Kp_hip_knee;
    Mat4<float> _Kd_hip_knee;
    Vec4<float> _q_act;
    Vec4<float> _qd_act;
    Vec4<float> _q_des;
    Vec4<float> _qd_des;

    size_t bounding_step;
    bool bounding;
  protected:
    DVec<float> _mhpc_ini;
    FBModelState<float> homeState;
    MHPCUserParameters userParameters;
};

#endif
