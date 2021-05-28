#ifndef MHPC_CONTROLLER
#define MHPC_CONTROLLER

#include <RobotController.h>
#include "MHPCUserParameters.h"
#include "MHPC_cpptypes.h"

#define STANDUP 1
#define PREBOUNDING 2
#define BOUNDING 3
#define JOINTPD 4
#define PASSIVE 0

class MHPC_Controller:public RobotController{
  public:
    MHPC_Controller():RobotController(),_mhpc_ini(cheetah::num_act_joint){
    _mhpc_ini.setZero();
    _K_DDP_data = new Mat4_14<float> [4500];
    _init_foot_pos = std::vector<Vec3<float>> (4, Vec3<float>::Zero());
    _init_joint_pos = std::vector<Vec3<float>> (4, Vec3<float>::Zero());
    _F_ff = std::vector<Vec3<float>> (4, Vec3<float>::Zero());  
    _foot_vel_prev = std::vector<Vec3<float>> (4, Vec3<float>::Zero());  
    _foot_vel_current = std::vector<Vec3<float>> (4, Vec3<float>::Zero()); 
    _preBounding_qDes = std::vector<Vec3<float>> (4, Vec3<float>::Zero());
    
    _state.bodyVelocity.setZero();
    _state.bodyPosition.setZero();
    _state.bodyOrientation.setZero();
    _state.q = DVec<float>::Zero(12);
    _state.qd = DVec<float>::Zero(12);
    _standUp_pDes.setZero();

    _bounding = false;
    _preBounding = false;
    _stand_up = false;
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

    void standUp_control_run();
    void standUp_control_enter();
    void standUp_control();
    void bounding_control_enter();
    void bounding_control_run();
    void bounding_control();
    void preBounding_control_enter();
    void preBounding_control_run();
    void preBounding_control();
    void jointPD_control();

    
    bool touchDown_check(int);
    bool takeoff_check(int);
    Vec4<float> getContactState();

    virtual ControlParameters* getUserControlParameters() {
      return &userParameters;
    }
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // overload new operator to avoid alignment issue
    std::vector<Vec4<float>> _tau_ff_data;
    std::vector<Vec4<float>> _qdes_data;
    std::vector<Vec4<float>> _qddes_data;
    std::vector<Vec3<float>> _pos_des_data;
    std::vector<Vec3<float>> _vel_des_data;
    std::vector<Vec3<float>> _F_ff;
    std::vector<Vec3<float>> _foot_vel_prev;
    std::vector<Vec3<float>> _foot_vel_current;
    std::vector<Vec3<float>> _init_foot_pos;
    std::vector<Vec3<float>> _init_joint_pos;
    std::vector<Vec3<float>> _preBounding_qDes;

    Mat4_14<float>* _K_DDP_data;

    Vec4<float> _tau_ff;  
    Mat4_14<float> _K;
    Mat4_3<float> _Kp_cart;
    Mat4_3<float> _Kd_cart; 
    Vec3<float> _pos_des;
    Vec3<float> _pos_act;
    Vec3<float> _vel_act;
    Vec3<float> _vel_des;
    Vec3<float> _loc_foot;
    Mat4<float> _Kp_hip_knee;
    Mat4<float> _Kd_hip_knee;
    Vec4<float> _q_act;
    Vec4<float> _qd_act;
    Vec4<float> _q_des;
    Vec4<float> _qd_des;
    Vec4<float> _progress;
    Vec4<bool> _first_takeoff;
    Vec4<float> _contactState;
    Vec3<float> _standUp_pDes;


    int iter_stand = 0;
    int iter_preBounding = 0;

    int _sim_len;
    int _stanceTime[4];
    int _duration[4] = {80,80,80,80};
    size_t _bounding_step;
    size_t _foot_ID[4] = {8,11,14,17};
    bool _bounding;
    bool _stand_up;
    bool _preBounding;
    bool _stanceFlag[4];    

    FBModelState<float> _state;
    LegControllerData<float> *_legDatas;
  protected:
    DVec<float> _mhpc_ini;
    FBModelState<float> homeState;
    MHPCUserParameters userParameters;
};

#endif
