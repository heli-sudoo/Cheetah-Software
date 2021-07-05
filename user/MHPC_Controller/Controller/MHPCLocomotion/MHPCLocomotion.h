#ifndef MHPC_LOCOMOTION_H
#define MHPC_LOCOMOTION_H
#include "MultiPhaseDDP.h"
#include "MHPC_CPPTypes.h"
#include "MHPC_CompoundTypes.h"
#include "PlanarFloatingBase.h"
#include "PlanarQuadruped.h"
#include "Gait.h"
#include "ReferenceGen.h"
#include "ControlFSMData.h"
#include "Controllers/StateEstimatorContainer.h"
#include "MHPCUserParameters.h"
#include "MHPCConstraints.h"
#include "MHPCCost.h"

template<typename T, typename TH>
class MHPCLocomotion:public MultiPhaseDDP<TH>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    MHPCLocomotion(MHPCUserParameters *Params, HSDDP_OPTION<TH>option_in);                
    ~MHPCLocomotion();
    void warmstart() override; // override default warm_start
    void initialization();
    void initialization(ControlFSMData<T> *datain); // overload initialization
    void reconfig_mhpc();
    void update_mhpc_if_needed();
    void solve_mhpc(); // modify the solve function in base class MultiphaseDDP
    void run();
    void get_one_solution(Vec14<T>&x, Vec4<T>&u, MatMN<T,4,14>&K, int step);

private:
    void memory_alloc();
    void memory_reset(); // set all data to zero
    void memory_free();
    void build_problem();
    void update_problem();
    void setup_command(ControlFSMData<T> *datain);
    void control_in_plane(); // bounding control in 2D
    void orientation_stabilization(); // regulate orientation in 3D if necessary



private:  
    // trajectory data
    ModelState<TH,xsize_WB, usize_WB, ysize_WB> ** ms_act_WB = nullptr;
    ModelState<TH,xsize_WB, usize_WB, ysize_WB> ** ms_nom_WB = nullptr;
    ModelState<TH,xsize_WB, usize_WB, ysize_WB> ** ms_ref_WB = nullptr;
    DynDerivative<TH, xsize_WB, usize_WB, ysize_WB> **dynpar_WB = nullptr; // dynamics partial along nominal trajectory
    RCostStruct<TH, xsize_WB, usize_WB, ysize_WB> **rcost_WB = nullptr;
    CostToGoStruct<TH, xsize_WB, usize_WB> **CTG_WB = nullptr;
    TCostStruct<TH, xsize_WB> *tcost_WB = nullptr;

    ModelState<TH,xsize_FB, usize_FB, ysize_FB> ** ms_act_FB = nullptr;
    ModelState<TH,xsize_FB, usize_FB, ysize_FB> ** ms_nom_FB = nullptr;
    ModelState<TH,xsize_FB, usize_FB, ysize_FB> ** ms_ref_FB = nullptr;
    DynDerivative<TH, xsize_FB, usize_FB, ysize_FB> **dynpar_FB = nullptr; // dynamics partial along nominal trajectory
    RCostStruct<TH, xsize_FB, usize_FB, ysize_FB> **rcost_FB = nullptr;
    CostToGoStruct<TH, xsize_FB, usize_FB> **CTG_FB = nullptr;
    TCostStruct<TH, xsize_FB> *tcost_FB = nullptr;   

    ModelState<TH,xsize_WB, usize_WB, ysize_WB> *ms_exec = nullptr; // solution bag for execution horizon
    CostToGoStruct<TH, xsize_WB, usize_WB> *CTG_exec = nullptr;

    // configuration data
    int n_wbphase, n_fbphase;
    double dt_wb, dt_fb;
    int cmode;
    DVec<int> mode_seq;
    DVec<float> timings;       // time duration of each phase
    DVec<int> N_TIMESTEPS; // num of time steps of each phase
    
    vector<int> pidx_WB;
    vector<int> pidx_FB;
    VecM<int, 2> touchdown2D;   // set to one for a short period of time after TD instant
    VecM<int, 2> takeoff2D;     // set to one for a short period of time after TO instant
    VecM<T, 14> state2D; // state estimate of planar quadruped

    // mhpc solution for one step
    VecM<T, 14> state2D_des;   
    VecM<T, 4> udes;    
    VecM<T, 7> qdes;
    VecM<T, 7> qddes;
    MatMN<T, 4, 14> Kddp; 

    // default initial condition for planar quadruped
    VecM<TH, 7> q0, qd0;
    VecM<TH, 14> x0; 

    int mhpcstep;

private:
    // types using lower precision T
    ControlFSMData<T> *data = nullptr;
    StateEstimate<T> *stateEstimate = nullptr;
    LegControllerData<T> *legData = nullptr;
    Gait *gait = nullptr;
    MHPCUserParameters *userParams = nullptr;

public:    
    // types using higher precision TH
    PlanarQuadruped<TH> *wbmodel = nullptr;
    PlanarFloatingBase<TH> *fbmodel = nullptr;
    ReferenceGen<TH> refGen;
    FBConstraint<TH> *fbconstrGen = nullptr;
    WBConstraint<TH> *wbconstrGen= nullptr;
    WBCost<TH> *wbcostGen = nullptr;
    FBCost<TH> *fbcostGen = nullptr;
    FootholdPlanner<TH> * footholdplanner = nullptr;
};




#endif //MHPC_LOCOMOTION_H