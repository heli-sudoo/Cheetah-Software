#include "MultiPhaseDDP.h"
#include "MHPC_CPPTypes.h"
#include "MHPC_CompoundTypes.h"
#include "PlanarFloatingBase.h"
#include "PlanarQuadruped.h"
#include "Gait.h"
#include "ReferenceGen.h"
#include "MHPCConstraints.h"
#include "MHPCCost.h"
#include "MHPCUserParameters.h"
#include "ControlFSMData.h"
#include "MHPCLocomotion.h"

int main(int argn, char *argv[])
{
	// problem setup
    int n_WBphases(8), n_FBphases(0); // default values
    int n_phases;
    casadi_real dt_WB(0.002), dt_FB(0.01);
    USRCMD<casadi_real> usrcmd = {1.5, -0.13, 0, 0, 0}; // vd y_COM yaw pitch roll
    casadi_real ground_height = -0.404;                 // ground height in world frame
    int cmode;

    // set initial condition (always start from WB state)
    VecM<casadi_real, 7> q0_WB, qd0_WB;
    VecM<casadi_real, 14> x0_WB;
    q0_WB << 0, -0.1093, -0.1542, 1.0957, -2.2033, 0.9742, -1.7098;
    qd0_WB << 0.9011, 0.2756, 0.7333, 0.0446, 0.0009, 1.3219, 2.7346;
    x0_WB << q0_WB, qd0_WB;

    if (argn > 3)
    {
        cmode = atoi(argv[1]);
        n_WBphases = atoi(argv[2]);
        n_FBphases = atoi(argv[3]);
    }
    else
    {
        printf("Usage: ./optimization current_mode n_WBphase n_FBphase \n");
        printf("End program!\n");
        exit(0);
        return (0);
    }
    n_phases = n_WBphases + n_FBphases;

    // build whole-body model
    PlanarQuadruped<casadi_real> wbquad(dt_WB);
    wbquad.build_quadruped();

    // build foothold planner and floating-base model
    FootholdPlanner<casadi_real> footholdPlanner(&wbquad, usrcmd.vel, ground_height);
    PlanarFloatingBase<casadi_real> fbquad(&footholdPlanner, dt_FB);

    // define gait
    Gait<casadi_real> Bounding;

    // build cost and constraint model
    WBCost<casadi_real> wbcostGen(dt_WB);
    FBCost<casadi_real> fbcostGen(dt_FB);
    WBConstraint<casadi_real> wbconstrGen(&wbquad);
    FBConstraint<casadi_real> fbconstrGen(&fbquad);

    // build HSDDP problem
    HSDDP_OPTION<casadi_real> option;
    option.ReB_active = 1;
    option.AL_active = 1;
    option.max_AL_iter = 2;
    option.max_DDP_iter = 3;

	MultiPhaseDDP<casadi_real>(n_phases, option);    

    // build an MHPCLocomotion object
    MHPCUserParameters mhpcparams;
    MHPCLocomotion<casadi_real> locomotion(&mhpcparams, option);

    
}