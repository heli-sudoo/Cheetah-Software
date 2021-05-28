#include "MHPCLocomotion.h"
#include "MHPCUtils.h"
#include "boundingPDControl.h"

template <typename T>
MHPCLocomotion<T>::MHPCLocomotion(MHPCUserParameters*params, HSDDP_OPTION<T>option_in):                
                MultiPhaseDDP<T>(params->n_wbphase+params->n_fbphase, 
                                 option_in),
                                 dt_wb(params->dt_wb),
                                 dt_fb(params->dt_fb),
                                 userParams(params),
                                 n_wbphase(params->n_wbphase),
                                 n_fbphase(params->n_fbphase),
                                 cmode(params->cmode)                                     
{
    mode_seq.setOnes(this->_n_phases);
    timings.setZero(this->_n_phases);
    N_TIMESTEPS.setZero(this->_n_phases);

    wbmodel = new PlanarQuadruped<T>(dt_wb);
    wbmodel->build_quadruped();
    footholdplanner = new FootholdPlanner<T>(wbmodel, 1.0, -0.404);
    fbmodel = new PlanarFloatingBase<T>(footholdplanner, dt_fb);
    wbcostGen = new WBCost<T>(dt_wb);
    fbcostGen = new FBCost<T>(dt_fb);
    wbconstrGen = new WBConstraint<T>(wbmodel);
    fbconstrGen = new FBConstraint<T>(fbmodel);

    this->_stateProj.setZero(xsize_FB, xsize_WB);
    this->_stateProj.block(0, 0, 3, 3) = DMat<T>::Identity(3, 3);
    this->_stateProj.block(3, 7, 3, 3) = DMat<T>::Identity(3, 3);

    /* set default initial condition */
    q0 << 0, -0.1093, -0.1542, 1.0957, -2.2033, 0.9742, -1.7098;
    qd0 << 0.9011, 0.2756, 0.7333, 0.0446, 0.0009, 1.3219, 2.7346;
    x0 << q0, qd0;

    /* initialize class members */
    memory_alloc(); // allocate memory for data
}

template <typename T>
void MHPCLocomotion<T>::initialization(ControlFSMData<T> * datain)
{
    setup_command(datain);
    initialization();
}

template <typename T>
void MHPCLocomotion<T>::initialization()
{
    this->set_initial_condition(x0);
    memory_reset();
    build_problem();
    warmstart();
    solve_mhpc();
    mhpcstep = 0;
    dt_wb = userParams->dt_wb;
    dt_fb = userParams->dt_fb;
    n_wbphase = userParams->n_wbphase;
    n_fbphase = userParams->n_fbphase;
    cmode = userParams->cmode;    
    data->_contactEstimator->init();      // initialize contact estimator to full stance
}

template <typename T>
void MHPCLocomotion<T>::run()
{
    // get current state (trunk state and leg data)
    stateEstimate = data->_stateEstimator->getResultHandle();
    legData = data->_legController->datas;
    state2D = Convert3DEstimateTo2D(stateEstimate, legData);

    data->_contactEstimator->run_one_step();
    data->_stateEstimator->setContactPhase(data->_contactEstimator->_contactState);

    touchdown2D = data->_contactEstimator->getTouchdownState2D();
    takeoff2D = data->_contactEstimator->getTakeoffState2D();
    update_mhpc_if_needed();
    get_one_solution(state2D_des, udes, Kddp, mhpcstep); // get one solution from the solution bag
    control_in_plane(); // bounding control in 2D
    orientation_stabilization(); // regulate orientation in 3D if necessary

    mhpcstep++;
}

template <typename T>
void MHPCLocomotion<T>::setup_command(ControlFSMData<T> *_data)
{
    data = _data;    
    gait = data->_gait;
}

template <typename T>
void MHPCLocomotion<T>::update_mhpc_if_needed()
{
    if (touchdown2D.isZero() && takeoff2D.isZero())
      return;

    /* if mode transition detected, update mhpc with the current state estimate*/
    this->set_initial_condition(state2D);     
    memory_reset();
    update_problem();
    warmstart();
    solve_mhpc();
    mhpcstep = 0; // reset indexing of mhpc solution bag to the begining
}

template <typename T>
void MHPCLocomotion<T>::reconfig_mhpc()
{
    memory_free();
    memory_alloc();
    initialization();
}

template <typename T>
void MHPCLocomotion<T>::control_in_plane()
{   
    Vec4<T> qj  = state2D.segment(3,4);
    Vec4<T> qjd = state2D.tail(4); 
    Vec4<T> qj_des  = state2D_des.segment(3,4);
    Vec4<T> qjd_des = state2D_des.tail(4);
    MatMN<T,4,3> Kp_body_ddp = Kddp.block(0,0,4,3);
    MatMN<T,4,3> Kd_body_ddp = Kddp.block(0,7,4,3);
    MatMN<T,4,4> Kp_joint_ddp = Kddp.block(0,3,4,4);
    MatMN<T,4,4> Kd_joint_ddp = Kddp.block(0,10,4,4);

    Vec3<double> Kp_temp = data->userParameters->kp_joint; 
    Vec3<double> Kd_temp = data->userParameters->kd_joint; 
    Mat3<T> Kp_pd = Kp_temp.cast<T>().asDiagonal();
    Mat3<T> Kd_pd = Kd_temp.cast<T>().asDiagonal();

    /* feedback from body state */
    T sp = 0.2, sd = 1; // scale the body state feeback if necessary
    udes += sp*Kp_joint_ddp*(state2D.head(3)-state2D_des.head(3))
            + sd*Kd_body_ddp*(state2D.segment(7,3)-state2D_des.segment(7,3));

    /* feedback from leg state*/
    udes += Kp_joint_ddp*(qj -qj_des)+ Kd_joint_ddp*(qjd - qjd_des);
    
    for(int leg(0); leg<4; ++leg)
    {
      int leg2D = leg/2; // 0 or 1  
      data->_legController->commands[leg].qDes[0] =  0;
      data->_legController->commands[leg].qDes[1] =  qj_des[2*leg2D];
      data->_legController->commands[leg].qDes[2] =  qj_des[2*leg2D+1];
      data->_legController->commands[leg].qdDes[0] = 0;
      data->_legController->commands[leg].qdDes[1] = qjd_des[2*leg2D];
      data->_legController->commands[leg].qdDes[2] = qjd_des[2*leg2D+1];
      data->_legController->commands[leg].tauFeedForward[0] = 0;
      data->_legController->commands[leg].tauFeedForward[1] = udes[2*leg2D]/2;    // divided by 2 from 2D to 3D
      data->_legController->commands[leg].tauFeedForward[2] = udes[2*leg2D+1]/2;
      data->_legController->commands[leg].kpJoint = Kp_pd;
      data->_legController->commands[leg].kdJoint = Kd_pd;
    }
}

template <typename T>
void MHPCLocomotion<T>::orientation_stabilization()
{
    /* regulate roll motion*/
      float tau_roll_fb, kp_roll = 20, kd_roll=2, dist;
      vector<Vec3<T>> F_ff(4, Vec3<T>::Zero());
      F_ff[0].setZero();
      F_ff[1].setZero();
      F_ff[2].setZero();
      F_ff[3].setZero();

      tau_roll_fb = -kp_roll*stateEstimate->rpy[0] - kd_roll*stateEstimate->omegaWorld[0];
      dist = 0.05;

      if (data->_contactEstimator->_stance_flag[0]) 
        {F_ff[0][2] = 0.5*tau_roll_fb/dist; }
      if (data->_contactEstimator->_stance_flag[1]) 
        {F_ff[1][2] = -0.5*tau_roll_fb/dist; }
      if (data->_contactEstimator->_stance_flag[2]) 
        {F_ff[2][2] = 0.5*tau_roll_fb/dist; }
      if (data->_contactEstimator->_stance_flag[3]) 
        {F_ff[3][2] = -0.5*tau_roll_fb/dist; }

      /* regulate yaw motion */
      float tau_yaw_fb, kp_yaw = 20, kd_yaw = 2;
      tau_yaw_fb = -kp_yaw *stateEstimate->rpy[2] - kd_yaw*stateEstimate->omegaWorld[2];
      dist = 0.25;
      
      if (data->_contactEstimator->_stance_flag[0]) 
        {F_ff[0][1] += -0.5*tau_yaw_fb/dist; }
      if (data->_contactEstimator->_stance_flag[1]) 
        {F_ff[1][1] += -0.5*tau_yaw_fb/dist; }
      if (data->_contactEstimator->_stance_flag[2]) 
        {F_ff[2][1] += 0.5*tau_yaw_fb/dist; }
      if (data->_contactEstimator->_stance_flag[3]) 
        {F_ff[3][1] += 0.5*tau_yaw_fb/dist; }  

      for (int leg(0); leg < 4; ++leg)
      {
        data->_legController->commands[leg].forceFeedForward = F_ff[leg];
      }
}


/*
  @brief: build multi-phase problem
  @params:
          cmode: current mode index
          usrcmd: user command
*/
template <typename T>
void MHPCLocomotion<T>::build_problem()
{
    mode_seq = gait->get_mode_seq(cmode, this->_n_phases); // compute mode index for each phase
    timings = gait->get_timings(mode_seq);                 // compute timings for each phase

    vector<ModelState<T, xsize_WB, usize_WB, ysize_WB> *> ref_WB_vec;
    vector<ModelState<T, xsize_FB, usize_FB, ysize_FB> *> ref_FB_vec;

    /* configure single-phase problem */
    for (size_t pidx = 0; pidx < this->_n_phases; pidx++)
    {
        if (pidx < n_wbphase) // If full-model whole body phase
        {
            N_TIMESTEPS[pidx] = round(timings[pidx] / dt_wb);
            this->_phases[pidx]->set_models(wbmodel, wbcostGen, wbconstrGen, &this->_option, dt_wb);
            this->_phases[pidx]->set_phase_config(mode_seq[pidx], pidx, N_TIMESTEPS[pidx]);
            this->_phases[pidx]->set_data(ms_act_WB[pidx], ms_nom_WB[pidx], dynpar_WB[pidx], rcost_WB[pidx],
                                          CTG_WB[pidx], &tcost_WB[pidx], ms_ref_WB[pidx]);
            ref_WB_vec.push_back(ms_ref_WB[pidx]);
        }
        else // If simple-model rigid body phase
        {
            N_TIMESTEPS[pidx] = round(timings[pidx] / dt_fb);
            this->_phases[pidx]->set_models(fbmodel, fbcostGen, fbconstrGen, &this->_option, dt_fb);
            this->_phases[pidx]->set_phase_config(mode_seq[pidx], pidx, N_TIMESTEPS[pidx]);
            this->_phases[pidx]->set_data(ms_act_FB[pidx - n_wbphase], ms_nom_FB[pidx - n_wbphase], dynpar_FB[pidx - n_wbphase], rcost_FB[pidx - n_wbphase],
                                          CTG_FB[pidx - n_wbphase], &tcost_FB[pidx - n_wbphase], ms_ref_FB[pidx - n_wbphase]);
            ref_FB_vec.push_back(ms_ref_FB[pidx - n_wbphase]);
        }
        (pidx < this->_n_phases - 1) ? this->_phases[pidx]->set_next_phase(this->_phases[pidx + 1]) // assign next phase
                                     : this->_phases[pidx]->set_next_phase(nullptr);
        this->_phases[pidx]->initialization();
    }

    /* Initialize reference generator */
    refGen.Initialization(n_wbphase, n_fbphase,
                           ref_WB_vec, ref_FB_vec,
                           dt_wb, dt_fb,
                           mode_seq, N_TIMESTEPS, data->_usrcmd->vel, data->_usrcmd->height);
    /* computes tracking reference for multi-phase problem */
    refGen.generate_ref(this->_x0);
}

template <typename T>
void MHPCLocomotion<T>::update_problem()
{
    vector<ModelState<T, xsize_WB, usize_WB, ysize_WB> *> ref_WB_vec;
    vector<ModelState<T, xsize_FB, usize_FB, ysize_FB> *> ref_FB_vec;

    int pidx_temp = pidx_WB.front();
    pidx_WB.erase(pidx_WB.begin());
    pidx_WB.push_back(pidx_temp);
    if (n_fbphase > 0)
    {
        pidx_temp = pidx_FB.front();
        pidx_FB.erase(pidx_FB.begin());
        pidx_FB.push_back(pidx_temp);
    }

    cmode = gait->get_next_mode(cmode);
    mode_seq = gait->get_mode_seq(cmode, this->_n_phases); // update mode sequence
    timings = gait->get_timings(mode_seq);                 // update phase timing

    /* update mode index tied to each single phase */
    for (size_t idx = 0; idx < this->_n_phases; idx++)
    {
        N_TIMESTEPS[idx] = round(timings[idx] / this->_phases[idx]->_dt);
        this->_phases[idx]->set_phase_config(mode_seq[idx], idx, N_TIMESTEPS[idx]);
    }

    /* update data tied to each single phase */
    int pidx;
    for (size_t idx = 0; idx < this->_n_phases; idx++)
    {
        if (idx < n_wbphase) // If full-model whole body phase
        {
            pidx = pidx_WB[idx];
            this->_phases[idx]->set_data(ms_act_WB[pidx], ms_nom_WB[pidx], dynpar_WB[pidx], rcost_WB[pidx],
                                         CTG_WB[pidx], &tcost_WB[pidx], ms_ref_WB[pidx]);
            ref_WB_vec.push_back(ms_ref_WB[pidx]);                                         
        }
        else // If simple-model rigid body phase
        {
            pidx = pidx_FB[idx -n_wbphase];
            this->_phases[idx]->set_data(ms_act_FB[pidx], ms_nom_FB[pidx], dynpar_FB[pidx], rcost_FB[pidx],
                                          CTG_FB[pidx], &tcost_FB[pidx], ms_ref_FB[pidx]);
            ref_FB_vec.push_back(ms_ref_FB[pidx]);                                
        }
        this->_phases[idx]->initialization();
    }

    /* Initialize reference generator */
    refGen.update_phase_config(mode_seq, N_TIMESTEPS);
    refGen.update_data(ref_WB_vec, ref_FB_vec);
    refGen.generate_ref(this->_x0);
}



/*
    Reimplements the solve() function as defined in base class MultiPhaseDDP by copying the solution 
    to the execution horizon 
*/
template <typename T>
void MHPCLocomotion<T>::solve_mhpc()
{
    MultiPhaseDDP<T>::solve();
    assert(("Whole-body phase needs to be at least one", n_wbphase>=1));

    /* copy nominal state and feedback information to the execution horizon from the first phase of the solution bag*/
    std::memcpy(ms_exec, (ModelState<T, xsize_WB, usize_WB, ysize_WB>*)this->_phases[0]->get_nominal_ms_ptr(), this->_phases[0]->_N_TIMESTEPS);
    std::memcpy(CTG_exec, (ModelState<T, xsize_WB, usize_WB, ysize_WB>*)this->_phases[0]->get_CTG_info_ptr(), this->_phases[0]->_N_TIMESTEPS);
    
    /* copy the second phase if any*/
    if (n_wbphase > 1)
    {
        std::memcpy(ms_exec + this->_phases[0]->_N_TIMESTEPS, 
                    (ModelState<T, xsize_WB, usize_WB, ysize_WB>*)this->_phases[1]->get_nominal_ms_ptr(), 
                    this->_phases[1]->_N_TIMESTEPS);
        std::memcpy(CTG_exec + this->_phases[0]->_N_TIMESTEPS, 
                    (ModelState<T, xsize_WB, usize_WB, ysize_WB>*)this->_phases[1]->get_CTG_info_ptr(), 
                    this->_phases[1]->_N_TIMESTEPS);
    }    
}

template <typename T>
void MHPCLocomotion<T>::get_one_solution(Vec14<T>&x, Vec4<T>&u, MatMN<T,4,14>&K, int step)
{
    int k(0);
    if (n_wbphase==1)
    {
        k = (step < this->_phases[0]->_N_TIMESTEPS-1) ? step : this->_phases[0]->_N_TIMESTEPS-1; 
    } 
    if (n_wbphase>1)
    {
        int N = this->_phases[0]->_N_TIMESTEPS + this->_phases[1]->_N_TIMESTEPS;
        k = (step < N -1) ? step : N - 1;
    }
    x = ms_exec[k].x;
    u = ms_exec[k].u;
    K = CTG_exec[k].K;
}


template <typename T>
void MHPCLocomotion<T>::warmstart()
{
    this->empty_bag();
    DVec<T> x0 = this->_x0;
    for (int idx = 0; idx < n_wbphase; idx++)
    {
        int pidx = pidx_WB[idx];
        ms_act_WB[pidx][0].x = x0;
        bounding_PDcontrol(wbmodel, ms_act_WB[pidx], mode_seq[idx], N_TIMESTEPS[idx]);
        if (idx + 1 < n_wbphase)
        {
            x0 = this->phase_transition(this->_phases[idx], this->_phases[idx + 1]);
        }
    }
    this->update_nominal_trajectory();
}

template <typename T>
void MHPCLocomotion<T>::memory_alloc()
{
    ms_act_WB = new ModelState<T, xsize_WB, usize_WB, ysize_WB> *[n_wbphase];
    ms_nom_WB = new ModelState<T, xsize_WB, usize_WB, ysize_WB> *[n_wbphase];
    ms_ref_WB = new ModelState<T, xsize_WB, usize_WB, ysize_WB> *[n_wbphase];
    dynpar_WB = new DynDerivative<T, xsize_WB, usize_WB, ysize_WB> *[n_wbphase];
    CTG_WB = new CostToGoStruct<T, xsize_WB, usize_WB> *[n_wbphase];
    rcost_WB = new RCostStruct<T, xsize_WB, usize_WB, ysize_WB> *[n_wbphase];
    tcost_WB = new TCostStruct<T, xsize_WB>[n_wbphase];

    ms_act_FB = new ModelState<T, xsize_FB, usize_FB, ysize_FB> *[n_fbphase];
    ms_nom_FB = new ModelState<T, xsize_FB, usize_FB, ysize_FB> *[n_fbphase];
    ms_ref_FB = new ModelState<T, xsize_FB, usize_FB, ysize_FB> *[n_fbphase];
    dynpar_FB = new DynDerivative<T, xsize_FB, usize_FB, ysize_FB> *[n_fbphase];
    CTG_FB = new CostToGoStruct<T, xsize_FB, usize_FB> *[n_fbphase];
    rcost_FB = new RCostStruct<T, xsize_FB, usize_FB, ysize_FB> *[n_fbphase];
    tcost_FB = new TCostStruct<T, xsize_FB>[n_fbphase];

    for (size_t idx = 0; idx < n_wbphase; idx++)
    {
        pidx_WB.push_back(idx);
        this->_phases[idx] = new SinglePhase<T, xsize_WB, usize_WB, ysize_WB>;
        ms_act_WB[idx] = new ModelState<T, xsize_WB, usize_WB, ysize_WB>[N_TIMESTEPS_MAX];
        ms_nom_WB[idx] = new ModelState<T, xsize_WB, usize_WB, ysize_WB>[N_TIMESTEPS_MAX];
        ms_ref_WB[idx] = new ModelState<T, xsize_WB, usize_WB, ysize_WB>[N_TIMESTEPS_MAX];
        dynpar_WB[idx] = new DynDerivative<T, xsize_WB, usize_WB, ysize_WB>[N_TIMESTEPS_MAX];
        CTG_WB[idx] = new CostToGoStruct<T, xsize_WB, usize_WB>[N_TIMESTEPS_MAX];
        rcost_WB[idx] = new RCostStruct<T, xsize_WB, usize_WB, ysize_WB>[N_TIMESTEPS_MAX];
    }

    for (size_t idx = 0; idx < n_fbphase; idx++)
    {
        pidx_FB.push_back(idx);
        this->_phases[idx + n_wbphase] = new SinglePhase<T, xsize_FB, usize_FB, ysize_FB>;
        ms_act_FB[idx] = new ModelState<T, xsize_FB, usize_FB, ysize_FB>[N_TIMESTEPS_MAX];
        ms_nom_FB[idx] = new ModelState<T, xsize_FB, usize_FB, ysize_FB>[N_TIMESTEPS_MAX];
        ms_ref_FB[idx] = new ModelState<T, xsize_FB, usize_FB, ysize_FB>[N_TIMESTEPS_MAX];
        dynpar_FB[idx] = new DynDerivative<T, xsize_FB, usize_FB, ysize_FB>[N_TIMESTEPS_MAX];
        CTG_FB[idx] = new CostToGoStruct<T, xsize_FB, usize_FB>[N_TIMESTEPS_MAX];
        rcost_FB[idx] = new RCostStruct<T, xsize_FB, usize_FB, ysize_FB>[N_TIMESTEPS_MAX];
    }
    ms_exec = new ModelState<T, xsize_WB, usize_WB, ysize_WB>[2*N_TIMESTEPS_MAX];
    CTG_exec = new CostToGoStruct<T, xsize_WB, usize_WB>[2*N_TIMESTEPS_MAX];
}


template <typename T>
void MHPCLocomotion<T>::memory_reset()
{
    for (size_t idx = 0; idx < n_wbphase; idx++)
    {
        std::memset(ms_act_WB[idx], 0, sizeof(ModelState<T, xsize_WB, usize_WB, ysize_WB>) * N_TIMESTEPS_MAX);
        std::memset(ms_nom_WB[idx], 0, sizeof(ModelState<T, xsize_WB, usize_WB, ysize_WB>) * N_TIMESTEPS_MAX);
        std::memset(ms_ref_WB[idx], 0, sizeof(ModelState<T, xsize_WB, usize_WB, ysize_WB>) * N_TIMESTEPS_MAX);
        std::memset(dynpar_WB[idx], 0, sizeof(DynDerivative<T, xsize_WB, usize_WB, ysize_WB>) * N_TIMESTEPS_MAX);
        std::memset(rcost_WB[idx], 0, sizeof(RCostStruct<T, xsize_WB, usize_WB, ysize_WB>) * N_TIMESTEPS_MAX);
        std::memset(CTG_WB[idx], 0, sizeof(CostToGoStruct<T, xsize_WB, usize_WB>) * N_TIMESTEPS_MAX);
        tcost_WB[idx].Zeros();
    }

    for (size_t idx = 0; idx < n_fbphase; idx++)
    {
        std::memset(ms_act_FB[idx], 0, sizeof(ModelState<T, xsize_FB, usize_FB, ysize_FB>) * N_TIMESTEPS_MAX);
        std::memset(ms_nom_FB[idx], 0, sizeof(ModelState<T, xsize_FB, usize_FB, ysize_FB>) * N_TIMESTEPS_MAX);
        std::memset(ms_ref_FB[idx], 0, sizeof(ModelState<T, xsize_FB, usize_FB, ysize_FB>) * N_TIMESTEPS_MAX);
        std::memset(dynpar_FB[idx], 0, sizeof(DynDerivative<T, xsize_FB, usize_FB, ysize_FB>) * N_TIMESTEPS_MAX);
        std::memset(rcost_FB[idx], 0, sizeof(RCostStruct<T, xsize_FB, usize_FB, ysize_FB>) * N_TIMESTEPS_MAX);
        std::memset(CTG_FB[idx], 0, sizeof(CostToGoStruct<T, xsize_FB, usize_FB>) * N_TIMESTEPS_MAX);
        tcost_FB[idx].Zeros();
    }
    std::memset(ms_exec, 0, sizeof(ModelState<T, xsize_WB, usize_WB, ysize_WB>) * 2*N_TIMESTEPS_MAX);
    std::memset(CTG_exec, 0, sizeof(CostToGoStruct<T, xsize_WB, usize_WB>) * 2*N_TIMESTEPS_MAX);    
}

template <typename T>
MHPCLocomotion<T>::~MHPCLocomotion()
{
    memory_free();
}

template <typename T>
void MHPCLocomotion<T>::memory_free()
{
    for (size_t i = 0; i < n_wbphase; i++)
    {
        delete[] ms_act_WB[i];
        delete[] ms_nom_WB[i];
        delete[] ms_ref_WB[i];
        delete[] dynpar_WB[i];
        delete[] CTG_WB[i];
        delete[] rcost_WB[i];
    }

    delete[] ms_act_WB;
    delete[] ms_nom_WB;
    delete[] ms_ref_WB;
    delete[] dynpar_WB;
    delete[] CTG_WB;
    delete[] rcost_WB;
    delete[] tcost_WB;

    ms_act_WB = nullptr;
    ms_nom_WB = nullptr;
    ms_ref_WB = nullptr;
    dynpar_WB = nullptr;
    CTG_WB = nullptr;
    rcost_WB = nullptr;
    tcost_WB = nullptr;

    delete [] ms_exec;
    delete [] CTG_exec;
    ms_exec = nullptr;
    CTG_exec = nullptr;

    for (size_t i = 0; i < n_fbphase; i++)
    {
        delete[] ms_act_FB[i];
        delete[] ms_nom_FB[i];
        delete[] ms_ref_FB[i];
        delete[] dynpar_FB[i];
        delete[] CTG_FB[i];
        delete[] rcost_FB[i];
    }

    delete[] ms_act_FB;
    delete[] ms_nom_FB;
    delete[] ms_ref_FB;
    delete[] dynpar_FB;
    delete[] CTG_FB;
    delete[] rcost_FB;
    delete[] tcost_FB;

    ms_act_FB = nullptr;
    ms_nom_FB = nullptr;
    ms_ref_FB = nullptr;
    dynpar_FB = nullptr;
    CTG_FB = nullptr;
    rcost_FB = nullptr;
    tcost_FB = nullptr;

    delete gait;
    delete wbmodel;
    delete fbmodel;
    delete fbconstrGen;
    delete wbconstrGen;
    delete wbcostGen;
    delete fbcostGen;

    gait = nullptr;
    wbmodel = nullptr;
    fbmodel = nullptr;
    fbconstrGen = nullptr;
    wbconstrGen= nullptr;
    wbcostGen = nullptr;
    fbcostGen = nullptr;

    for (size_t i = 0; i < this->_n_phases; i++)
    {
        delete this->_phases[i];
        this->_phases[i] = nullptr;
    }


}

template class MHPCLocomotion<casadi_real>;
// template class MHPCLocomotion<double>;