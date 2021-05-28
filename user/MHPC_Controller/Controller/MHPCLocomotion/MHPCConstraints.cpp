#include "MHPCConstraints.h"
#include "CasadiGen.h"

template <typename T>
FBConstraint<T>::FBConstraint(RobotBase<T> *model) : Constraint<T>(model)
{
    // assume no terminal constriant and path constraint for floating-base model planning
    this->_num_pconstr = std::vector<size_t>(4, 0);
    this->_num_tconstr = std::vector<size_t>(4, 0);
    this->_params = std::vector<AL_REB_PARAMETER<T>>(4, AL_REB_PARAMETER<T>(0,0));
}

template <typename T>
WBConstraint<T>::WBConstraint(RobotBase<T> *model) : Constraint<T>(model)
{
    _num_torque_limit = 2 * usize_WB;
    _num_joint_limit = 2 * usize_WB;
    _num_GRF_constraint = 3;

    this->_num_pconstr.push_back(_num_torque_limit + _num_joint_limit + _num_GRF_constraint);
    this->_num_tconstr.push_back(0);

    this->_num_pconstr.push_back(_num_torque_limit + _num_joint_limit);
    this->_num_tconstr.push_back(1);

    this->_num_pconstr.push_back(_num_torque_limit + _num_joint_limit + _num_GRF_constraint);
    this->_num_tconstr.push_back(0);

    this->_num_pconstr.push_back(_num_torque_limit + _num_joint_limit);
    this->_num_tconstr.push_back(1);

    for (size_t midx = 0; midx < 4; midx++)
    {
        this->_params.push_back(AL_REB_PARAMETER<T>(this->_num_tconstr[midx], this->_num_pconstr[midx]));
    }

    initialize_AL_REB_PARAMS();

    /* Initizalize ineqaulity constraint */ 
    C_torque.setZero(_num_torque_limit, usize_WB);
    b_torque.setZero(_num_torque_limit);
    C_joint.setZero(_num_joint_limit, usize_WB);
    b_joint.setZero(_num_joint_limit);
    C_grf.setZero(_num_GRF_constraint, ysize_WB);
    b_grf.setZero(_num_GRF_constraint);

    C_torque << -DMat<T>::Identity(usize_WB, usize_WB),
                DMat<T>::Identity(usize_WB, usize_WB);
    b_torque.setConstant(33);

    C_joint << -DMat<T>::Identity(usize_WB, usize_WB),
                DMat<T>::Identity(usize_WB, usize_WB);
    b_joint << PI / 4, -0.1, 1.15 * PI, -0.1,
               PI, PI - 0.2, .1, PI - 0.2;

}

template <typename T>
void WBConstraint<T>::initialize_AL_REB_PARAMS()
{
    // first mode AL and ReB parameters
    this->_params[0].delta.setConstant(0.1);
    this->_params[0].delta_min.setConstant(0.01);
    this->_params[0].eps_ReB << 0.01 * DVec<T>::Ones(_num_torque_limit), DVec<T>::Zero(_num_joint_limit), 0.01 * DVec<T>::Ones(_num_GRF_constraint);
    this->_params[0].reb_empty = false;

    // second mode AL and ReB parameters
    this->_params[1].delta.setConstant(0.1);
    this->_params[1].delta_min.setConstant(0.01);
    this->_params[1].eps_ReB << 0.01 * DVec<T>::Ones(_num_torque_limit), DVec<T>::Zero(_num_joint_limit);
    this->_params[1].sigma = 5;
    this->_params[1].reb_empty = false;
    this->_params[1].al_empty = false;

    // third mode AL and ReB parameters
    this->_params[2].delta.setConstant(0.1);
    this->_params[2].delta_min.setConstant(0.01);
    this->_params[2].eps_ReB << 0.01 * DVec<T>::Ones(_num_torque_limit), DVec<T>::Zero(_num_joint_limit), 0.01 * DVec<T>::Ones(_num_GRF_constraint);
    this->_params[2].reb_empty = false;

    // fourth mode AL and ReB parameters
    this->_params[3].delta.setConstant(0.1);
    this->_params[3].delta_min.setConstant(0.01);
    this->_params[3].eps_ReB << 0.01 * DVec<T>::Ones(_num_torque_limit), DVec<T>::Zero(_num_joint_limit);
    this->_params[3].sigma = 5;
    this->_params[3].reb_empty = false;
    this->_params[3].al_empty = false;
}

template <typename T>
void WBConstraint<T>::terminal_constraint(ModelState<T, xsize_WB, usize_WB, ysize_WB> &mstate,
                                          vector<TConstrStruct<T, xsize_WB>> &tconstraint,
                                          int modeidx)
{
    std::vector<casadi_real *> arg = {mstate.x.data()};
    std::vector<casadi_real *> res = {&tconstraint[0].h, tconstraint[0].hx.data(), tconstraint[0].hxx.data()};
    if (2 == modeidx)
    {
        casadi_interface(arg, res, tconstraint[0].hxx.size(), WB_FL1_terminal_constr, WB_FL1_terminal_constr_sparsity_out, WB_FL1_terminal_constr_work);
    }
    else if (4 == modeidx)
    {
        casadi_interface(arg, res, tconstraint[0].hxx.size(), WB_FL2_terminal_constr, WB_FL2_terminal_constr_sparsity_out, WB_FL2_terminal_constr_work);
    }
}

template <typename T>
void WBConstraint<T>::path_constraint(ModelState<T, xsize_WB, usize_WB, ysize_WB> &mstate,
                                      vector<IneqConstrStruct<T, xsize_WB, usize_WB, ysize_WB>>& pconstraint,
                                      int modeidx)
{
    
    torque_limit(mstate, pconstraint.data(), modeidx);
    joint_limit(mstate, pconstraint.data() + _num_torque_limit, modeidx);

    if (1==modeidx || 3==modeidx)
    {
        GRF_constraint(mstate, pconstraint.data() + _num_torque_limit + _num_joint_limit, modeidx);
    }
}

template <typename T>
void WBConstraint<T>::torque_limit(ModelState<T, xsize_WB, usize_WB, ysize_WB> &mstate,
                                   IneqConstrStruct<T, xsize_WB, usize_WB, ysize_WB> *pc,
                                   int modeidx)
{   
    for (size_t idx = 0; idx < _num_torque_limit; idx++)
    {
        (pc+idx)->g = (C_torque.row(idx) * mstate.u + b_torque[idx]);
        (pc+idx)->gu = C_torque.row(idx).transpose();
    }
}

template <typename T>
void WBConstraint<T>::GRF_constraint(ModelState<T, xsize_WB, usize_WB, ysize_WB> &mstate,
                                     IneqConstrStruct<T, xsize_WB, usize_WB, ysize_WB> *pc,
                                     int modeidx)
{
    if ((2 == modeidx) || (4 == modeidx))
        return;

    if (1 == modeidx)
    {
        C_grf << 0, 0, 0, 1,
                 0, 0, -1, _friccoeff,
                 0, 0, 1, _friccoeff;
    }
    else if (3 == modeidx)
    {
        C_grf << 0, 1, 0, 0,
                -1, _friccoeff, 0, 0,
                1, _friccoeff, 0, 0;
    }

    for (size_t idx = 0; idx < _num_GRF_constraint; idx++)
    {
        (pc+idx)->g = C_grf.row(idx) * mstate.y + b_grf(idx);
        (pc+idx)->gy = C_grf.row(idx).transpose();

    }
}

template <typename T>
void WBConstraint<T>::joint_limit(ModelState<T, xsize_WB, usize_WB, ysize_WB> &mstate,
                                  IneqConstrStruct<T, xsize_WB, usize_WB, ysize_WB> *pc,
                                  int modeidx)
{  
    for (size_t idx = 0; idx < _num_joint_limit; idx++)
    {
        (pc+idx)->g = C_joint.row(idx) * mstate.x.segment(3,usize_WB) + b_joint[idx]; 
        (pc+idx)->gx.segment(3, usize_WB) = C_joint.row(idx).transpose(); // # joint = usize_WB
    }
}

template class FBConstraint<casadi_real>;
template class WBConstraint<casadi_real>;