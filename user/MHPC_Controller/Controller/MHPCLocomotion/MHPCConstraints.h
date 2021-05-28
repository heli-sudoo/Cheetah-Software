# ifndef MHPC_CONSTRAINTS_H
# define MHPC_CONSTRAINTS_H

#include "MHPC_CPPTypes.h"
#include "MHPC_CompoundTypes.h"
#include "ConstraintsBase.h"
#include "PlanarRobot.h"

template <typename T>
class FBConstraint : public Constraint<T>
{
public:
    FBConstraint(RobotBase<T> *model);

    void terminal_constraint(ModelState<T, xsize_FB, usize_FB, ysize_FB> &, vector<TConstrStruct<T, xsize_FB>> &, int) override {}

    void path_constraint(ModelState<T, xsize_FB, usize_FB, ysize_FB> &, vector<IneqConstrStruct<T, xsize_FB, usize_FB, ysize_FB>> &, int) override {}
};

template <typename T>
class WBConstraint : public Constraint<T>
{
public:
    size_t _num_torque_limit;
    size_t _num_joint_limit;
    size_t _num_GRF_constraint;
    DMat<T> C_torque, C_joint, C_grf;
    DVec<T> b_torque, b_joint, b_grf;
    T  _friccoeff = 0.6; // static friction coefficient

public:
    WBConstraint(RobotBase<T> *model);

    void terminal_constraint(ModelState<T, xsize_WB, usize_WB, ysize_WB> &, vector<TConstrStruct<T, xsize_WB>> &, int) override;

    void path_constraint(ModelState<T, xsize_WB, usize_WB, ysize_WB> &, vector<IneqConstrStruct<T, xsize_WB, usize_WB, ysize_WB>> &, int) override;

    void torque_limit(ModelState<T, xsize_WB, usize_WB, ysize_WB> &, IneqConstrStruct<T, xsize_WB, usize_WB, ysize_WB>*, int); // torque limit constraint

    void GRF_constraint(ModelState<T, xsize_WB, usize_WB, ysize_WB> &, IneqConstrStruct<T, xsize_WB, usize_WB, ysize_WB>*, int); // non-negative normal GRF and friction constraint

    void joint_limit(ModelState<T, xsize_WB, usize_WB, ysize_WB> &, IneqConstrStruct<T, xsize_WB, usize_WB, ysize_WB>*, int); // joint limit constraint

    void initialize_AL_REB_PARAMS();
};

# endif // MHPC_CONSTRAINTS_H