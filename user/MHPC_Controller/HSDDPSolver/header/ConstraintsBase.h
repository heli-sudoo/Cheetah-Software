#ifndef CONSTRAINTSBASE_H
#define CONSTRAINTSBASE_H

#include "MHPC_CPPTypes.h"
#include "MHPC_CompoundTypes.h"
#include "PlanarRobot.h"

#include <vector>

template <typename T>
class Constraint
{
protected:
    RobotBase<T> *_model;

    std::vector<AL_REB_PARAMETER<T>> _params;

    std::vector<size_t> _num_tconstr, _num_pconstr;

public:
    Constraint(){}

    Constraint(RobotBase<T> *model) : _model(model){}

    virtual ~ Constraint() = default;

    void get_AL_REB_PARAMS(AL_REB_PARAMETER<T> &param, int mode) { param = _params[mode - 1]; }

    size_t get_num_pconstraint(int modeidx) // get # pconstraint for mode modeidx
    {
        assert((modeidx <= _num_pconstr.size() && modeidx > 0));
        return _num_pconstr[modeidx - 1];
    }

    size_t get_num_tconstraint(int modeidx) // get # tconstraint for mode modeidx
    {
        assert((modeidx <= _num_tconstr.size() && modeidx > 0));
        return _num_tconstr[modeidx - 1];
    }

    virtual void terminal_constraint(ModelState<T, xsize_WB, usize_WB, ysize_WB> &, vector<TConstrStruct<T, xsize_WB>> &, int) {}

    virtual void terminal_constraint(ModelState<T, xsize_FB, usize_FB, ysize_FB> &, vector<TConstrStruct<T, xsize_FB>> &, int) {}

    virtual void path_constraint(ModelState<T, xsize_WB, usize_WB, ysize_WB> &,  vector<IneqConstrStruct<T, xsize_WB, usize_WB, ysize_WB>> &, int) {}

    virtual void path_constraint(ModelState<T, xsize_FB, usize_FB, ysize_FB> &,  vector<IneqConstrStruct<T, xsize_FB, usize_FB, ysize_FB>> &, int) {}

    virtual void initialize_AL_REB_PARAMS(){};
};

// template class Constraint<casadi_real>;


# endif // CONSTRAINTSBASE_H