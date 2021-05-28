#ifndef COSTBASE_H
#define COSTBASE_H

#include "MHPC_CPPTypes.h"
#include "MHPC_CompoundTypes.h"
#include "PlanarRobot.h"
#include <vector>

template <typename T>
class CostAbstract
{
public:
    virtual void running_cost(ModelState<T, xsize_WB, usize_WB, ysize_WB> &,
                              ModelState<T, xsize_WB, usize_WB, ysize_WB> &,
                              RCostStruct<T, xsize_WB, usize_WB, ysize_WB> &, int) {}

    virtual void running_cost(ModelState<T, xsize_FB, usize_FB, ysize_FB> &,
                              ModelState<T, xsize_FB, usize_FB, ysize_FB> &,
                              RCostStruct<T, xsize_FB, usize_FB, ysize_FB> &, int) {}

    virtual void running_cost_par(ModelState<T, xsize_WB, usize_WB, ysize_WB> &,
                                  ModelState<T, xsize_WB, usize_WB, ysize_WB> &,
                                  RCostStruct<T, xsize_WB, usize_WB, ysize_WB> &, int) {}

    virtual void running_cost_par(ModelState<T, xsize_FB, usize_FB, ysize_FB> &,
                                  ModelState<T, xsize_FB, usize_FB, ysize_FB> &,
                                  RCostStruct<T, xsize_FB, usize_FB, ysize_FB> &, int) {}

    virtual void terminal_cost(ModelState<T, xsize_WB, usize_WB, ysize_WB> &,
                               ModelState<T, xsize_WB, usize_WB, ysize_WB> &,
                               TCostStruct<T, xsize_WB> &, int) {}

    virtual void terminal_cost(ModelState<T, xsize_FB, usize_FB, ysize_FB> &,
                               ModelState<T, xsize_FB, usize_FB, ysize_FB> &,
                               TCostStruct<T, xsize_FB> &, int) {}

    virtual void terminal_cost_par(ModelState<T, xsize_WB, usize_WB, ysize_WB> &,
                                   ModelState<T, xsize_WB, usize_WB, ysize_WB> &,
                                   TCostStruct<T, xsize_WB> &, int) {}

    virtual void terminal_cost_par(ModelState<T, xsize_FB, usize_FB, ysize_FB> &,
                                   ModelState<T, xsize_FB, usize_FB, ysize_FB> &,
                                   TCostStruct<T, xsize_FB> &, int) {}
};

template <typename T, size_t XSIZE, size_t USIZE, size_t YSIZE>
class Cost : public CostAbstract<T>
{
public:
    T _dt;
    MatMN<T, XSIZE, XSIZE> *_Q = nullptr, *_Qf = nullptr;
    MatMN<T, USIZE, USIZE> *_R = nullptr;
    MatMN<T, YSIZE, YSIZE> *_S = nullptr;
    size_t _n_modes;
    static constexpr size_t _xsize = XSIZE, _usize = USIZE, _ysize = YSIZE;

public:
    Cost(T dt, size_t n_modes) : _dt(dt), _n_modes(n_modes) {}

    virtual void set_weighting_matrices(){};

    void running_cost(ModelState<T, XSIZE, USIZE, YSIZE> &,
                      ModelState<T, XSIZE, USIZE, YSIZE> &,
                      RCostStruct<T, XSIZE, USIZE, YSIZE> &, int) override;

    void running_cost_par(ModelState<T, XSIZE, USIZE, YSIZE> &,
                          ModelState<T, XSIZE, USIZE, YSIZE> &,
                          RCostStruct<T, XSIZE, USIZE, YSIZE> &, int) override;

    void terminal_cost(ModelState<T, XSIZE, USIZE, YSIZE> &,
                       ModelState<T, XSIZE, USIZE, YSIZE> &,
                       TCostStruct<T, XSIZE> &, int) override;

    void terminal_cost_par(ModelState<T, XSIZE, USIZE, YSIZE> &,
                           ModelState<T, XSIZE, USIZE, YSIZE> &,
                           TCostStruct<T, XSIZE> &, int) override;

    ~Cost()
    {
        delete[] _Q;
        _Q = nullptr;
        delete[] _R;
        _R = nullptr;
        delete[] _S;
        _S = nullptr;
        delete[] _Qf;
        _Qf = nullptr;
    }
};

#endif // COSTBASE_H