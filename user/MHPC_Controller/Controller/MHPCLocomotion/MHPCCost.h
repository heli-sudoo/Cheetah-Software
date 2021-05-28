# ifndef MHPCCOST_H
# define MHPCCOST_H

#include "MHPC_CPPTypes.h"
#include "MHPC_CompoundTypes.h"
#include "CostBase.h"

template <typename T>
class WBCost : public Cost<T, xsize_WB, usize_WB, ysize_WB>
{
private:
    /* data */
public:
    WBCost(T dt);
    void set_weighting_matrices() override;
};

template <typename T>
class FBCost : public Cost<T, xsize_FB, usize_FB, ysize_FB>
{
private:
    /* data */
public:
    FBCost(T dt);
    void set_weighting_matrices() override;
};

#endif //MHPCCOST_H
