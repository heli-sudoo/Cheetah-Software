#ifndef GAIT
#define GAIT
#include "MHPC_CPPTypes.h"
#include<assert.h>
#include<math.h>
enum class GaitType
{
    STAND,
    BOUND,
    PRONK
};

template<typename T>
class Gait
{
private:
    DVec<int> _gait_mode;
    DVec<T>  _gait_timing;
    std::string _gait_name;

public:
    Gait()
    {
        _gait_mode = VecM<int, 4>(1,2,3,4);
        _gait_name = "BOUND";
        _gait_timing = VecM<T,4>(0.08, 0.1, 0.08, 0.1);
    }

    Gait(GaitType gait)
    {
        switch (gait) 
        {
        case GaitType::BOUND:
            _gait_mode = VecM<int,4>(1,2,3,4);
            _gait_name = "BOUND";
            _gait_timing = VecM<T,4>(0.08, 0.1, 0.08, 0.1);
            break;
        
        default:
            _gait_mode = VecM<int,4>(1,2,3,4);
            _gait_name = "BOUND";
            _gait_timing = VecM<T,4>(0.08, 0.08, 0.08, 0.08);
            break;
        }
    }

    int get_next_mode(int current_mode)
    {   
        int idx = 0;
        for (idx=0; idx < _gait_mode.size(); idx++)
        {
            if(current_mode == _gait_mode[idx]) {return _gait_mode[(idx+1)%_gait_mode.size()];}        
        }                
    } 
    
    DVec<int> get_mode_seq(int current_mode, int num_phases) 
    {
        DVec<int> mode_seq(num_phases);
        assert(num_phases >= 1);
        mode_seq[0] = current_mode;

        for (int pidx(0); pidx < num_phases-1; pidx++)
        {
            mode_seq[pidx+1] = get_next_mode(mode_seq[pidx]);
        } 
        return mode_seq;       
    }

    DVec<T> get_timings(DVec<int> &mode_seq)
    {
        DVec<T> timings(mode_seq.size());
        for (size_t idx = 0; idx < mode_seq.size(); idx++)
        {
            timings[idx] = _gait_timing[mode_seq[idx]-1];
        }

        return timings;        
    }
};


#endif // GAIT