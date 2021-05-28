#include "CasadiGen.h"
void casadi_interface(vector<casadi_real *> ARG, vector<casadi_real *> RES, int max_sz_res,
                      int f(const casadi_real **, casadi_real **, int_T *, casadi_real *, int),
                      const int_T *f_sparse_out(int_T),
                      int f_work(int_T *, int_T *, int_T *, int_T *))
{
    // function evaluation
    casadi_real *arg[ARG.size()], *res[RES.size()];
    int_T sz_arg, sz_res, n_arg, n_res, sz_iw, sz_w, *iw = nullptr;
    casadi_real *w = nullptr;

    // get the size of each input and output variables
    f_work(&sz_arg, &sz_res, &sz_iw, &sz_w);

    assert(ARG.size()<=sz_arg);
    assert(RES.size()<=sz_res);

    for (size_t idx_arg = 0; idx_arg < ARG.size(); idx_arg++)
    {
        arg[idx_arg] = ARG[idx_arg];
    }

    for (size_t idx_res = 0; idx_res < RES.size(); idx_res++)
    {
        res[idx_res] = new casadi_real[max_sz_res];
    }    
       
    iw = new int_T[sz_iw];
    w = new casadi_real[sz_w];

    // get function output
    f((const casadi_real **)arg, res, iw, w, 1);

    // get sparsity pattern
    const int_T *sppattern;
    const int_T *rowinfo, *colinfo;
    int nnz;

    int nrow, ncol;
    for (size_t idx_res = 0; idx_res < sz_res; idx_res++)
    {
        sppattern = f_sparse_out(idx_res); // get the sparsity pattern (pointer to const array) for the ith output
        nrow = sppattern[0];
        ncol = sppattern[1];                

        colinfo = sppattern + 2;
        rowinfo = colinfo + ncol + 1;
        nnz = colinfo[ncol];

        // get data from res to RES
        int nzidx = 0;
        for (int colidx = 0; colidx < ncol; colidx++)
        {   
            while (nzidx < colinfo[colidx+1])
            {
                RES[idx_res][rowinfo[nzidx]+nrow*colidx] = res[idx_res][nzidx];
                nzidx++;
            }
            
        }
        
    }
    
    // delete dynamic memory
    if(nullptr != iw) delete [] iw;
    if(nullptr !=w) delete [] w;

    for (int idx_res = 0; idx_res < RES.size(); idx_res++)
    {
        if(nullptr != res[idx_res]) delete [] res[idx_res];
    }

}
