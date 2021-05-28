#ifndef CASADIGEN_H
#define CASADIGEN_H

#define int_T long long int

#include "MHPC_CPPTypes.h" // overwrite casadi_real in casadi generated functions
#include "Dyn_FL.h"
#include "Dyn_FL_par.h"
#include "Dyn_FS.h"
#include "Dyn_FS_par.h"
#include "Dyn_BS.h"
#include "Dyn_BS_par.h"
#include "Jacob_F.h"
#include "Jacob_B.h"
#include "Imp_B.h"
#include "Imp_B_par.h"
#include "Imp_F.h"
#include "Imp_F_par.h"
#include "FBDynamics.h"
#include "FBDynamics_par.h"
#include "WB_FL1_terminal_constr.h"
#include "WB_FL2_terminal_constr.h"
#include "Link0Jacobian.h"
#include "Link1Jacobian.h"
#include "Link2Jacobian.h"
#include "Link3Jacobian.h"
#include "Link4Jacobian.h"


/*
  @brief: Get the numerical evaluation of a CasadiGen function and the output sparsity pattern
  @params: 
          arg: casadi_real ptr to an array of pointers whose element points to an input variable
          res: casadi_real ptr to an array of pointers whose element points to an output variable
          max_sz_res: maximum size of output variables
*/
void casadi_interface(vector<casadi_real *> ARG, vector<casadi_real *> RES, int max_sz_res,
                      int f(const casadi_real **, casadi_real **, int_T *, casadi_real *, int),
                      const int_T *f_sparse_out(int_T),
                      int f_work(int_T *, int_T *, int_T *, int_T *));

#endif //CASADIGEN_H