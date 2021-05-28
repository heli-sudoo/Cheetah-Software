/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

int Link1Jacobian(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int Link1Jacobian_alloc_mem(void);
int Link1Jacobian_init_mem(int mem);
void Link1Jacobian_free_mem(int mem);
int Link1Jacobian_checkout(void);
void Link1Jacobian_release(int mem);
void Link1Jacobian_incref(void);
void Link1Jacobian_decref(void);
casadi_int Link1Jacobian_n_out(void);
casadi_int Link1Jacobian_n_in(void);
casadi_real Link1Jacobian_default_in(casadi_int i);
const char* Link1Jacobian_name_in(casadi_int i);
const char* Link1Jacobian_name_out(casadi_int i);
const casadi_int* Link1Jacobian_sparsity_in(casadi_int i);
const casadi_int* Link1Jacobian_sparsity_out(casadi_int i);
int Link1Jacobian_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#ifdef __cplusplus
} /* extern "C" */
#endif
