/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) WB_FL2_terminal_constr_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real float
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s1[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s2[21] = {1, 14, 0, 0, 1, 2, 2, 2, 3, 4, 4, 4, 4, 4, 4, 4, 4, 0, 0, 0, 0};
static const casadi_int casadi_s3[26] = {14, 14, 0, 0, 0, 3, 3, 3, 6, 9, 9, 9, 9, 9, 9, 9, 9, 2, 5, 6, 2, 5, 6, 2, 5, 6};

/* WB_FL2_terminal_constr:(i0[14])->(o0,o1[1x14,4nz],o2[14x14,9nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a3, a4, a5, a6, a7, a8, a9;
  a0=-1.9500000000000001e-001;
  a1=arg[0]? arg[0][2] : 0;
  a2=cos(a1);
  a3=arg[0]? arg[0][5] : 0;
  a4=cos(a3);
  a5=(a2*a4);
  a6=sin(a1);
  a7=sin(a3);
  a8=(a6*a7);
  a5=(a5-a8);
  a8=arg[0]? arg[0][6] : 0;
  a9=cos(a8);
  a10=(a5*a9);
  a11=cos(a3);
  a12=(a6*a11);
  a13=sin(a3);
  a14=(a2*a13);
  a12=(a12+a14);
  a14=sin(a8);
  a15=(a12*a14);
  a10=(a10-a15);
  a10=(a0*a10);
  a15=-2.0899999999999999e-001;
  a16=(a15*a5);
  a17=arg[0]? arg[0][1] : 0;
  a18=-1.9000000000000000e-001;
  a18=(a18*a6);
  a17=(a17-a18);
  a16=(a16+a17);
  a10=(a10+a16);
  a16=-4.0400000000000003e-001;
  a10=(a10-a16);
  if (res[0]!=0) res[0][0]=a10;
  a10=1.;
  if (res[1]!=0) res[1][0]=a10;
  a10=cos(a1);
  a16=1.9000000000000000e-001;
  a17=1.9500000000000001e-001;
  a18=(a17*a14);
  a19=(a11*a18);
  a19=(a16+a19);
  a20=(a0*a9);
  a20=(a15+a20);
  a21=(a7*a20);
  a19=(a19-a21);
  a10=(a10*a19);
  a19=sin(a1);
  a21=(a13*a18);
  a22=(a4*a20);
  a21=(a21+a22);
  a19=(a19*a21);
  a10=(a10-a19);
  if (res[1]!=0) res[1][1]=a10;
  a10=cos(a3);
  a19=(a2*a18);
  a10=(a10*a19);
  a19=sin(a3);
  a18=(a6*a18);
  a19=(a19*a18);
  a10=(a10-a19);
  a19=cos(a3);
  a18=(a6*a20);
  a19=(a19*a18);
  a10=(a10-a19);
  a19=sin(a3);
  a20=(a2*a20);
  a19=(a19*a20);
  a10=(a10-a19);
  if (res[1]!=0) res[1][2]=a10;
  a10=cos(a8);
  a19=(a17*a12);
  a10=(a10*a19);
  a19=sin(a8);
  a20=(a0*a5);
  a19=(a19*a20);
  a10=(a10-a19);
  if (res[1]!=0) res[1][3]=a10;
  a14=(a17*a14);
  a10=(a11*a14);
  a16=(a16+a10);
  a9=(a0*a9);
  a15=(a15+a9);
  a9=(a7*a15);
  a16=(a16-a9);
  a9=sin(a1);
  a16=(a16*a9);
  a9=(a13*a14);
  a10=(a4*a15);
  a9=(a9+a10);
  a10=cos(a1);
  a9=(a9*a10);
  a16=(a16+a9);
  a16=(-a16);
  if (res[2]!=0) res[2][0]=a16;
  a16=cos(a1);
  a9=sin(a3);
  a9=(a14*a9);
  a10=cos(a3);
  a10=(a15*a10);
  a9=(a9+a10);
  a9=(a16*a9);
  a1=sin(a1);
  a10=cos(a3);
  a10=(a14*a10);
  a19=sin(a3);
  a19=(a15*a19);
  a10=(a10-a19);
  a10=(a1*a10);
  a9=(a9+a10);
  a9=(-a9);
  if (res[2]!=0) res[2][1]=a9;
  a10=cos(a8);
  a10=(a17*a10);
  a11=(a11*a10);
  a19=sin(a8);
  a19=(a0*a19);
  a7=(a7*a19);
  a11=(a11+a7);
  a16=(a16*a11);
  a13=(a13*a10);
  a4=(a4*a19);
  a13=(a13-a4);
  a1=(a1*a13);
  a16=(a16-a1);
  if (res[2]!=0) res[2][2]=a16;
  if (res[2]!=0) res[2][3]=a9;
  a9=(a6*a15);
  a1=sin(a3);
  a9=(a9*a1);
  a1=(a2*a14);
  a13=sin(a3);
  a1=(a1*a13);
  a14=(a6*a14);
  a13=cos(a3);
  a14=(a14*a13);
  a1=(a1+a14);
  a9=(a9-a1);
  a15=(a2*a15);
  a1=cos(a3);
  a15=(a15*a1);
  a9=(a9-a15);
  if (res[2]!=0) res[2][4]=a9;
  a9=cos(a3);
  a15=(a2*a10);
  a9=(a9*a15);
  a15=sin(a3);
  a10=(a6*a10);
  a15=(a15*a10);
  a9=(a9-a15);
  a15=cos(a3);
  a6=(a6*a19);
  a15=(a15*a6);
  a9=(a9+a15);
  a3=sin(a3);
  a2=(a2*a19);
  a3=(a3*a2);
  a9=(a9+a3);
  if (res[2]!=0) res[2][5]=a9;
  if (res[2]!=0) res[2][6]=a16;
  if (res[2]!=0) res[2][7]=a9;
  a17=(a17*a12);
  a12=sin(a8);
  a17=(a17*a12);
  a0=(a0*a5);
  a8=cos(a8);
  a0=(a0*a8);
  a17=(a17+a0);
  a17=(-a17);
  if (res[2]!=0) res[2][8]=a17;
  return 0;
}

CASADI_SYMBOL_EXPORT int WB_FL2_terminal_constr(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int WB_FL2_terminal_constr_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int WB_FL2_terminal_constr_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void WB_FL2_terminal_constr_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int WB_FL2_terminal_constr_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void WB_FL2_terminal_constr_release(int mem) {
}

CASADI_SYMBOL_EXPORT void WB_FL2_terminal_constr_incref(void) {
}

CASADI_SYMBOL_EXPORT void WB_FL2_terminal_constr_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int WB_FL2_terminal_constr_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int WB_FL2_terminal_constr_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real WB_FL2_terminal_constr_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* WB_FL2_terminal_constr_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* WB_FL2_terminal_constr_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* WB_FL2_terminal_constr_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* WB_FL2_terminal_constr_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    case 1: return casadi_s2;
    case 2: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int WB_FL2_terminal_constr_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
