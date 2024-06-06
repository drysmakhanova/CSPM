/* Produced by CVXGEN, 2022-04-22 18:20:13 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void multbymA(double *lhs, double *rhs) {
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.M[0])-rhs[1]*(params.M[18])-rhs[2]*(params.M[36]);
  lhs[1] = -rhs[0]*(params.M[1])-rhs[1]*(params.M[19])-rhs[2]*(params.M[37]);
  lhs[2] = -rhs[0]*(params.M[2])-rhs[1]*(params.M[20])-rhs[2]*(params.M[38]);
  lhs[3] = -rhs[0]*(params.M[3])-rhs[1]*(params.M[21])-rhs[2]*(params.M[39]);
  lhs[4] = -rhs[0]*(params.M[4])-rhs[1]*(params.M[22])-rhs[2]*(params.M[40]);
  lhs[5] = -rhs[0]*(params.M[5])-rhs[1]*(params.M[23])-rhs[2]*(params.M[41]);
  lhs[6] = -rhs[0]*(params.M[6])-rhs[1]*(params.M[24])-rhs[2]*(params.M[42]);
  lhs[7] = -rhs[0]*(params.M[7])-rhs[1]*(params.M[25])-rhs[2]*(params.M[43]);
  lhs[8] = -rhs[0]*(params.M[8])-rhs[1]*(params.M[26])-rhs[2]*(params.M[44]);
  lhs[9] = -rhs[0]*(params.M[9])-rhs[1]*(params.M[27])-rhs[2]*(params.M[45]);
  lhs[10] = -rhs[0]*(params.M[10])-rhs[1]*(params.M[28])-rhs[2]*(params.M[46]);
  lhs[11] = -rhs[0]*(params.M[11])-rhs[1]*(params.M[29])-rhs[2]*(params.M[47]);
  lhs[12] = -rhs[0]*(params.M[12])-rhs[1]*(params.M[30])-rhs[2]*(params.M[48]);
  lhs[13] = -rhs[0]*(params.M[13])-rhs[1]*(params.M[31])-rhs[2]*(params.M[49]);
  lhs[14] = -rhs[0]*(params.M[14])-rhs[1]*(params.M[32])-rhs[2]*(params.M[50]);
  lhs[15] = -rhs[0]*(params.M[15])-rhs[1]*(params.M[33])-rhs[2]*(params.M[51]);
  lhs[16] = -rhs[0]*(params.M[16])-rhs[1]*(params.M[34])-rhs[2]*(params.M[52]);
  lhs[17] = -rhs[0]*(params.M[17])-rhs[1]*(params.M[35])-rhs[2]*(params.M[53]);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.M[0])-rhs[1]*(params.M[1])-rhs[2]*(params.M[2])-rhs[3]*(params.M[3])-rhs[4]*(params.M[4])-rhs[5]*(params.M[5])-rhs[6]*(params.M[6])-rhs[7]*(params.M[7])-rhs[8]*(params.M[8])-rhs[9]*(params.M[9])-rhs[10]*(params.M[10])-rhs[11]*(params.M[11])-rhs[12]*(params.M[12])-rhs[13]*(params.M[13])-rhs[14]*(params.M[14])-rhs[15]*(params.M[15])-rhs[16]*(params.M[16])-rhs[17]*(params.M[17]);
  lhs[1] = -rhs[0]*(params.M[18])-rhs[1]*(params.M[19])-rhs[2]*(params.M[20])-rhs[3]*(params.M[21])-rhs[4]*(params.M[22])-rhs[5]*(params.M[23])-rhs[6]*(params.M[24])-rhs[7]*(params.M[25])-rhs[8]*(params.M[26])-rhs[9]*(params.M[27])-rhs[10]*(params.M[28])-rhs[11]*(params.M[29])-rhs[12]*(params.M[30])-rhs[13]*(params.M[31])-rhs[14]*(params.M[32])-rhs[15]*(params.M[33])-rhs[16]*(params.M[34])-rhs[17]*(params.M[35]);
  lhs[2] = -rhs[0]*(params.M[36])-rhs[1]*(params.M[37])-rhs[2]*(params.M[38])-rhs[3]*(params.M[39])-rhs[4]*(params.M[40])-rhs[5]*(params.M[41])-rhs[6]*(params.M[42])-rhs[7]*(params.M[43])-rhs[8]*(params.M[44])-rhs[9]*(params.M[45])-rhs[10]*(params.M[46])-rhs[11]*(params.M[47])-rhs[12]*(params.M[48])-rhs[13]*(params.M[49])-rhs[14]*(params.M[50])-rhs[15]*(params.M[51])-rhs[16]*(params.M[52])-rhs[17]*(params.M[53]);
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2);
  lhs[1] = rhs[1]*(2);
  lhs[2] = rhs[2]*(2);
}
void fillq(void) {
  work.q[0] = -2*params.ref[0];
  work.q[1] = -2*params.ref[1];
  work.q[2] = -2*params.ref[2];
}
void fillh(void) {
  work.h[0] = params.m[0];
  work.h[1] = params.m[1];
  work.h[2] = params.m[2];
  work.h[3] = params.m[3];
  work.h[4] = params.m[4];
  work.h[5] = params.m[5];
  work.h[6] = params.m[6];
  work.h[7] = params.m[7];
  work.h[8] = params.m[8];
  work.h[9] = params.m[9];
  work.h[10] = params.m[10];
  work.h[11] = params.m[11];
  work.h[12] = params.m[12];
  work.h[13] = params.m[13];
  work.h[14] = params.m[14];
  work.h[15] = params.m[15];
  work.h[16] = params.m[16];
  work.h[17] = params.m[17];
}
void fillb(void) {
}
void pre_ops(void) {
  work.quad_657348546560[0] = params.ref[0]*params.ref[0]+params.ref[1]*params.ref[1]+params.ref[2]*params.ref[2];
}
