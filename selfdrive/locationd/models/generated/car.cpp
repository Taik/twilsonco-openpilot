#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_1078694211982077466) {
   out_1078694211982077466[0] = delta_x[0] + nom_x[0];
   out_1078694211982077466[1] = delta_x[1] + nom_x[1];
   out_1078694211982077466[2] = delta_x[2] + nom_x[2];
   out_1078694211982077466[3] = delta_x[3] + nom_x[3];
   out_1078694211982077466[4] = delta_x[4] + nom_x[4];
   out_1078694211982077466[5] = delta_x[5] + nom_x[5];
   out_1078694211982077466[6] = delta_x[6] + nom_x[6];
   out_1078694211982077466[7] = delta_x[7] + nom_x[7];
   out_1078694211982077466[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_9039723021082541259) {
   out_9039723021082541259[0] = -nom_x[0] + true_x[0];
   out_9039723021082541259[1] = -nom_x[1] + true_x[1];
   out_9039723021082541259[2] = -nom_x[2] + true_x[2];
   out_9039723021082541259[3] = -nom_x[3] + true_x[3];
   out_9039723021082541259[4] = -nom_x[4] + true_x[4];
   out_9039723021082541259[5] = -nom_x[5] + true_x[5];
   out_9039723021082541259[6] = -nom_x[6] + true_x[6];
   out_9039723021082541259[7] = -nom_x[7] + true_x[7];
   out_9039723021082541259[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3566115125440726837) {
   out_3566115125440726837[0] = 1.0;
   out_3566115125440726837[1] = 0;
   out_3566115125440726837[2] = 0;
   out_3566115125440726837[3] = 0;
   out_3566115125440726837[4] = 0;
   out_3566115125440726837[5] = 0;
   out_3566115125440726837[6] = 0;
   out_3566115125440726837[7] = 0;
   out_3566115125440726837[8] = 0;
   out_3566115125440726837[9] = 0;
   out_3566115125440726837[10] = 1.0;
   out_3566115125440726837[11] = 0;
   out_3566115125440726837[12] = 0;
   out_3566115125440726837[13] = 0;
   out_3566115125440726837[14] = 0;
   out_3566115125440726837[15] = 0;
   out_3566115125440726837[16] = 0;
   out_3566115125440726837[17] = 0;
   out_3566115125440726837[18] = 0;
   out_3566115125440726837[19] = 0;
   out_3566115125440726837[20] = 1.0;
   out_3566115125440726837[21] = 0;
   out_3566115125440726837[22] = 0;
   out_3566115125440726837[23] = 0;
   out_3566115125440726837[24] = 0;
   out_3566115125440726837[25] = 0;
   out_3566115125440726837[26] = 0;
   out_3566115125440726837[27] = 0;
   out_3566115125440726837[28] = 0;
   out_3566115125440726837[29] = 0;
   out_3566115125440726837[30] = 1.0;
   out_3566115125440726837[31] = 0;
   out_3566115125440726837[32] = 0;
   out_3566115125440726837[33] = 0;
   out_3566115125440726837[34] = 0;
   out_3566115125440726837[35] = 0;
   out_3566115125440726837[36] = 0;
   out_3566115125440726837[37] = 0;
   out_3566115125440726837[38] = 0;
   out_3566115125440726837[39] = 0;
   out_3566115125440726837[40] = 1.0;
   out_3566115125440726837[41] = 0;
   out_3566115125440726837[42] = 0;
   out_3566115125440726837[43] = 0;
   out_3566115125440726837[44] = 0;
   out_3566115125440726837[45] = 0;
   out_3566115125440726837[46] = 0;
   out_3566115125440726837[47] = 0;
   out_3566115125440726837[48] = 0;
   out_3566115125440726837[49] = 0;
   out_3566115125440726837[50] = 1.0;
   out_3566115125440726837[51] = 0;
   out_3566115125440726837[52] = 0;
   out_3566115125440726837[53] = 0;
   out_3566115125440726837[54] = 0;
   out_3566115125440726837[55] = 0;
   out_3566115125440726837[56] = 0;
   out_3566115125440726837[57] = 0;
   out_3566115125440726837[58] = 0;
   out_3566115125440726837[59] = 0;
   out_3566115125440726837[60] = 1.0;
   out_3566115125440726837[61] = 0;
   out_3566115125440726837[62] = 0;
   out_3566115125440726837[63] = 0;
   out_3566115125440726837[64] = 0;
   out_3566115125440726837[65] = 0;
   out_3566115125440726837[66] = 0;
   out_3566115125440726837[67] = 0;
   out_3566115125440726837[68] = 0;
   out_3566115125440726837[69] = 0;
   out_3566115125440726837[70] = 1.0;
   out_3566115125440726837[71] = 0;
   out_3566115125440726837[72] = 0;
   out_3566115125440726837[73] = 0;
   out_3566115125440726837[74] = 0;
   out_3566115125440726837[75] = 0;
   out_3566115125440726837[76] = 0;
   out_3566115125440726837[77] = 0;
   out_3566115125440726837[78] = 0;
   out_3566115125440726837[79] = 0;
   out_3566115125440726837[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5563845995787045166) {
   out_5563845995787045166[0] = state[0];
   out_5563845995787045166[1] = state[1];
   out_5563845995787045166[2] = state[2];
   out_5563845995787045166[3] = state[3];
   out_5563845995787045166[4] = state[4];
   out_5563845995787045166[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5563845995787045166[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5563845995787045166[7] = state[7];
   out_5563845995787045166[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5883604685601594109) {
   out_5883604685601594109[0] = 1;
   out_5883604685601594109[1] = 0;
   out_5883604685601594109[2] = 0;
   out_5883604685601594109[3] = 0;
   out_5883604685601594109[4] = 0;
   out_5883604685601594109[5] = 0;
   out_5883604685601594109[6] = 0;
   out_5883604685601594109[7] = 0;
   out_5883604685601594109[8] = 0;
   out_5883604685601594109[9] = 0;
   out_5883604685601594109[10] = 1;
   out_5883604685601594109[11] = 0;
   out_5883604685601594109[12] = 0;
   out_5883604685601594109[13] = 0;
   out_5883604685601594109[14] = 0;
   out_5883604685601594109[15] = 0;
   out_5883604685601594109[16] = 0;
   out_5883604685601594109[17] = 0;
   out_5883604685601594109[18] = 0;
   out_5883604685601594109[19] = 0;
   out_5883604685601594109[20] = 1;
   out_5883604685601594109[21] = 0;
   out_5883604685601594109[22] = 0;
   out_5883604685601594109[23] = 0;
   out_5883604685601594109[24] = 0;
   out_5883604685601594109[25] = 0;
   out_5883604685601594109[26] = 0;
   out_5883604685601594109[27] = 0;
   out_5883604685601594109[28] = 0;
   out_5883604685601594109[29] = 0;
   out_5883604685601594109[30] = 1;
   out_5883604685601594109[31] = 0;
   out_5883604685601594109[32] = 0;
   out_5883604685601594109[33] = 0;
   out_5883604685601594109[34] = 0;
   out_5883604685601594109[35] = 0;
   out_5883604685601594109[36] = 0;
   out_5883604685601594109[37] = 0;
   out_5883604685601594109[38] = 0;
   out_5883604685601594109[39] = 0;
   out_5883604685601594109[40] = 1;
   out_5883604685601594109[41] = 0;
   out_5883604685601594109[42] = 0;
   out_5883604685601594109[43] = 0;
   out_5883604685601594109[44] = 0;
   out_5883604685601594109[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5883604685601594109[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5883604685601594109[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5883604685601594109[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5883604685601594109[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5883604685601594109[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5883604685601594109[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5883604685601594109[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5883604685601594109[53] = -9.8000000000000007*dt;
   out_5883604685601594109[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5883604685601594109[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5883604685601594109[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5883604685601594109[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5883604685601594109[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5883604685601594109[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5883604685601594109[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5883604685601594109[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5883604685601594109[62] = 0;
   out_5883604685601594109[63] = 0;
   out_5883604685601594109[64] = 0;
   out_5883604685601594109[65] = 0;
   out_5883604685601594109[66] = 0;
   out_5883604685601594109[67] = 0;
   out_5883604685601594109[68] = 0;
   out_5883604685601594109[69] = 0;
   out_5883604685601594109[70] = 1;
   out_5883604685601594109[71] = 0;
   out_5883604685601594109[72] = 0;
   out_5883604685601594109[73] = 0;
   out_5883604685601594109[74] = 0;
   out_5883604685601594109[75] = 0;
   out_5883604685601594109[76] = 0;
   out_5883604685601594109[77] = 0;
   out_5883604685601594109[78] = 0;
   out_5883604685601594109[79] = 0;
   out_5883604685601594109[80] = 1;
}
void h_25(double *state, double *unused, double *out_5225710209594518100) {
   out_5225710209594518100[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2743426639728721431) {
   out_2743426639728721431[0] = 0;
   out_2743426639728721431[1] = 0;
   out_2743426639728721431[2] = 0;
   out_2743426639728721431[3] = 0;
   out_2743426639728721431[4] = 0;
   out_2743426639728721431[5] = 0;
   out_2743426639728721431[6] = 1;
   out_2743426639728721431[7] = 0;
   out_2743426639728721431[8] = 0;
}
void h_24(double *state, double *unused, double *out_3821121116768031143) {
   out_3821121116768031143[0] = state[4];
   out_3821121116768031143[1] = state[5];
}
void H_24(double *state, double *unused, double *out_6475252247911634960) {
   out_6475252247911634960[0] = 0;
   out_6475252247911634960[1] = 0;
   out_6475252247911634960[2] = 0;
   out_6475252247911634960[3] = 0;
   out_6475252247911634960[4] = 1;
   out_6475252247911634960[5] = 0;
   out_6475252247911634960[6] = 0;
   out_6475252247911634960[7] = 0;
   out_6475252247911634960[8] = 0;
   out_6475252247911634960[9] = 0;
   out_6475252247911634960[10] = 0;
   out_6475252247911634960[11] = 0;
   out_6475252247911634960[12] = 0;
   out_6475252247911634960[13] = 0;
   out_6475252247911634960[14] = 1;
   out_6475252247911634960[15] = 0;
   out_6475252247911634960[16] = 0;
   out_6475252247911634960[17] = 0;
}
void h_30(double *state, double *unused, double *out_5500904271879023989) {
   out_5500904271879023989[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1784269690398886767) {
   out_1784269690398886767[0] = 0;
   out_1784269690398886767[1] = 0;
   out_1784269690398886767[2] = 0;
   out_1784269690398886767[3] = 0;
   out_1784269690398886767[4] = 1;
   out_1784269690398886767[5] = 0;
   out_1784269690398886767[6] = 0;
   out_1784269690398886767[7] = 0;
   out_1784269690398886767[8] = 0;
}
void h_26(double *state, double *unused, double *out_6444414124780686424) {
   out_6444414124780686424[0] = state[7];
}
void H_26(double *state, double *unused, double *out_998076679145334793) {
   out_998076679145334793[0] = 0;
   out_998076679145334793[1] = 0;
   out_998076679145334793[2] = 0;
   out_998076679145334793[3] = 0;
   out_998076679145334793[4] = 0;
   out_998076679145334793[5] = 0;
   out_998076679145334793[6] = 0;
   out_998076679145334793[7] = 1;
   out_998076679145334793[8] = 0;
}
void h_27(double *state, double *unused, double *out_1777052494198547679) {
   out_1777052494198547679[0] = state[3];
}
void H_27(double *state, double *unused, double *out_439324380785056450) {
   out_439324380785056450[0] = 0;
   out_439324380785056450[1] = 0;
   out_439324380785056450[2] = 0;
   out_439324380785056450[3] = 1;
   out_439324380785056450[4] = 0;
   out_439324380785056450[5] = 0;
   out_439324380785056450[6] = 0;
   out_439324380785056450[7] = 0;
   out_439324380785056450[8] = 0;
}
void h_29(double *state, double *unused, double *out_7944166534169681982) {
   out_7944166534169681982[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1274038346084494583) {
   out_1274038346084494583[0] = 0;
   out_1274038346084494583[1] = 1;
   out_1274038346084494583[2] = 0;
   out_1274038346084494583[3] = 0;
   out_1274038346084494583[4] = 0;
   out_1274038346084494583[5] = 0;
   out_1274038346084494583[6] = 0;
   out_1274038346084494583[7] = 0;
   out_1274038346084494583[8] = 0;
}
void h_28(double *state, double *unused, double *out_2863779142555012006) {
   out_2863779142555012006[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6356437363154025157) {
   out_6356437363154025157[0] = 1;
   out_6356437363154025157[1] = 0;
   out_6356437363154025157[2] = 0;
   out_6356437363154025157[3] = 0;
   out_6356437363154025157[4] = 0;
   out_6356437363154025157[5] = 0;
   out_6356437363154025157[6] = 0;
   out_6356437363154025157[7] = 0;
   out_6356437363154025157[8] = 0;
}
void h_31(double *state, double *unused, double *out_5328094369230510157) {
   out_5328094369230510157[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1624284781378686269) {
   out_1624284781378686269[0] = 0;
   out_1624284781378686269[1] = 0;
   out_1624284781378686269[2] = 0;
   out_1624284781378686269[3] = 0;
   out_1624284781378686269[4] = 0;
   out_1624284781378686269[5] = 0;
   out_1624284781378686269[6] = 0;
   out_1624284781378686269[7] = 0;
   out_1624284781378686269[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_1078694211982077466) {
  err_fun(nom_x, delta_x, out_1078694211982077466);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9039723021082541259) {
  inv_err_fun(nom_x, true_x, out_9039723021082541259);
}
void car_H_mod_fun(double *state, double *out_3566115125440726837) {
  H_mod_fun(state, out_3566115125440726837);
}
void car_f_fun(double *state, double dt, double *out_5563845995787045166) {
  f_fun(state,  dt, out_5563845995787045166);
}
void car_F_fun(double *state, double dt, double *out_5883604685601594109) {
  F_fun(state,  dt, out_5883604685601594109);
}
void car_h_25(double *state, double *unused, double *out_5225710209594518100) {
  h_25(state, unused, out_5225710209594518100);
}
void car_H_25(double *state, double *unused, double *out_2743426639728721431) {
  H_25(state, unused, out_2743426639728721431);
}
void car_h_24(double *state, double *unused, double *out_3821121116768031143) {
  h_24(state, unused, out_3821121116768031143);
}
void car_H_24(double *state, double *unused, double *out_6475252247911634960) {
  H_24(state, unused, out_6475252247911634960);
}
void car_h_30(double *state, double *unused, double *out_5500904271879023989) {
  h_30(state, unused, out_5500904271879023989);
}
void car_H_30(double *state, double *unused, double *out_1784269690398886767) {
  H_30(state, unused, out_1784269690398886767);
}
void car_h_26(double *state, double *unused, double *out_6444414124780686424) {
  h_26(state, unused, out_6444414124780686424);
}
void car_H_26(double *state, double *unused, double *out_998076679145334793) {
  H_26(state, unused, out_998076679145334793);
}
void car_h_27(double *state, double *unused, double *out_1777052494198547679) {
  h_27(state, unused, out_1777052494198547679);
}
void car_H_27(double *state, double *unused, double *out_439324380785056450) {
  H_27(state, unused, out_439324380785056450);
}
void car_h_29(double *state, double *unused, double *out_7944166534169681982) {
  h_29(state, unused, out_7944166534169681982);
}
void car_H_29(double *state, double *unused, double *out_1274038346084494583) {
  H_29(state, unused, out_1274038346084494583);
}
void car_h_28(double *state, double *unused, double *out_2863779142555012006) {
  h_28(state, unused, out_2863779142555012006);
}
void car_H_28(double *state, double *unused, double *out_6356437363154025157) {
  H_28(state, unused, out_6356437363154025157);
}
void car_h_31(double *state, double *unused, double *out_5328094369230510157) {
  h_31(state, unused, out_5328094369230510157);
}
void car_H_31(double *state, double *unused, double *out_1624284781378686269) {
  H_31(state, unused, out_1624284781378686269);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
