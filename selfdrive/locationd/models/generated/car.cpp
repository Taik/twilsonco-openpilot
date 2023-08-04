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
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2939202982062152065) {
   out_2939202982062152065[0] = delta_x[0] + nom_x[0];
   out_2939202982062152065[1] = delta_x[1] + nom_x[1];
   out_2939202982062152065[2] = delta_x[2] + nom_x[2];
   out_2939202982062152065[3] = delta_x[3] + nom_x[3];
   out_2939202982062152065[4] = delta_x[4] + nom_x[4];
   out_2939202982062152065[5] = delta_x[5] + nom_x[5];
   out_2939202982062152065[6] = delta_x[6] + nom_x[6];
   out_2939202982062152065[7] = delta_x[7] + nom_x[7];
   out_2939202982062152065[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1376135891411693707) {
   out_1376135891411693707[0] = -nom_x[0] + true_x[0];
   out_1376135891411693707[1] = -nom_x[1] + true_x[1];
   out_1376135891411693707[2] = -nom_x[2] + true_x[2];
   out_1376135891411693707[3] = -nom_x[3] + true_x[3];
   out_1376135891411693707[4] = -nom_x[4] + true_x[4];
   out_1376135891411693707[5] = -nom_x[5] + true_x[5];
   out_1376135891411693707[6] = -nom_x[6] + true_x[6];
   out_1376135891411693707[7] = -nom_x[7] + true_x[7];
   out_1376135891411693707[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_549565208155142469) {
   out_549565208155142469[0] = 1.0;
   out_549565208155142469[1] = 0;
   out_549565208155142469[2] = 0;
   out_549565208155142469[3] = 0;
   out_549565208155142469[4] = 0;
   out_549565208155142469[5] = 0;
   out_549565208155142469[6] = 0;
   out_549565208155142469[7] = 0;
   out_549565208155142469[8] = 0;
   out_549565208155142469[9] = 0;
   out_549565208155142469[10] = 1.0;
   out_549565208155142469[11] = 0;
   out_549565208155142469[12] = 0;
   out_549565208155142469[13] = 0;
   out_549565208155142469[14] = 0;
   out_549565208155142469[15] = 0;
   out_549565208155142469[16] = 0;
   out_549565208155142469[17] = 0;
   out_549565208155142469[18] = 0;
   out_549565208155142469[19] = 0;
   out_549565208155142469[20] = 1.0;
   out_549565208155142469[21] = 0;
   out_549565208155142469[22] = 0;
   out_549565208155142469[23] = 0;
   out_549565208155142469[24] = 0;
   out_549565208155142469[25] = 0;
   out_549565208155142469[26] = 0;
   out_549565208155142469[27] = 0;
   out_549565208155142469[28] = 0;
   out_549565208155142469[29] = 0;
   out_549565208155142469[30] = 1.0;
   out_549565208155142469[31] = 0;
   out_549565208155142469[32] = 0;
   out_549565208155142469[33] = 0;
   out_549565208155142469[34] = 0;
   out_549565208155142469[35] = 0;
   out_549565208155142469[36] = 0;
   out_549565208155142469[37] = 0;
   out_549565208155142469[38] = 0;
   out_549565208155142469[39] = 0;
   out_549565208155142469[40] = 1.0;
   out_549565208155142469[41] = 0;
   out_549565208155142469[42] = 0;
   out_549565208155142469[43] = 0;
   out_549565208155142469[44] = 0;
   out_549565208155142469[45] = 0;
   out_549565208155142469[46] = 0;
   out_549565208155142469[47] = 0;
   out_549565208155142469[48] = 0;
   out_549565208155142469[49] = 0;
   out_549565208155142469[50] = 1.0;
   out_549565208155142469[51] = 0;
   out_549565208155142469[52] = 0;
   out_549565208155142469[53] = 0;
   out_549565208155142469[54] = 0;
   out_549565208155142469[55] = 0;
   out_549565208155142469[56] = 0;
   out_549565208155142469[57] = 0;
   out_549565208155142469[58] = 0;
   out_549565208155142469[59] = 0;
   out_549565208155142469[60] = 1.0;
   out_549565208155142469[61] = 0;
   out_549565208155142469[62] = 0;
   out_549565208155142469[63] = 0;
   out_549565208155142469[64] = 0;
   out_549565208155142469[65] = 0;
   out_549565208155142469[66] = 0;
   out_549565208155142469[67] = 0;
   out_549565208155142469[68] = 0;
   out_549565208155142469[69] = 0;
   out_549565208155142469[70] = 1.0;
   out_549565208155142469[71] = 0;
   out_549565208155142469[72] = 0;
   out_549565208155142469[73] = 0;
   out_549565208155142469[74] = 0;
   out_549565208155142469[75] = 0;
   out_549565208155142469[76] = 0;
   out_549565208155142469[77] = 0;
   out_549565208155142469[78] = 0;
   out_549565208155142469[79] = 0;
   out_549565208155142469[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_746317309293841027) {
   out_746317309293841027[0] = state[0];
   out_746317309293841027[1] = state[1];
   out_746317309293841027[2] = state[2];
   out_746317309293841027[3] = state[3];
   out_746317309293841027[4] = state[4];
   out_746317309293841027[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_746317309293841027[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_746317309293841027[7] = state[7];
   out_746317309293841027[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7394503540686598109) {
   out_7394503540686598109[0] = 1;
   out_7394503540686598109[1] = 0;
   out_7394503540686598109[2] = 0;
   out_7394503540686598109[3] = 0;
   out_7394503540686598109[4] = 0;
   out_7394503540686598109[5] = 0;
   out_7394503540686598109[6] = 0;
   out_7394503540686598109[7] = 0;
   out_7394503540686598109[8] = 0;
   out_7394503540686598109[9] = 0;
   out_7394503540686598109[10] = 1;
   out_7394503540686598109[11] = 0;
   out_7394503540686598109[12] = 0;
   out_7394503540686598109[13] = 0;
   out_7394503540686598109[14] = 0;
   out_7394503540686598109[15] = 0;
   out_7394503540686598109[16] = 0;
   out_7394503540686598109[17] = 0;
   out_7394503540686598109[18] = 0;
   out_7394503540686598109[19] = 0;
   out_7394503540686598109[20] = 1;
   out_7394503540686598109[21] = 0;
   out_7394503540686598109[22] = 0;
   out_7394503540686598109[23] = 0;
   out_7394503540686598109[24] = 0;
   out_7394503540686598109[25] = 0;
   out_7394503540686598109[26] = 0;
   out_7394503540686598109[27] = 0;
   out_7394503540686598109[28] = 0;
   out_7394503540686598109[29] = 0;
   out_7394503540686598109[30] = 1;
   out_7394503540686598109[31] = 0;
   out_7394503540686598109[32] = 0;
   out_7394503540686598109[33] = 0;
   out_7394503540686598109[34] = 0;
   out_7394503540686598109[35] = 0;
   out_7394503540686598109[36] = 0;
   out_7394503540686598109[37] = 0;
   out_7394503540686598109[38] = 0;
   out_7394503540686598109[39] = 0;
   out_7394503540686598109[40] = 1;
   out_7394503540686598109[41] = 0;
   out_7394503540686598109[42] = 0;
   out_7394503540686598109[43] = 0;
   out_7394503540686598109[44] = 0;
   out_7394503540686598109[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7394503540686598109[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7394503540686598109[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7394503540686598109[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7394503540686598109[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7394503540686598109[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7394503540686598109[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7394503540686598109[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7394503540686598109[53] = -9.8000000000000007*dt;
   out_7394503540686598109[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7394503540686598109[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7394503540686598109[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7394503540686598109[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7394503540686598109[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7394503540686598109[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7394503540686598109[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7394503540686598109[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7394503540686598109[62] = 0;
   out_7394503540686598109[63] = 0;
   out_7394503540686598109[64] = 0;
   out_7394503540686598109[65] = 0;
   out_7394503540686598109[66] = 0;
   out_7394503540686598109[67] = 0;
   out_7394503540686598109[68] = 0;
   out_7394503540686598109[69] = 0;
   out_7394503540686598109[70] = 1;
   out_7394503540686598109[71] = 0;
   out_7394503540686598109[72] = 0;
   out_7394503540686598109[73] = 0;
   out_7394503540686598109[74] = 0;
   out_7394503540686598109[75] = 0;
   out_7394503540686598109[76] = 0;
   out_7394503540686598109[77] = 0;
   out_7394503540686598109[78] = 0;
   out_7394503540686598109[79] = 0;
   out_7394503540686598109[80] = 1;
}
void h_25(double *state, double *unused, double *out_2154740370469115095) {
   out_2154740370469115095[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5133228550106326038) {
   out_5133228550106326038[0] = 0;
   out_5133228550106326038[1] = 0;
   out_5133228550106326038[2] = 0;
   out_5133228550106326038[3] = 0;
   out_5133228550106326038[4] = 0;
   out_5133228550106326038[5] = 0;
   out_5133228550106326038[6] = 1;
   out_5133228550106326038[7] = 0;
   out_5133228550106326038[8] = 0;
}
void h_24(double *state, double *unused, double *out_753491134740954591) {
   out_753491134740954591[0] = state[4];
   out_753491134740954591[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5895123889926298685) {
   out_5895123889926298685[0] = 0;
   out_5895123889926298685[1] = 0;
   out_5895123889926298685[2] = 0;
   out_5895123889926298685[3] = 0;
   out_5895123889926298685[4] = 1;
   out_5895123889926298685[5] = 0;
   out_5895123889926298685[6] = 0;
   out_5895123889926298685[7] = 0;
   out_5895123889926298685[8] = 0;
   out_5895123889926298685[9] = 0;
   out_5895123889926298685[10] = 0;
   out_5895123889926298685[11] = 0;
   out_5895123889926298685[12] = 0;
   out_5895123889926298685[13] = 0;
   out_5895123889926298685[14] = 1;
   out_5895123889926298685[15] = 0;
   out_5895123889926298685[16] = 0;
   out_5895123889926298685[17] = 0;
}
void h_30(double *state, double *unused, double *out_1879546308184609206) {
   out_1879546308184609206[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6396825182111608823) {
   out_6396825182111608823[0] = 0;
   out_6396825182111608823[1] = 0;
   out_6396825182111608823[2] = 0;
   out_6396825182111608823[3] = 0;
   out_6396825182111608823[4] = 1;
   out_6396825182111608823[5] = 0;
   out_6396825182111608823[6] = 0;
   out_6396825182111608823[7] = 0;
   out_6396825182111608823[8] = 0;
}
void h_26(double *state, double *unused, double *out_6881219575845454851) {
   out_6881219575845454851[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1391725231232269814) {
   out_1391725231232269814[0] = 0;
   out_1391725231232269814[1] = 0;
   out_1391725231232269814[2] = 0;
   out_1391725231232269814[3] = 0;
   out_1391725231232269814[4] = 0;
   out_1391725231232269814[5] = 0;
   out_1391725231232269814[6] = 0;
   out_1391725231232269814[7] = 1;
   out_1391725231232269814[8] = 0;
}
void h_27(double *state, double *unused, double *out_3098250223370777530) {
   out_3098250223370777530[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8571588493912033734) {
   out_8571588493912033734[0] = 0;
   out_8571588493912033734[1] = 0;
   out_8571588493912033734[2] = 0;
   out_8571588493912033734[3] = 1;
   out_8571588493912033734[4] = 0;
   out_8571588493912033734[5] = 0;
   out_8571588493912033734[6] = 0;
   out_8571588493912033734[7] = 0;
   out_8571588493912033734[8] = 0;
}
void h_29(double *state, double *unused, double *out_5295819597833679222) {
   out_5295819597833679222[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5886593837797216639) {
   out_5886593837797216639[0] = 0;
   out_5886593837797216639[1] = 1;
   out_5886593837797216639[2] = 0;
   out_5886593837797216639[3] = 0;
   out_5886593837797216639[4] = 0;
   out_5886593837797216639[5] = 0;
   out_5886593837797216639[6] = 0;
   out_5886593837797216639[7] = 0;
   out_5886593837797216639[8] = 0;
}
void h_28(double *state, double *unused, double *out_307271827440529977) {
   out_307271827440529977[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3079393835858436275) {
   out_3079393835858436275[0] = 1;
   out_3079393835858436275[1] = 0;
   out_3079393835858436275[2] = 0;
   out_3079393835858436275[3] = 0;
   out_3079393835858436275[4] = 0;
   out_3079393835858436275[5] = 0;
   out_3079393835858436275[6] = 0;
   out_3079393835858436275[7] = 0;
   out_3079393835858436275[8] = 0;
}
void h_31(double *state, double *unused, double *out_1997553702117985950) {
   out_1997553702117985950[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5163874511983286466) {
   out_5163874511983286466[0] = 0;
   out_5163874511983286466[1] = 0;
   out_5163874511983286466[2] = 0;
   out_5163874511983286466[3] = 0;
   out_5163874511983286466[4] = 0;
   out_5163874511983286466[5] = 0;
   out_5163874511983286466[6] = 0;
   out_5163874511983286466[7] = 0;
   out_5163874511983286466[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2939202982062152065) {
  err_fun(nom_x, delta_x, out_2939202982062152065);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1376135891411693707) {
  inv_err_fun(nom_x, true_x, out_1376135891411693707);
}
void car_H_mod_fun(double *state, double *out_549565208155142469) {
  H_mod_fun(state, out_549565208155142469);
}
void car_f_fun(double *state, double dt, double *out_746317309293841027) {
  f_fun(state,  dt, out_746317309293841027);
}
void car_F_fun(double *state, double dt, double *out_7394503540686598109) {
  F_fun(state,  dt, out_7394503540686598109);
}
void car_h_25(double *state, double *unused, double *out_2154740370469115095) {
  h_25(state, unused, out_2154740370469115095);
}
void car_H_25(double *state, double *unused, double *out_5133228550106326038) {
  H_25(state, unused, out_5133228550106326038);
}
void car_h_24(double *state, double *unused, double *out_753491134740954591) {
  h_24(state, unused, out_753491134740954591);
}
void car_H_24(double *state, double *unused, double *out_5895123889926298685) {
  H_24(state, unused, out_5895123889926298685);
}
void car_h_30(double *state, double *unused, double *out_1879546308184609206) {
  h_30(state, unused, out_1879546308184609206);
}
void car_H_30(double *state, double *unused, double *out_6396825182111608823) {
  H_30(state, unused, out_6396825182111608823);
}
void car_h_26(double *state, double *unused, double *out_6881219575845454851) {
  h_26(state, unused, out_6881219575845454851);
}
void car_H_26(double *state, double *unused, double *out_1391725231232269814) {
  H_26(state, unused, out_1391725231232269814);
}
void car_h_27(double *state, double *unused, double *out_3098250223370777530) {
  h_27(state, unused, out_3098250223370777530);
}
void car_H_27(double *state, double *unused, double *out_8571588493912033734) {
  H_27(state, unused, out_8571588493912033734);
}
void car_h_29(double *state, double *unused, double *out_5295819597833679222) {
  h_29(state, unused, out_5295819597833679222);
}
void car_H_29(double *state, double *unused, double *out_5886593837797216639) {
  H_29(state, unused, out_5886593837797216639);
}
void car_h_28(double *state, double *unused, double *out_307271827440529977) {
  h_28(state, unused, out_307271827440529977);
}
void car_H_28(double *state, double *unused, double *out_3079393835858436275) {
  H_28(state, unused, out_3079393835858436275);
}
void car_h_31(double *state, double *unused, double *out_1997553702117985950) {
  h_31(state, unused, out_1997553702117985950);
}
void car_H_31(double *state, double *unused, double *out_5163874511983286466) {
  H_31(state, unused, out_5163874511983286466);
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
