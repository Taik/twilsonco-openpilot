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
void err_fun(double *nom_x, double *delta_x, double *out_8858193099505922130) {
   out_8858193099505922130[0] = delta_x[0] + nom_x[0];
   out_8858193099505922130[1] = delta_x[1] + nom_x[1];
   out_8858193099505922130[2] = delta_x[2] + nom_x[2];
   out_8858193099505922130[3] = delta_x[3] + nom_x[3];
   out_8858193099505922130[4] = delta_x[4] + nom_x[4];
   out_8858193099505922130[5] = delta_x[5] + nom_x[5];
   out_8858193099505922130[6] = delta_x[6] + nom_x[6];
   out_8858193099505922130[7] = delta_x[7] + nom_x[7];
   out_8858193099505922130[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6125827550351398334) {
   out_6125827550351398334[0] = -nom_x[0] + true_x[0];
   out_6125827550351398334[1] = -nom_x[1] + true_x[1];
   out_6125827550351398334[2] = -nom_x[2] + true_x[2];
   out_6125827550351398334[3] = -nom_x[3] + true_x[3];
   out_6125827550351398334[4] = -nom_x[4] + true_x[4];
   out_6125827550351398334[5] = -nom_x[5] + true_x[5];
   out_6125827550351398334[6] = -nom_x[6] + true_x[6];
   out_6125827550351398334[7] = -nom_x[7] + true_x[7];
   out_6125827550351398334[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2420972627039917597) {
   out_2420972627039917597[0] = 1.0;
   out_2420972627039917597[1] = 0;
   out_2420972627039917597[2] = 0;
   out_2420972627039917597[3] = 0;
   out_2420972627039917597[4] = 0;
   out_2420972627039917597[5] = 0;
   out_2420972627039917597[6] = 0;
   out_2420972627039917597[7] = 0;
   out_2420972627039917597[8] = 0;
   out_2420972627039917597[9] = 0;
   out_2420972627039917597[10] = 1.0;
   out_2420972627039917597[11] = 0;
   out_2420972627039917597[12] = 0;
   out_2420972627039917597[13] = 0;
   out_2420972627039917597[14] = 0;
   out_2420972627039917597[15] = 0;
   out_2420972627039917597[16] = 0;
   out_2420972627039917597[17] = 0;
   out_2420972627039917597[18] = 0;
   out_2420972627039917597[19] = 0;
   out_2420972627039917597[20] = 1.0;
   out_2420972627039917597[21] = 0;
   out_2420972627039917597[22] = 0;
   out_2420972627039917597[23] = 0;
   out_2420972627039917597[24] = 0;
   out_2420972627039917597[25] = 0;
   out_2420972627039917597[26] = 0;
   out_2420972627039917597[27] = 0;
   out_2420972627039917597[28] = 0;
   out_2420972627039917597[29] = 0;
   out_2420972627039917597[30] = 1.0;
   out_2420972627039917597[31] = 0;
   out_2420972627039917597[32] = 0;
   out_2420972627039917597[33] = 0;
   out_2420972627039917597[34] = 0;
   out_2420972627039917597[35] = 0;
   out_2420972627039917597[36] = 0;
   out_2420972627039917597[37] = 0;
   out_2420972627039917597[38] = 0;
   out_2420972627039917597[39] = 0;
   out_2420972627039917597[40] = 1.0;
   out_2420972627039917597[41] = 0;
   out_2420972627039917597[42] = 0;
   out_2420972627039917597[43] = 0;
   out_2420972627039917597[44] = 0;
   out_2420972627039917597[45] = 0;
   out_2420972627039917597[46] = 0;
   out_2420972627039917597[47] = 0;
   out_2420972627039917597[48] = 0;
   out_2420972627039917597[49] = 0;
   out_2420972627039917597[50] = 1.0;
   out_2420972627039917597[51] = 0;
   out_2420972627039917597[52] = 0;
   out_2420972627039917597[53] = 0;
   out_2420972627039917597[54] = 0;
   out_2420972627039917597[55] = 0;
   out_2420972627039917597[56] = 0;
   out_2420972627039917597[57] = 0;
   out_2420972627039917597[58] = 0;
   out_2420972627039917597[59] = 0;
   out_2420972627039917597[60] = 1.0;
   out_2420972627039917597[61] = 0;
   out_2420972627039917597[62] = 0;
   out_2420972627039917597[63] = 0;
   out_2420972627039917597[64] = 0;
   out_2420972627039917597[65] = 0;
   out_2420972627039917597[66] = 0;
   out_2420972627039917597[67] = 0;
   out_2420972627039917597[68] = 0;
   out_2420972627039917597[69] = 0;
   out_2420972627039917597[70] = 1.0;
   out_2420972627039917597[71] = 0;
   out_2420972627039917597[72] = 0;
   out_2420972627039917597[73] = 0;
   out_2420972627039917597[74] = 0;
   out_2420972627039917597[75] = 0;
   out_2420972627039917597[76] = 0;
   out_2420972627039917597[77] = 0;
   out_2420972627039917597[78] = 0;
   out_2420972627039917597[79] = 0;
   out_2420972627039917597[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6131450845666027670) {
   out_6131450845666027670[0] = state[0];
   out_6131450845666027670[1] = state[1];
   out_6131450845666027670[2] = state[2];
   out_6131450845666027670[3] = state[3];
   out_6131450845666027670[4] = state[4];
   out_6131450845666027670[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6131450845666027670[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6131450845666027670[7] = state[7];
   out_6131450845666027670[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1611502613691033176) {
   out_1611502613691033176[0] = 1;
   out_1611502613691033176[1] = 0;
   out_1611502613691033176[2] = 0;
   out_1611502613691033176[3] = 0;
   out_1611502613691033176[4] = 0;
   out_1611502613691033176[5] = 0;
   out_1611502613691033176[6] = 0;
   out_1611502613691033176[7] = 0;
   out_1611502613691033176[8] = 0;
   out_1611502613691033176[9] = 0;
   out_1611502613691033176[10] = 1;
   out_1611502613691033176[11] = 0;
   out_1611502613691033176[12] = 0;
   out_1611502613691033176[13] = 0;
   out_1611502613691033176[14] = 0;
   out_1611502613691033176[15] = 0;
   out_1611502613691033176[16] = 0;
   out_1611502613691033176[17] = 0;
   out_1611502613691033176[18] = 0;
   out_1611502613691033176[19] = 0;
   out_1611502613691033176[20] = 1;
   out_1611502613691033176[21] = 0;
   out_1611502613691033176[22] = 0;
   out_1611502613691033176[23] = 0;
   out_1611502613691033176[24] = 0;
   out_1611502613691033176[25] = 0;
   out_1611502613691033176[26] = 0;
   out_1611502613691033176[27] = 0;
   out_1611502613691033176[28] = 0;
   out_1611502613691033176[29] = 0;
   out_1611502613691033176[30] = 1;
   out_1611502613691033176[31] = 0;
   out_1611502613691033176[32] = 0;
   out_1611502613691033176[33] = 0;
   out_1611502613691033176[34] = 0;
   out_1611502613691033176[35] = 0;
   out_1611502613691033176[36] = 0;
   out_1611502613691033176[37] = 0;
   out_1611502613691033176[38] = 0;
   out_1611502613691033176[39] = 0;
   out_1611502613691033176[40] = 1;
   out_1611502613691033176[41] = 0;
   out_1611502613691033176[42] = 0;
   out_1611502613691033176[43] = 0;
   out_1611502613691033176[44] = 0;
   out_1611502613691033176[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1611502613691033176[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1611502613691033176[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1611502613691033176[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1611502613691033176[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1611502613691033176[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1611502613691033176[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1611502613691033176[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1611502613691033176[53] = -9.8000000000000007*dt;
   out_1611502613691033176[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1611502613691033176[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1611502613691033176[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1611502613691033176[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1611502613691033176[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1611502613691033176[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1611502613691033176[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1611502613691033176[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1611502613691033176[62] = 0;
   out_1611502613691033176[63] = 0;
   out_1611502613691033176[64] = 0;
   out_1611502613691033176[65] = 0;
   out_1611502613691033176[66] = 0;
   out_1611502613691033176[67] = 0;
   out_1611502613691033176[68] = 0;
   out_1611502613691033176[69] = 0;
   out_1611502613691033176[70] = 1;
   out_1611502613691033176[71] = 0;
   out_1611502613691033176[72] = 0;
   out_1611502613691033176[73] = 0;
   out_1611502613691033176[74] = 0;
   out_1611502613691033176[75] = 0;
   out_1611502613691033176[76] = 0;
   out_1611502613691033176[77] = 0;
   out_1611502613691033176[78] = 0;
   out_1611502613691033176[79] = 0;
   out_1611502613691033176[80] = 1;
}
void h_25(double *state, double *unused, double *out_7412045671842276054) {
   out_7412045671842276054[0] = state[6];
}
void H_25(double *state, double *unused, double *out_229619803497515274) {
   out_229619803497515274[0] = 0;
   out_229619803497515274[1] = 0;
   out_229619803497515274[2] = 0;
   out_229619803497515274[3] = 0;
   out_229619803497515274[4] = 0;
   out_229619803497515274[5] = 0;
   out_229619803497515274[6] = 1;
   out_229619803497515274[7] = 0;
   out_229619803497515274[8] = 0;
}
void h_24(double *state, double *unused, double *out_4498318601322565789) {
   out_4498318601322565789[0] = state[4];
   out_4498318601322565789[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1996087980481353288) {
   out_1996087980481353288[0] = 0;
   out_1996087980481353288[1] = 0;
   out_1996087980481353288[2] = 0;
   out_1996087980481353288[3] = 0;
   out_1996087980481353288[4] = 1;
   out_1996087980481353288[5] = 0;
   out_1996087980481353288[6] = 0;
   out_1996087980481353288[7] = 0;
   out_1996087980481353288[8] = 0;
   out_1996087980481353288[9] = 0;
   out_1996087980481353288[10] = 0;
   out_1996087980481353288[11] = 0;
   out_1996087980481353288[12] = 0;
   out_1996087980481353288[13] = 0;
   out_1996087980481353288[14] = 1;
   out_1996087980481353288[15] = 0;
   out_1996087980481353288[16] = 0;
   out_1996087980481353288[17] = 0;
}
void h_30(double *state, double *unused, double *out_4446870475390980399) {
   out_4446870475390980399[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6687070537994101481) {
   out_6687070537994101481[0] = 0;
   out_6687070537994101481[1] = 0;
   out_6687070537994101481[2] = 0;
   out_6687070537994101481[3] = 0;
   out_6687070537994101481[4] = 1;
   out_6687070537994101481[5] = 0;
   out_6687070537994101481[6] = 0;
   out_6687070537994101481[7] = 0;
   out_6687070537994101481[8] = 0;
}
void h_26(double *state, double *unused, double *out_1311114438602757557) {
   out_1311114438602757557[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3074906166263285327) {
   out_3074906166263285327[0] = 0;
   out_3074906166263285327[1] = 0;
   out_3074906166263285327[2] = 0;
   out_3074906166263285327[3] = 0;
   out_3074906166263285327[4] = 0;
   out_3074906166263285327[5] = 0;
   out_3074906166263285327[6] = 0;
   out_3074906166263285327[7] = 1;
   out_3074906166263285327[8] = 0;
}
void h_27(double *state, double *unused, double *out_5808791186031950615) {
   out_5808791186031950615[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4512307226193676570) {
   out_4512307226193676570[0] = 0;
   out_4512307226193676570[1] = 0;
   out_4512307226193676570[2] = 0;
   out_4512307226193676570[3] = 1;
   out_4512307226193676570[4] = 0;
   out_4512307226193676570[5] = 0;
   out_4512307226193676570[6] = 0;
   out_4512307226193676570[7] = 0;
   out_4512307226193676570[8] = 0;
}
void h_29(double *state, double *unused, double *out_554802792269865246) {
   out_554802792269865246[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7197301882308493665) {
   out_7197301882308493665[0] = 0;
   out_7197301882308493665[1] = 1;
   out_7197301882308493665[2] = 0;
   out_7197301882308493665[3] = 0;
   out_7197301882308493665[4] = 0;
   out_7197301882308493665[5] = 0;
   out_7197301882308493665[6] = 0;
   out_7197301882308493665[7] = 0;
   out_7197301882308493665[8] = 0;
}
void h_28(double *state, double *unused, double *out_6976534987108092352) {
   out_6976534987108092352[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2283454517745405037) {
   out_2283454517745405037[0] = 1;
   out_2283454517745405037[1] = 0;
   out_2283454517745405037[2] = 0;
   out_2283454517745405037[3] = 0;
   out_2283454517745405037[4] = 0;
   out_2283454517745405037[5] = 0;
   out_2283454517745405037[6] = 0;
   out_2283454517745405037[7] = 0;
   out_2283454517745405037[8] = 0;
}
void h_31(double *state, double *unused, double *out_3774596143588253704) {
   out_3774596143588253704[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6847055447014301979) {
   out_6847055447014301979[0] = 0;
   out_6847055447014301979[1] = 0;
   out_6847055447014301979[2] = 0;
   out_6847055447014301979[3] = 0;
   out_6847055447014301979[4] = 0;
   out_6847055447014301979[5] = 0;
   out_6847055447014301979[6] = 0;
   out_6847055447014301979[7] = 0;
   out_6847055447014301979[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8858193099505922130) {
  err_fun(nom_x, delta_x, out_8858193099505922130);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6125827550351398334) {
  inv_err_fun(nom_x, true_x, out_6125827550351398334);
}
void car_H_mod_fun(double *state, double *out_2420972627039917597) {
  H_mod_fun(state, out_2420972627039917597);
}
void car_f_fun(double *state, double dt, double *out_6131450845666027670) {
  f_fun(state,  dt, out_6131450845666027670);
}
void car_F_fun(double *state, double dt, double *out_1611502613691033176) {
  F_fun(state,  dt, out_1611502613691033176);
}
void car_h_25(double *state, double *unused, double *out_7412045671842276054) {
  h_25(state, unused, out_7412045671842276054);
}
void car_H_25(double *state, double *unused, double *out_229619803497515274) {
  H_25(state, unused, out_229619803497515274);
}
void car_h_24(double *state, double *unused, double *out_4498318601322565789) {
  h_24(state, unused, out_4498318601322565789);
}
void car_H_24(double *state, double *unused, double *out_1996087980481353288) {
  H_24(state, unused, out_1996087980481353288);
}
void car_h_30(double *state, double *unused, double *out_4446870475390980399) {
  h_30(state, unused, out_4446870475390980399);
}
void car_H_30(double *state, double *unused, double *out_6687070537994101481) {
  H_30(state, unused, out_6687070537994101481);
}
void car_h_26(double *state, double *unused, double *out_1311114438602757557) {
  h_26(state, unused, out_1311114438602757557);
}
void car_H_26(double *state, double *unused, double *out_3074906166263285327) {
  H_26(state, unused, out_3074906166263285327);
}
void car_h_27(double *state, double *unused, double *out_5808791186031950615) {
  h_27(state, unused, out_5808791186031950615);
}
void car_H_27(double *state, double *unused, double *out_4512307226193676570) {
  H_27(state, unused, out_4512307226193676570);
}
void car_h_29(double *state, double *unused, double *out_554802792269865246) {
  h_29(state, unused, out_554802792269865246);
}
void car_H_29(double *state, double *unused, double *out_7197301882308493665) {
  H_29(state, unused, out_7197301882308493665);
}
void car_h_28(double *state, double *unused, double *out_6976534987108092352) {
  h_28(state, unused, out_6976534987108092352);
}
void car_H_28(double *state, double *unused, double *out_2283454517745405037) {
  H_28(state, unused, out_2283454517745405037);
}
void car_h_31(double *state, double *unused, double *out_3774596143588253704) {
  h_31(state, unused, out_3774596143588253704);
}
void car_H_31(double *state, double *unused, double *out_6847055447014301979) {
  H_31(state, unused, out_6847055447014301979);
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
