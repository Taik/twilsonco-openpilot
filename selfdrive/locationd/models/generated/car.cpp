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
void err_fun(double *nom_x, double *delta_x, double *out_4636062031133054768) {
   out_4636062031133054768[0] = delta_x[0] + nom_x[0];
   out_4636062031133054768[1] = delta_x[1] + nom_x[1];
   out_4636062031133054768[2] = delta_x[2] + nom_x[2];
   out_4636062031133054768[3] = delta_x[3] + nom_x[3];
   out_4636062031133054768[4] = delta_x[4] + nom_x[4];
   out_4636062031133054768[5] = delta_x[5] + nom_x[5];
   out_4636062031133054768[6] = delta_x[6] + nom_x[6];
   out_4636062031133054768[7] = delta_x[7] + nom_x[7];
   out_4636062031133054768[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6030581718255978326) {
   out_6030581718255978326[0] = -nom_x[0] + true_x[0];
   out_6030581718255978326[1] = -nom_x[1] + true_x[1];
   out_6030581718255978326[2] = -nom_x[2] + true_x[2];
   out_6030581718255978326[3] = -nom_x[3] + true_x[3];
   out_6030581718255978326[4] = -nom_x[4] + true_x[4];
   out_6030581718255978326[5] = -nom_x[5] + true_x[5];
   out_6030581718255978326[6] = -nom_x[6] + true_x[6];
   out_6030581718255978326[7] = -nom_x[7] + true_x[7];
   out_6030581718255978326[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2936385218361774249) {
   out_2936385218361774249[0] = 1.0;
   out_2936385218361774249[1] = 0;
   out_2936385218361774249[2] = 0;
   out_2936385218361774249[3] = 0;
   out_2936385218361774249[4] = 0;
   out_2936385218361774249[5] = 0;
   out_2936385218361774249[6] = 0;
   out_2936385218361774249[7] = 0;
   out_2936385218361774249[8] = 0;
   out_2936385218361774249[9] = 0;
   out_2936385218361774249[10] = 1.0;
   out_2936385218361774249[11] = 0;
   out_2936385218361774249[12] = 0;
   out_2936385218361774249[13] = 0;
   out_2936385218361774249[14] = 0;
   out_2936385218361774249[15] = 0;
   out_2936385218361774249[16] = 0;
   out_2936385218361774249[17] = 0;
   out_2936385218361774249[18] = 0;
   out_2936385218361774249[19] = 0;
   out_2936385218361774249[20] = 1.0;
   out_2936385218361774249[21] = 0;
   out_2936385218361774249[22] = 0;
   out_2936385218361774249[23] = 0;
   out_2936385218361774249[24] = 0;
   out_2936385218361774249[25] = 0;
   out_2936385218361774249[26] = 0;
   out_2936385218361774249[27] = 0;
   out_2936385218361774249[28] = 0;
   out_2936385218361774249[29] = 0;
   out_2936385218361774249[30] = 1.0;
   out_2936385218361774249[31] = 0;
   out_2936385218361774249[32] = 0;
   out_2936385218361774249[33] = 0;
   out_2936385218361774249[34] = 0;
   out_2936385218361774249[35] = 0;
   out_2936385218361774249[36] = 0;
   out_2936385218361774249[37] = 0;
   out_2936385218361774249[38] = 0;
   out_2936385218361774249[39] = 0;
   out_2936385218361774249[40] = 1.0;
   out_2936385218361774249[41] = 0;
   out_2936385218361774249[42] = 0;
   out_2936385218361774249[43] = 0;
   out_2936385218361774249[44] = 0;
   out_2936385218361774249[45] = 0;
   out_2936385218361774249[46] = 0;
   out_2936385218361774249[47] = 0;
   out_2936385218361774249[48] = 0;
   out_2936385218361774249[49] = 0;
   out_2936385218361774249[50] = 1.0;
   out_2936385218361774249[51] = 0;
   out_2936385218361774249[52] = 0;
   out_2936385218361774249[53] = 0;
   out_2936385218361774249[54] = 0;
   out_2936385218361774249[55] = 0;
   out_2936385218361774249[56] = 0;
   out_2936385218361774249[57] = 0;
   out_2936385218361774249[58] = 0;
   out_2936385218361774249[59] = 0;
   out_2936385218361774249[60] = 1.0;
   out_2936385218361774249[61] = 0;
   out_2936385218361774249[62] = 0;
   out_2936385218361774249[63] = 0;
   out_2936385218361774249[64] = 0;
   out_2936385218361774249[65] = 0;
   out_2936385218361774249[66] = 0;
   out_2936385218361774249[67] = 0;
   out_2936385218361774249[68] = 0;
   out_2936385218361774249[69] = 0;
   out_2936385218361774249[70] = 1.0;
   out_2936385218361774249[71] = 0;
   out_2936385218361774249[72] = 0;
   out_2936385218361774249[73] = 0;
   out_2936385218361774249[74] = 0;
   out_2936385218361774249[75] = 0;
   out_2936385218361774249[76] = 0;
   out_2936385218361774249[77] = 0;
   out_2936385218361774249[78] = 0;
   out_2936385218361774249[79] = 0;
   out_2936385218361774249[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6336615873933685425) {
   out_6336615873933685425[0] = state[0];
   out_6336615873933685425[1] = state[1];
   out_6336615873933685425[2] = state[2];
   out_6336615873933685425[3] = state[3];
   out_6336615873933685425[4] = state[4];
   out_6336615873933685425[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6336615873933685425[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6336615873933685425[7] = state[7];
   out_6336615873933685425[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5006846382452517549) {
   out_5006846382452517549[0] = 1;
   out_5006846382452517549[1] = 0;
   out_5006846382452517549[2] = 0;
   out_5006846382452517549[3] = 0;
   out_5006846382452517549[4] = 0;
   out_5006846382452517549[5] = 0;
   out_5006846382452517549[6] = 0;
   out_5006846382452517549[7] = 0;
   out_5006846382452517549[8] = 0;
   out_5006846382452517549[9] = 0;
   out_5006846382452517549[10] = 1;
   out_5006846382452517549[11] = 0;
   out_5006846382452517549[12] = 0;
   out_5006846382452517549[13] = 0;
   out_5006846382452517549[14] = 0;
   out_5006846382452517549[15] = 0;
   out_5006846382452517549[16] = 0;
   out_5006846382452517549[17] = 0;
   out_5006846382452517549[18] = 0;
   out_5006846382452517549[19] = 0;
   out_5006846382452517549[20] = 1;
   out_5006846382452517549[21] = 0;
   out_5006846382452517549[22] = 0;
   out_5006846382452517549[23] = 0;
   out_5006846382452517549[24] = 0;
   out_5006846382452517549[25] = 0;
   out_5006846382452517549[26] = 0;
   out_5006846382452517549[27] = 0;
   out_5006846382452517549[28] = 0;
   out_5006846382452517549[29] = 0;
   out_5006846382452517549[30] = 1;
   out_5006846382452517549[31] = 0;
   out_5006846382452517549[32] = 0;
   out_5006846382452517549[33] = 0;
   out_5006846382452517549[34] = 0;
   out_5006846382452517549[35] = 0;
   out_5006846382452517549[36] = 0;
   out_5006846382452517549[37] = 0;
   out_5006846382452517549[38] = 0;
   out_5006846382452517549[39] = 0;
   out_5006846382452517549[40] = 1;
   out_5006846382452517549[41] = 0;
   out_5006846382452517549[42] = 0;
   out_5006846382452517549[43] = 0;
   out_5006846382452517549[44] = 0;
   out_5006846382452517549[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5006846382452517549[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5006846382452517549[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5006846382452517549[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5006846382452517549[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5006846382452517549[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5006846382452517549[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5006846382452517549[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5006846382452517549[53] = -9.8000000000000007*dt;
   out_5006846382452517549[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5006846382452517549[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5006846382452517549[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5006846382452517549[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5006846382452517549[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5006846382452517549[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5006846382452517549[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5006846382452517549[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5006846382452517549[62] = 0;
   out_5006846382452517549[63] = 0;
   out_5006846382452517549[64] = 0;
   out_5006846382452517549[65] = 0;
   out_5006846382452517549[66] = 0;
   out_5006846382452517549[67] = 0;
   out_5006846382452517549[68] = 0;
   out_5006846382452517549[69] = 0;
   out_5006846382452517549[70] = 1;
   out_5006846382452517549[71] = 0;
   out_5006846382452517549[72] = 0;
   out_5006846382452517549[73] = 0;
   out_5006846382452517549[74] = 0;
   out_5006846382452517549[75] = 0;
   out_5006846382452517549[76] = 0;
   out_5006846382452517549[77] = 0;
   out_5006846382452517549[78] = 0;
   out_5006846382452517549[79] = 0;
   out_5006846382452517549[80] = 1;
}
void h_25(double *state, double *unused, double *out_680600810615390925) {
   out_680600810615390925[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5768343168375162758) {
   out_5768343168375162758[0] = 0;
   out_5768343168375162758[1] = 0;
   out_5768343168375162758[2] = 0;
   out_5768343168375162758[3] = 0;
   out_5768343168375162758[4] = 0;
   out_5768343168375162758[5] = 0;
   out_5768343168375162758[6] = 1;
   out_5768343168375162758[7] = 0;
   out_5768343168375162758[8] = 0;
}
void h_24(double *state, double *unused, double *out_1697116264620455802) {
   out_1697116264620455802[0] = state[4];
   out_1697116264620455802[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3595693569369663192) {
   out_3595693569369663192[0] = 0;
   out_3595693569369663192[1] = 0;
   out_3595693569369663192[2] = 0;
   out_3595693569369663192[3] = 0;
   out_3595693569369663192[4] = 1;
   out_3595693569369663192[5] = 0;
   out_3595693569369663192[6] = 0;
   out_3595693569369663192[7] = 0;
   out_3595693569369663192[8] = 0;
   out_3595693569369663192[9] = 0;
   out_3595693569369663192[10] = 0;
   out_3595693569369663192[11] = 0;
   out_3595693569369663192[12] = 0;
   out_3595693569369663192[13] = 0;
   out_3595693569369663192[14] = 1;
   out_3595693569369663192[15] = 0;
   out_3595693569369663192[16] = 0;
   out_3595693569369663192[17] = 0;
}
void h_30(double *state, double *unused, double *out_7959276666103150959) {
   out_7959276666103150959[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5761710563842772103) {
   out_5761710563842772103[0] = 0;
   out_5761710563842772103[1] = 0;
   out_5761710563842772103[2] = 0;
   out_5761710563842772103[3] = 0;
   out_5761710563842772103[4] = 1;
   out_5761710563842772103[5] = 0;
   out_5761710563842772103[6] = 0;
   out_5761710563842772103[7] = 0;
   out_5761710563842772103[8] = 0;
}
void h_26(double *state, double *unused, double *out_5836448100731834965) {
   out_5836448100731834965[0] = state[7];
}
void H_26(double *state, double *unused, double *out_9072869138135963359) {
   out_9072869138135963359[0] = 0;
   out_9072869138135963359[1] = 0;
   out_9072869138135963359[2] = 0;
   out_9072869138135963359[3] = 0;
   out_9072869138135963359[4] = 0;
   out_9072869138135963359[5] = 0;
   out_9072869138135963359[6] = 0;
   out_9072869138135963359[7] = 1;
   out_9072869138135963359[8] = 0;
}
void h_27(double *state, double *unused, double *out_5682274078276236570) {
   out_5682274078276236570[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7936473875643197014) {
   out_7936473875643197014[0] = 0;
   out_7936473875643197014[1] = 0;
   out_7936473875643197014[2] = 0;
   out_7936473875643197014[3] = 1;
   out_7936473875643197014[4] = 0;
   out_7936473875643197014[5] = 0;
   out_7936473875643197014[6] = 0;
   out_7936473875643197014[7] = 0;
   out_7936473875643197014[8] = 0;
}
void h_29(double *state, double *unused, double *out_4231642685647353403) {
   out_4231642685647353403[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8796907471196803569) {
   out_8796907471196803569[0] = 0;
   out_8796907471196803569[1] = 1;
   out_8796907471196803569[2] = 0;
   out_8796907471196803569[3] = 0;
   out_8796907471196803569[4] = 0;
   out_8796907471196803569[5] = 0;
   out_8796907471196803569[6] = 0;
   out_8796907471196803569[7] = 0;
   out_8796907471196803569[8] = 0;
}
void h_28(double *state, double *unused, double *out_6664358171611534682) {
   out_6664358171611534682[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3714508454127272995) {
   out_3714508454127272995[0] = 1;
   out_3714508454127272995[1] = 0;
   out_3714508454127272995[2] = 0;
   out_3714508454127272995[3] = 0;
   out_3714508454127272995[4] = 0;
   out_3714508454127272995[5] = 0;
   out_3714508454127272995[6] = 0;
   out_3714508454127272995[7] = 0;
   out_3714508454127272995[8] = 0;
}
void h_31(double *state, double *unused, double *out_405406748330885036) {
   out_405406748330885036[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5601725654822571605) {
   out_5601725654822571605[0] = 0;
   out_5601725654822571605[1] = 0;
   out_5601725654822571605[2] = 0;
   out_5601725654822571605[3] = 0;
   out_5601725654822571605[4] = 0;
   out_5601725654822571605[5] = 0;
   out_5601725654822571605[6] = 0;
   out_5601725654822571605[7] = 0;
   out_5601725654822571605[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4636062031133054768) {
  err_fun(nom_x, delta_x, out_4636062031133054768);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6030581718255978326) {
  inv_err_fun(nom_x, true_x, out_6030581718255978326);
}
void car_H_mod_fun(double *state, double *out_2936385218361774249) {
  H_mod_fun(state, out_2936385218361774249);
}
void car_f_fun(double *state, double dt, double *out_6336615873933685425) {
  f_fun(state,  dt, out_6336615873933685425);
}
void car_F_fun(double *state, double dt, double *out_5006846382452517549) {
  F_fun(state,  dt, out_5006846382452517549);
}
void car_h_25(double *state, double *unused, double *out_680600810615390925) {
  h_25(state, unused, out_680600810615390925);
}
void car_H_25(double *state, double *unused, double *out_5768343168375162758) {
  H_25(state, unused, out_5768343168375162758);
}
void car_h_24(double *state, double *unused, double *out_1697116264620455802) {
  h_24(state, unused, out_1697116264620455802);
}
void car_H_24(double *state, double *unused, double *out_3595693569369663192) {
  H_24(state, unused, out_3595693569369663192);
}
void car_h_30(double *state, double *unused, double *out_7959276666103150959) {
  h_30(state, unused, out_7959276666103150959);
}
void car_H_30(double *state, double *unused, double *out_5761710563842772103) {
  H_30(state, unused, out_5761710563842772103);
}
void car_h_26(double *state, double *unused, double *out_5836448100731834965) {
  h_26(state, unused, out_5836448100731834965);
}
void car_H_26(double *state, double *unused, double *out_9072869138135963359) {
  H_26(state, unused, out_9072869138135963359);
}
void car_h_27(double *state, double *unused, double *out_5682274078276236570) {
  h_27(state, unused, out_5682274078276236570);
}
void car_H_27(double *state, double *unused, double *out_7936473875643197014) {
  H_27(state, unused, out_7936473875643197014);
}
void car_h_29(double *state, double *unused, double *out_4231642685647353403) {
  h_29(state, unused, out_4231642685647353403);
}
void car_H_29(double *state, double *unused, double *out_8796907471196803569) {
  H_29(state, unused, out_8796907471196803569);
}
void car_h_28(double *state, double *unused, double *out_6664358171611534682) {
  h_28(state, unused, out_6664358171611534682);
}
void car_H_28(double *state, double *unused, double *out_3714508454127272995) {
  H_28(state, unused, out_3714508454127272995);
}
void car_h_31(double *state, double *unused, double *out_405406748330885036) {
  h_31(state, unused, out_405406748330885036);
}
void car_H_31(double *state, double *unused, double *out_5601725654822571605) {
  H_31(state, unused, out_5601725654822571605);
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
