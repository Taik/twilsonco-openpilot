#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4991393669533314019) {
   out_4991393669533314019[0] = delta_x[0] + nom_x[0];
   out_4991393669533314019[1] = delta_x[1] + nom_x[1];
   out_4991393669533314019[2] = delta_x[2] + nom_x[2];
   out_4991393669533314019[3] = delta_x[3] + nom_x[3];
   out_4991393669533314019[4] = delta_x[4] + nom_x[4];
   out_4991393669533314019[5] = delta_x[5] + nom_x[5];
   out_4991393669533314019[6] = delta_x[6] + nom_x[6];
   out_4991393669533314019[7] = delta_x[7] + nom_x[7];
   out_4991393669533314019[8] = delta_x[8] + nom_x[8];
   out_4991393669533314019[9] = delta_x[9] + nom_x[9];
   out_4991393669533314019[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2774114983703919595) {
   out_2774114983703919595[0] = -nom_x[0] + true_x[0];
   out_2774114983703919595[1] = -nom_x[1] + true_x[1];
   out_2774114983703919595[2] = -nom_x[2] + true_x[2];
   out_2774114983703919595[3] = -nom_x[3] + true_x[3];
   out_2774114983703919595[4] = -nom_x[4] + true_x[4];
   out_2774114983703919595[5] = -nom_x[5] + true_x[5];
   out_2774114983703919595[6] = -nom_x[6] + true_x[6];
   out_2774114983703919595[7] = -nom_x[7] + true_x[7];
   out_2774114983703919595[8] = -nom_x[8] + true_x[8];
   out_2774114983703919595[9] = -nom_x[9] + true_x[9];
   out_2774114983703919595[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_6730346335144126854) {
   out_6730346335144126854[0] = 1.0;
   out_6730346335144126854[1] = 0;
   out_6730346335144126854[2] = 0;
   out_6730346335144126854[3] = 0;
   out_6730346335144126854[4] = 0;
   out_6730346335144126854[5] = 0;
   out_6730346335144126854[6] = 0;
   out_6730346335144126854[7] = 0;
   out_6730346335144126854[8] = 0;
   out_6730346335144126854[9] = 0;
   out_6730346335144126854[10] = 0;
   out_6730346335144126854[11] = 0;
   out_6730346335144126854[12] = 1.0;
   out_6730346335144126854[13] = 0;
   out_6730346335144126854[14] = 0;
   out_6730346335144126854[15] = 0;
   out_6730346335144126854[16] = 0;
   out_6730346335144126854[17] = 0;
   out_6730346335144126854[18] = 0;
   out_6730346335144126854[19] = 0;
   out_6730346335144126854[20] = 0;
   out_6730346335144126854[21] = 0;
   out_6730346335144126854[22] = 0;
   out_6730346335144126854[23] = 0;
   out_6730346335144126854[24] = 1.0;
   out_6730346335144126854[25] = 0;
   out_6730346335144126854[26] = 0;
   out_6730346335144126854[27] = 0;
   out_6730346335144126854[28] = 0;
   out_6730346335144126854[29] = 0;
   out_6730346335144126854[30] = 0;
   out_6730346335144126854[31] = 0;
   out_6730346335144126854[32] = 0;
   out_6730346335144126854[33] = 0;
   out_6730346335144126854[34] = 0;
   out_6730346335144126854[35] = 0;
   out_6730346335144126854[36] = 1.0;
   out_6730346335144126854[37] = 0;
   out_6730346335144126854[38] = 0;
   out_6730346335144126854[39] = 0;
   out_6730346335144126854[40] = 0;
   out_6730346335144126854[41] = 0;
   out_6730346335144126854[42] = 0;
   out_6730346335144126854[43] = 0;
   out_6730346335144126854[44] = 0;
   out_6730346335144126854[45] = 0;
   out_6730346335144126854[46] = 0;
   out_6730346335144126854[47] = 0;
   out_6730346335144126854[48] = 1.0;
   out_6730346335144126854[49] = 0;
   out_6730346335144126854[50] = 0;
   out_6730346335144126854[51] = 0;
   out_6730346335144126854[52] = 0;
   out_6730346335144126854[53] = 0;
   out_6730346335144126854[54] = 0;
   out_6730346335144126854[55] = 0;
   out_6730346335144126854[56] = 0;
   out_6730346335144126854[57] = 0;
   out_6730346335144126854[58] = 0;
   out_6730346335144126854[59] = 0;
   out_6730346335144126854[60] = 1.0;
   out_6730346335144126854[61] = 0;
   out_6730346335144126854[62] = 0;
   out_6730346335144126854[63] = 0;
   out_6730346335144126854[64] = 0;
   out_6730346335144126854[65] = 0;
   out_6730346335144126854[66] = 0;
   out_6730346335144126854[67] = 0;
   out_6730346335144126854[68] = 0;
   out_6730346335144126854[69] = 0;
   out_6730346335144126854[70] = 0;
   out_6730346335144126854[71] = 0;
   out_6730346335144126854[72] = 1.0;
   out_6730346335144126854[73] = 0;
   out_6730346335144126854[74] = 0;
   out_6730346335144126854[75] = 0;
   out_6730346335144126854[76] = 0;
   out_6730346335144126854[77] = 0;
   out_6730346335144126854[78] = 0;
   out_6730346335144126854[79] = 0;
   out_6730346335144126854[80] = 0;
   out_6730346335144126854[81] = 0;
   out_6730346335144126854[82] = 0;
   out_6730346335144126854[83] = 0;
   out_6730346335144126854[84] = 1.0;
   out_6730346335144126854[85] = 0;
   out_6730346335144126854[86] = 0;
   out_6730346335144126854[87] = 0;
   out_6730346335144126854[88] = 0;
   out_6730346335144126854[89] = 0;
   out_6730346335144126854[90] = 0;
   out_6730346335144126854[91] = 0;
   out_6730346335144126854[92] = 0;
   out_6730346335144126854[93] = 0;
   out_6730346335144126854[94] = 0;
   out_6730346335144126854[95] = 0;
   out_6730346335144126854[96] = 1.0;
   out_6730346335144126854[97] = 0;
   out_6730346335144126854[98] = 0;
   out_6730346335144126854[99] = 0;
   out_6730346335144126854[100] = 0;
   out_6730346335144126854[101] = 0;
   out_6730346335144126854[102] = 0;
   out_6730346335144126854[103] = 0;
   out_6730346335144126854[104] = 0;
   out_6730346335144126854[105] = 0;
   out_6730346335144126854[106] = 0;
   out_6730346335144126854[107] = 0;
   out_6730346335144126854[108] = 1.0;
   out_6730346335144126854[109] = 0;
   out_6730346335144126854[110] = 0;
   out_6730346335144126854[111] = 0;
   out_6730346335144126854[112] = 0;
   out_6730346335144126854[113] = 0;
   out_6730346335144126854[114] = 0;
   out_6730346335144126854[115] = 0;
   out_6730346335144126854[116] = 0;
   out_6730346335144126854[117] = 0;
   out_6730346335144126854[118] = 0;
   out_6730346335144126854[119] = 0;
   out_6730346335144126854[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_5306142855013818139) {
   out_5306142855013818139[0] = dt*state[3] + state[0];
   out_5306142855013818139[1] = dt*state[4] + state[1];
   out_5306142855013818139[2] = dt*state[5] + state[2];
   out_5306142855013818139[3] = state[3];
   out_5306142855013818139[4] = state[4];
   out_5306142855013818139[5] = state[5];
   out_5306142855013818139[6] = dt*state[7] + state[6];
   out_5306142855013818139[7] = dt*state[8] + state[7];
   out_5306142855013818139[8] = state[8];
   out_5306142855013818139[9] = state[9];
   out_5306142855013818139[10] = state[10];
}
void F_fun(double *state, double dt, double *out_3786678109369211231) {
   out_3786678109369211231[0] = 1;
   out_3786678109369211231[1] = 0;
   out_3786678109369211231[2] = 0;
   out_3786678109369211231[3] = dt;
   out_3786678109369211231[4] = 0;
   out_3786678109369211231[5] = 0;
   out_3786678109369211231[6] = 0;
   out_3786678109369211231[7] = 0;
   out_3786678109369211231[8] = 0;
   out_3786678109369211231[9] = 0;
   out_3786678109369211231[10] = 0;
   out_3786678109369211231[11] = 0;
   out_3786678109369211231[12] = 1;
   out_3786678109369211231[13] = 0;
   out_3786678109369211231[14] = 0;
   out_3786678109369211231[15] = dt;
   out_3786678109369211231[16] = 0;
   out_3786678109369211231[17] = 0;
   out_3786678109369211231[18] = 0;
   out_3786678109369211231[19] = 0;
   out_3786678109369211231[20] = 0;
   out_3786678109369211231[21] = 0;
   out_3786678109369211231[22] = 0;
   out_3786678109369211231[23] = 0;
   out_3786678109369211231[24] = 1;
   out_3786678109369211231[25] = 0;
   out_3786678109369211231[26] = 0;
   out_3786678109369211231[27] = dt;
   out_3786678109369211231[28] = 0;
   out_3786678109369211231[29] = 0;
   out_3786678109369211231[30] = 0;
   out_3786678109369211231[31] = 0;
   out_3786678109369211231[32] = 0;
   out_3786678109369211231[33] = 0;
   out_3786678109369211231[34] = 0;
   out_3786678109369211231[35] = 0;
   out_3786678109369211231[36] = 1;
   out_3786678109369211231[37] = 0;
   out_3786678109369211231[38] = 0;
   out_3786678109369211231[39] = 0;
   out_3786678109369211231[40] = 0;
   out_3786678109369211231[41] = 0;
   out_3786678109369211231[42] = 0;
   out_3786678109369211231[43] = 0;
   out_3786678109369211231[44] = 0;
   out_3786678109369211231[45] = 0;
   out_3786678109369211231[46] = 0;
   out_3786678109369211231[47] = 0;
   out_3786678109369211231[48] = 1;
   out_3786678109369211231[49] = 0;
   out_3786678109369211231[50] = 0;
   out_3786678109369211231[51] = 0;
   out_3786678109369211231[52] = 0;
   out_3786678109369211231[53] = 0;
   out_3786678109369211231[54] = 0;
   out_3786678109369211231[55] = 0;
   out_3786678109369211231[56] = 0;
   out_3786678109369211231[57] = 0;
   out_3786678109369211231[58] = 0;
   out_3786678109369211231[59] = 0;
   out_3786678109369211231[60] = 1;
   out_3786678109369211231[61] = 0;
   out_3786678109369211231[62] = 0;
   out_3786678109369211231[63] = 0;
   out_3786678109369211231[64] = 0;
   out_3786678109369211231[65] = 0;
   out_3786678109369211231[66] = 0;
   out_3786678109369211231[67] = 0;
   out_3786678109369211231[68] = 0;
   out_3786678109369211231[69] = 0;
   out_3786678109369211231[70] = 0;
   out_3786678109369211231[71] = 0;
   out_3786678109369211231[72] = 1;
   out_3786678109369211231[73] = dt;
   out_3786678109369211231[74] = 0;
   out_3786678109369211231[75] = 0;
   out_3786678109369211231[76] = 0;
   out_3786678109369211231[77] = 0;
   out_3786678109369211231[78] = 0;
   out_3786678109369211231[79] = 0;
   out_3786678109369211231[80] = 0;
   out_3786678109369211231[81] = 0;
   out_3786678109369211231[82] = 0;
   out_3786678109369211231[83] = 0;
   out_3786678109369211231[84] = 1;
   out_3786678109369211231[85] = dt;
   out_3786678109369211231[86] = 0;
   out_3786678109369211231[87] = 0;
   out_3786678109369211231[88] = 0;
   out_3786678109369211231[89] = 0;
   out_3786678109369211231[90] = 0;
   out_3786678109369211231[91] = 0;
   out_3786678109369211231[92] = 0;
   out_3786678109369211231[93] = 0;
   out_3786678109369211231[94] = 0;
   out_3786678109369211231[95] = 0;
   out_3786678109369211231[96] = 1;
   out_3786678109369211231[97] = 0;
   out_3786678109369211231[98] = 0;
   out_3786678109369211231[99] = 0;
   out_3786678109369211231[100] = 0;
   out_3786678109369211231[101] = 0;
   out_3786678109369211231[102] = 0;
   out_3786678109369211231[103] = 0;
   out_3786678109369211231[104] = 0;
   out_3786678109369211231[105] = 0;
   out_3786678109369211231[106] = 0;
   out_3786678109369211231[107] = 0;
   out_3786678109369211231[108] = 1;
   out_3786678109369211231[109] = 0;
   out_3786678109369211231[110] = 0;
   out_3786678109369211231[111] = 0;
   out_3786678109369211231[112] = 0;
   out_3786678109369211231[113] = 0;
   out_3786678109369211231[114] = 0;
   out_3786678109369211231[115] = 0;
   out_3786678109369211231[116] = 0;
   out_3786678109369211231[117] = 0;
   out_3786678109369211231[118] = 0;
   out_3786678109369211231[119] = 0;
   out_3786678109369211231[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_1396935052484491170) {
   out_1396935052484491170[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_8126508195366532936) {
   out_8126508195366532936[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8126508195366532936[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8126508195366532936[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8126508195366532936[3] = 0;
   out_8126508195366532936[4] = 0;
   out_8126508195366532936[5] = 0;
   out_8126508195366532936[6] = 1;
   out_8126508195366532936[7] = 0;
   out_8126508195366532936[8] = 0;
   out_8126508195366532936[9] = 0;
   out_8126508195366532936[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_3696039159246344318) {
   out_3696039159246344318[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_1052657384503181039) {
   out_1052657384503181039[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1052657384503181039[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1052657384503181039[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1052657384503181039[3] = 0;
   out_1052657384503181039[4] = 0;
   out_1052657384503181039[5] = 0;
   out_1052657384503181039[6] = 1;
   out_1052657384503181039[7] = 0;
   out_1052657384503181039[8] = 0;
   out_1052657384503181039[9] = 1;
   out_1052657384503181039[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_3520472109361379988) {
   out_3520472109361379988[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_6147765845821346871) {
   out_6147765845821346871[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6147765845821346871[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6147765845821346871[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6147765845821346871[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6147765845821346871[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6147765845821346871[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6147765845821346871[6] = 0;
   out_6147765845821346871[7] = 1;
   out_6147765845821346871[8] = 0;
   out_6147765845821346871[9] = 0;
   out_6147765845821346871[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_3520472109361379988) {
   out_3520472109361379988[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_6147765845821346871) {
   out_6147765845821346871[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6147765845821346871[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6147765845821346871[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6147765845821346871[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6147765845821346871[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6147765845821346871[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6147765845821346871[6] = 0;
   out_6147765845821346871[7] = 1;
   out_6147765845821346871[8] = 0;
   out_6147765845821346871[9] = 0;
   out_6147765845821346871[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_4991393669533314019) {
  err_fun(nom_x, delta_x, out_4991393669533314019);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_2774114983703919595) {
  inv_err_fun(nom_x, true_x, out_2774114983703919595);
}
void gnss_H_mod_fun(double *state, double *out_6730346335144126854) {
  H_mod_fun(state, out_6730346335144126854);
}
void gnss_f_fun(double *state, double dt, double *out_5306142855013818139) {
  f_fun(state,  dt, out_5306142855013818139);
}
void gnss_F_fun(double *state, double dt, double *out_3786678109369211231) {
  F_fun(state,  dt, out_3786678109369211231);
}
void gnss_h_6(double *state, double *sat_pos, double *out_1396935052484491170) {
  h_6(state, sat_pos, out_1396935052484491170);
}
void gnss_H_6(double *state, double *sat_pos, double *out_8126508195366532936) {
  H_6(state, sat_pos, out_8126508195366532936);
}
void gnss_h_20(double *state, double *sat_pos, double *out_3696039159246344318) {
  h_20(state, sat_pos, out_3696039159246344318);
}
void gnss_H_20(double *state, double *sat_pos, double *out_1052657384503181039) {
  H_20(state, sat_pos, out_1052657384503181039);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_3520472109361379988) {
  h_7(state, sat_pos_vel, out_3520472109361379988);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_6147765845821346871) {
  H_7(state, sat_pos_vel, out_6147765845821346871);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_3520472109361379988) {
  h_21(state, sat_pos_vel, out_3520472109361379988);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_6147765845821346871) {
  H_21(state, sat_pos_vel, out_6147765845821346871);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
