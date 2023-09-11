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
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6982813324875031433) {
   out_6982813324875031433[0] = delta_x[0] + nom_x[0];
   out_6982813324875031433[1] = delta_x[1] + nom_x[1];
   out_6982813324875031433[2] = delta_x[2] + nom_x[2];
   out_6982813324875031433[3] = delta_x[3] + nom_x[3];
   out_6982813324875031433[4] = delta_x[4] + nom_x[4];
   out_6982813324875031433[5] = delta_x[5] + nom_x[5];
   out_6982813324875031433[6] = delta_x[6] + nom_x[6];
   out_6982813324875031433[7] = delta_x[7] + nom_x[7];
   out_6982813324875031433[8] = delta_x[8] + nom_x[8];
   out_6982813324875031433[9] = delta_x[9] + nom_x[9];
   out_6982813324875031433[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8822363379104221878) {
   out_8822363379104221878[0] = -nom_x[0] + true_x[0];
   out_8822363379104221878[1] = -nom_x[1] + true_x[1];
   out_8822363379104221878[2] = -nom_x[2] + true_x[2];
   out_8822363379104221878[3] = -nom_x[3] + true_x[3];
   out_8822363379104221878[4] = -nom_x[4] + true_x[4];
   out_8822363379104221878[5] = -nom_x[5] + true_x[5];
   out_8822363379104221878[6] = -nom_x[6] + true_x[6];
   out_8822363379104221878[7] = -nom_x[7] + true_x[7];
   out_8822363379104221878[8] = -nom_x[8] + true_x[8];
   out_8822363379104221878[9] = -nom_x[9] + true_x[9];
   out_8822363379104221878[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_5122475045724401360) {
   out_5122475045724401360[0] = 1.0;
   out_5122475045724401360[1] = 0;
   out_5122475045724401360[2] = 0;
   out_5122475045724401360[3] = 0;
   out_5122475045724401360[4] = 0;
   out_5122475045724401360[5] = 0;
   out_5122475045724401360[6] = 0;
   out_5122475045724401360[7] = 0;
   out_5122475045724401360[8] = 0;
   out_5122475045724401360[9] = 0;
   out_5122475045724401360[10] = 0;
   out_5122475045724401360[11] = 0;
   out_5122475045724401360[12] = 1.0;
   out_5122475045724401360[13] = 0;
   out_5122475045724401360[14] = 0;
   out_5122475045724401360[15] = 0;
   out_5122475045724401360[16] = 0;
   out_5122475045724401360[17] = 0;
   out_5122475045724401360[18] = 0;
   out_5122475045724401360[19] = 0;
   out_5122475045724401360[20] = 0;
   out_5122475045724401360[21] = 0;
   out_5122475045724401360[22] = 0;
   out_5122475045724401360[23] = 0;
   out_5122475045724401360[24] = 1.0;
   out_5122475045724401360[25] = 0;
   out_5122475045724401360[26] = 0;
   out_5122475045724401360[27] = 0;
   out_5122475045724401360[28] = 0;
   out_5122475045724401360[29] = 0;
   out_5122475045724401360[30] = 0;
   out_5122475045724401360[31] = 0;
   out_5122475045724401360[32] = 0;
   out_5122475045724401360[33] = 0;
   out_5122475045724401360[34] = 0;
   out_5122475045724401360[35] = 0;
   out_5122475045724401360[36] = 1.0;
   out_5122475045724401360[37] = 0;
   out_5122475045724401360[38] = 0;
   out_5122475045724401360[39] = 0;
   out_5122475045724401360[40] = 0;
   out_5122475045724401360[41] = 0;
   out_5122475045724401360[42] = 0;
   out_5122475045724401360[43] = 0;
   out_5122475045724401360[44] = 0;
   out_5122475045724401360[45] = 0;
   out_5122475045724401360[46] = 0;
   out_5122475045724401360[47] = 0;
   out_5122475045724401360[48] = 1.0;
   out_5122475045724401360[49] = 0;
   out_5122475045724401360[50] = 0;
   out_5122475045724401360[51] = 0;
   out_5122475045724401360[52] = 0;
   out_5122475045724401360[53] = 0;
   out_5122475045724401360[54] = 0;
   out_5122475045724401360[55] = 0;
   out_5122475045724401360[56] = 0;
   out_5122475045724401360[57] = 0;
   out_5122475045724401360[58] = 0;
   out_5122475045724401360[59] = 0;
   out_5122475045724401360[60] = 1.0;
   out_5122475045724401360[61] = 0;
   out_5122475045724401360[62] = 0;
   out_5122475045724401360[63] = 0;
   out_5122475045724401360[64] = 0;
   out_5122475045724401360[65] = 0;
   out_5122475045724401360[66] = 0;
   out_5122475045724401360[67] = 0;
   out_5122475045724401360[68] = 0;
   out_5122475045724401360[69] = 0;
   out_5122475045724401360[70] = 0;
   out_5122475045724401360[71] = 0;
   out_5122475045724401360[72] = 1.0;
   out_5122475045724401360[73] = 0;
   out_5122475045724401360[74] = 0;
   out_5122475045724401360[75] = 0;
   out_5122475045724401360[76] = 0;
   out_5122475045724401360[77] = 0;
   out_5122475045724401360[78] = 0;
   out_5122475045724401360[79] = 0;
   out_5122475045724401360[80] = 0;
   out_5122475045724401360[81] = 0;
   out_5122475045724401360[82] = 0;
   out_5122475045724401360[83] = 0;
   out_5122475045724401360[84] = 1.0;
   out_5122475045724401360[85] = 0;
   out_5122475045724401360[86] = 0;
   out_5122475045724401360[87] = 0;
   out_5122475045724401360[88] = 0;
   out_5122475045724401360[89] = 0;
   out_5122475045724401360[90] = 0;
   out_5122475045724401360[91] = 0;
   out_5122475045724401360[92] = 0;
   out_5122475045724401360[93] = 0;
   out_5122475045724401360[94] = 0;
   out_5122475045724401360[95] = 0;
   out_5122475045724401360[96] = 1.0;
   out_5122475045724401360[97] = 0;
   out_5122475045724401360[98] = 0;
   out_5122475045724401360[99] = 0;
   out_5122475045724401360[100] = 0;
   out_5122475045724401360[101] = 0;
   out_5122475045724401360[102] = 0;
   out_5122475045724401360[103] = 0;
   out_5122475045724401360[104] = 0;
   out_5122475045724401360[105] = 0;
   out_5122475045724401360[106] = 0;
   out_5122475045724401360[107] = 0;
   out_5122475045724401360[108] = 1.0;
   out_5122475045724401360[109] = 0;
   out_5122475045724401360[110] = 0;
   out_5122475045724401360[111] = 0;
   out_5122475045724401360[112] = 0;
   out_5122475045724401360[113] = 0;
   out_5122475045724401360[114] = 0;
   out_5122475045724401360[115] = 0;
   out_5122475045724401360[116] = 0;
   out_5122475045724401360[117] = 0;
   out_5122475045724401360[118] = 0;
   out_5122475045724401360[119] = 0;
   out_5122475045724401360[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_8767392952015677118) {
   out_8767392952015677118[0] = dt*state[3] + state[0];
   out_8767392952015677118[1] = dt*state[4] + state[1];
   out_8767392952015677118[2] = dt*state[5] + state[2];
   out_8767392952015677118[3] = state[3];
   out_8767392952015677118[4] = state[4];
   out_8767392952015677118[5] = state[5];
   out_8767392952015677118[6] = dt*state[7] + state[6];
   out_8767392952015677118[7] = dt*state[8] + state[7];
   out_8767392952015677118[8] = state[8];
   out_8767392952015677118[9] = state[9];
   out_8767392952015677118[10] = state[10];
}
void F_fun(double *state, double dt, double *out_6519335429311450011) {
   out_6519335429311450011[0] = 1;
   out_6519335429311450011[1] = 0;
   out_6519335429311450011[2] = 0;
   out_6519335429311450011[3] = dt;
   out_6519335429311450011[4] = 0;
   out_6519335429311450011[5] = 0;
   out_6519335429311450011[6] = 0;
   out_6519335429311450011[7] = 0;
   out_6519335429311450011[8] = 0;
   out_6519335429311450011[9] = 0;
   out_6519335429311450011[10] = 0;
   out_6519335429311450011[11] = 0;
   out_6519335429311450011[12] = 1;
   out_6519335429311450011[13] = 0;
   out_6519335429311450011[14] = 0;
   out_6519335429311450011[15] = dt;
   out_6519335429311450011[16] = 0;
   out_6519335429311450011[17] = 0;
   out_6519335429311450011[18] = 0;
   out_6519335429311450011[19] = 0;
   out_6519335429311450011[20] = 0;
   out_6519335429311450011[21] = 0;
   out_6519335429311450011[22] = 0;
   out_6519335429311450011[23] = 0;
   out_6519335429311450011[24] = 1;
   out_6519335429311450011[25] = 0;
   out_6519335429311450011[26] = 0;
   out_6519335429311450011[27] = dt;
   out_6519335429311450011[28] = 0;
   out_6519335429311450011[29] = 0;
   out_6519335429311450011[30] = 0;
   out_6519335429311450011[31] = 0;
   out_6519335429311450011[32] = 0;
   out_6519335429311450011[33] = 0;
   out_6519335429311450011[34] = 0;
   out_6519335429311450011[35] = 0;
   out_6519335429311450011[36] = 1;
   out_6519335429311450011[37] = 0;
   out_6519335429311450011[38] = 0;
   out_6519335429311450011[39] = 0;
   out_6519335429311450011[40] = 0;
   out_6519335429311450011[41] = 0;
   out_6519335429311450011[42] = 0;
   out_6519335429311450011[43] = 0;
   out_6519335429311450011[44] = 0;
   out_6519335429311450011[45] = 0;
   out_6519335429311450011[46] = 0;
   out_6519335429311450011[47] = 0;
   out_6519335429311450011[48] = 1;
   out_6519335429311450011[49] = 0;
   out_6519335429311450011[50] = 0;
   out_6519335429311450011[51] = 0;
   out_6519335429311450011[52] = 0;
   out_6519335429311450011[53] = 0;
   out_6519335429311450011[54] = 0;
   out_6519335429311450011[55] = 0;
   out_6519335429311450011[56] = 0;
   out_6519335429311450011[57] = 0;
   out_6519335429311450011[58] = 0;
   out_6519335429311450011[59] = 0;
   out_6519335429311450011[60] = 1;
   out_6519335429311450011[61] = 0;
   out_6519335429311450011[62] = 0;
   out_6519335429311450011[63] = 0;
   out_6519335429311450011[64] = 0;
   out_6519335429311450011[65] = 0;
   out_6519335429311450011[66] = 0;
   out_6519335429311450011[67] = 0;
   out_6519335429311450011[68] = 0;
   out_6519335429311450011[69] = 0;
   out_6519335429311450011[70] = 0;
   out_6519335429311450011[71] = 0;
   out_6519335429311450011[72] = 1;
   out_6519335429311450011[73] = dt;
   out_6519335429311450011[74] = 0;
   out_6519335429311450011[75] = 0;
   out_6519335429311450011[76] = 0;
   out_6519335429311450011[77] = 0;
   out_6519335429311450011[78] = 0;
   out_6519335429311450011[79] = 0;
   out_6519335429311450011[80] = 0;
   out_6519335429311450011[81] = 0;
   out_6519335429311450011[82] = 0;
   out_6519335429311450011[83] = 0;
   out_6519335429311450011[84] = 1;
   out_6519335429311450011[85] = dt;
   out_6519335429311450011[86] = 0;
   out_6519335429311450011[87] = 0;
   out_6519335429311450011[88] = 0;
   out_6519335429311450011[89] = 0;
   out_6519335429311450011[90] = 0;
   out_6519335429311450011[91] = 0;
   out_6519335429311450011[92] = 0;
   out_6519335429311450011[93] = 0;
   out_6519335429311450011[94] = 0;
   out_6519335429311450011[95] = 0;
   out_6519335429311450011[96] = 1;
   out_6519335429311450011[97] = 0;
   out_6519335429311450011[98] = 0;
   out_6519335429311450011[99] = 0;
   out_6519335429311450011[100] = 0;
   out_6519335429311450011[101] = 0;
   out_6519335429311450011[102] = 0;
   out_6519335429311450011[103] = 0;
   out_6519335429311450011[104] = 0;
   out_6519335429311450011[105] = 0;
   out_6519335429311450011[106] = 0;
   out_6519335429311450011[107] = 0;
   out_6519335429311450011[108] = 1;
   out_6519335429311450011[109] = 0;
   out_6519335429311450011[110] = 0;
   out_6519335429311450011[111] = 0;
   out_6519335429311450011[112] = 0;
   out_6519335429311450011[113] = 0;
   out_6519335429311450011[114] = 0;
   out_6519335429311450011[115] = 0;
   out_6519335429311450011[116] = 0;
   out_6519335429311450011[117] = 0;
   out_6519335429311450011[118] = 0;
   out_6519335429311450011[119] = 0;
   out_6519335429311450011[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_8775600633747649685) {
   out_8775600633747649685[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_1847989328191366990) {
   out_1847989328191366990[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1847989328191366990[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1847989328191366990[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1847989328191366990[3] = 0;
   out_1847989328191366990[4] = 0;
   out_1847989328191366990[5] = 0;
   out_1847989328191366990[6] = 1;
   out_1847989328191366990[7] = 0;
   out_1847989328191366990[8] = 0;
   out_1847989328191366990[9] = 0;
   out_1847989328191366990[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_6848000075585344735) {
   out_6848000075585344735[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_4350122977225074540) {
   out_4350122977225074540[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4350122977225074540[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4350122977225074540[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4350122977225074540[3] = 0;
   out_4350122977225074540[4] = 0;
   out_4350122977225074540[5] = 0;
   out_4350122977225074540[6] = 1;
   out_4350122977225074540[7] = 0;
   out_4350122977225074540[8] = 0;
   out_4350122977225074540[9] = 1;
   out_4350122977225074540[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_3822185985007378568) {
   out_3822185985007378568[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_4973339453802444283) {
   out_4973339453802444283[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4973339453802444283[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4973339453802444283[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4973339453802444283[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4973339453802444283[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4973339453802444283[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4973339453802444283[6] = 0;
   out_4973339453802444283[7] = 1;
   out_4973339453802444283[8] = 0;
   out_4973339453802444283[9] = 0;
   out_4973339453802444283[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_3822185985007378568) {
   out_3822185985007378568[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_4973339453802444283) {
   out_4973339453802444283[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4973339453802444283[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4973339453802444283[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4973339453802444283[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4973339453802444283[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4973339453802444283[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4973339453802444283[6] = 0;
   out_4973339453802444283[7] = 1;
   out_4973339453802444283[8] = 0;
   out_4973339453802444283[9] = 0;
   out_4973339453802444283[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6982813324875031433) {
  err_fun(nom_x, delta_x, out_6982813324875031433);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_8822363379104221878) {
  inv_err_fun(nom_x, true_x, out_8822363379104221878);
}
void gnss_H_mod_fun(double *state, double *out_5122475045724401360) {
  H_mod_fun(state, out_5122475045724401360);
}
void gnss_f_fun(double *state, double dt, double *out_8767392952015677118) {
  f_fun(state,  dt, out_8767392952015677118);
}
void gnss_F_fun(double *state, double dt, double *out_6519335429311450011) {
  F_fun(state,  dt, out_6519335429311450011);
}
void gnss_h_6(double *state, double *sat_pos, double *out_8775600633747649685) {
  h_6(state, sat_pos, out_8775600633747649685);
}
void gnss_H_6(double *state, double *sat_pos, double *out_1847989328191366990) {
  H_6(state, sat_pos, out_1847989328191366990);
}
void gnss_h_20(double *state, double *sat_pos, double *out_6848000075585344735) {
  h_20(state, sat_pos, out_6848000075585344735);
}
void gnss_H_20(double *state, double *sat_pos, double *out_4350122977225074540) {
  H_20(state, sat_pos, out_4350122977225074540);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_3822185985007378568) {
  h_7(state, sat_pos_vel, out_3822185985007378568);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_4973339453802444283) {
  H_7(state, sat_pos_vel, out_4973339453802444283);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_3822185985007378568) {
  h_21(state, sat_pos_vel, out_3822185985007378568);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_4973339453802444283) {
  H_21(state, sat_pos_vel, out_4973339453802444283);
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
