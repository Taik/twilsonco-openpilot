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
void err_fun(double *nom_x, double *delta_x, double *out_3686789247502404896) {
   out_3686789247502404896[0] = delta_x[0] + nom_x[0];
   out_3686789247502404896[1] = delta_x[1] + nom_x[1];
   out_3686789247502404896[2] = delta_x[2] + nom_x[2];
   out_3686789247502404896[3] = delta_x[3] + nom_x[3];
   out_3686789247502404896[4] = delta_x[4] + nom_x[4];
   out_3686789247502404896[5] = delta_x[5] + nom_x[5];
   out_3686789247502404896[6] = delta_x[6] + nom_x[6];
   out_3686789247502404896[7] = delta_x[7] + nom_x[7];
   out_3686789247502404896[8] = delta_x[8] + nom_x[8];
   out_3686789247502404896[9] = delta_x[9] + nom_x[9];
   out_3686789247502404896[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4567148213281321427) {
   out_4567148213281321427[0] = -nom_x[0] + true_x[0];
   out_4567148213281321427[1] = -nom_x[1] + true_x[1];
   out_4567148213281321427[2] = -nom_x[2] + true_x[2];
   out_4567148213281321427[3] = -nom_x[3] + true_x[3];
   out_4567148213281321427[4] = -nom_x[4] + true_x[4];
   out_4567148213281321427[5] = -nom_x[5] + true_x[5];
   out_4567148213281321427[6] = -nom_x[6] + true_x[6];
   out_4567148213281321427[7] = -nom_x[7] + true_x[7];
   out_4567148213281321427[8] = -nom_x[8] + true_x[8];
   out_4567148213281321427[9] = -nom_x[9] + true_x[9];
   out_4567148213281321427[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_6175002328322028331) {
   out_6175002328322028331[0] = 1.0;
   out_6175002328322028331[1] = 0;
   out_6175002328322028331[2] = 0;
   out_6175002328322028331[3] = 0;
   out_6175002328322028331[4] = 0;
   out_6175002328322028331[5] = 0;
   out_6175002328322028331[6] = 0;
   out_6175002328322028331[7] = 0;
   out_6175002328322028331[8] = 0;
   out_6175002328322028331[9] = 0;
   out_6175002328322028331[10] = 0;
   out_6175002328322028331[11] = 0;
   out_6175002328322028331[12] = 1.0;
   out_6175002328322028331[13] = 0;
   out_6175002328322028331[14] = 0;
   out_6175002328322028331[15] = 0;
   out_6175002328322028331[16] = 0;
   out_6175002328322028331[17] = 0;
   out_6175002328322028331[18] = 0;
   out_6175002328322028331[19] = 0;
   out_6175002328322028331[20] = 0;
   out_6175002328322028331[21] = 0;
   out_6175002328322028331[22] = 0;
   out_6175002328322028331[23] = 0;
   out_6175002328322028331[24] = 1.0;
   out_6175002328322028331[25] = 0;
   out_6175002328322028331[26] = 0;
   out_6175002328322028331[27] = 0;
   out_6175002328322028331[28] = 0;
   out_6175002328322028331[29] = 0;
   out_6175002328322028331[30] = 0;
   out_6175002328322028331[31] = 0;
   out_6175002328322028331[32] = 0;
   out_6175002328322028331[33] = 0;
   out_6175002328322028331[34] = 0;
   out_6175002328322028331[35] = 0;
   out_6175002328322028331[36] = 1.0;
   out_6175002328322028331[37] = 0;
   out_6175002328322028331[38] = 0;
   out_6175002328322028331[39] = 0;
   out_6175002328322028331[40] = 0;
   out_6175002328322028331[41] = 0;
   out_6175002328322028331[42] = 0;
   out_6175002328322028331[43] = 0;
   out_6175002328322028331[44] = 0;
   out_6175002328322028331[45] = 0;
   out_6175002328322028331[46] = 0;
   out_6175002328322028331[47] = 0;
   out_6175002328322028331[48] = 1.0;
   out_6175002328322028331[49] = 0;
   out_6175002328322028331[50] = 0;
   out_6175002328322028331[51] = 0;
   out_6175002328322028331[52] = 0;
   out_6175002328322028331[53] = 0;
   out_6175002328322028331[54] = 0;
   out_6175002328322028331[55] = 0;
   out_6175002328322028331[56] = 0;
   out_6175002328322028331[57] = 0;
   out_6175002328322028331[58] = 0;
   out_6175002328322028331[59] = 0;
   out_6175002328322028331[60] = 1.0;
   out_6175002328322028331[61] = 0;
   out_6175002328322028331[62] = 0;
   out_6175002328322028331[63] = 0;
   out_6175002328322028331[64] = 0;
   out_6175002328322028331[65] = 0;
   out_6175002328322028331[66] = 0;
   out_6175002328322028331[67] = 0;
   out_6175002328322028331[68] = 0;
   out_6175002328322028331[69] = 0;
   out_6175002328322028331[70] = 0;
   out_6175002328322028331[71] = 0;
   out_6175002328322028331[72] = 1.0;
   out_6175002328322028331[73] = 0;
   out_6175002328322028331[74] = 0;
   out_6175002328322028331[75] = 0;
   out_6175002328322028331[76] = 0;
   out_6175002328322028331[77] = 0;
   out_6175002328322028331[78] = 0;
   out_6175002328322028331[79] = 0;
   out_6175002328322028331[80] = 0;
   out_6175002328322028331[81] = 0;
   out_6175002328322028331[82] = 0;
   out_6175002328322028331[83] = 0;
   out_6175002328322028331[84] = 1.0;
   out_6175002328322028331[85] = 0;
   out_6175002328322028331[86] = 0;
   out_6175002328322028331[87] = 0;
   out_6175002328322028331[88] = 0;
   out_6175002328322028331[89] = 0;
   out_6175002328322028331[90] = 0;
   out_6175002328322028331[91] = 0;
   out_6175002328322028331[92] = 0;
   out_6175002328322028331[93] = 0;
   out_6175002328322028331[94] = 0;
   out_6175002328322028331[95] = 0;
   out_6175002328322028331[96] = 1.0;
   out_6175002328322028331[97] = 0;
   out_6175002328322028331[98] = 0;
   out_6175002328322028331[99] = 0;
   out_6175002328322028331[100] = 0;
   out_6175002328322028331[101] = 0;
   out_6175002328322028331[102] = 0;
   out_6175002328322028331[103] = 0;
   out_6175002328322028331[104] = 0;
   out_6175002328322028331[105] = 0;
   out_6175002328322028331[106] = 0;
   out_6175002328322028331[107] = 0;
   out_6175002328322028331[108] = 1.0;
   out_6175002328322028331[109] = 0;
   out_6175002328322028331[110] = 0;
   out_6175002328322028331[111] = 0;
   out_6175002328322028331[112] = 0;
   out_6175002328322028331[113] = 0;
   out_6175002328322028331[114] = 0;
   out_6175002328322028331[115] = 0;
   out_6175002328322028331[116] = 0;
   out_6175002328322028331[117] = 0;
   out_6175002328322028331[118] = 0;
   out_6175002328322028331[119] = 0;
   out_6175002328322028331[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_4336716201713451957) {
   out_4336716201713451957[0] = dt*state[3] + state[0];
   out_4336716201713451957[1] = dt*state[4] + state[1];
   out_4336716201713451957[2] = dt*state[5] + state[2];
   out_4336716201713451957[3] = state[3];
   out_4336716201713451957[4] = state[4];
   out_4336716201713451957[5] = state[5];
   out_4336716201713451957[6] = dt*state[7] + state[6];
   out_4336716201713451957[7] = dt*state[8] + state[7];
   out_4336716201713451957[8] = state[8];
   out_4336716201713451957[9] = state[9];
   out_4336716201713451957[10] = state[10];
}
void F_fun(double *state, double dt, double *out_6104947861706444775) {
   out_6104947861706444775[0] = 1;
   out_6104947861706444775[1] = 0;
   out_6104947861706444775[2] = 0;
   out_6104947861706444775[3] = dt;
   out_6104947861706444775[4] = 0;
   out_6104947861706444775[5] = 0;
   out_6104947861706444775[6] = 0;
   out_6104947861706444775[7] = 0;
   out_6104947861706444775[8] = 0;
   out_6104947861706444775[9] = 0;
   out_6104947861706444775[10] = 0;
   out_6104947861706444775[11] = 0;
   out_6104947861706444775[12] = 1;
   out_6104947861706444775[13] = 0;
   out_6104947861706444775[14] = 0;
   out_6104947861706444775[15] = dt;
   out_6104947861706444775[16] = 0;
   out_6104947861706444775[17] = 0;
   out_6104947861706444775[18] = 0;
   out_6104947861706444775[19] = 0;
   out_6104947861706444775[20] = 0;
   out_6104947861706444775[21] = 0;
   out_6104947861706444775[22] = 0;
   out_6104947861706444775[23] = 0;
   out_6104947861706444775[24] = 1;
   out_6104947861706444775[25] = 0;
   out_6104947861706444775[26] = 0;
   out_6104947861706444775[27] = dt;
   out_6104947861706444775[28] = 0;
   out_6104947861706444775[29] = 0;
   out_6104947861706444775[30] = 0;
   out_6104947861706444775[31] = 0;
   out_6104947861706444775[32] = 0;
   out_6104947861706444775[33] = 0;
   out_6104947861706444775[34] = 0;
   out_6104947861706444775[35] = 0;
   out_6104947861706444775[36] = 1;
   out_6104947861706444775[37] = 0;
   out_6104947861706444775[38] = 0;
   out_6104947861706444775[39] = 0;
   out_6104947861706444775[40] = 0;
   out_6104947861706444775[41] = 0;
   out_6104947861706444775[42] = 0;
   out_6104947861706444775[43] = 0;
   out_6104947861706444775[44] = 0;
   out_6104947861706444775[45] = 0;
   out_6104947861706444775[46] = 0;
   out_6104947861706444775[47] = 0;
   out_6104947861706444775[48] = 1;
   out_6104947861706444775[49] = 0;
   out_6104947861706444775[50] = 0;
   out_6104947861706444775[51] = 0;
   out_6104947861706444775[52] = 0;
   out_6104947861706444775[53] = 0;
   out_6104947861706444775[54] = 0;
   out_6104947861706444775[55] = 0;
   out_6104947861706444775[56] = 0;
   out_6104947861706444775[57] = 0;
   out_6104947861706444775[58] = 0;
   out_6104947861706444775[59] = 0;
   out_6104947861706444775[60] = 1;
   out_6104947861706444775[61] = 0;
   out_6104947861706444775[62] = 0;
   out_6104947861706444775[63] = 0;
   out_6104947861706444775[64] = 0;
   out_6104947861706444775[65] = 0;
   out_6104947861706444775[66] = 0;
   out_6104947861706444775[67] = 0;
   out_6104947861706444775[68] = 0;
   out_6104947861706444775[69] = 0;
   out_6104947861706444775[70] = 0;
   out_6104947861706444775[71] = 0;
   out_6104947861706444775[72] = 1;
   out_6104947861706444775[73] = dt;
   out_6104947861706444775[74] = 0;
   out_6104947861706444775[75] = 0;
   out_6104947861706444775[76] = 0;
   out_6104947861706444775[77] = 0;
   out_6104947861706444775[78] = 0;
   out_6104947861706444775[79] = 0;
   out_6104947861706444775[80] = 0;
   out_6104947861706444775[81] = 0;
   out_6104947861706444775[82] = 0;
   out_6104947861706444775[83] = 0;
   out_6104947861706444775[84] = 1;
   out_6104947861706444775[85] = dt;
   out_6104947861706444775[86] = 0;
   out_6104947861706444775[87] = 0;
   out_6104947861706444775[88] = 0;
   out_6104947861706444775[89] = 0;
   out_6104947861706444775[90] = 0;
   out_6104947861706444775[91] = 0;
   out_6104947861706444775[92] = 0;
   out_6104947861706444775[93] = 0;
   out_6104947861706444775[94] = 0;
   out_6104947861706444775[95] = 0;
   out_6104947861706444775[96] = 1;
   out_6104947861706444775[97] = 0;
   out_6104947861706444775[98] = 0;
   out_6104947861706444775[99] = 0;
   out_6104947861706444775[100] = 0;
   out_6104947861706444775[101] = 0;
   out_6104947861706444775[102] = 0;
   out_6104947861706444775[103] = 0;
   out_6104947861706444775[104] = 0;
   out_6104947861706444775[105] = 0;
   out_6104947861706444775[106] = 0;
   out_6104947861706444775[107] = 0;
   out_6104947861706444775[108] = 1;
   out_6104947861706444775[109] = 0;
   out_6104947861706444775[110] = 0;
   out_6104947861706444775[111] = 0;
   out_6104947861706444775[112] = 0;
   out_6104947861706444775[113] = 0;
   out_6104947861706444775[114] = 0;
   out_6104947861706444775[115] = 0;
   out_6104947861706444775[116] = 0;
   out_6104947861706444775[117] = 0;
   out_6104947861706444775[118] = 0;
   out_6104947861706444775[119] = 0;
   out_6104947861706444775[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_1150344248106113082) {
   out_1150344248106113082[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_394388827836502853) {
   out_394388827836502853[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_394388827836502853[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_394388827836502853[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_394388827836502853[3] = 0;
   out_394388827836502853[4] = 0;
   out_394388827836502853[5] = 0;
   out_394388827836502853[6] = 1;
   out_394388827836502853[7] = 0;
   out_394388827836502853[8] = 0;
   out_394388827836502853[9] = 0;
   out_394388827836502853[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_2337833637223709483) {
   out_2337833637223709483[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_7352886989259104254) {
   out_7352886989259104254[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7352886989259104254[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7352886989259104254[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7352886989259104254[3] = 0;
   out_7352886989259104254[4] = 0;
   out_7352886989259104254[5] = 0;
   out_7352886989259104254[6] = 1;
   out_7352886989259104254[7] = 0;
   out_7352886989259104254[8] = 0;
   out_7352886989259104254[9] = 1;
   out_7352886989259104254[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_902647274384620902) {
   out_902647274384620902[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_185624918626144292) {
   out_185624918626144292[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_185624918626144292[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_185624918626144292[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_185624918626144292[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_185624918626144292[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_185624918626144292[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_185624918626144292[6] = 0;
   out_185624918626144292[7] = 1;
   out_185624918626144292[8] = 0;
   out_185624918626144292[9] = 0;
   out_185624918626144292[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_902647274384620902) {
   out_902647274384620902[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_185624918626144292) {
   out_185624918626144292[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_185624918626144292[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_185624918626144292[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_185624918626144292[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_185624918626144292[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_185624918626144292[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_185624918626144292[6] = 0;
   out_185624918626144292[7] = 1;
   out_185624918626144292[8] = 0;
   out_185624918626144292[9] = 0;
   out_185624918626144292[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_3686789247502404896) {
  err_fun(nom_x, delta_x, out_3686789247502404896);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_4567148213281321427) {
  inv_err_fun(nom_x, true_x, out_4567148213281321427);
}
void gnss_H_mod_fun(double *state, double *out_6175002328322028331) {
  H_mod_fun(state, out_6175002328322028331);
}
void gnss_f_fun(double *state, double dt, double *out_4336716201713451957) {
  f_fun(state,  dt, out_4336716201713451957);
}
void gnss_F_fun(double *state, double dt, double *out_6104947861706444775) {
  F_fun(state,  dt, out_6104947861706444775);
}
void gnss_h_6(double *state, double *sat_pos, double *out_1150344248106113082) {
  h_6(state, sat_pos, out_1150344248106113082);
}
void gnss_H_6(double *state, double *sat_pos, double *out_394388827836502853) {
  H_6(state, sat_pos, out_394388827836502853);
}
void gnss_h_20(double *state, double *sat_pos, double *out_2337833637223709483) {
  h_20(state, sat_pos, out_2337833637223709483);
}
void gnss_H_20(double *state, double *sat_pos, double *out_7352886989259104254) {
  H_20(state, sat_pos, out_7352886989259104254);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_902647274384620902) {
  h_7(state, sat_pos_vel, out_902647274384620902);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_185624918626144292) {
  H_7(state, sat_pos_vel, out_185624918626144292);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_902647274384620902) {
  h_21(state, sat_pos_vel, out_902647274384620902);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_185624918626144292) {
  H_21(state, sat_pos_vel, out_185624918626144292);
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
