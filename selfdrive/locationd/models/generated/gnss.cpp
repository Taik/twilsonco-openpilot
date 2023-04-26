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
void err_fun(double *nom_x, double *delta_x, double *out_8719033105959425466) {
   out_8719033105959425466[0] = delta_x[0] + nom_x[0];
   out_8719033105959425466[1] = delta_x[1] + nom_x[1];
   out_8719033105959425466[2] = delta_x[2] + nom_x[2];
   out_8719033105959425466[3] = delta_x[3] + nom_x[3];
   out_8719033105959425466[4] = delta_x[4] + nom_x[4];
   out_8719033105959425466[5] = delta_x[5] + nom_x[5];
   out_8719033105959425466[6] = delta_x[6] + nom_x[6];
   out_8719033105959425466[7] = delta_x[7] + nom_x[7];
   out_8719033105959425466[8] = delta_x[8] + nom_x[8];
   out_8719033105959425466[9] = delta_x[9] + nom_x[9];
   out_8719033105959425466[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7604825264931021996) {
   out_7604825264931021996[0] = -nom_x[0] + true_x[0];
   out_7604825264931021996[1] = -nom_x[1] + true_x[1];
   out_7604825264931021996[2] = -nom_x[2] + true_x[2];
   out_7604825264931021996[3] = -nom_x[3] + true_x[3];
   out_7604825264931021996[4] = -nom_x[4] + true_x[4];
   out_7604825264931021996[5] = -nom_x[5] + true_x[5];
   out_7604825264931021996[6] = -nom_x[6] + true_x[6];
   out_7604825264931021996[7] = -nom_x[7] + true_x[7];
   out_7604825264931021996[8] = -nom_x[8] + true_x[8];
   out_7604825264931021996[9] = -nom_x[9] + true_x[9];
   out_7604825264931021996[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_51336901519045214) {
   out_51336901519045214[0] = 1.0;
   out_51336901519045214[1] = 0;
   out_51336901519045214[2] = 0;
   out_51336901519045214[3] = 0;
   out_51336901519045214[4] = 0;
   out_51336901519045214[5] = 0;
   out_51336901519045214[6] = 0;
   out_51336901519045214[7] = 0;
   out_51336901519045214[8] = 0;
   out_51336901519045214[9] = 0;
   out_51336901519045214[10] = 0;
   out_51336901519045214[11] = 0;
   out_51336901519045214[12] = 1.0;
   out_51336901519045214[13] = 0;
   out_51336901519045214[14] = 0;
   out_51336901519045214[15] = 0;
   out_51336901519045214[16] = 0;
   out_51336901519045214[17] = 0;
   out_51336901519045214[18] = 0;
   out_51336901519045214[19] = 0;
   out_51336901519045214[20] = 0;
   out_51336901519045214[21] = 0;
   out_51336901519045214[22] = 0;
   out_51336901519045214[23] = 0;
   out_51336901519045214[24] = 1.0;
   out_51336901519045214[25] = 0;
   out_51336901519045214[26] = 0;
   out_51336901519045214[27] = 0;
   out_51336901519045214[28] = 0;
   out_51336901519045214[29] = 0;
   out_51336901519045214[30] = 0;
   out_51336901519045214[31] = 0;
   out_51336901519045214[32] = 0;
   out_51336901519045214[33] = 0;
   out_51336901519045214[34] = 0;
   out_51336901519045214[35] = 0;
   out_51336901519045214[36] = 1.0;
   out_51336901519045214[37] = 0;
   out_51336901519045214[38] = 0;
   out_51336901519045214[39] = 0;
   out_51336901519045214[40] = 0;
   out_51336901519045214[41] = 0;
   out_51336901519045214[42] = 0;
   out_51336901519045214[43] = 0;
   out_51336901519045214[44] = 0;
   out_51336901519045214[45] = 0;
   out_51336901519045214[46] = 0;
   out_51336901519045214[47] = 0;
   out_51336901519045214[48] = 1.0;
   out_51336901519045214[49] = 0;
   out_51336901519045214[50] = 0;
   out_51336901519045214[51] = 0;
   out_51336901519045214[52] = 0;
   out_51336901519045214[53] = 0;
   out_51336901519045214[54] = 0;
   out_51336901519045214[55] = 0;
   out_51336901519045214[56] = 0;
   out_51336901519045214[57] = 0;
   out_51336901519045214[58] = 0;
   out_51336901519045214[59] = 0;
   out_51336901519045214[60] = 1.0;
   out_51336901519045214[61] = 0;
   out_51336901519045214[62] = 0;
   out_51336901519045214[63] = 0;
   out_51336901519045214[64] = 0;
   out_51336901519045214[65] = 0;
   out_51336901519045214[66] = 0;
   out_51336901519045214[67] = 0;
   out_51336901519045214[68] = 0;
   out_51336901519045214[69] = 0;
   out_51336901519045214[70] = 0;
   out_51336901519045214[71] = 0;
   out_51336901519045214[72] = 1.0;
   out_51336901519045214[73] = 0;
   out_51336901519045214[74] = 0;
   out_51336901519045214[75] = 0;
   out_51336901519045214[76] = 0;
   out_51336901519045214[77] = 0;
   out_51336901519045214[78] = 0;
   out_51336901519045214[79] = 0;
   out_51336901519045214[80] = 0;
   out_51336901519045214[81] = 0;
   out_51336901519045214[82] = 0;
   out_51336901519045214[83] = 0;
   out_51336901519045214[84] = 1.0;
   out_51336901519045214[85] = 0;
   out_51336901519045214[86] = 0;
   out_51336901519045214[87] = 0;
   out_51336901519045214[88] = 0;
   out_51336901519045214[89] = 0;
   out_51336901519045214[90] = 0;
   out_51336901519045214[91] = 0;
   out_51336901519045214[92] = 0;
   out_51336901519045214[93] = 0;
   out_51336901519045214[94] = 0;
   out_51336901519045214[95] = 0;
   out_51336901519045214[96] = 1.0;
   out_51336901519045214[97] = 0;
   out_51336901519045214[98] = 0;
   out_51336901519045214[99] = 0;
   out_51336901519045214[100] = 0;
   out_51336901519045214[101] = 0;
   out_51336901519045214[102] = 0;
   out_51336901519045214[103] = 0;
   out_51336901519045214[104] = 0;
   out_51336901519045214[105] = 0;
   out_51336901519045214[106] = 0;
   out_51336901519045214[107] = 0;
   out_51336901519045214[108] = 1.0;
   out_51336901519045214[109] = 0;
   out_51336901519045214[110] = 0;
   out_51336901519045214[111] = 0;
   out_51336901519045214[112] = 0;
   out_51336901519045214[113] = 0;
   out_51336901519045214[114] = 0;
   out_51336901519045214[115] = 0;
   out_51336901519045214[116] = 0;
   out_51336901519045214[117] = 0;
   out_51336901519045214[118] = 0;
   out_51336901519045214[119] = 0;
   out_51336901519045214[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_1334164227221272553) {
   out_1334164227221272553[0] = dt*state[3] + state[0];
   out_1334164227221272553[1] = dt*state[4] + state[1];
   out_1334164227221272553[2] = dt*state[5] + state[2];
   out_1334164227221272553[3] = state[3];
   out_1334164227221272553[4] = state[4];
   out_1334164227221272553[5] = state[5];
   out_1334164227221272553[6] = dt*state[7] + state[6];
   out_1334164227221272553[7] = dt*state[8] + state[7];
   out_1334164227221272553[8] = state[8];
   out_1334164227221272553[9] = state[9];
   out_1334164227221272553[10] = state[10];
}
void F_fun(double *state, double dt, double *out_522197488900160010) {
   out_522197488900160010[0] = 1;
   out_522197488900160010[1] = 0;
   out_522197488900160010[2] = 0;
   out_522197488900160010[3] = dt;
   out_522197488900160010[4] = 0;
   out_522197488900160010[5] = 0;
   out_522197488900160010[6] = 0;
   out_522197488900160010[7] = 0;
   out_522197488900160010[8] = 0;
   out_522197488900160010[9] = 0;
   out_522197488900160010[10] = 0;
   out_522197488900160010[11] = 0;
   out_522197488900160010[12] = 1;
   out_522197488900160010[13] = 0;
   out_522197488900160010[14] = 0;
   out_522197488900160010[15] = dt;
   out_522197488900160010[16] = 0;
   out_522197488900160010[17] = 0;
   out_522197488900160010[18] = 0;
   out_522197488900160010[19] = 0;
   out_522197488900160010[20] = 0;
   out_522197488900160010[21] = 0;
   out_522197488900160010[22] = 0;
   out_522197488900160010[23] = 0;
   out_522197488900160010[24] = 1;
   out_522197488900160010[25] = 0;
   out_522197488900160010[26] = 0;
   out_522197488900160010[27] = dt;
   out_522197488900160010[28] = 0;
   out_522197488900160010[29] = 0;
   out_522197488900160010[30] = 0;
   out_522197488900160010[31] = 0;
   out_522197488900160010[32] = 0;
   out_522197488900160010[33] = 0;
   out_522197488900160010[34] = 0;
   out_522197488900160010[35] = 0;
   out_522197488900160010[36] = 1;
   out_522197488900160010[37] = 0;
   out_522197488900160010[38] = 0;
   out_522197488900160010[39] = 0;
   out_522197488900160010[40] = 0;
   out_522197488900160010[41] = 0;
   out_522197488900160010[42] = 0;
   out_522197488900160010[43] = 0;
   out_522197488900160010[44] = 0;
   out_522197488900160010[45] = 0;
   out_522197488900160010[46] = 0;
   out_522197488900160010[47] = 0;
   out_522197488900160010[48] = 1;
   out_522197488900160010[49] = 0;
   out_522197488900160010[50] = 0;
   out_522197488900160010[51] = 0;
   out_522197488900160010[52] = 0;
   out_522197488900160010[53] = 0;
   out_522197488900160010[54] = 0;
   out_522197488900160010[55] = 0;
   out_522197488900160010[56] = 0;
   out_522197488900160010[57] = 0;
   out_522197488900160010[58] = 0;
   out_522197488900160010[59] = 0;
   out_522197488900160010[60] = 1;
   out_522197488900160010[61] = 0;
   out_522197488900160010[62] = 0;
   out_522197488900160010[63] = 0;
   out_522197488900160010[64] = 0;
   out_522197488900160010[65] = 0;
   out_522197488900160010[66] = 0;
   out_522197488900160010[67] = 0;
   out_522197488900160010[68] = 0;
   out_522197488900160010[69] = 0;
   out_522197488900160010[70] = 0;
   out_522197488900160010[71] = 0;
   out_522197488900160010[72] = 1;
   out_522197488900160010[73] = dt;
   out_522197488900160010[74] = 0;
   out_522197488900160010[75] = 0;
   out_522197488900160010[76] = 0;
   out_522197488900160010[77] = 0;
   out_522197488900160010[78] = 0;
   out_522197488900160010[79] = 0;
   out_522197488900160010[80] = 0;
   out_522197488900160010[81] = 0;
   out_522197488900160010[82] = 0;
   out_522197488900160010[83] = 0;
   out_522197488900160010[84] = 1;
   out_522197488900160010[85] = dt;
   out_522197488900160010[86] = 0;
   out_522197488900160010[87] = 0;
   out_522197488900160010[88] = 0;
   out_522197488900160010[89] = 0;
   out_522197488900160010[90] = 0;
   out_522197488900160010[91] = 0;
   out_522197488900160010[92] = 0;
   out_522197488900160010[93] = 0;
   out_522197488900160010[94] = 0;
   out_522197488900160010[95] = 0;
   out_522197488900160010[96] = 1;
   out_522197488900160010[97] = 0;
   out_522197488900160010[98] = 0;
   out_522197488900160010[99] = 0;
   out_522197488900160010[100] = 0;
   out_522197488900160010[101] = 0;
   out_522197488900160010[102] = 0;
   out_522197488900160010[103] = 0;
   out_522197488900160010[104] = 0;
   out_522197488900160010[105] = 0;
   out_522197488900160010[106] = 0;
   out_522197488900160010[107] = 0;
   out_522197488900160010[108] = 1;
   out_522197488900160010[109] = 0;
   out_522197488900160010[110] = 0;
   out_522197488900160010[111] = 0;
   out_522197488900160010[112] = 0;
   out_522197488900160010[113] = 0;
   out_522197488900160010[114] = 0;
   out_522197488900160010[115] = 0;
   out_522197488900160010[116] = 0;
   out_522197488900160010[117] = 0;
   out_522197488900160010[118] = 0;
   out_522197488900160010[119] = 0;
   out_522197488900160010[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_268232154873960878) {
   out_268232154873960878[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_1693479458403604436) {
   out_1693479458403604436[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1693479458403604436[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1693479458403604436[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1693479458403604436[3] = 0;
   out_1693479458403604436[4] = 0;
   out_1693479458403604436[5] = 0;
   out_1693479458403604436[6] = 1;
   out_1693479458403604436[7] = 0;
   out_1693479458403604436[8] = 0;
   out_1693479458403604436[9] = 0;
   out_1693479458403604436[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_6070021782984203072) {
   out_6070021782984203072[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_5552429774338222019) {
   out_5552429774338222019[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5552429774338222019[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5552429774338222019[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5552429774338222019[3] = 0;
   out_5552429774338222019[4] = 0;
   out_5552429774338222019[5] = 0;
   out_5552429774338222019[6] = 1;
   out_5552429774338222019[7] = 0;
   out_5552429774338222019[8] = 0;
   out_5552429774338222019[9] = 1;
   out_5552429774338222019[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_310073623623055319) {
   out_310073623623055319[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_3027623563310936522) {
   out_3027623563310936522[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3027623563310936522[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3027623563310936522[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3027623563310936522[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3027623563310936522[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3027623563310936522[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3027623563310936522[6] = 0;
   out_3027623563310936522[7] = 1;
   out_3027623563310936522[8] = 0;
   out_3027623563310936522[9] = 0;
   out_3027623563310936522[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_310073623623055319) {
   out_310073623623055319[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_3027623563310936522) {
   out_3027623563310936522[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3027623563310936522[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3027623563310936522[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3027623563310936522[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3027623563310936522[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3027623563310936522[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3027623563310936522[6] = 0;
   out_3027623563310936522[7] = 1;
   out_3027623563310936522[8] = 0;
   out_3027623563310936522[9] = 0;
   out_3027623563310936522[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_8719033105959425466) {
  err_fun(nom_x, delta_x, out_8719033105959425466);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_7604825264931021996) {
  inv_err_fun(nom_x, true_x, out_7604825264931021996);
}
void gnss_H_mod_fun(double *state, double *out_51336901519045214) {
  H_mod_fun(state, out_51336901519045214);
}
void gnss_f_fun(double *state, double dt, double *out_1334164227221272553) {
  f_fun(state,  dt, out_1334164227221272553);
}
void gnss_F_fun(double *state, double dt, double *out_522197488900160010) {
  F_fun(state,  dt, out_522197488900160010);
}
void gnss_h_6(double *state, double *sat_pos, double *out_268232154873960878) {
  h_6(state, sat_pos, out_268232154873960878);
}
void gnss_H_6(double *state, double *sat_pos, double *out_1693479458403604436) {
  H_6(state, sat_pos, out_1693479458403604436);
}
void gnss_h_20(double *state, double *sat_pos, double *out_6070021782984203072) {
  h_20(state, sat_pos, out_6070021782984203072);
}
void gnss_H_20(double *state, double *sat_pos, double *out_5552429774338222019) {
  H_20(state, sat_pos, out_5552429774338222019);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_310073623623055319) {
  h_7(state, sat_pos_vel, out_310073623623055319);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_3027623563310936522) {
  H_7(state, sat_pos_vel, out_3027623563310936522);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_310073623623055319) {
  h_21(state, sat_pos_vel, out_310073623623055319);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_3027623563310936522) {
  H_21(state, sat_pos_vel, out_3027623563310936522);
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
