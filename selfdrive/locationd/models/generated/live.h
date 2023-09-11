#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_1197758814798533388);
void live_err_fun(double *nom_x, double *delta_x, double *out_4286215774710425169);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_3781385446178574208);
void live_H_mod_fun(double *state, double *out_5816692744659529436);
void live_f_fun(double *state, double dt, double *out_5690960620373720106);
void live_F_fun(double *state, double dt, double *out_1708363142813755465);
void live_h_4(double *state, double *unused, double *out_1528163716891991779);
void live_H_4(double *state, double *unused, double *out_6451929225734141090);
void live_h_9(double *state, double *unused, double *out_5075190892892801008);
void live_H_9(double *state, double *unused, double *out_835289709530306380);
void live_h_10(double *state, double *unused, double *out_842381642263115200);
void live_H_10(double *state, double *unused, double *out_5010370349131368794);
void live_h_12(double *state, double *unused, double *out_5441043766607214819);
void live_H_12(double *state, double *unused, double *out_5613556470932677530);
void live_h_35(double *state, double *unused, double *out_1872850465453208056);
void live_H_35(double *state, double *unused, double *out_1313090214622834414);
void live_h_32(double *state, double *unused, double *out_6110767287042918546);
void live_H_32(double *state, double *unused, double *out_1573391208225620082);
void live_h_13(double *state, double *unused, double *out_549733664283291714);
void live_H_13(double *state, double *unused, double *out_1107750194629656687);
void live_h_14(double *state, double *unused, double *out_5075190892892801008);
void live_H_14(double *state, double *unused, double *out_835289709530306380);
void live_h_33(double *state, double *unused, double *out_1123921101808459626);
void live_H_33(double *state, double *unused, double *out_6937067565813002773);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}