#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_1078694211982077466);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9039723021082541259);
void car_H_mod_fun(double *state, double *out_3566115125440726837);
void car_f_fun(double *state, double dt, double *out_5563845995787045166);
void car_F_fun(double *state, double dt, double *out_5883604685601594109);
void car_h_25(double *state, double *unused, double *out_5225710209594518100);
void car_H_25(double *state, double *unused, double *out_2743426639728721431);
void car_h_24(double *state, double *unused, double *out_3821121116768031143);
void car_H_24(double *state, double *unused, double *out_6475252247911634960);
void car_h_30(double *state, double *unused, double *out_5500904271879023989);
void car_H_30(double *state, double *unused, double *out_1784269690398886767);
void car_h_26(double *state, double *unused, double *out_6444414124780686424);
void car_H_26(double *state, double *unused, double *out_998076679145334793);
void car_h_27(double *state, double *unused, double *out_1777052494198547679);
void car_H_27(double *state, double *unused, double *out_439324380785056450);
void car_h_29(double *state, double *unused, double *out_7944166534169681982);
void car_H_29(double *state, double *unused, double *out_1274038346084494583);
void car_h_28(double *state, double *unused, double *out_2863779142555012006);
void car_H_28(double *state, double *unused, double *out_6356437363154025157);
void car_h_31(double *state, double *unused, double *out_5328094369230510157);
void car_H_31(double *state, double *unused, double *out_1624284781378686269);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}