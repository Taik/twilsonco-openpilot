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
void live_H(double *in_vec, double *out_8035984677720566691);
void live_err_fun(double *nom_x, double *delta_x, double *out_8420573368845911328);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6777199985856691982);
void live_H_mod_fun(double *state, double *out_7266160212655929723);
void live_f_fun(double *state, double dt, double *out_685833055220742968);
void live_F_fun(double *state, double dt, double *out_6925511053866020455);
void live_h_4(double *state, double *unused, double *out_6466845048239706322);
void live_H_4(double *state, double *unused, double *out_407181918403328470);
void live_h_9(double *state, double *unused, double *out_749653915449883533);
void live_H_9(double *state, double *unused, double *out_648371565032919115);
void live_h_10(double *state, double *unused, double *out_8534172667793870162);
void live_H_10(double *state, double *unused, double *out_4975612423287271250);
void live_h_12(double *state, double *unused, double *out_6515126630711997761);
void live_H_12(double *state, double *unused, double *out_5426638326435290265);
void live_h_35(double *state, double *unused, double *out_7782822813653033061);
void live_H_35(double *state, double *unused, double *out_3773843975775935846);
void live_h_32(double *state, double *unused, double *out_108926050794950082);
void live_H_32(double *state, double *unused, double *out_3141020527918180510);
void live_h_13(double *state, double *unused, double *out_2871024965946799999);
void live_H_13(double *state, double *unused, double *out_5162091226028516826);
void live_h_14(double *state, double *unused, double *out_749653915449883533);
void live_H_14(double *state, double *unused, double *out_648371565032919115);
void live_h_33(double *state, double *unused, double *out_6040282944703340401);
void live_H_33(double *state, double *unused, double *out_6924400980414793450);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}