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
void car_err_fun(double *nom_x, double *delta_x, double *out_2939202982062152065);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1376135891411693707);
void car_H_mod_fun(double *state, double *out_549565208155142469);
void car_f_fun(double *state, double dt, double *out_746317309293841027);
void car_F_fun(double *state, double dt, double *out_7394503540686598109);
void car_h_25(double *state, double *unused, double *out_2154740370469115095);
void car_H_25(double *state, double *unused, double *out_5133228550106326038);
void car_h_24(double *state, double *unused, double *out_753491134740954591);
void car_H_24(double *state, double *unused, double *out_5895123889926298685);
void car_h_30(double *state, double *unused, double *out_1879546308184609206);
void car_H_30(double *state, double *unused, double *out_6396825182111608823);
void car_h_26(double *state, double *unused, double *out_6881219575845454851);
void car_H_26(double *state, double *unused, double *out_1391725231232269814);
void car_h_27(double *state, double *unused, double *out_3098250223370777530);
void car_H_27(double *state, double *unused, double *out_8571588493912033734);
void car_h_29(double *state, double *unused, double *out_5295819597833679222);
void car_H_29(double *state, double *unused, double *out_5886593837797216639);
void car_h_28(double *state, double *unused, double *out_307271827440529977);
void car_H_28(double *state, double *unused, double *out_3079393835858436275);
void car_h_31(double *state, double *unused, double *out_1997553702117985950);
void car_H_31(double *state, double *unused, double *out_5163874511983286466);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}