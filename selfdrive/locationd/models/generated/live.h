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
void live_H(double *in_vec, double *out_8586613551564000760);
void live_err_fun(double *nom_x, double *delta_x, double *out_8865145637849526754);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_9116737328858028060);
void live_H_mod_fun(double *state, double *out_7196489520599866412);
void live_f_fun(double *state, double dt, double *out_8123343203449822581);
void live_F_fun(double *state, double dt, double *out_1106023987106368781);
void live_h_4(double *state, double *unused, double *out_580970961733077966);
void live_H_4(double *state, double *unused, double *out_2314552585202212419);
void live_h_9(double *state, double *unused, double *out_3687165426871612897);
void live_H_9(double *state, double *unused, double *out_8844972553242891727);
void live_h_10(double *state, double *unused, double *out_4340920366832618183);
void live_H_10(double *state, double *unused, double *out_4568283767898204571);
void live_h_12(double *state, double *unused, double *out_4155938323785247986);
void live_H_12(double *state, double *unused, double *out_7334008993234174214);
void live_h_35(double *state, double *unused, double *out_919478754092228704);
void live_H_35(double *state, double *unused, double *out_8367172048150363693);
void live_h_32(double *state, double *unused, double *out_3030361748292745760);
void live_H_32(double *state, double *unused, double *out_5662147535248613756);
void live_h_13(double *state, double *unused, double *out_5552089620285866186);
void live_H_13(double *state, double *unused, double *out_8516502728882848364);
void live_h_14(double *state, double *unused, double *out_3687165426871612897);
void live_H_14(double *state, double *unused, double *out_8844972553242891727);
void live_h_33(double *state, double *unused, double *out_6584163880527017259);
void live_H_33(double *state, double *unused, double *out_8831771647213677399);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}