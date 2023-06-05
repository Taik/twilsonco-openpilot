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
void live_H(double *in_vec, double *out_7930137500395005439);
void live_err_fun(double *nom_x, double *delta_x, double *out_1511445506936860753);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4492291371470343697);
void live_H_mod_fun(double *state, double *out_6277349731035049140);
void live_f_fun(double *state, double dt, double *out_2633777933531838554);
void live_F_fun(double *state, double dt, double *out_5635087947563813180);
void live_h_4(double *state, double *unused, double *out_5877431186897083912);
void live_H_4(double *state, double *unused, double *out_3803620381983396311);
void live_h_9(double *state, double *unused, double *out_4086184597025535118);
void live_H_9(double *state, double *unused, double *out_3562430735353805666);
void live_h_10(double *state, double *unused, double *out_1506774930050350103);
void live_H_10(double *state, double *unused, double *out_282797064244797466);
void live_h_12(double *state, double *unused, double *out_5403427492213584531);
void live_H_12(double *state, double *unused, double *out_1215836026048565484);
void live_h_35(double *state, double *unused, double *out_7911039660552771080);
void live_H_35(double *state, double *unused, double *out_436958324610788935);
void live_h_32(double *state, double *unused, double *out_4702994774248972860);
void live_H_32(double *state, double *unused, double *out_5364346047596823623);
void live_h_13(double *state, double *unused, double *out_2974338752368555092);
void live_H_13(double *state, double *unused, double *out_1777836100001051822);
void live_h_14(double *state, double *unused, double *out_4086184597025535118);
void live_H_14(double *state, double *unused, double *out_3562430735353805666);
void live_h_33(double *state, double *unused, double *out_5902435596341600660);
void live_H_33(double *state, double *unused, double *out_2713598680028068669);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}