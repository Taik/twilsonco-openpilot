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
void car_err_fun(double *nom_x, double *delta_x, double *out_8858193099505922130);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6125827550351398334);
void car_H_mod_fun(double *state, double *out_2420972627039917597);
void car_f_fun(double *state, double dt, double *out_6131450845666027670);
void car_F_fun(double *state, double dt, double *out_1611502613691033176);
void car_h_25(double *state, double *unused, double *out_7412045671842276054);
void car_H_25(double *state, double *unused, double *out_229619803497515274);
void car_h_24(double *state, double *unused, double *out_4498318601322565789);
void car_H_24(double *state, double *unused, double *out_1996087980481353288);
void car_h_30(double *state, double *unused, double *out_4446870475390980399);
void car_H_30(double *state, double *unused, double *out_6687070537994101481);
void car_h_26(double *state, double *unused, double *out_1311114438602757557);
void car_H_26(double *state, double *unused, double *out_3074906166263285327);
void car_h_27(double *state, double *unused, double *out_5808791186031950615);
void car_H_27(double *state, double *unused, double *out_4512307226193676570);
void car_h_29(double *state, double *unused, double *out_554802792269865246);
void car_H_29(double *state, double *unused, double *out_7197301882308493665);
void car_h_28(double *state, double *unused, double *out_6976534987108092352);
void car_H_28(double *state, double *unused, double *out_2283454517745405037);
void car_h_31(double *state, double *unused, double *out_3774596143588253704);
void car_H_31(double *state, double *unused, double *out_6847055447014301979);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}