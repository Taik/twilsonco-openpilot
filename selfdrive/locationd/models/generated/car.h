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
void car_err_fun(double *nom_x, double *delta_x, double *out_4636062031133054768);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6030581718255978326);
void car_H_mod_fun(double *state, double *out_2936385218361774249);
void car_f_fun(double *state, double dt, double *out_6336615873933685425);
void car_F_fun(double *state, double dt, double *out_5006846382452517549);
void car_h_25(double *state, double *unused, double *out_680600810615390925);
void car_H_25(double *state, double *unused, double *out_5768343168375162758);
void car_h_24(double *state, double *unused, double *out_1697116264620455802);
void car_H_24(double *state, double *unused, double *out_3595693569369663192);
void car_h_30(double *state, double *unused, double *out_7959276666103150959);
void car_H_30(double *state, double *unused, double *out_5761710563842772103);
void car_h_26(double *state, double *unused, double *out_5836448100731834965);
void car_H_26(double *state, double *unused, double *out_9072869138135963359);
void car_h_27(double *state, double *unused, double *out_5682274078276236570);
void car_H_27(double *state, double *unused, double *out_7936473875643197014);
void car_h_29(double *state, double *unused, double *out_4231642685647353403);
void car_H_29(double *state, double *unused, double *out_8796907471196803569);
void car_h_28(double *state, double *unused, double *out_6664358171611534682);
void car_H_28(double *state, double *unused, double *out_3714508454127272995);
void car_h_31(double *state, double *unused, double *out_405406748330885036);
void car_H_31(double *state, double *unused, double *out_5601725654822571605);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}