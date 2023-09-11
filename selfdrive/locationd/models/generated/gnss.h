#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6982813324875031433);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_8822363379104221878);
void gnss_H_mod_fun(double *state, double *out_5122475045724401360);
void gnss_f_fun(double *state, double dt, double *out_8767392952015677118);
void gnss_F_fun(double *state, double dt, double *out_6519335429311450011);
void gnss_h_6(double *state, double *sat_pos, double *out_8775600633747649685);
void gnss_H_6(double *state, double *sat_pos, double *out_1847989328191366990);
void gnss_h_20(double *state, double *sat_pos, double *out_6848000075585344735);
void gnss_H_20(double *state, double *sat_pos, double *out_4350122977225074540);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_3822185985007378568);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_4973339453802444283);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_3822185985007378568);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_4973339453802444283);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}