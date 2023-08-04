#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_3686789247502404896);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_4567148213281321427);
void gnss_H_mod_fun(double *state, double *out_6175002328322028331);
void gnss_f_fun(double *state, double dt, double *out_4336716201713451957);
void gnss_F_fun(double *state, double dt, double *out_6104947861706444775);
void gnss_h_6(double *state, double *sat_pos, double *out_1150344248106113082);
void gnss_H_6(double *state, double *sat_pos, double *out_394388827836502853);
void gnss_h_20(double *state, double *sat_pos, double *out_2337833637223709483);
void gnss_H_20(double *state, double *sat_pos, double *out_7352886989259104254);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_902647274384620902);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_185624918626144292);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_902647274384620902);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_185624918626144292);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}