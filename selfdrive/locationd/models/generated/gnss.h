#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_4991393669533314019);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_2774114983703919595);
void gnss_H_mod_fun(double *state, double *out_6730346335144126854);
void gnss_f_fun(double *state, double dt, double *out_5306142855013818139);
void gnss_F_fun(double *state, double dt, double *out_3786678109369211231);
void gnss_h_6(double *state, double *sat_pos, double *out_1396935052484491170);
void gnss_H_6(double *state, double *sat_pos, double *out_8126508195366532936);
void gnss_h_20(double *state, double *sat_pos, double *out_3696039159246344318);
void gnss_H_20(double *state, double *sat_pos, double *out_1052657384503181039);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_3520472109361379988);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_6147765845821346871);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_3520472109361379988);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_6147765845821346871);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}