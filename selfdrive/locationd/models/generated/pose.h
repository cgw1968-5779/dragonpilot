#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_6415792483428760966);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6377608227770965724);
void pose_H_mod_fun(double *state, double *out_6347051622433270619);
void pose_f_fun(double *state, double dt, double *out_9196882787902943927);
void pose_F_fun(double *state, double dt, double *out_5866301831384964850);
void pose_h_4(double *state, double *unused, double *out_1536508825138900327);
void pose_H_4(double *state, double *unused, double *out_7544999104316168723);
void pose_h_10(double *state, double *unused, double *out_6489509158998458427);
void pose_H_10(double *state, double *unused, double *out_6104186488438151746);
void pose_h_13(double *state, double *unused, double *out_2124120058566848093);
void pose_H_13(double *state, double *unused, double *out_3711243641013644699);
void pose_h_14(double *state, double *unused, double *out_2918682735475695430);
void pose_H_14(double *state, double *unused, double *out_4462210672020796427);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}