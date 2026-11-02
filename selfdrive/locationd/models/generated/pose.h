#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_3399544854207247872);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6784639363269531457);
void pose_H_mod_fun(double *state, double *out_3714060151064540699);
void pose_f_fun(double *state, double dt, double *out_8177302778648908743);
void pose_F_fun(double *state, double dt, double *out_5608595110335033478);
void pose_h_4(double *state, double *unused, double *out_6884326003512242794);
void pose_H_4(double *state, double *unused, double *out_2658547600726984974);
void pose_h_10(double *state, double *unused, double *out_8751108212963429724);
void pose_H_10(double *state, double *unused, double *out_9115722006580025533);
void pose_h_13(double *state, double *unused, double *out_4274029695233563681);
void pose_H_13(double *state, double *unused, double *out_5870821426059317775);
void pose_h_14(double *state, double *unused, double *out_6042798886861941258);
void pose_H_14(double *state, double *unused, double *out_6621788457066469503);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}