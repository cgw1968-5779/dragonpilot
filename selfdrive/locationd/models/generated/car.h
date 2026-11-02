#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_3163716875487303097);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2959949742471833250);
void car_H_mod_fun(double *state, double *out_3971242436043389113);
void car_f_fun(double *state, double dt, double *out_3474729257374299696);
void car_F_fun(double *state, double dt, double *out_7804021070923170269);
void car_h_25(double *state, double *unused, double *out_668706547536053124);
void car_H_25(double *state, double *unused, double *out_448261786053897436);
void car_h_24(double *state, double *unused, double *out_1762144102910258338);
void car_H_24(double *state, double *unused, double *out_2620911385059397002);
void car_h_30(double *state, double *unused, double *out_7439541773886404060);
void car_H_30(double *state, double *unused, double *out_6468428555437719319);
void car_h_26(double *state, double *unused, double *out_2097674538000373497);
void car_H_26(double *state, double *unused, double *out_4189765104927953660);
void car_h_27(double *state, double *unused, double *out_4332966720124792521);
void car_H_27(double *state, double *unused, double *out_2752364044997562417);
void car_h_29(double *state, double *unused, double *out_7970416248378814871);
void car_H_29(double *state, double *unused, double *out_4465726771867113450);
void car_h_28(double *state, double *unused, double *out_5395940433716066769);
void car_H_28(double *state, double *unused, double *out_2502096500301787199);
void car_h_31(double *state, double *unused, double *out_781924845092830043);
void car_H_31(double *state, double *unused, double *out_417615824176937008);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}