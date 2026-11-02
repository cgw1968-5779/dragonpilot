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
void car_err_fun(double *nom_x, double *delta_x, double *out_2280476542227183180);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9051257053526000705);
void car_H_mod_fun(double *state, double *out_6457573013710524247);
void car_f_fun(double *state, double dt, double *out_1118574319908399978);
void car_F_fun(double *state, double dt, double *out_884454746947933335);
void car_h_25(double *state, double *unused, double *out_6720018844695025700);
void car_H_25(double *state, double *unused, double *out_4067833027025167564);
void car_h_24(double *state, double *unused, double *out_1370033712900151212);
void car_H_24(double *state, double *unused, double *out_509907428222844428);
void car_h_30(double *state, double *unused, double *out_4907223068185748781);
void car_H_30(double *state, double *unused, double *out_8595529357152775762);
void car_h_26(double *state, double *unused, double *out_959740751069687356);
void car_H_26(double *state, double *unused, double *out_7809336345899223788);
void car_h_27(double *state, double *unused, double *out_2823228601404989965);
void car_H_27(double *state, double *unused, double *out_7676451404756350943);
void car_h_29(double *state, double *unused, double *out_1229380413321304906);
void car_H_29(double *state, double *unused, double *out_8085298012838383578);
void car_h_28(double *state, double *unused, double *out_2377558556468636828);
void car_H_28(double *state, double *unused, double *out_5279047043801637464);
void car_h_31(double *state, double *unused, double *out_5318054386544568732);
void car_H_31(double *state, double *unused, double *out_8435544448132575264);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}