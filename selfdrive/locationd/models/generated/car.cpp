#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2280476542227183180) {
   out_2280476542227183180[0] = delta_x[0] + nom_x[0];
   out_2280476542227183180[1] = delta_x[1] + nom_x[1];
   out_2280476542227183180[2] = delta_x[2] + nom_x[2];
   out_2280476542227183180[3] = delta_x[3] + nom_x[3];
   out_2280476542227183180[4] = delta_x[4] + nom_x[4];
   out_2280476542227183180[5] = delta_x[5] + nom_x[5];
   out_2280476542227183180[6] = delta_x[6] + nom_x[6];
   out_2280476542227183180[7] = delta_x[7] + nom_x[7];
   out_2280476542227183180[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_9051257053526000705) {
   out_9051257053526000705[0] = -nom_x[0] + true_x[0];
   out_9051257053526000705[1] = -nom_x[1] + true_x[1];
   out_9051257053526000705[2] = -nom_x[2] + true_x[2];
   out_9051257053526000705[3] = -nom_x[3] + true_x[3];
   out_9051257053526000705[4] = -nom_x[4] + true_x[4];
   out_9051257053526000705[5] = -nom_x[5] + true_x[5];
   out_9051257053526000705[6] = -nom_x[6] + true_x[6];
   out_9051257053526000705[7] = -nom_x[7] + true_x[7];
   out_9051257053526000705[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6457573013710524247) {
   out_6457573013710524247[0] = 1.0;
   out_6457573013710524247[1] = 0.0;
   out_6457573013710524247[2] = 0.0;
   out_6457573013710524247[3] = 0.0;
   out_6457573013710524247[4] = 0.0;
   out_6457573013710524247[5] = 0.0;
   out_6457573013710524247[6] = 0.0;
   out_6457573013710524247[7] = 0.0;
   out_6457573013710524247[8] = 0.0;
   out_6457573013710524247[9] = 0.0;
   out_6457573013710524247[10] = 1.0;
   out_6457573013710524247[11] = 0.0;
   out_6457573013710524247[12] = 0.0;
   out_6457573013710524247[13] = 0.0;
   out_6457573013710524247[14] = 0.0;
   out_6457573013710524247[15] = 0.0;
   out_6457573013710524247[16] = 0.0;
   out_6457573013710524247[17] = 0.0;
   out_6457573013710524247[18] = 0.0;
   out_6457573013710524247[19] = 0.0;
   out_6457573013710524247[20] = 1.0;
   out_6457573013710524247[21] = 0.0;
   out_6457573013710524247[22] = 0.0;
   out_6457573013710524247[23] = 0.0;
   out_6457573013710524247[24] = 0.0;
   out_6457573013710524247[25] = 0.0;
   out_6457573013710524247[26] = 0.0;
   out_6457573013710524247[27] = 0.0;
   out_6457573013710524247[28] = 0.0;
   out_6457573013710524247[29] = 0.0;
   out_6457573013710524247[30] = 1.0;
   out_6457573013710524247[31] = 0.0;
   out_6457573013710524247[32] = 0.0;
   out_6457573013710524247[33] = 0.0;
   out_6457573013710524247[34] = 0.0;
   out_6457573013710524247[35] = 0.0;
   out_6457573013710524247[36] = 0.0;
   out_6457573013710524247[37] = 0.0;
   out_6457573013710524247[38] = 0.0;
   out_6457573013710524247[39] = 0.0;
   out_6457573013710524247[40] = 1.0;
   out_6457573013710524247[41] = 0.0;
   out_6457573013710524247[42] = 0.0;
   out_6457573013710524247[43] = 0.0;
   out_6457573013710524247[44] = 0.0;
   out_6457573013710524247[45] = 0.0;
   out_6457573013710524247[46] = 0.0;
   out_6457573013710524247[47] = 0.0;
   out_6457573013710524247[48] = 0.0;
   out_6457573013710524247[49] = 0.0;
   out_6457573013710524247[50] = 1.0;
   out_6457573013710524247[51] = 0.0;
   out_6457573013710524247[52] = 0.0;
   out_6457573013710524247[53] = 0.0;
   out_6457573013710524247[54] = 0.0;
   out_6457573013710524247[55] = 0.0;
   out_6457573013710524247[56] = 0.0;
   out_6457573013710524247[57] = 0.0;
   out_6457573013710524247[58] = 0.0;
   out_6457573013710524247[59] = 0.0;
   out_6457573013710524247[60] = 1.0;
   out_6457573013710524247[61] = 0.0;
   out_6457573013710524247[62] = 0.0;
   out_6457573013710524247[63] = 0.0;
   out_6457573013710524247[64] = 0.0;
   out_6457573013710524247[65] = 0.0;
   out_6457573013710524247[66] = 0.0;
   out_6457573013710524247[67] = 0.0;
   out_6457573013710524247[68] = 0.0;
   out_6457573013710524247[69] = 0.0;
   out_6457573013710524247[70] = 1.0;
   out_6457573013710524247[71] = 0.0;
   out_6457573013710524247[72] = 0.0;
   out_6457573013710524247[73] = 0.0;
   out_6457573013710524247[74] = 0.0;
   out_6457573013710524247[75] = 0.0;
   out_6457573013710524247[76] = 0.0;
   out_6457573013710524247[77] = 0.0;
   out_6457573013710524247[78] = 0.0;
   out_6457573013710524247[79] = 0.0;
   out_6457573013710524247[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1118574319908399978) {
   out_1118574319908399978[0] = state[0];
   out_1118574319908399978[1] = state[1];
   out_1118574319908399978[2] = state[2];
   out_1118574319908399978[3] = state[3];
   out_1118574319908399978[4] = state[4];
   out_1118574319908399978[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1118574319908399978[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1118574319908399978[7] = state[7];
   out_1118574319908399978[8] = state[8];
}
void F_fun(double *state, double dt, double *out_884454746947933335) {
   out_884454746947933335[0] = 1;
   out_884454746947933335[1] = 0;
   out_884454746947933335[2] = 0;
   out_884454746947933335[3] = 0;
   out_884454746947933335[4] = 0;
   out_884454746947933335[5] = 0;
   out_884454746947933335[6] = 0;
   out_884454746947933335[7] = 0;
   out_884454746947933335[8] = 0;
   out_884454746947933335[9] = 0;
   out_884454746947933335[10] = 1;
   out_884454746947933335[11] = 0;
   out_884454746947933335[12] = 0;
   out_884454746947933335[13] = 0;
   out_884454746947933335[14] = 0;
   out_884454746947933335[15] = 0;
   out_884454746947933335[16] = 0;
   out_884454746947933335[17] = 0;
   out_884454746947933335[18] = 0;
   out_884454746947933335[19] = 0;
   out_884454746947933335[20] = 1;
   out_884454746947933335[21] = 0;
   out_884454746947933335[22] = 0;
   out_884454746947933335[23] = 0;
   out_884454746947933335[24] = 0;
   out_884454746947933335[25] = 0;
   out_884454746947933335[26] = 0;
   out_884454746947933335[27] = 0;
   out_884454746947933335[28] = 0;
   out_884454746947933335[29] = 0;
   out_884454746947933335[30] = 1;
   out_884454746947933335[31] = 0;
   out_884454746947933335[32] = 0;
   out_884454746947933335[33] = 0;
   out_884454746947933335[34] = 0;
   out_884454746947933335[35] = 0;
   out_884454746947933335[36] = 0;
   out_884454746947933335[37] = 0;
   out_884454746947933335[38] = 0;
   out_884454746947933335[39] = 0;
   out_884454746947933335[40] = 1;
   out_884454746947933335[41] = 0;
   out_884454746947933335[42] = 0;
   out_884454746947933335[43] = 0;
   out_884454746947933335[44] = 0;
   out_884454746947933335[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_884454746947933335[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_884454746947933335[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_884454746947933335[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_884454746947933335[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_884454746947933335[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_884454746947933335[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_884454746947933335[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_884454746947933335[53] = -9.8100000000000005*dt;
   out_884454746947933335[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_884454746947933335[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_884454746947933335[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_884454746947933335[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_884454746947933335[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_884454746947933335[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_884454746947933335[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_884454746947933335[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_884454746947933335[62] = 0;
   out_884454746947933335[63] = 0;
   out_884454746947933335[64] = 0;
   out_884454746947933335[65] = 0;
   out_884454746947933335[66] = 0;
   out_884454746947933335[67] = 0;
   out_884454746947933335[68] = 0;
   out_884454746947933335[69] = 0;
   out_884454746947933335[70] = 1;
   out_884454746947933335[71] = 0;
   out_884454746947933335[72] = 0;
   out_884454746947933335[73] = 0;
   out_884454746947933335[74] = 0;
   out_884454746947933335[75] = 0;
   out_884454746947933335[76] = 0;
   out_884454746947933335[77] = 0;
   out_884454746947933335[78] = 0;
   out_884454746947933335[79] = 0;
   out_884454746947933335[80] = 1;
}
void h_25(double *state, double *unused, double *out_6720018844695025700) {
   out_6720018844695025700[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4067833027025167564) {
   out_4067833027025167564[0] = 0;
   out_4067833027025167564[1] = 0;
   out_4067833027025167564[2] = 0;
   out_4067833027025167564[3] = 0;
   out_4067833027025167564[4] = 0;
   out_4067833027025167564[5] = 0;
   out_4067833027025167564[6] = 1;
   out_4067833027025167564[7] = 0;
   out_4067833027025167564[8] = 0;
}
void h_24(double *state, double *unused, double *out_1370033712900151212) {
   out_1370033712900151212[0] = state[4];
   out_1370033712900151212[1] = state[5];
}
void H_24(double *state, double *unused, double *out_509907428222844428) {
   out_509907428222844428[0] = 0;
   out_509907428222844428[1] = 0;
   out_509907428222844428[2] = 0;
   out_509907428222844428[3] = 0;
   out_509907428222844428[4] = 1;
   out_509907428222844428[5] = 0;
   out_509907428222844428[6] = 0;
   out_509907428222844428[7] = 0;
   out_509907428222844428[8] = 0;
   out_509907428222844428[9] = 0;
   out_509907428222844428[10] = 0;
   out_509907428222844428[11] = 0;
   out_509907428222844428[12] = 0;
   out_509907428222844428[13] = 0;
   out_509907428222844428[14] = 1;
   out_509907428222844428[15] = 0;
   out_509907428222844428[16] = 0;
   out_509907428222844428[17] = 0;
}
void h_30(double *state, double *unused, double *out_4907223068185748781) {
   out_4907223068185748781[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8595529357152775762) {
   out_8595529357152775762[0] = 0;
   out_8595529357152775762[1] = 0;
   out_8595529357152775762[2] = 0;
   out_8595529357152775762[3] = 0;
   out_8595529357152775762[4] = 1;
   out_8595529357152775762[5] = 0;
   out_8595529357152775762[6] = 0;
   out_8595529357152775762[7] = 0;
   out_8595529357152775762[8] = 0;
}
void h_26(double *state, double *unused, double *out_959740751069687356) {
   out_959740751069687356[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7809336345899223788) {
   out_7809336345899223788[0] = 0;
   out_7809336345899223788[1] = 0;
   out_7809336345899223788[2] = 0;
   out_7809336345899223788[3] = 0;
   out_7809336345899223788[4] = 0;
   out_7809336345899223788[5] = 0;
   out_7809336345899223788[6] = 0;
   out_7809336345899223788[7] = 1;
   out_7809336345899223788[8] = 0;
}
void h_27(double *state, double *unused, double *out_2823228601404989965) {
   out_2823228601404989965[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7676451404756350943) {
   out_7676451404756350943[0] = 0;
   out_7676451404756350943[1] = 0;
   out_7676451404756350943[2] = 0;
   out_7676451404756350943[3] = 1;
   out_7676451404756350943[4] = 0;
   out_7676451404756350943[5] = 0;
   out_7676451404756350943[6] = 0;
   out_7676451404756350943[7] = 0;
   out_7676451404756350943[8] = 0;
}
void h_29(double *state, double *unused, double *out_1229380413321304906) {
   out_1229380413321304906[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8085298012838383578) {
   out_8085298012838383578[0] = 0;
   out_8085298012838383578[1] = 1;
   out_8085298012838383578[2] = 0;
   out_8085298012838383578[3] = 0;
   out_8085298012838383578[4] = 0;
   out_8085298012838383578[5] = 0;
   out_8085298012838383578[6] = 0;
   out_8085298012838383578[7] = 0;
   out_8085298012838383578[8] = 0;
}
void h_28(double *state, double *unused, double *out_2377558556468636828) {
   out_2377558556468636828[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5279047043801637464) {
   out_5279047043801637464[0] = 1;
   out_5279047043801637464[1] = 0;
   out_5279047043801637464[2] = 0;
   out_5279047043801637464[3] = 0;
   out_5279047043801637464[4] = 0;
   out_5279047043801637464[5] = 0;
   out_5279047043801637464[6] = 0;
   out_5279047043801637464[7] = 0;
   out_5279047043801637464[8] = 0;
}
void h_31(double *state, double *unused, double *out_5318054386544568732) {
   out_5318054386544568732[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8435544448132575264) {
   out_8435544448132575264[0] = 0;
   out_8435544448132575264[1] = 0;
   out_8435544448132575264[2] = 0;
   out_8435544448132575264[3] = 0;
   out_8435544448132575264[4] = 0;
   out_8435544448132575264[5] = 0;
   out_8435544448132575264[6] = 0;
   out_8435544448132575264[7] = 0;
   out_8435544448132575264[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_2280476542227183180) {
  err_fun(nom_x, delta_x, out_2280476542227183180);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9051257053526000705) {
  inv_err_fun(nom_x, true_x, out_9051257053526000705);
}
void car_H_mod_fun(double *state, double *out_6457573013710524247) {
  H_mod_fun(state, out_6457573013710524247);
}
void car_f_fun(double *state, double dt, double *out_1118574319908399978) {
  f_fun(state,  dt, out_1118574319908399978);
}
void car_F_fun(double *state, double dt, double *out_884454746947933335) {
  F_fun(state,  dt, out_884454746947933335);
}
void car_h_25(double *state, double *unused, double *out_6720018844695025700) {
  h_25(state, unused, out_6720018844695025700);
}
void car_H_25(double *state, double *unused, double *out_4067833027025167564) {
  H_25(state, unused, out_4067833027025167564);
}
void car_h_24(double *state, double *unused, double *out_1370033712900151212) {
  h_24(state, unused, out_1370033712900151212);
}
void car_H_24(double *state, double *unused, double *out_509907428222844428) {
  H_24(state, unused, out_509907428222844428);
}
void car_h_30(double *state, double *unused, double *out_4907223068185748781) {
  h_30(state, unused, out_4907223068185748781);
}
void car_H_30(double *state, double *unused, double *out_8595529357152775762) {
  H_30(state, unused, out_8595529357152775762);
}
void car_h_26(double *state, double *unused, double *out_959740751069687356) {
  h_26(state, unused, out_959740751069687356);
}
void car_H_26(double *state, double *unused, double *out_7809336345899223788) {
  H_26(state, unused, out_7809336345899223788);
}
void car_h_27(double *state, double *unused, double *out_2823228601404989965) {
  h_27(state, unused, out_2823228601404989965);
}
void car_H_27(double *state, double *unused, double *out_7676451404756350943) {
  H_27(state, unused, out_7676451404756350943);
}
void car_h_29(double *state, double *unused, double *out_1229380413321304906) {
  h_29(state, unused, out_1229380413321304906);
}
void car_H_29(double *state, double *unused, double *out_8085298012838383578) {
  H_29(state, unused, out_8085298012838383578);
}
void car_h_28(double *state, double *unused, double *out_2377558556468636828) {
  h_28(state, unused, out_2377558556468636828);
}
void car_H_28(double *state, double *unused, double *out_5279047043801637464) {
  H_28(state, unused, out_5279047043801637464);
}
void car_h_31(double *state, double *unused, double *out_5318054386544568732) {
  h_31(state, unused, out_5318054386544568732);
}
void car_H_31(double *state, double *unused, double *out_8435544448132575264) {
  H_31(state, unused, out_8435544448132575264);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
