#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3399544854207247872) {
   out_3399544854207247872[0] = delta_x[0] + nom_x[0];
   out_3399544854207247872[1] = delta_x[1] + nom_x[1];
   out_3399544854207247872[2] = delta_x[2] + nom_x[2];
   out_3399544854207247872[3] = delta_x[3] + nom_x[3];
   out_3399544854207247872[4] = delta_x[4] + nom_x[4];
   out_3399544854207247872[5] = delta_x[5] + nom_x[5];
   out_3399544854207247872[6] = delta_x[6] + nom_x[6];
   out_3399544854207247872[7] = delta_x[7] + nom_x[7];
   out_3399544854207247872[8] = delta_x[8] + nom_x[8];
   out_3399544854207247872[9] = delta_x[9] + nom_x[9];
   out_3399544854207247872[10] = delta_x[10] + nom_x[10];
   out_3399544854207247872[11] = delta_x[11] + nom_x[11];
   out_3399544854207247872[12] = delta_x[12] + nom_x[12];
   out_3399544854207247872[13] = delta_x[13] + nom_x[13];
   out_3399544854207247872[14] = delta_x[14] + nom_x[14];
   out_3399544854207247872[15] = delta_x[15] + nom_x[15];
   out_3399544854207247872[16] = delta_x[16] + nom_x[16];
   out_3399544854207247872[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6784639363269531457) {
   out_6784639363269531457[0] = -nom_x[0] + true_x[0];
   out_6784639363269531457[1] = -nom_x[1] + true_x[1];
   out_6784639363269531457[2] = -nom_x[2] + true_x[2];
   out_6784639363269531457[3] = -nom_x[3] + true_x[3];
   out_6784639363269531457[4] = -nom_x[4] + true_x[4];
   out_6784639363269531457[5] = -nom_x[5] + true_x[5];
   out_6784639363269531457[6] = -nom_x[6] + true_x[6];
   out_6784639363269531457[7] = -nom_x[7] + true_x[7];
   out_6784639363269531457[8] = -nom_x[8] + true_x[8];
   out_6784639363269531457[9] = -nom_x[9] + true_x[9];
   out_6784639363269531457[10] = -nom_x[10] + true_x[10];
   out_6784639363269531457[11] = -nom_x[11] + true_x[11];
   out_6784639363269531457[12] = -nom_x[12] + true_x[12];
   out_6784639363269531457[13] = -nom_x[13] + true_x[13];
   out_6784639363269531457[14] = -nom_x[14] + true_x[14];
   out_6784639363269531457[15] = -nom_x[15] + true_x[15];
   out_6784639363269531457[16] = -nom_x[16] + true_x[16];
   out_6784639363269531457[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_3714060151064540699) {
   out_3714060151064540699[0] = 1.0;
   out_3714060151064540699[1] = 0.0;
   out_3714060151064540699[2] = 0.0;
   out_3714060151064540699[3] = 0.0;
   out_3714060151064540699[4] = 0.0;
   out_3714060151064540699[5] = 0.0;
   out_3714060151064540699[6] = 0.0;
   out_3714060151064540699[7] = 0.0;
   out_3714060151064540699[8] = 0.0;
   out_3714060151064540699[9] = 0.0;
   out_3714060151064540699[10] = 0.0;
   out_3714060151064540699[11] = 0.0;
   out_3714060151064540699[12] = 0.0;
   out_3714060151064540699[13] = 0.0;
   out_3714060151064540699[14] = 0.0;
   out_3714060151064540699[15] = 0.0;
   out_3714060151064540699[16] = 0.0;
   out_3714060151064540699[17] = 0.0;
   out_3714060151064540699[18] = 0.0;
   out_3714060151064540699[19] = 1.0;
   out_3714060151064540699[20] = 0.0;
   out_3714060151064540699[21] = 0.0;
   out_3714060151064540699[22] = 0.0;
   out_3714060151064540699[23] = 0.0;
   out_3714060151064540699[24] = 0.0;
   out_3714060151064540699[25] = 0.0;
   out_3714060151064540699[26] = 0.0;
   out_3714060151064540699[27] = 0.0;
   out_3714060151064540699[28] = 0.0;
   out_3714060151064540699[29] = 0.0;
   out_3714060151064540699[30] = 0.0;
   out_3714060151064540699[31] = 0.0;
   out_3714060151064540699[32] = 0.0;
   out_3714060151064540699[33] = 0.0;
   out_3714060151064540699[34] = 0.0;
   out_3714060151064540699[35] = 0.0;
   out_3714060151064540699[36] = 0.0;
   out_3714060151064540699[37] = 0.0;
   out_3714060151064540699[38] = 1.0;
   out_3714060151064540699[39] = 0.0;
   out_3714060151064540699[40] = 0.0;
   out_3714060151064540699[41] = 0.0;
   out_3714060151064540699[42] = 0.0;
   out_3714060151064540699[43] = 0.0;
   out_3714060151064540699[44] = 0.0;
   out_3714060151064540699[45] = 0.0;
   out_3714060151064540699[46] = 0.0;
   out_3714060151064540699[47] = 0.0;
   out_3714060151064540699[48] = 0.0;
   out_3714060151064540699[49] = 0.0;
   out_3714060151064540699[50] = 0.0;
   out_3714060151064540699[51] = 0.0;
   out_3714060151064540699[52] = 0.0;
   out_3714060151064540699[53] = 0.0;
   out_3714060151064540699[54] = 0.0;
   out_3714060151064540699[55] = 0.0;
   out_3714060151064540699[56] = 0.0;
   out_3714060151064540699[57] = 1.0;
   out_3714060151064540699[58] = 0.0;
   out_3714060151064540699[59] = 0.0;
   out_3714060151064540699[60] = 0.0;
   out_3714060151064540699[61] = 0.0;
   out_3714060151064540699[62] = 0.0;
   out_3714060151064540699[63] = 0.0;
   out_3714060151064540699[64] = 0.0;
   out_3714060151064540699[65] = 0.0;
   out_3714060151064540699[66] = 0.0;
   out_3714060151064540699[67] = 0.0;
   out_3714060151064540699[68] = 0.0;
   out_3714060151064540699[69] = 0.0;
   out_3714060151064540699[70] = 0.0;
   out_3714060151064540699[71] = 0.0;
   out_3714060151064540699[72] = 0.0;
   out_3714060151064540699[73] = 0.0;
   out_3714060151064540699[74] = 0.0;
   out_3714060151064540699[75] = 0.0;
   out_3714060151064540699[76] = 1.0;
   out_3714060151064540699[77] = 0.0;
   out_3714060151064540699[78] = 0.0;
   out_3714060151064540699[79] = 0.0;
   out_3714060151064540699[80] = 0.0;
   out_3714060151064540699[81] = 0.0;
   out_3714060151064540699[82] = 0.0;
   out_3714060151064540699[83] = 0.0;
   out_3714060151064540699[84] = 0.0;
   out_3714060151064540699[85] = 0.0;
   out_3714060151064540699[86] = 0.0;
   out_3714060151064540699[87] = 0.0;
   out_3714060151064540699[88] = 0.0;
   out_3714060151064540699[89] = 0.0;
   out_3714060151064540699[90] = 0.0;
   out_3714060151064540699[91] = 0.0;
   out_3714060151064540699[92] = 0.0;
   out_3714060151064540699[93] = 0.0;
   out_3714060151064540699[94] = 0.0;
   out_3714060151064540699[95] = 1.0;
   out_3714060151064540699[96] = 0.0;
   out_3714060151064540699[97] = 0.0;
   out_3714060151064540699[98] = 0.0;
   out_3714060151064540699[99] = 0.0;
   out_3714060151064540699[100] = 0.0;
   out_3714060151064540699[101] = 0.0;
   out_3714060151064540699[102] = 0.0;
   out_3714060151064540699[103] = 0.0;
   out_3714060151064540699[104] = 0.0;
   out_3714060151064540699[105] = 0.0;
   out_3714060151064540699[106] = 0.0;
   out_3714060151064540699[107] = 0.0;
   out_3714060151064540699[108] = 0.0;
   out_3714060151064540699[109] = 0.0;
   out_3714060151064540699[110] = 0.0;
   out_3714060151064540699[111] = 0.0;
   out_3714060151064540699[112] = 0.0;
   out_3714060151064540699[113] = 0.0;
   out_3714060151064540699[114] = 1.0;
   out_3714060151064540699[115] = 0.0;
   out_3714060151064540699[116] = 0.0;
   out_3714060151064540699[117] = 0.0;
   out_3714060151064540699[118] = 0.0;
   out_3714060151064540699[119] = 0.0;
   out_3714060151064540699[120] = 0.0;
   out_3714060151064540699[121] = 0.0;
   out_3714060151064540699[122] = 0.0;
   out_3714060151064540699[123] = 0.0;
   out_3714060151064540699[124] = 0.0;
   out_3714060151064540699[125] = 0.0;
   out_3714060151064540699[126] = 0.0;
   out_3714060151064540699[127] = 0.0;
   out_3714060151064540699[128] = 0.0;
   out_3714060151064540699[129] = 0.0;
   out_3714060151064540699[130] = 0.0;
   out_3714060151064540699[131] = 0.0;
   out_3714060151064540699[132] = 0.0;
   out_3714060151064540699[133] = 1.0;
   out_3714060151064540699[134] = 0.0;
   out_3714060151064540699[135] = 0.0;
   out_3714060151064540699[136] = 0.0;
   out_3714060151064540699[137] = 0.0;
   out_3714060151064540699[138] = 0.0;
   out_3714060151064540699[139] = 0.0;
   out_3714060151064540699[140] = 0.0;
   out_3714060151064540699[141] = 0.0;
   out_3714060151064540699[142] = 0.0;
   out_3714060151064540699[143] = 0.0;
   out_3714060151064540699[144] = 0.0;
   out_3714060151064540699[145] = 0.0;
   out_3714060151064540699[146] = 0.0;
   out_3714060151064540699[147] = 0.0;
   out_3714060151064540699[148] = 0.0;
   out_3714060151064540699[149] = 0.0;
   out_3714060151064540699[150] = 0.0;
   out_3714060151064540699[151] = 0.0;
   out_3714060151064540699[152] = 1.0;
   out_3714060151064540699[153] = 0.0;
   out_3714060151064540699[154] = 0.0;
   out_3714060151064540699[155] = 0.0;
   out_3714060151064540699[156] = 0.0;
   out_3714060151064540699[157] = 0.0;
   out_3714060151064540699[158] = 0.0;
   out_3714060151064540699[159] = 0.0;
   out_3714060151064540699[160] = 0.0;
   out_3714060151064540699[161] = 0.0;
   out_3714060151064540699[162] = 0.0;
   out_3714060151064540699[163] = 0.0;
   out_3714060151064540699[164] = 0.0;
   out_3714060151064540699[165] = 0.0;
   out_3714060151064540699[166] = 0.0;
   out_3714060151064540699[167] = 0.0;
   out_3714060151064540699[168] = 0.0;
   out_3714060151064540699[169] = 0.0;
   out_3714060151064540699[170] = 0.0;
   out_3714060151064540699[171] = 1.0;
   out_3714060151064540699[172] = 0.0;
   out_3714060151064540699[173] = 0.0;
   out_3714060151064540699[174] = 0.0;
   out_3714060151064540699[175] = 0.0;
   out_3714060151064540699[176] = 0.0;
   out_3714060151064540699[177] = 0.0;
   out_3714060151064540699[178] = 0.0;
   out_3714060151064540699[179] = 0.0;
   out_3714060151064540699[180] = 0.0;
   out_3714060151064540699[181] = 0.0;
   out_3714060151064540699[182] = 0.0;
   out_3714060151064540699[183] = 0.0;
   out_3714060151064540699[184] = 0.0;
   out_3714060151064540699[185] = 0.0;
   out_3714060151064540699[186] = 0.0;
   out_3714060151064540699[187] = 0.0;
   out_3714060151064540699[188] = 0.0;
   out_3714060151064540699[189] = 0.0;
   out_3714060151064540699[190] = 1.0;
   out_3714060151064540699[191] = 0.0;
   out_3714060151064540699[192] = 0.0;
   out_3714060151064540699[193] = 0.0;
   out_3714060151064540699[194] = 0.0;
   out_3714060151064540699[195] = 0.0;
   out_3714060151064540699[196] = 0.0;
   out_3714060151064540699[197] = 0.0;
   out_3714060151064540699[198] = 0.0;
   out_3714060151064540699[199] = 0.0;
   out_3714060151064540699[200] = 0.0;
   out_3714060151064540699[201] = 0.0;
   out_3714060151064540699[202] = 0.0;
   out_3714060151064540699[203] = 0.0;
   out_3714060151064540699[204] = 0.0;
   out_3714060151064540699[205] = 0.0;
   out_3714060151064540699[206] = 0.0;
   out_3714060151064540699[207] = 0.0;
   out_3714060151064540699[208] = 0.0;
   out_3714060151064540699[209] = 1.0;
   out_3714060151064540699[210] = 0.0;
   out_3714060151064540699[211] = 0.0;
   out_3714060151064540699[212] = 0.0;
   out_3714060151064540699[213] = 0.0;
   out_3714060151064540699[214] = 0.0;
   out_3714060151064540699[215] = 0.0;
   out_3714060151064540699[216] = 0.0;
   out_3714060151064540699[217] = 0.0;
   out_3714060151064540699[218] = 0.0;
   out_3714060151064540699[219] = 0.0;
   out_3714060151064540699[220] = 0.0;
   out_3714060151064540699[221] = 0.0;
   out_3714060151064540699[222] = 0.0;
   out_3714060151064540699[223] = 0.0;
   out_3714060151064540699[224] = 0.0;
   out_3714060151064540699[225] = 0.0;
   out_3714060151064540699[226] = 0.0;
   out_3714060151064540699[227] = 0.0;
   out_3714060151064540699[228] = 1.0;
   out_3714060151064540699[229] = 0.0;
   out_3714060151064540699[230] = 0.0;
   out_3714060151064540699[231] = 0.0;
   out_3714060151064540699[232] = 0.0;
   out_3714060151064540699[233] = 0.0;
   out_3714060151064540699[234] = 0.0;
   out_3714060151064540699[235] = 0.0;
   out_3714060151064540699[236] = 0.0;
   out_3714060151064540699[237] = 0.0;
   out_3714060151064540699[238] = 0.0;
   out_3714060151064540699[239] = 0.0;
   out_3714060151064540699[240] = 0.0;
   out_3714060151064540699[241] = 0.0;
   out_3714060151064540699[242] = 0.0;
   out_3714060151064540699[243] = 0.0;
   out_3714060151064540699[244] = 0.0;
   out_3714060151064540699[245] = 0.0;
   out_3714060151064540699[246] = 0.0;
   out_3714060151064540699[247] = 1.0;
   out_3714060151064540699[248] = 0.0;
   out_3714060151064540699[249] = 0.0;
   out_3714060151064540699[250] = 0.0;
   out_3714060151064540699[251] = 0.0;
   out_3714060151064540699[252] = 0.0;
   out_3714060151064540699[253] = 0.0;
   out_3714060151064540699[254] = 0.0;
   out_3714060151064540699[255] = 0.0;
   out_3714060151064540699[256] = 0.0;
   out_3714060151064540699[257] = 0.0;
   out_3714060151064540699[258] = 0.0;
   out_3714060151064540699[259] = 0.0;
   out_3714060151064540699[260] = 0.0;
   out_3714060151064540699[261] = 0.0;
   out_3714060151064540699[262] = 0.0;
   out_3714060151064540699[263] = 0.0;
   out_3714060151064540699[264] = 0.0;
   out_3714060151064540699[265] = 0.0;
   out_3714060151064540699[266] = 1.0;
   out_3714060151064540699[267] = 0.0;
   out_3714060151064540699[268] = 0.0;
   out_3714060151064540699[269] = 0.0;
   out_3714060151064540699[270] = 0.0;
   out_3714060151064540699[271] = 0.0;
   out_3714060151064540699[272] = 0.0;
   out_3714060151064540699[273] = 0.0;
   out_3714060151064540699[274] = 0.0;
   out_3714060151064540699[275] = 0.0;
   out_3714060151064540699[276] = 0.0;
   out_3714060151064540699[277] = 0.0;
   out_3714060151064540699[278] = 0.0;
   out_3714060151064540699[279] = 0.0;
   out_3714060151064540699[280] = 0.0;
   out_3714060151064540699[281] = 0.0;
   out_3714060151064540699[282] = 0.0;
   out_3714060151064540699[283] = 0.0;
   out_3714060151064540699[284] = 0.0;
   out_3714060151064540699[285] = 1.0;
   out_3714060151064540699[286] = 0.0;
   out_3714060151064540699[287] = 0.0;
   out_3714060151064540699[288] = 0.0;
   out_3714060151064540699[289] = 0.0;
   out_3714060151064540699[290] = 0.0;
   out_3714060151064540699[291] = 0.0;
   out_3714060151064540699[292] = 0.0;
   out_3714060151064540699[293] = 0.0;
   out_3714060151064540699[294] = 0.0;
   out_3714060151064540699[295] = 0.0;
   out_3714060151064540699[296] = 0.0;
   out_3714060151064540699[297] = 0.0;
   out_3714060151064540699[298] = 0.0;
   out_3714060151064540699[299] = 0.0;
   out_3714060151064540699[300] = 0.0;
   out_3714060151064540699[301] = 0.0;
   out_3714060151064540699[302] = 0.0;
   out_3714060151064540699[303] = 0.0;
   out_3714060151064540699[304] = 1.0;
   out_3714060151064540699[305] = 0.0;
   out_3714060151064540699[306] = 0.0;
   out_3714060151064540699[307] = 0.0;
   out_3714060151064540699[308] = 0.0;
   out_3714060151064540699[309] = 0.0;
   out_3714060151064540699[310] = 0.0;
   out_3714060151064540699[311] = 0.0;
   out_3714060151064540699[312] = 0.0;
   out_3714060151064540699[313] = 0.0;
   out_3714060151064540699[314] = 0.0;
   out_3714060151064540699[315] = 0.0;
   out_3714060151064540699[316] = 0.0;
   out_3714060151064540699[317] = 0.0;
   out_3714060151064540699[318] = 0.0;
   out_3714060151064540699[319] = 0.0;
   out_3714060151064540699[320] = 0.0;
   out_3714060151064540699[321] = 0.0;
   out_3714060151064540699[322] = 0.0;
   out_3714060151064540699[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_8177302778648908743) {
   out_8177302778648908743[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_8177302778648908743[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_8177302778648908743[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_8177302778648908743[3] = dt*state[12] + state[3];
   out_8177302778648908743[4] = dt*state[13] + state[4];
   out_8177302778648908743[5] = dt*state[14] + state[5];
   out_8177302778648908743[6] = state[6];
   out_8177302778648908743[7] = state[7];
   out_8177302778648908743[8] = state[8];
   out_8177302778648908743[9] = state[9];
   out_8177302778648908743[10] = state[10];
   out_8177302778648908743[11] = state[11];
   out_8177302778648908743[12] = state[12];
   out_8177302778648908743[13] = state[13];
   out_8177302778648908743[14] = state[14];
   out_8177302778648908743[15] = state[15];
   out_8177302778648908743[16] = state[16];
   out_8177302778648908743[17] = state[17];
}
void F_fun(double *state, double dt, double *out_5608595110335033478) {
   out_5608595110335033478[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5608595110335033478[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5608595110335033478[2] = 0;
   out_5608595110335033478[3] = 0;
   out_5608595110335033478[4] = 0;
   out_5608595110335033478[5] = 0;
   out_5608595110335033478[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5608595110335033478[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5608595110335033478[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5608595110335033478[9] = 0;
   out_5608595110335033478[10] = 0;
   out_5608595110335033478[11] = 0;
   out_5608595110335033478[12] = 0;
   out_5608595110335033478[13] = 0;
   out_5608595110335033478[14] = 0;
   out_5608595110335033478[15] = 0;
   out_5608595110335033478[16] = 0;
   out_5608595110335033478[17] = 0;
   out_5608595110335033478[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5608595110335033478[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5608595110335033478[20] = 0;
   out_5608595110335033478[21] = 0;
   out_5608595110335033478[22] = 0;
   out_5608595110335033478[23] = 0;
   out_5608595110335033478[24] = 0;
   out_5608595110335033478[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5608595110335033478[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5608595110335033478[27] = 0;
   out_5608595110335033478[28] = 0;
   out_5608595110335033478[29] = 0;
   out_5608595110335033478[30] = 0;
   out_5608595110335033478[31] = 0;
   out_5608595110335033478[32] = 0;
   out_5608595110335033478[33] = 0;
   out_5608595110335033478[34] = 0;
   out_5608595110335033478[35] = 0;
   out_5608595110335033478[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5608595110335033478[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5608595110335033478[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5608595110335033478[39] = 0;
   out_5608595110335033478[40] = 0;
   out_5608595110335033478[41] = 0;
   out_5608595110335033478[42] = 0;
   out_5608595110335033478[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5608595110335033478[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5608595110335033478[45] = 0;
   out_5608595110335033478[46] = 0;
   out_5608595110335033478[47] = 0;
   out_5608595110335033478[48] = 0;
   out_5608595110335033478[49] = 0;
   out_5608595110335033478[50] = 0;
   out_5608595110335033478[51] = 0;
   out_5608595110335033478[52] = 0;
   out_5608595110335033478[53] = 0;
   out_5608595110335033478[54] = 0;
   out_5608595110335033478[55] = 0;
   out_5608595110335033478[56] = 0;
   out_5608595110335033478[57] = 1;
   out_5608595110335033478[58] = 0;
   out_5608595110335033478[59] = 0;
   out_5608595110335033478[60] = 0;
   out_5608595110335033478[61] = 0;
   out_5608595110335033478[62] = 0;
   out_5608595110335033478[63] = 0;
   out_5608595110335033478[64] = 0;
   out_5608595110335033478[65] = 0;
   out_5608595110335033478[66] = dt;
   out_5608595110335033478[67] = 0;
   out_5608595110335033478[68] = 0;
   out_5608595110335033478[69] = 0;
   out_5608595110335033478[70] = 0;
   out_5608595110335033478[71] = 0;
   out_5608595110335033478[72] = 0;
   out_5608595110335033478[73] = 0;
   out_5608595110335033478[74] = 0;
   out_5608595110335033478[75] = 0;
   out_5608595110335033478[76] = 1;
   out_5608595110335033478[77] = 0;
   out_5608595110335033478[78] = 0;
   out_5608595110335033478[79] = 0;
   out_5608595110335033478[80] = 0;
   out_5608595110335033478[81] = 0;
   out_5608595110335033478[82] = 0;
   out_5608595110335033478[83] = 0;
   out_5608595110335033478[84] = 0;
   out_5608595110335033478[85] = dt;
   out_5608595110335033478[86] = 0;
   out_5608595110335033478[87] = 0;
   out_5608595110335033478[88] = 0;
   out_5608595110335033478[89] = 0;
   out_5608595110335033478[90] = 0;
   out_5608595110335033478[91] = 0;
   out_5608595110335033478[92] = 0;
   out_5608595110335033478[93] = 0;
   out_5608595110335033478[94] = 0;
   out_5608595110335033478[95] = 1;
   out_5608595110335033478[96] = 0;
   out_5608595110335033478[97] = 0;
   out_5608595110335033478[98] = 0;
   out_5608595110335033478[99] = 0;
   out_5608595110335033478[100] = 0;
   out_5608595110335033478[101] = 0;
   out_5608595110335033478[102] = 0;
   out_5608595110335033478[103] = 0;
   out_5608595110335033478[104] = dt;
   out_5608595110335033478[105] = 0;
   out_5608595110335033478[106] = 0;
   out_5608595110335033478[107] = 0;
   out_5608595110335033478[108] = 0;
   out_5608595110335033478[109] = 0;
   out_5608595110335033478[110] = 0;
   out_5608595110335033478[111] = 0;
   out_5608595110335033478[112] = 0;
   out_5608595110335033478[113] = 0;
   out_5608595110335033478[114] = 1;
   out_5608595110335033478[115] = 0;
   out_5608595110335033478[116] = 0;
   out_5608595110335033478[117] = 0;
   out_5608595110335033478[118] = 0;
   out_5608595110335033478[119] = 0;
   out_5608595110335033478[120] = 0;
   out_5608595110335033478[121] = 0;
   out_5608595110335033478[122] = 0;
   out_5608595110335033478[123] = 0;
   out_5608595110335033478[124] = 0;
   out_5608595110335033478[125] = 0;
   out_5608595110335033478[126] = 0;
   out_5608595110335033478[127] = 0;
   out_5608595110335033478[128] = 0;
   out_5608595110335033478[129] = 0;
   out_5608595110335033478[130] = 0;
   out_5608595110335033478[131] = 0;
   out_5608595110335033478[132] = 0;
   out_5608595110335033478[133] = 1;
   out_5608595110335033478[134] = 0;
   out_5608595110335033478[135] = 0;
   out_5608595110335033478[136] = 0;
   out_5608595110335033478[137] = 0;
   out_5608595110335033478[138] = 0;
   out_5608595110335033478[139] = 0;
   out_5608595110335033478[140] = 0;
   out_5608595110335033478[141] = 0;
   out_5608595110335033478[142] = 0;
   out_5608595110335033478[143] = 0;
   out_5608595110335033478[144] = 0;
   out_5608595110335033478[145] = 0;
   out_5608595110335033478[146] = 0;
   out_5608595110335033478[147] = 0;
   out_5608595110335033478[148] = 0;
   out_5608595110335033478[149] = 0;
   out_5608595110335033478[150] = 0;
   out_5608595110335033478[151] = 0;
   out_5608595110335033478[152] = 1;
   out_5608595110335033478[153] = 0;
   out_5608595110335033478[154] = 0;
   out_5608595110335033478[155] = 0;
   out_5608595110335033478[156] = 0;
   out_5608595110335033478[157] = 0;
   out_5608595110335033478[158] = 0;
   out_5608595110335033478[159] = 0;
   out_5608595110335033478[160] = 0;
   out_5608595110335033478[161] = 0;
   out_5608595110335033478[162] = 0;
   out_5608595110335033478[163] = 0;
   out_5608595110335033478[164] = 0;
   out_5608595110335033478[165] = 0;
   out_5608595110335033478[166] = 0;
   out_5608595110335033478[167] = 0;
   out_5608595110335033478[168] = 0;
   out_5608595110335033478[169] = 0;
   out_5608595110335033478[170] = 0;
   out_5608595110335033478[171] = 1;
   out_5608595110335033478[172] = 0;
   out_5608595110335033478[173] = 0;
   out_5608595110335033478[174] = 0;
   out_5608595110335033478[175] = 0;
   out_5608595110335033478[176] = 0;
   out_5608595110335033478[177] = 0;
   out_5608595110335033478[178] = 0;
   out_5608595110335033478[179] = 0;
   out_5608595110335033478[180] = 0;
   out_5608595110335033478[181] = 0;
   out_5608595110335033478[182] = 0;
   out_5608595110335033478[183] = 0;
   out_5608595110335033478[184] = 0;
   out_5608595110335033478[185] = 0;
   out_5608595110335033478[186] = 0;
   out_5608595110335033478[187] = 0;
   out_5608595110335033478[188] = 0;
   out_5608595110335033478[189] = 0;
   out_5608595110335033478[190] = 1;
   out_5608595110335033478[191] = 0;
   out_5608595110335033478[192] = 0;
   out_5608595110335033478[193] = 0;
   out_5608595110335033478[194] = 0;
   out_5608595110335033478[195] = 0;
   out_5608595110335033478[196] = 0;
   out_5608595110335033478[197] = 0;
   out_5608595110335033478[198] = 0;
   out_5608595110335033478[199] = 0;
   out_5608595110335033478[200] = 0;
   out_5608595110335033478[201] = 0;
   out_5608595110335033478[202] = 0;
   out_5608595110335033478[203] = 0;
   out_5608595110335033478[204] = 0;
   out_5608595110335033478[205] = 0;
   out_5608595110335033478[206] = 0;
   out_5608595110335033478[207] = 0;
   out_5608595110335033478[208] = 0;
   out_5608595110335033478[209] = 1;
   out_5608595110335033478[210] = 0;
   out_5608595110335033478[211] = 0;
   out_5608595110335033478[212] = 0;
   out_5608595110335033478[213] = 0;
   out_5608595110335033478[214] = 0;
   out_5608595110335033478[215] = 0;
   out_5608595110335033478[216] = 0;
   out_5608595110335033478[217] = 0;
   out_5608595110335033478[218] = 0;
   out_5608595110335033478[219] = 0;
   out_5608595110335033478[220] = 0;
   out_5608595110335033478[221] = 0;
   out_5608595110335033478[222] = 0;
   out_5608595110335033478[223] = 0;
   out_5608595110335033478[224] = 0;
   out_5608595110335033478[225] = 0;
   out_5608595110335033478[226] = 0;
   out_5608595110335033478[227] = 0;
   out_5608595110335033478[228] = 1;
   out_5608595110335033478[229] = 0;
   out_5608595110335033478[230] = 0;
   out_5608595110335033478[231] = 0;
   out_5608595110335033478[232] = 0;
   out_5608595110335033478[233] = 0;
   out_5608595110335033478[234] = 0;
   out_5608595110335033478[235] = 0;
   out_5608595110335033478[236] = 0;
   out_5608595110335033478[237] = 0;
   out_5608595110335033478[238] = 0;
   out_5608595110335033478[239] = 0;
   out_5608595110335033478[240] = 0;
   out_5608595110335033478[241] = 0;
   out_5608595110335033478[242] = 0;
   out_5608595110335033478[243] = 0;
   out_5608595110335033478[244] = 0;
   out_5608595110335033478[245] = 0;
   out_5608595110335033478[246] = 0;
   out_5608595110335033478[247] = 1;
   out_5608595110335033478[248] = 0;
   out_5608595110335033478[249] = 0;
   out_5608595110335033478[250] = 0;
   out_5608595110335033478[251] = 0;
   out_5608595110335033478[252] = 0;
   out_5608595110335033478[253] = 0;
   out_5608595110335033478[254] = 0;
   out_5608595110335033478[255] = 0;
   out_5608595110335033478[256] = 0;
   out_5608595110335033478[257] = 0;
   out_5608595110335033478[258] = 0;
   out_5608595110335033478[259] = 0;
   out_5608595110335033478[260] = 0;
   out_5608595110335033478[261] = 0;
   out_5608595110335033478[262] = 0;
   out_5608595110335033478[263] = 0;
   out_5608595110335033478[264] = 0;
   out_5608595110335033478[265] = 0;
   out_5608595110335033478[266] = 1;
   out_5608595110335033478[267] = 0;
   out_5608595110335033478[268] = 0;
   out_5608595110335033478[269] = 0;
   out_5608595110335033478[270] = 0;
   out_5608595110335033478[271] = 0;
   out_5608595110335033478[272] = 0;
   out_5608595110335033478[273] = 0;
   out_5608595110335033478[274] = 0;
   out_5608595110335033478[275] = 0;
   out_5608595110335033478[276] = 0;
   out_5608595110335033478[277] = 0;
   out_5608595110335033478[278] = 0;
   out_5608595110335033478[279] = 0;
   out_5608595110335033478[280] = 0;
   out_5608595110335033478[281] = 0;
   out_5608595110335033478[282] = 0;
   out_5608595110335033478[283] = 0;
   out_5608595110335033478[284] = 0;
   out_5608595110335033478[285] = 1;
   out_5608595110335033478[286] = 0;
   out_5608595110335033478[287] = 0;
   out_5608595110335033478[288] = 0;
   out_5608595110335033478[289] = 0;
   out_5608595110335033478[290] = 0;
   out_5608595110335033478[291] = 0;
   out_5608595110335033478[292] = 0;
   out_5608595110335033478[293] = 0;
   out_5608595110335033478[294] = 0;
   out_5608595110335033478[295] = 0;
   out_5608595110335033478[296] = 0;
   out_5608595110335033478[297] = 0;
   out_5608595110335033478[298] = 0;
   out_5608595110335033478[299] = 0;
   out_5608595110335033478[300] = 0;
   out_5608595110335033478[301] = 0;
   out_5608595110335033478[302] = 0;
   out_5608595110335033478[303] = 0;
   out_5608595110335033478[304] = 1;
   out_5608595110335033478[305] = 0;
   out_5608595110335033478[306] = 0;
   out_5608595110335033478[307] = 0;
   out_5608595110335033478[308] = 0;
   out_5608595110335033478[309] = 0;
   out_5608595110335033478[310] = 0;
   out_5608595110335033478[311] = 0;
   out_5608595110335033478[312] = 0;
   out_5608595110335033478[313] = 0;
   out_5608595110335033478[314] = 0;
   out_5608595110335033478[315] = 0;
   out_5608595110335033478[316] = 0;
   out_5608595110335033478[317] = 0;
   out_5608595110335033478[318] = 0;
   out_5608595110335033478[319] = 0;
   out_5608595110335033478[320] = 0;
   out_5608595110335033478[321] = 0;
   out_5608595110335033478[322] = 0;
   out_5608595110335033478[323] = 1;
}
void h_4(double *state, double *unused, double *out_6884326003512242794) {
   out_6884326003512242794[0] = state[6] + state[9];
   out_6884326003512242794[1] = state[7] + state[10];
   out_6884326003512242794[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_2658547600726984974) {
   out_2658547600726984974[0] = 0;
   out_2658547600726984974[1] = 0;
   out_2658547600726984974[2] = 0;
   out_2658547600726984974[3] = 0;
   out_2658547600726984974[4] = 0;
   out_2658547600726984974[5] = 0;
   out_2658547600726984974[6] = 1;
   out_2658547600726984974[7] = 0;
   out_2658547600726984974[8] = 0;
   out_2658547600726984974[9] = 1;
   out_2658547600726984974[10] = 0;
   out_2658547600726984974[11] = 0;
   out_2658547600726984974[12] = 0;
   out_2658547600726984974[13] = 0;
   out_2658547600726984974[14] = 0;
   out_2658547600726984974[15] = 0;
   out_2658547600726984974[16] = 0;
   out_2658547600726984974[17] = 0;
   out_2658547600726984974[18] = 0;
   out_2658547600726984974[19] = 0;
   out_2658547600726984974[20] = 0;
   out_2658547600726984974[21] = 0;
   out_2658547600726984974[22] = 0;
   out_2658547600726984974[23] = 0;
   out_2658547600726984974[24] = 0;
   out_2658547600726984974[25] = 1;
   out_2658547600726984974[26] = 0;
   out_2658547600726984974[27] = 0;
   out_2658547600726984974[28] = 1;
   out_2658547600726984974[29] = 0;
   out_2658547600726984974[30] = 0;
   out_2658547600726984974[31] = 0;
   out_2658547600726984974[32] = 0;
   out_2658547600726984974[33] = 0;
   out_2658547600726984974[34] = 0;
   out_2658547600726984974[35] = 0;
   out_2658547600726984974[36] = 0;
   out_2658547600726984974[37] = 0;
   out_2658547600726984974[38] = 0;
   out_2658547600726984974[39] = 0;
   out_2658547600726984974[40] = 0;
   out_2658547600726984974[41] = 0;
   out_2658547600726984974[42] = 0;
   out_2658547600726984974[43] = 0;
   out_2658547600726984974[44] = 1;
   out_2658547600726984974[45] = 0;
   out_2658547600726984974[46] = 0;
   out_2658547600726984974[47] = 1;
   out_2658547600726984974[48] = 0;
   out_2658547600726984974[49] = 0;
   out_2658547600726984974[50] = 0;
   out_2658547600726984974[51] = 0;
   out_2658547600726984974[52] = 0;
   out_2658547600726984974[53] = 0;
}
void h_10(double *state, double *unused, double *out_8751108212963429724) {
   out_8751108212963429724[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_8751108212963429724[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_8751108212963429724[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_9115722006580025533) {
   out_9115722006580025533[0] = 0;
   out_9115722006580025533[1] = 9.8100000000000005*cos(state[1]);
   out_9115722006580025533[2] = 0;
   out_9115722006580025533[3] = 0;
   out_9115722006580025533[4] = -state[8];
   out_9115722006580025533[5] = state[7];
   out_9115722006580025533[6] = 0;
   out_9115722006580025533[7] = state[5];
   out_9115722006580025533[8] = -state[4];
   out_9115722006580025533[9] = 0;
   out_9115722006580025533[10] = 0;
   out_9115722006580025533[11] = 0;
   out_9115722006580025533[12] = 1;
   out_9115722006580025533[13] = 0;
   out_9115722006580025533[14] = 0;
   out_9115722006580025533[15] = 1;
   out_9115722006580025533[16] = 0;
   out_9115722006580025533[17] = 0;
   out_9115722006580025533[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_9115722006580025533[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_9115722006580025533[20] = 0;
   out_9115722006580025533[21] = state[8];
   out_9115722006580025533[22] = 0;
   out_9115722006580025533[23] = -state[6];
   out_9115722006580025533[24] = -state[5];
   out_9115722006580025533[25] = 0;
   out_9115722006580025533[26] = state[3];
   out_9115722006580025533[27] = 0;
   out_9115722006580025533[28] = 0;
   out_9115722006580025533[29] = 0;
   out_9115722006580025533[30] = 0;
   out_9115722006580025533[31] = 1;
   out_9115722006580025533[32] = 0;
   out_9115722006580025533[33] = 0;
   out_9115722006580025533[34] = 1;
   out_9115722006580025533[35] = 0;
   out_9115722006580025533[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_9115722006580025533[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_9115722006580025533[38] = 0;
   out_9115722006580025533[39] = -state[7];
   out_9115722006580025533[40] = state[6];
   out_9115722006580025533[41] = 0;
   out_9115722006580025533[42] = state[4];
   out_9115722006580025533[43] = -state[3];
   out_9115722006580025533[44] = 0;
   out_9115722006580025533[45] = 0;
   out_9115722006580025533[46] = 0;
   out_9115722006580025533[47] = 0;
   out_9115722006580025533[48] = 0;
   out_9115722006580025533[49] = 0;
   out_9115722006580025533[50] = 1;
   out_9115722006580025533[51] = 0;
   out_9115722006580025533[52] = 0;
   out_9115722006580025533[53] = 1;
}
void h_13(double *state, double *unused, double *out_4274029695233563681) {
   out_4274029695233563681[0] = state[3];
   out_4274029695233563681[1] = state[4];
   out_4274029695233563681[2] = state[5];
}
void H_13(double *state, double *unused, double *out_5870821426059317775) {
   out_5870821426059317775[0] = 0;
   out_5870821426059317775[1] = 0;
   out_5870821426059317775[2] = 0;
   out_5870821426059317775[3] = 1;
   out_5870821426059317775[4] = 0;
   out_5870821426059317775[5] = 0;
   out_5870821426059317775[6] = 0;
   out_5870821426059317775[7] = 0;
   out_5870821426059317775[8] = 0;
   out_5870821426059317775[9] = 0;
   out_5870821426059317775[10] = 0;
   out_5870821426059317775[11] = 0;
   out_5870821426059317775[12] = 0;
   out_5870821426059317775[13] = 0;
   out_5870821426059317775[14] = 0;
   out_5870821426059317775[15] = 0;
   out_5870821426059317775[16] = 0;
   out_5870821426059317775[17] = 0;
   out_5870821426059317775[18] = 0;
   out_5870821426059317775[19] = 0;
   out_5870821426059317775[20] = 0;
   out_5870821426059317775[21] = 0;
   out_5870821426059317775[22] = 1;
   out_5870821426059317775[23] = 0;
   out_5870821426059317775[24] = 0;
   out_5870821426059317775[25] = 0;
   out_5870821426059317775[26] = 0;
   out_5870821426059317775[27] = 0;
   out_5870821426059317775[28] = 0;
   out_5870821426059317775[29] = 0;
   out_5870821426059317775[30] = 0;
   out_5870821426059317775[31] = 0;
   out_5870821426059317775[32] = 0;
   out_5870821426059317775[33] = 0;
   out_5870821426059317775[34] = 0;
   out_5870821426059317775[35] = 0;
   out_5870821426059317775[36] = 0;
   out_5870821426059317775[37] = 0;
   out_5870821426059317775[38] = 0;
   out_5870821426059317775[39] = 0;
   out_5870821426059317775[40] = 0;
   out_5870821426059317775[41] = 1;
   out_5870821426059317775[42] = 0;
   out_5870821426059317775[43] = 0;
   out_5870821426059317775[44] = 0;
   out_5870821426059317775[45] = 0;
   out_5870821426059317775[46] = 0;
   out_5870821426059317775[47] = 0;
   out_5870821426059317775[48] = 0;
   out_5870821426059317775[49] = 0;
   out_5870821426059317775[50] = 0;
   out_5870821426059317775[51] = 0;
   out_5870821426059317775[52] = 0;
   out_5870821426059317775[53] = 0;
}
void h_14(double *state, double *unused, double *out_6042798886861941258) {
   out_6042798886861941258[0] = state[6];
   out_6042798886861941258[1] = state[7];
   out_6042798886861941258[2] = state[8];
}
void H_14(double *state, double *unused, double *out_6621788457066469503) {
   out_6621788457066469503[0] = 0;
   out_6621788457066469503[1] = 0;
   out_6621788457066469503[2] = 0;
   out_6621788457066469503[3] = 0;
   out_6621788457066469503[4] = 0;
   out_6621788457066469503[5] = 0;
   out_6621788457066469503[6] = 1;
   out_6621788457066469503[7] = 0;
   out_6621788457066469503[8] = 0;
   out_6621788457066469503[9] = 0;
   out_6621788457066469503[10] = 0;
   out_6621788457066469503[11] = 0;
   out_6621788457066469503[12] = 0;
   out_6621788457066469503[13] = 0;
   out_6621788457066469503[14] = 0;
   out_6621788457066469503[15] = 0;
   out_6621788457066469503[16] = 0;
   out_6621788457066469503[17] = 0;
   out_6621788457066469503[18] = 0;
   out_6621788457066469503[19] = 0;
   out_6621788457066469503[20] = 0;
   out_6621788457066469503[21] = 0;
   out_6621788457066469503[22] = 0;
   out_6621788457066469503[23] = 0;
   out_6621788457066469503[24] = 0;
   out_6621788457066469503[25] = 1;
   out_6621788457066469503[26] = 0;
   out_6621788457066469503[27] = 0;
   out_6621788457066469503[28] = 0;
   out_6621788457066469503[29] = 0;
   out_6621788457066469503[30] = 0;
   out_6621788457066469503[31] = 0;
   out_6621788457066469503[32] = 0;
   out_6621788457066469503[33] = 0;
   out_6621788457066469503[34] = 0;
   out_6621788457066469503[35] = 0;
   out_6621788457066469503[36] = 0;
   out_6621788457066469503[37] = 0;
   out_6621788457066469503[38] = 0;
   out_6621788457066469503[39] = 0;
   out_6621788457066469503[40] = 0;
   out_6621788457066469503[41] = 0;
   out_6621788457066469503[42] = 0;
   out_6621788457066469503[43] = 0;
   out_6621788457066469503[44] = 1;
   out_6621788457066469503[45] = 0;
   out_6621788457066469503[46] = 0;
   out_6621788457066469503[47] = 0;
   out_6621788457066469503[48] = 0;
   out_6621788457066469503[49] = 0;
   out_6621788457066469503[50] = 0;
   out_6621788457066469503[51] = 0;
   out_6621788457066469503[52] = 0;
   out_6621788457066469503[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_3399544854207247872) {
  err_fun(nom_x, delta_x, out_3399544854207247872);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6784639363269531457) {
  inv_err_fun(nom_x, true_x, out_6784639363269531457);
}
void pose_H_mod_fun(double *state, double *out_3714060151064540699) {
  H_mod_fun(state, out_3714060151064540699);
}
void pose_f_fun(double *state, double dt, double *out_8177302778648908743) {
  f_fun(state,  dt, out_8177302778648908743);
}
void pose_F_fun(double *state, double dt, double *out_5608595110335033478) {
  F_fun(state,  dt, out_5608595110335033478);
}
void pose_h_4(double *state, double *unused, double *out_6884326003512242794) {
  h_4(state, unused, out_6884326003512242794);
}
void pose_H_4(double *state, double *unused, double *out_2658547600726984974) {
  H_4(state, unused, out_2658547600726984974);
}
void pose_h_10(double *state, double *unused, double *out_8751108212963429724) {
  h_10(state, unused, out_8751108212963429724);
}
void pose_H_10(double *state, double *unused, double *out_9115722006580025533) {
  H_10(state, unused, out_9115722006580025533);
}
void pose_h_13(double *state, double *unused, double *out_4274029695233563681) {
  h_13(state, unused, out_4274029695233563681);
}
void pose_H_13(double *state, double *unused, double *out_5870821426059317775) {
  H_13(state, unused, out_5870821426059317775);
}
void pose_h_14(double *state, double *unused, double *out_6042798886861941258) {
  h_14(state, unused, out_6042798886861941258);
}
void pose_H_14(double *state, double *unused, double *out_6621788457066469503) {
  H_14(state, unused, out_6621788457066469503);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
