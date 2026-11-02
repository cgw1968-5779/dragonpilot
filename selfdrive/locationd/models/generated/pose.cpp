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
void err_fun(double *nom_x, double *delta_x, double *out_6415792483428760966) {
   out_6415792483428760966[0] = delta_x[0] + nom_x[0];
   out_6415792483428760966[1] = delta_x[1] + nom_x[1];
   out_6415792483428760966[2] = delta_x[2] + nom_x[2];
   out_6415792483428760966[3] = delta_x[3] + nom_x[3];
   out_6415792483428760966[4] = delta_x[4] + nom_x[4];
   out_6415792483428760966[5] = delta_x[5] + nom_x[5];
   out_6415792483428760966[6] = delta_x[6] + nom_x[6];
   out_6415792483428760966[7] = delta_x[7] + nom_x[7];
   out_6415792483428760966[8] = delta_x[8] + nom_x[8];
   out_6415792483428760966[9] = delta_x[9] + nom_x[9];
   out_6415792483428760966[10] = delta_x[10] + nom_x[10];
   out_6415792483428760966[11] = delta_x[11] + nom_x[11];
   out_6415792483428760966[12] = delta_x[12] + nom_x[12];
   out_6415792483428760966[13] = delta_x[13] + nom_x[13];
   out_6415792483428760966[14] = delta_x[14] + nom_x[14];
   out_6415792483428760966[15] = delta_x[15] + nom_x[15];
   out_6415792483428760966[16] = delta_x[16] + nom_x[16];
   out_6415792483428760966[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6377608227770965724) {
   out_6377608227770965724[0] = -nom_x[0] + true_x[0];
   out_6377608227770965724[1] = -nom_x[1] + true_x[1];
   out_6377608227770965724[2] = -nom_x[2] + true_x[2];
   out_6377608227770965724[3] = -nom_x[3] + true_x[3];
   out_6377608227770965724[4] = -nom_x[4] + true_x[4];
   out_6377608227770965724[5] = -nom_x[5] + true_x[5];
   out_6377608227770965724[6] = -nom_x[6] + true_x[6];
   out_6377608227770965724[7] = -nom_x[7] + true_x[7];
   out_6377608227770965724[8] = -nom_x[8] + true_x[8];
   out_6377608227770965724[9] = -nom_x[9] + true_x[9];
   out_6377608227770965724[10] = -nom_x[10] + true_x[10];
   out_6377608227770965724[11] = -nom_x[11] + true_x[11];
   out_6377608227770965724[12] = -nom_x[12] + true_x[12];
   out_6377608227770965724[13] = -nom_x[13] + true_x[13];
   out_6377608227770965724[14] = -nom_x[14] + true_x[14];
   out_6377608227770965724[15] = -nom_x[15] + true_x[15];
   out_6377608227770965724[16] = -nom_x[16] + true_x[16];
   out_6377608227770965724[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_6347051622433270619) {
   out_6347051622433270619[0] = 1.0;
   out_6347051622433270619[1] = 0.0;
   out_6347051622433270619[2] = 0.0;
   out_6347051622433270619[3] = 0.0;
   out_6347051622433270619[4] = 0.0;
   out_6347051622433270619[5] = 0.0;
   out_6347051622433270619[6] = 0.0;
   out_6347051622433270619[7] = 0.0;
   out_6347051622433270619[8] = 0.0;
   out_6347051622433270619[9] = 0.0;
   out_6347051622433270619[10] = 0.0;
   out_6347051622433270619[11] = 0.0;
   out_6347051622433270619[12] = 0.0;
   out_6347051622433270619[13] = 0.0;
   out_6347051622433270619[14] = 0.0;
   out_6347051622433270619[15] = 0.0;
   out_6347051622433270619[16] = 0.0;
   out_6347051622433270619[17] = 0.0;
   out_6347051622433270619[18] = 0.0;
   out_6347051622433270619[19] = 1.0;
   out_6347051622433270619[20] = 0.0;
   out_6347051622433270619[21] = 0.0;
   out_6347051622433270619[22] = 0.0;
   out_6347051622433270619[23] = 0.0;
   out_6347051622433270619[24] = 0.0;
   out_6347051622433270619[25] = 0.0;
   out_6347051622433270619[26] = 0.0;
   out_6347051622433270619[27] = 0.0;
   out_6347051622433270619[28] = 0.0;
   out_6347051622433270619[29] = 0.0;
   out_6347051622433270619[30] = 0.0;
   out_6347051622433270619[31] = 0.0;
   out_6347051622433270619[32] = 0.0;
   out_6347051622433270619[33] = 0.0;
   out_6347051622433270619[34] = 0.0;
   out_6347051622433270619[35] = 0.0;
   out_6347051622433270619[36] = 0.0;
   out_6347051622433270619[37] = 0.0;
   out_6347051622433270619[38] = 1.0;
   out_6347051622433270619[39] = 0.0;
   out_6347051622433270619[40] = 0.0;
   out_6347051622433270619[41] = 0.0;
   out_6347051622433270619[42] = 0.0;
   out_6347051622433270619[43] = 0.0;
   out_6347051622433270619[44] = 0.0;
   out_6347051622433270619[45] = 0.0;
   out_6347051622433270619[46] = 0.0;
   out_6347051622433270619[47] = 0.0;
   out_6347051622433270619[48] = 0.0;
   out_6347051622433270619[49] = 0.0;
   out_6347051622433270619[50] = 0.0;
   out_6347051622433270619[51] = 0.0;
   out_6347051622433270619[52] = 0.0;
   out_6347051622433270619[53] = 0.0;
   out_6347051622433270619[54] = 0.0;
   out_6347051622433270619[55] = 0.0;
   out_6347051622433270619[56] = 0.0;
   out_6347051622433270619[57] = 1.0;
   out_6347051622433270619[58] = 0.0;
   out_6347051622433270619[59] = 0.0;
   out_6347051622433270619[60] = 0.0;
   out_6347051622433270619[61] = 0.0;
   out_6347051622433270619[62] = 0.0;
   out_6347051622433270619[63] = 0.0;
   out_6347051622433270619[64] = 0.0;
   out_6347051622433270619[65] = 0.0;
   out_6347051622433270619[66] = 0.0;
   out_6347051622433270619[67] = 0.0;
   out_6347051622433270619[68] = 0.0;
   out_6347051622433270619[69] = 0.0;
   out_6347051622433270619[70] = 0.0;
   out_6347051622433270619[71] = 0.0;
   out_6347051622433270619[72] = 0.0;
   out_6347051622433270619[73] = 0.0;
   out_6347051622433270619[74] = 0.0;
   out_6347051622433270619[75] = 0.0;
   out_6347051622433270619[76] = 1.0;
   out_6347051622433270619[77] = 0.0;
   out_6347051622433270619[78] = 0.0;
   out_6347051622433270619[79] = 0.0;
   out_6347051622433270619[80] = 0.0;
   out_6347051622433270619[81] = 0.0;
   out_6347051622433270619[82] = 0.0;
   out_6347051622433270619[83] = 0.0;
   out_6347051622433270619[84] = 0.0;
   out_6347051622433270619[85] = 0.0;
   out_6347051622433270619[86] = 0.0;
   out_6347051622433270619[87] = 0.0;
   out_6347051622433270619[88] = 0.0;
   out_6347051622433270619[89] = 0.0;
   out_6347051622433270619[90] = 0.0;
   out_6347051622433270619[91] = 0.0;
   out_6347051622433270619[92] = 0.0;
   out_6347051622433270619[93] = 0.0;
   out_6347051622433270619[94] = 0.0;
   out_6347051622433270619[95] = 1.0;
   out_6347051622433270619[96] = 0.0;
   out_6347051622433270619[97] = 0.0;
   out_6347051622433270619[98] = 0.0;
   out_6347051622433270619[99] = 0.0;
   out_6347051622433270619[100] = 0.0;
   out_6347051622433270619[101] = 0.0;
   out_6347051622433270619[102] = 0.0;
   out_6347051622433270619[103] = 0.0;
   out_6347051622433270619[104] = 0.0;
   out_6347051622433270619[105] = 0.0;
   out_6347051622433270619[106] = 0.0;
   out_6347051622433270619[107] = 0.0;
   out_6347051622433270619[108] = 0.0;
   out_6347051622433270619[109] = 0.0;
   out_6347051622433270619[110] = 0.0;
   out_6347051622433270619[111] = 0.0;
   out_6347051622433270619[112] = 0.0;
   out_6347051622433270619[113] = 0.0;
   out_6347051622433270619[114] = 1.0;
   out_6347051622433270619[115] = 0.0;
   out_6347051622433270619[116] = 0.0;
   out_6347051622433270619[117] = 0.0;
   out_6347051622433270619[118] = 0.0;
   out_6347051622433270619[119] = 0.0;
   out_6347051622433270619[120] = 0.0;
   out_6347051622433270619[121] = 0.0;
   out_6347051622433270619[122] = 0.0;
   out_6347051622433270619[123] = 0.0;
   out_6347051622433270619[124] = 0.0;
   out_6347051622433270619[125] = 0.0;
   out_6347051622433270619[126] = 0.0;
   out_6347051622433270619[127] = 0.0;
   out_6347051622433270619[128] = 0.0;
   out_6347051622433270619[129] = 0.0;
   out_6347051622433270619[130] = 0.0;
   out_6347051622433270619[131] = 0.0;
   out_6347051622433270619[132] = 0.0;
   out_6347051622433270619[133] = 1.0;
   out_6347051622433270619[134] = 0.0;
   out_6347051622433270619[135] = 0.0;
   out_6347051622433270619[136] = 0.0;
   out_6347051622433270619[137] = 0.0;
   out_6347051622433270619[138] = 0.0;
   out_6347051622433270619[139] = 0.0;
   out_6347051622433270619[140] = 0.0;
   out_6347051622433270619[141] = 0.0;
   out_6347051622433270619[142] = 0.0;
   out_6347051622433270619[143] = 0.0;
   out_6347051622433270619[144] = 0.0;
   out_6347051622433270619[145] = 0.0;
   out_6347051622433270619[146] = 0.0;
   out_6347051622433270619[147] = 0.0;
   out_6347051622433270619[148] = 0.0;
   out_6347051622433270619[149] = 0.0;
   out_6347051622433270619[150] = 0.0;
   out_6347051622433270619[151] = 0.0;
   out_6347051622433270619[152] = 1.0;
   out_6347051622433270619[153] = 0.0;
   out_6347051622433270619[154] = 0.0;
   out_6347051622433270619[155] = 0.0;
   out_6347051622433270619[156] = 0.0;
   out_6347051622433270619[157] = 0.0;
   out_6347051622433270619[158] = 0.0;
   out_6347051622433270619[159] = 0.0;
   out_6347051622433270619[160] = 0.0;
   out_6347051622433270619[161] = 0.0;
   out_6347051622433270619[162] = 0.0;
   out_6347051622433270619[163] = 0.0;
   out_6347051622433270619[164] = 0.0;
   out_6347051622433270619[165] = 0.0;
   out_6347051622433270619[166] = 0.0;
   out_6347051622433270619[167] = 0.0;
   out_6347051622433270619[168] = 0.0;
   out_6347051622433270619[169] = 0.0;
   out_6347051622433270619[170] = 0.0;
   out_6347051622433270619[171] = 1.0;
   out_6347051622433270619[172] = 0.0;
   out_6347051622433270619[173] = 0.0;
   out_6347051622433270619[174] = 0.0;
   out_6347051622433270619[175] = 0.0;
   out_6347051622433270619[176] = 0.0;
   out_6347051622433270619[177] = 0.0;
   out_6347051622433270619[178] = 0.0;
   out_6347051622433270619[179] = 0.0;
   out_6347051622433270619[180] = 0.0;
   out_6347051622433270619[181] = 0.0;
   out_6347051622433270619[182] = 0.0;
   out_6347051622433270619[183] = 0.0;
   out_6347051622433270619[184] = 0.0;
   out_6347051622433270619[185] = 0.0;
   out_6347051622433270619[186] = 0.0;
   out_6347051622433270619[187] = 0.0;
   out_6347051622433270619[188] = 0.0;
   out_6347051622433270619[189] = 0.0;
   out_6347051622433270619[190] = 1.0;
   out_6347051622433270619[191] = 0.0;
   out_6347051622433270619[192] = 0.0;
   out_6347051622433270619[193] = 0.0;
   out_6347051622433270619[194] = 0.0;
   out_6347051622433270619[195] = 0.0;
   out_6347051622433270619[196] = 0.0;
   out_6347051622433270619[197] = 0.0;
   out_6347051622433270619[198] = 0.0;
   out_6347051622433270619[199] = 0.0;
   out_6347051622433270619[200] = 0.0;
   out_6347051622433270619[201] = 0.0;
   out_6347051622433270619[202] = 0.0;
   out_6347051622433270619[203] = 0.0;
   out_6347051622433270619[204] = 0.0;
   out_6347051622433270619[205] = 0.0;
   out_6347051622433270619[206] = 0.0;
   out_6347051622433270619[207] = 0.0;
   out_6347051622433270619[208] = 0.0;
   out_6347051622433270619[209] = 1.0;
   out_6347051622433270619[210] = 0.0;
   out_6347051622433270619[211] = 0.0;
   out_6347051622433270619[212] = 0.0;
   out_6347051622433270619[213] = 0.0;
   out_6347051622433270619[214] = 0.0;
   out_6347051622433270619[215] = 0.0;
   out_6347051622433270619[216] = 0.0;
   out_6347051622433270619[217] = 0.0;
   out_6347051622433270619[218] = 0.0;
   out_6347051622433270619[219] = 0.0;
   out_6347051622433270619[220] = 0.0;
   out_6347051622433270619[221] = 0.0;
   out_6347051622433270619[222] = 0.0;
   out_6347051622433270619[223] = 0.0;
   out_6347051622433270619[224] = 0.0;
   out_6347051622433270619[225] = 0.0;
   out_6347051622433270619[226] = 0.0;
   out_6347051622433270619[227] = 0.0;
   out_6347051622433270619[228] = 1.0;
   out_6347051622433270619[229] = 0.0;
   out_6347051622433270619[230] = 0.0;
   out_6347051622433270619[231] = 0.0;
   out_6347051622433270619[232] = 0.0;
   out_6347051622433270619[233] = 0.0;
   out_6347051622433270619[234] = 0.0;
   out_6347051622433270619[235] = 0.0;
   out_6347051622433270619[236] = 0.0;
   out_6347051622433270619[237] = 0.0;
   out_6347051622433270619[238] = 0.0;
   out_6347051622433270619[239] = 0.0;
   out_6347051622433270619[240] = 0.0;
   out_6347051622433270619[241] = 0.0;
   out_6347051622433270619[242] = 0.0;
   out_6347051622433270619[243] = 0.0;
   out_6347051622433270619[244] = 0.0;
   out_6347051622433270619[245] = 0.0;
   out_6347051622433270619[246] = 0.0;
   out_6347051622433270619[247] = 1.0;
   out_6347051622433270619[248] = 0.0;
   out_6347051622433270619[249] = 0.0;
   out_6347051622433270619[250] = 0.0;
   out_6347051622433270619[251] = 0.0;
   out_6347051622433270619[252] = 0.0;
   out_6347051622433270619[253] = 0.0;
   out_6347051622433270619[254] = 0.0;
   out_6347051622433270619[255] = 0.0;
   out_6347051622433270619[256] = 0.0;
   out_6347051622433270619[257] = 0.0;
   out_6347051622433270619[258] = 0.0;
   out_6347051622433270619[259] = 0.0;
   out_6347051622433270619[260] = 0.0;
   out_6347051622433270619[261] = 0.0;
   out_6347051622433270619[262] = 0.0;
   out_6347051622433270619[263] = 0.0;
   out_6347051622433270619[264] = 0.0;
   out_6347051622433270619[265] = 0.0;
   out_6347051622433270619[266] = 1.0;
   out_6347051622433270619[267] = 0.0;
   out_6347051622433270619[268] = 0.0;
   out_6347051622433270619[269] = 0.0;
   out_6347051622433270619[270] = 0.0;
   out_6347051622433270619[271] = 0.0;
   out_6347051622433270619[272] = 0.0;
   out_6347051622433270619[273] = 0.0;
   out_6347051622433270619[274] = 0.0;
   out_6347051622433270619[275] = 0.0;
   out_6347051622433270619[276] = 0.0;
   out_6347051622433270619[277] = 0.0;
   out_6347051622433270619[278] = 0.0;
   out_6347051622433270619[279] = 0.0;
   out_6347051622433270619[280] = 0.0;
   out_6347051622433270619[281] = 0.0;
   out_6347051622433270619[282] = 0.0;
   out_6347051622433270619[283] = 0.0;
   out_6347051622433270619[284] = 0.0;
   out_6347051622433270619[285] = 1.0;
   out_6347051622433270619[286] = 0.0;
   out_6347051622433270619[287] = 0.0;
   out_6347051622433270619[288] = 0.0;
   out_6347051622433270619[289] = 0.0;
   out_6347051622433270619[290] = 0.0;
   out_6347051622433270619[291] = 0.0;
   out_6347051622433270619[292] = 0.0;
   out_6347051622433270619[293] = 0.0;
   out_6347051622433270619[294] = 0.0;
   out_6347051622433270619[295] = 0.0;
   out_6347051622433270619[296] = 0.0;
   out_6347051622433270619[297] = 0.0;
   out_6347051622433270619[298] = 0.0;
   out_6347051622433270619[299] = 0.0;
   out_6347051622433270619[300] = 0.0;
   out_6347051622433270619[301] = 0.0;
   out_6347051622433270619[302] = 0.0;
   out_6347051622433270619[303] = 0.0;
   out_6347051622433270619[304] = 1.0;
   out_6347051622433270619[305] = 0.0;
   out_6347051622433270619[306] = 0.0;
   out_6347051622433270619[307] = 0.0;
   out_6347051622433270619[308] = 0.0;
   out_6347051622433270619[309] = 0.0;
   out_6347051622433270619[310] = 0.0;
   out_6347051622433270619[311] = 0.0;
   out_6347051622433270619[312] = 0.0;
   out_6347051622433270619[313] = 0.0;
   out_6347051622433270619[314] = 0.0;
   out_6347051622433270619[315] = 0.0;
   out_6347051622433270619[316] = 0.0;
   out_6347051622433270619[317] = 0.0;
   out_6347051622433270619[318] = 0.0;
   out_6347051622433270619[319] = 0.0;
   out_6347051622433270619[320] = 0.0;
   out_6347051622433270619[321] = 0.0;
   out_6347051622433270619[322] = 0.0;
   out_6347051622433270619[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_9196882787902943927) {
   out_9196882787902943927[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_9196882787902943927[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_9196882787902943927[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_9196882787902943927[3] = dt*state[12] + state[3];
   out_9196882787902943927[4] = dt*state[13] + state[4];
   out_9196882787902943927[5] = dt*state[14] + state[5];
   out_9196882787902943927[6] = state[6];
   out_9196882787902943927[7] = state[7];
   out_9196882787902943927[8] = state[8];
   out_9196882787902943927[9] = state[9];
   out_9196882787902943927[10] = state[10];
   out_9196882787902943927[11] = state[11];
   out_9196882787902943927[12] = state[12];
   out_9196882787902943927[13] = state[13];
   out_9196882787902943927[14] = state[14];
   out_9196882787902943927[15] = state[15];
   out_9196882787902943927[16] = state[16];
   out_9196882787902943927[17] = state[17];
}
void F_fun(double *state, double dt, double *out_5866301831384964850) {
   out_5866301831384964850[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5866301831384964850[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5866301831384964850[2] = 0;
   out_5866301831384964850[3] = 0;
   out_5866301831384964850[4] = 0;
   out_5866301831384964850[5] = 0;
   out_5866301831384964850[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5866301831384964850[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5866301831384964850[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5866301831384964850[9] = 0;
   out_5866301831384964850[10] = 0;
   out_5866301831384964850[11] = 0;
   out_5866301831384964850[12] = 0;
   out_5866301831384964850[13] = 0;
   out_5866301831384964850[14] = 0;
   out_5866301831384964850[15] = 0;
   out_5866301831384964850[16] = 0;
   out_5866301831384964850[17] = 0;
   out_5866301831384964850[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5866301831384964850[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5866301831384964850[20] = 0;
   out_5866301831384964850[21] = 0;
   out_5866301831384964850[22] = 0;
   out_5866301831384964850[23] = 0;
   out_5866301831384964850[24] = 0;
   out_5866301831384964850[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5866301831384964850[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5866301831384964850[27] = 0;
   out_5866301831384964850[28] = 0;
   out_5866301831384964850[29] = 0;
   out_5866301831384964850[30] = 0;
   out_5866301831384964850[31] = 0;
   out_5866301831384964850[32] = 0;
   out_5866301831384964850[33] = 0;
   out_5866301831384964850[34] = 0;
   out_5866301831384964850[35] = 0;
   out_5866301831384964850[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5866301831384964850[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5866301831384964850[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5866301831384964850[39] = 0;
   out_5866301831384964850[40] = 0;
   out_5866301831384964850[41] = 0;
   out_5866301831384964850[42] = 0;
   out_5866301831384964850[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5866301831384964850[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5866301831384964850[45] = 0;
   out_5866301831384964850[46] = 0;
   out_5866301831384964850[47] = 0;
   out_5866301831384964850[48] = 0;
   out_5866301831384964850[49] = 0;
   out_5866301831384964850[50] = 0;
   out_5866301831384964850[51] = 0;
   out_5866301831384964850[52] = 0;
   out_5866301831384964850[53] = 0;
   out_5866301831384964850[54] = 0;
   out_5866301831384964850[55] = 0;
   out_5866301831384964850[56] = 0;
   out_5866301831384964850[57] = 1;
   out_5866301831384964850[58] = 0;
   out_5866301831384964850[59] = 0;
   out_5866301831384964850[60] = 0;
   out_5866301831384964850[61] = 0;
   out_5866301831384964850[62] = 0;
   out_5866301831384964850[63] = 0;
   out_5866301831384964850[64] = 0;
   out_5866301831384964850[65] = 0;
   out_5866301831384964850[66] = dt;
   out_5866301831384964850[67] = 0;
   out_5866301831384964850[68] = 0;
   out_5866301831384964850[69] = 0;
   out_5866301831384964850[70] = 0;
   out_5866301831384964850[71] = 0;
   out_5866301831384964850[72] = 0;
   out_5866301831384964850[73] = 0;
   out_5866301831384964850[74] = 0;
   out_5866301831384964850[75] = 0;
   out_5866301831384964850[76] = 1;
   out_5866301831384964850[77] = 0;
   out_5866301831384964850[78] = 0;
   out_5866301831384964850[79] = 0;
   out_5866301831384964850[80] = 0;
   out_5866301831384964850[81] = 0;
   out_5866301831384964850[82] = 0;
   out_5866301831384964850[83] = 0;
   out_5866301831384964850[84] = 0;
   out_5866301831384964850[85] = dt;
   out_5866301831384964850[86] = 0;
   out_5866301831384964850[87] = 0;
   out_5866301831384964850[88] = 0;
   out_5866301831384964850[89] = 0;
   out_5866301831384964850[90] = 0;
   out_5866301831384964850[91] = 0;
   out_5866301831384964850[92] = 0;
   out_5866301831384964850[93] = 0;
   out_5866301831384964850[94] = 0;
   out_5866301831384964850[95] = 1;
   out_5866301831384964850[96] = 0;
   out_5866301831384964850[97] = 0;
   out_5866301831384964850[98] = 0;
   out_5866301831384964850[99] = 0;
   out_5866301831384964850[100] = 0;
   out_5866301831384964850[101] = 0;
   out_5866301831384964850[102] = 0;
   out_5866301831384964850[103] = 0;
   out_5866301831384964850[104] = dt;
   out_5866301831384964850[105] = 0;
   out_5866301831384964850[106] = 0;
   out_5866301831384964850[107] = 0;
   out_5866301831384964850[108] = 0;
   out_5866301831384964850[109] = 0;
   out_5866301831384964850[110] = 0;
   out_5866301831384964850[111] = 0;
   out_5866301831384964850[112] = 0;
   out_5866301831384964850[113] = 0;
   out_5866301831384964850[114] = 1;
   out_5866301831384964850[115] = 0;
   out_5866301831384964850[116] = 0;
   out_5866301831384964850[117] = 0;
   out_5866301831384964850[118] = 0;
   out_5866301831384964850[119] = 0;
   out_5866301831384964850[120] = 0;
   out_5866301831384964850[121] = 0;
   out_5866301831384964850[122] = 0;
   out_5866301831384964850[123] = 0;
   out_5866301831384964850[124] = 0;
   out_5866301831384964850[125] = 0;
   out_5866301831384964850[126] = 0;
   out_5866301831384964850[127] = 0;
   out_5866301831384964850[128] = 0;
   out_5866301831384964850[129] = 0;
   out_5866301831384964850[130] = 0;
   out_5866301831384964850[131] = 0;
   out_5866301831384964850[132] = 0;
   out_5866301831384964850[133] = 1;
   out_5866301831384964850[134] = 0;
   out_5866301831384964850[135] = 0;
   out_5866301831384964850[136] = 0;
   out_5866301831384964850[137] = 0;
   out_5866301831384964850[138] = 0;
   out_5866301831384964850[139] = 0;
   out_5866301831384964850[140] = 0;
   out_5866301831384964850[141] = 0;
   out_5866301831384964850[142] = 0;
   out_5866301831384964850[143] = 0;
   out_5866301831384964850[144] = 0;
   out_5866301831384964850[145] = 0;
   out_5866301831384964850[146] = 0;
   out_5866301831384964850[147] = 0;
   out_5866301831384964850[148] = 0;
   out_5866301831384964850[149] = 0;
   out_5866301831384964850[150] = 0;
   out_5866301831384964850[151] = 0;
   out_5866301831384964850[152] = 1;
   out_5866301831384964850[153] = 0;
   out_5866301831384964850[154] = 0;
   out_5866301831384964850[155] = 0;
   out_5866301831384964850[156] = 0;
   out_5866301831384964850[157] = 0;
   out_5866301831384964850[158] = 0;
   out_5866301831384964850[159] = 0;
   out_5866301831384964850[160] = 0;
   out_5866301831384964850[161] = 0;
   out_5866301831384964850[162] = 0;
   out_5866301831384964850[163] = 0;
   out_5866301831384964850[164] = 0;
   out_5866301831384964850[165] = 0;
   out_5866301831384964850[166] = 0;
   out_5866301831384964850[167] = 0;
   out_5866301831384964850[168] = 0;
   out_5866301831384964850[169] = 0;
   out_5866301831384964850[170] = 0;
   out_5866301831384964850[171] = 1;
   out_5866301831384964850[172] = 0;
   out_5866301831384964850[173] = 0;
   out_5866301831384964850[174] = 0;
   out_5866301831384964850[175] = 0;
   out_5866301831384964850[176] = 0;
   out_5866301831384964850[177] = 0;
   out_5866301831384964850[178] = 0;
   out_5866301831384964850[179] = 0;
   out_5866301831384964850[180] = 0;
   out_5866301831384964850[181] = 0;
   out_5866301831384964850[182] = 0;
   out_5866301831384964850[183] = 0;
   out_5866301831384964850[184] = 0;
   out_5866301831384964850[185] = 0;
   out_5866301831384964850[186] = 0;
   out_5866301831384964850[187] = 0;
   out_5866301831384964850[188] = 0;
   out_5866301831384964850[189] = 0;
   out_5866301831384964850[190] = 1;
   out_5866301831384964850[191] = 0;
   out_5866301831384964850[192] = 0;
   out_5866301831384964850[193] = 0;
   out_5866301831384964850[194] = 0;
   out_5866301831384964850[195] = 0;
   out_5866301831384964850[196] = 0;
   out_5866301831384964850[197] = 0;
   out_5866301831384964850[198] = 0;
   out_5866301831384964850[199] = 0;
   out_5866301831384964850[200] = 0;
   out_5866301831384964850[201] = 0;
   out_5866301831384964850[202] = 0;
   out_5866301831384964850[203] = 0;
   out_5866301831384964850[204] = 0;
   out_5866301831384964850[205] = 0;
   out_5866301831384964850[206] = 0;
   out_5866301831384964850[207] = 0;
   out_5866301831384964850[208] = 0;
   out_5866301831384964850[209] = 1;
   out_5866301831384964850[210] = 0;
   out_5866301831384964850[211] = 0;
   out_5866301831384964850[212] = 0;
   out_5866301831384964850[213] = 0;
   out_5866301831384964850[214] = 0;
   out_5866301831384964850[215] = 0;
   out_5866301831384964850[216] = 0;
   out_5866301831384964850[217] = 0;
   out_5866301831384964850[218] = 0;
   out_5866301831384964850[219] = 0;
   out_5866301831384964850[220] = 0;
   out_5866301831384964850[221] = 0;
   out_5866301831384964850[222] = 0;
   out_5866301831384964850[223] = 0;
   out_5866301831384964850[224] = 0;
   out_5866301831384964850[225] = 0;
   out_5866301831384964850[226] = 0;
   out_5866301831384964850[227] = 0;
   out_5866301831384964850[228] = 1;
   out_5866301831384964850[229] = 0;
   out_5866301831384964850[230] = 0;
   out_5866301831384964850[231] = 0;
   out_5866301831384964850[232] = 0;
   out_5866301831384964850[233] = 0;
   out_5866301831384964850[234] = 0;
   out_5866301831384964850[235] = 0;
   out_5866301831384964850[236] = 0;
   out_5866301831384964850[237] = 0;
   out_5866301831384964850[238] = 0;
   out_5866301831384964850[239] = 0;
   out_5866301831384964850[240] = 0;
   out_5866301831384964850[241] = 0;
   out_5866301831384964850[242] = 0;
   out_5866301831384964850[243] = 0;
   out_5866301831384964850[244] = 0;
   out_5866301831384964850[245] = 0;
   out_5866301831384964850[246] = 0;
   out_5866301831384964850[247] = 1;
   out_5866301831384964850[248] = 0;
   out_5866301831384964850[249] = 0;
   out_5866301831384964850[250] = 0;
   out_5866301831384964850[251] = 0;
   out_5866301831384964850[252] = 0;
   out_5866301831384964850[253] = 0;
   out_5866301831384964850[254] = 0;
   out_5866301831384964850[255] = 0;
   out_5866301831384964850[256] = 0;
   out_5866301831384964850[257] = 0;
   out_5866301831384964850[258] = 0;
   out_5866301831384964850[259] = 0;
   out_5866301831384964850[260] = 0;
   out_5866301831384964850[261] = 0;
   out_5866301831384964850[262] = 0;
   out_5866301831384964850[263] = 0;
   out_5866301831384964850[264] = 0;
   out_5866301831384964850[265] = 0;
   out_5866301831384964850[266] = 1;
   out_5866301831384964850[267] = 0;
   out_5866301831384964850[268] = 0;
   out_5866301831384964850[269] = 0;
   out_5866301831384964850[270] = 0;
   out_5866301831384964850[271] = 0;
   out_5866301831384964850[272] = 0;
   out_5866301831384964850[273] = 0;
   out_5866301831384964850[274] = 0;
   out_5866301831384964850[275] = 0;
   out_5866301831384964850[276] = 0;
   out_5866301831384964850[277] = 0;
   out_5866301831384964850[278] = 0;
   out_5866301831384964850[279] = 0;
   out_5866301831384964850[280] = 0;
   out_5866301831384964850[281] = 0;
   out_5866301831384964850[282] = 0;
   out_5866301831384964850[283] = 0;
   out_5866301831384964850[284] = 0;
   out_5866301831384964850[285] = 1;
   out_5866301831384964850[286] = 0;
   out_5866301831384964850[287] = 0;
   out_5866301831384964850[288] = 0;
   out_5866301831384964850[289] = 0;
   out_5866301831384964850[290] = 0;
   out_5866301831384964850[291] = 0;
   out_5866301831384964850[292] = 0;
   out_5866301831384964850[293] = 0;
   out_5866301831384964850[294] = 0;
   out_5866301831384964850[295] = 0;
   out_5866301831384964850[296] = 0;
   out_5866301831384964850[297] = 0;
   out_5866301831384964850[298] = 0;
   out_5866301831384964850[299] = 0;
   out_5866301831384964850[300] = 0;
   out_5866301831384964850[301] = 0;
   out_5866301831384964850[302] = 0;
   out_5866301831384964850[303] = 0;
   out_5866301831384964850[304] = 1;
   out_5866301831384964850[305] = 0;
   out_5866301831384964850[306] = 0;
   out_5866301831384964850[307] = 0;
   out_5866301831384964850[308] = 0;
   out_5866301831384964850[309] = 0;
   out_5866301831384964850[310] = 0;
   out_5866301831384964850[311] = 0;
   out_5866301831384964850[312] = 0;
   out_5866301831384964850[313] = 0;
   out_5866301831384964850[314] = 0;
   out_5866301831384964850[315] = 0;
   out_5866301831384964850[316] = 0;
   out_5866301831384964850[317] = 0;
   out_5866301831384964850[318] = 0;
   out_5866301831384964850[319] = 0;
   out_5866301831384964850[320] = 0;
   out_5866301831384964850[321] = 0;
   out_5866301831384964850[322] = 0;
   out_5866301831384964850[323] = 1;
}
void h_4(double *state, double *unused, double *out_1536508825138900327) {
   out_1536508825138900327[0] = state[6] + state[9];
   out_1536508825138900327[1] = state[7] + state[10];
   out_1536508825138900327[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_7544999104316168723) {
   out_7544999104316168723[0] = 0;
   out_7544999104316168723[1] = 0;
   out_7544999104316168723[2] = 0;
   out_7544999104316168723[3] = 0;
   out_7544999104316168723[4] = 0;
   out_7544999104316168723[5] = 0;
   out_7544999104316168723[6] = 1;
   out_7544999104316168723[7] = 0;
   out_7544999104316168723[8] = 0;
   out_7544999104316168723[9] = 1;
   out_7544999104316168723[10] = 0;
   out_7544999104316168723[11] = 0;
   out_7544999104316168723[12] = 0;
   out_7544999104316168723[13] = 0;
   out_7544999104316168723[14] = 0;
   out_7544999104316168723[15] = 0;
   out_7544999104316168723[16] = 0;
   out_7544999104316168723[17] = 0;
   out_7544999104316168723[18] = 0;
   out_7544999104316168723[19] = 0;
   out_7544999104316168723[20] = 0;
   out_7544999104316168723[21] = 0;
   out_7544999104316168723[22] = 0;
   out_7544999104316168723[23] = 0;
   out_7544999104316168723[24] = 0;
   out_7544999104316168723[25] = 1;
   out_7544999104316168723[26] = 0;
   out_7544999104316168723[27] = 0;
   out_7544999104316168723[28] = 1;
   out_7544999104316168723[29] = 0;
   out_7544999104316168723[30] = 0;
   out_7544999104316168723[31] = 0;
   out_7544999104316168723[32] = 0;
   out_7544999104316168723[33] = 0;
   out_7544999104316168723[34] = 0;
   out_7544999104316168723[35] = 0;
   out_7544999104316168723[36] = 0;
   out_7544999104316168723[37] = 0;
   out_7544999104316168723[38] = 0;
   out_7544999104316168723[39] = 0;
   out_7544999104316168723[40] = 0;
   out_7544999104316168723[41] = 0;
   out_7544999104316168723[42] = 0;
   out_7544999104316168723[43] = 0;
   out_7544999104316168723[44] = 1;
   out_7544999104316168723[45] = 0;
   out_7544999104316168723[46] = 0;
   out_7544999104316168723[47] = 1;
   out_7544999104316168723[48] = 0;
   out_7544999104316168723[49] = 0;
   out_7544999104316168723[50] = 0;
   out_7544999104316168723[51] = 0;
   out_7544999104316168723[52] = 0;
   out_7544999104316168723[53] = 0;
}
void h_10(double *state, double *unused, double *out_6489509158998458427) {
   out_6489509158998458427[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_6489509158998458427[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_6489509158998458427[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_6104186488438151746) {
   out_6104186488438151746[0] = 0;
   out_6104186488438151746[1] = 9.8100000000000005*cos(state[1]);
   out_6104186488438151746[2] = 0;
   out_6104186488438151746[3] = 0;
   out_6104186488438151746[4] = -state[8];
   out_6104186488438151746[5] = state[7];
   out_6104186488438151746[6] = 0;
   out_6104186488438151746[7] = state[5];
   out_6104186488438151746[8] = -state[4];
   out_6104186488438151746[9] = 0;
   out_6104186488438151746[10] = 0;
   out_6104186488438151746[11] = 0;
   out_6104186488438151746[12] = 1;
   out_6104186488438151746[13] = 0;
   out_6104186488438151746[14] = 0;
   out_6104186488438151746[15] = 1;
   out_6104186488438151746[16] = 0;
   out_6104186488438151746[17] = 0;
   out_6104186488438151746[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_6104186488438151746[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_6104186488438151746[20] = 0;
   out_6104186488438151746[21] = state[8];
   out_6104186488438151746[22] = 0;
   out_6104186488438151746[23] = -state[6];
   out_6104186488438151746[24] = -state[5];
   out_6104186488438151746[25] = 0;
   out_6104186488438151746[26] = state[3];
   out_6104186488438151746[27] = 0;
   out_6104186488438151746[28] = 0;
   out_6104186488438151746[29] = 0;
   out_6104186488438151746[30] = 0;
   out_6104186488438151746[31] = 1;
   out_6104186488438151746[32] = 0;
   out_6104186488438151746[33] = 0;
   out_6104186488438151746[34] = 1;
   out_6104186488438151746[35] = 0;
   out_6104186488438151746[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_6104186488438151746[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_6104186488438151746[38] = 0;
   out_6104186488438151746[39] = -state[7];
   out_6104186488438151746[40] = state[6];
   out_6104186488438151746[41] = 0;
   out_6104186488438151746[42] = state[4];
   out_6104186488438151746[43] = -state[3];
   out_6104186488438151746[44] = 0;
   out_6104186488438151746[45] = 0;
   out_6104186488438151746[46] = 0;
   out_6104186488438151746[47] = 0;
   out_6104186488438151746[48] = 0;
   out_6104186488438151746[49] = 0;
   out_6104186488438151746[50] = 1;
   out_6104186488438151746[51] = 0;
   out_6104186488438151746[52] = 0;
   out_6104186488438151746[53] = 1;
}
void h_13(double *state, double *unused, double *out_2124120058566848093) {
   out_2124120058566848093[0] = state[3];
   out_2124120058566848093[1] = state[4];
   out_2124120058566848093[2] = state[5];
}
void H_13(double *state, double *unused, double *out_3711243641013644699) {
   out_3711243641013644699[0] = 0;
   out_3711243641013644699[1] = 0;
   out_3711243641013644699[2] = 0;
   out_3711243641013644699[3] = 1;
   out_3711243641013644699[4] = 0;
   out_3711243641013644699[5] = 0;
   out_3711243641013644699[6] = 0;
   out_3711243641013644699[7] = 0;
   out_3711243641013644699[8] = 0;
   out_3711243641013644699[9] = 0;
   out_3711243641013644699[10] = 0;
   out_3711243641013644699[11] = 0;
   out_3711243641013644699[12] = 0;
   out_3711243641013644699[13] = 0;
   out_3711243641013644699[14] = 0;
   out_3711243641013644699[15] = 0;
   out_3711243641013644699[16] = 0;
   out_3711243641013644699[17] = 0;
   out_3711243641013644699[18] = 0;
   out_3711243641013644699[19] = 0;
   out_3711243641013644699[20] = 0;
   out_3711243641013644699[21] = 0;
   out_3711243641013644699[22] = 1;
   out_3711243641013644699[23] = 0;
   out_3711243641013644699[24] = 0;
   out_3711243641013644699[25] = 0;
   out_3711243641013644699[26] = 0;
   out_3711243641013644699[27] = 0;
   out_3711243641013644699[28] = 0;
   out_3711243641013644699[29] = 0;
   out_3711243641013644699[30] = 0;
   out_3711243641013644699[31] = 0;
   out_3711243641013644699[32] = 0;
   out_3711243641013644699[33] = 0;
   out_3711243641013644699[34] = 0;
   out_3711243641013644699[35] = 0;
   out_3711243641013644699[36] = 0;
   out_3711243641013644699[37] = 0;
   out_3711243641013644699[38] = 0;
   out_3711243641013644699[39] = 0;
   out_3711243641013644699[40] = 0;
   out_3711243641013644699[41] = 1;
   out_3711243641013644699[42] = 0;
   out_3711243641013644699[43] = 0;
   out_3711243641013644699[44] = 0;
   out_3711243641013644699[45] = 0;
   out_3711243641013644699[46] = 0;
   out_3711243641013644699[47] = 0;
   out_3711243641013644699[48] = 0;
   out_3711243641013644699[49] = 0;
   out_3711243641013644699[50] = 0;
   out_3711243641013644699[51] = 0;
   out_3711243641013644699[52] = 0;
   out_3711243641013644699[53] = 0;
}
void h_14(double *state, double *unused, double *out_2918682735475695430) {
   out_2918682735475695430[0] = state[6];
   out_2918682735475695430[1] = state[7];
   out_2918682735475695430[2] = state[8];
}
void H_14(double *state, double *unused, double *out_4462210672020796427) {
   out_4462210672020796427[0] = 0;
   out_4462210672020796427[1] = 0;
   out_4462210672020796427[2] = 0;
   out_4462210672020796427[3] = 0;
   out_4462210672020796427[4] = 0;
   out_4462210672020796427[5] = 0;
   out_4462210672020796427[6] = 1;
   out_4462210672020796427[7] = 0;
   out_4462210672020796427[8] = 0;
   out_4462210672020796427[9] = 0;
   out_4462210672020796427[10] = 0;
   out_4462210672020796427[11] = 0;
   out_4462210672020796427[12] = 0;
   out_4462210672020796427[13] = 0;
   out_4462210672020796427[14] = 0;
   out_4462210672020796427[15] = 0;
   out_4462210672020796427[16] = 0;
   out_4462210672020796427[17] = 0;
   out_4462210672020796427[18] = 0;
   out_4462210672020796427[19] = 0;
   out_4462210672020796427[20] = 0;
   out_4462210672020796427[21] = 0;
   out_4462210672020796427[22] = 0;
   out_4462210672020796427[23] = 0;
   out_4462210672020796427[24] = 0;
   out_4462210672020796427[25] = 1;
   out_4462210672020796427[26] = 0;
   out_4462210672020796427[27] = 0;
   out_4462210672020796427[28] = 0;
   out_4462210672020796427[29] = 0;
   out_4462210672020796427[30] = 0;
   out_4462210672020796427[31] = 0;
   out_4462210672020796427[32] = 0;
   out_4462210672020796427[33] = 0;
   out_4462210672020796427[34] = 0;
   out_4462210672020796427[35] = 0;
   out_4462210672020796427[36] = 0;
   out_4462210672020796427[37] = 0;
   out_4462210672020796427[38] = 0;
   out_4462210672020796427[39] = 0;
   out_4462210672020796427[40] = 0;
   out_4462210672020796427[41] = 0;
   out_4462210672020796427[42] = 0;
   out_4462210672020796427[43] = 0;
   out_4462210672020796427[44] = 1;
   out_4462210672020796427[45] = 0;
   out_4462210672020796427[46] = 0;
   out_4462210672020796427[47] = 0;
   out_4462210672020796427[48] = 0;
   out_4462210672020796427[49] = 0;
   out_4462210672020796427[50] = 0;
   out_4462210672020796427[51] = 0;
   out_4462210672020796427[52] = 0;
   out_4462210672020796427[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_6415792483428760966) {
  err_fun(nom_x, delta_x, out_6415792483428760966);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6377608227770965724) {
  inv_err_fun(nom_x, true_x, out_6377608227770965724);
}
void pose_H_mod_fun(double *state, double *out_6347051622433270619) {
  H_mod_fun(state, out_6347051622433270619);
}
void pose_f_fun(double *state, double dt, double *out_9196882787902943927) {
  f_fun(state,  dt, out_9196882787902943927);
}
void pose_F_fun(double *state, double dt, double *out_5866301831384964850) {
  F_fun(state,  dt, out_5866301831384964850);
}
void pose_h_4(double *state, double *unused, double *out_1536508825138900327) {
  h_4(state, unused, out_1536508825138900327);
}
void pose_H_4(double *state, double *unused, double *out_7544999104316168723) {
  H_4(state, unused, out_7544999104316168723);
}
void pose_h_10(double *state, double *unused, double *out_6489509158998458427) {
  h_10(state, unused, out_6489509158998458427);
}
void pose_H_10(double *state, double *unused, double *out_6104186488438151746) {
  H_10(state, unused, out_6104186488438151746);
}
void pose_h_13(double *state, double *unused, double *out_2124120058566848093) {
  h_13(state, unused, out_2124120058566848093);
}
void pose_H_13(double *state, double *unused, double *out_3711243641013644699) {
  H_13(state, unused, out_3711243641013644699);
}
void pose_h_14(double *state, double *unused, double *out_2918682735475695430) {
  h_14(state, unused, out_2918682735475695430);
}
void pose_H_14(double *state, double *unused, double *out_4462210672020796427) {
  H_14(state, unused, out_4462210672020796427);
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
