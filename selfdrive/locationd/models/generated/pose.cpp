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
void err_fun(double *nom_x, double *delta_x, double *out_2226712134560383445) {
   out_2226712134560383445[0] = delta_x[0] + nom_x[0];
   out_2226712134560383445[1] = delta_x[1] + nom_x[1];
   out_2226712134560383445[2] = delta_x[2] + nom_x[2];
   out_2226712134560383445[3] = delta_x[3] + nom_x[3];
   out_2226712134560383445[4] = delta_x[4] + nom_x[4];
   out_2226712134560383445[5] = delta_x[5] + nom_x[5];
   out_2226712134560383445[6] = delta_x[6] + nom_x[6];
   out_2226712134560383445[7] = delta_x[7] + nom_x[7];
   out_2226712134560383445[8] = delta_x[8] + nom_x[8];
   out_2226712134560383445[9] = delta_x[9] + nom_x[9];
   out_2226712134560383445[10] = delta_x[10] + nom_x[10];
   out_2226712134560383445[11] = delta_x[11] + nom_x[11];
   out_2226712134560383445[12] = delta_x[12] + nom_x[12];
   out_2226712134560383445[13] = delta_x[13] + nom_x[13];
   out_2226712134560383445[14] = delta_x[14] + nom_x[14];
   out_2226712134560383445[15] = delta_x[15] + nom_x[15];
   out_2226712134560383445[16] = delta_x[16] + nom_x[16];
   out_2226712134560383445[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5991885512389717825) {
   out_5991885512389717825[0] = -nom_x[0] + true_x[0];
   out_5991885512389717825[1] = -nom_x[1] + true_x[1];
   out_5991885512389717825[2] = -nom_x[2] + true_x[2];
   out_5991885512389717825[3] = -nom_x[3] + true_x[3];
   out_5991885512389717825[4] = -nom_x[4] + true_x[4];
   out_5991885512389717825[5] = -nom_x[5] + true_x[5];
   out_5991885512389717825[6] = -nom_x[6] + true_x[6];
   out_5991885512389717825[7] = -nom_x[7] + true_x[7];
   out_5991885512389717825[8] = -nom_x[8] + true_x[8];
   out_5991885512389717825[9] = -nom_x[9] + true_x[9];
   out_5991885512389717825[10] = -nom_x[10] + true_x[10];
   out_5991885512389717825[11] = -nom_x[11] + true_x[11];
   out_5991885512389717825[12] = -nom_x[12] + true_x[12];
   out_5991885512389717825[13] = -nom_x[13] + true_x[13];
   out_5991885512389717825[14] = -nom_x[14] + true_x[14];
   out_5991885512389717825[15] = -nom_x[15] + true_x[15];
   out_5991885512389717825[16] = -nom_x[16] + true_x[16];
   out_5991885512389717825[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_4761019588596262838) {
   out_4761019588596262838[0] = 1.0;
   out_4761019588596262838[1] = 0.0;
   out_4761019588596262838[2] = 0.0;
   out_4761019588596262838[3] = 0.0;
   out_4761019588596262838[4] = 0.0;
   out_4761019588596262838[5] = 0.0;
   out_4761019588596262838[6] = 0.0;
   out_4761019588596262838[7] = 0.0;
   out_4761019588596262838[8] = 0.0;
   out_4761019588596262838[9] = 0.0;
   out_4761019588596262838[10] = 0.0;
   out_4761019588596262838[11] = 0.0;
   out_4761019588596262838[12] = 0.0;
   out_4761019588596262838[13] = 0.0;
   out_4761019588596262838[14] = 0.0;
   out_4761019588596262838[15] = 0.0;
   out_4761019588596262838[16] = 0.0;
   out_4761019588596262838[17] = 0.0;
   out_4761019588596262838[18] = 0.0;
   out_4761019588596262838[19] = 1.0;
   out_4761019588596262838[20] = 0.0;
   out_4761019588596262838[21] = 0.0;
   out_4761019588596262838[22] = 0.0;
   out_4761019588596262838[23] = 0.0;
   out_4761019588596262838[24] = 0.0;
   out_4761019588596262838[25] = 0.0;
   out_4761019588596262838[26] = 0.0;
   out_4761019588596262838[27] = 0.0;
   out_4761019588596262838[28] = 0.0;
   out_4761019588596262838[29] = 0.0;
   out_4761019588596262838[30] = 0.0;
   out_4761019588596262838[31] = 0.0;
   out_4761019588596262838[32] = 0.0;
   out_4761019588596262838[33] = 0.0;
   out_4761019588596262838[34] = 0.0;
   out_4761019588596262838[35] = 0.0;
   out_4761019588596262838[36] = 0.0;
   out_4761019588596262838[37] = 0.0;
   out_4761019588596262838[38] = 1.0;
   out_4761019588596262838[39] = 0.0;
   out_4761019588596262838[40] = 0.0;
   out_4761019588596262838[41] = 0.0;
   out_4761019588596262838[42] = 0.0;
   out_4761019588596262838[43] = 0.0;
   out_4761019588596262838[44] = 0.0;
   out_4761019588596262838[45] = 0.0;
   out_4761019588596262838[46] = 0.0;
   out_4761019588596262838[47] = 0.0;
   out_4761019588596262838[48] = 0.0;
   out_4761019588596262838[49] = 0.0;
   out_4761019588596262838[50] = 0.0;
   out_4761019588596262838[51] = 0.0;
   out_4761019588596262838[52] = 0.0;
   out_4761019588596262838[53] = 0.0;
   out_4761019588596262838[54] = 0.0;
   out_4761019588596262838[55] = 0.0;
   out_4761019588596262838[56] = 0.0;
   out_4761019588596262838[57] = 1.0;
   out_4761019588596262838[58] = 0.0;
   out_4761019588596262838[59] = 0.0;
   out_4761019588596262838[60] = 0.0;
   out_4761019588596262838[61] = 0.0;
   out_4761019588596262838[62] = 0.0;
   out_4761019588596262838[63] = 0.0;
   out_4761019588596262838[64] = 0.0;
   out_4761019588596262838[65] = 0.0;
   out_4761019588596262838[66] = 0.0;
   out_4761019588596262838[67] = 0.0;
   out_4761019588596262838[68] = 0.0;
   out_4761019588596262838[69] = 0.0;
   out_4761019588596262838[70] = 0.0;
   out_4761019588596262838[71] = 0.0;
   out_4761019588596262838[72] = 0.0;
   out_4761019588596262838[73] = 0.0;
   out_4761019588596262838[74] = 0.0;
   out_4761019588596262838[75] = 0.0;
   out_4761019588596262838[76] = 1.0;
   out_4761019588596262838[77] = 0.0;
   out_4761019588596262838[78] = 0.0;
   out_4761019588596262838[79] = 0.0;
   out_4761019588596262838[80] = 0.0;
   out_4761019588596262838[81] = 0.0;
   out_4761019588596262838[82] = 0.0;
   out_4761019588596262838[83] = 0.0;
   out_4761019588596262838[84] = 0.0;
   out_4761019588596262838[85] = 0.0;
   out_4761019588596262838[86] = 0.0;
   out_4761019588596262838[87] = 0.0;
   out_4761019588596262838[88] = 0.0;
   out_4761019588596262838[89] = 0.0;
   out_4761019588596262838[90] = 0.0;
   out_4761019588596262838[91] = 0.0;
   out_4761019588596262838[92] = 0.0;
   out_4761019588596262838[93] = 0.0;
   out_4761019588596262838[94] = 0.0;
   out_4761019588596262838[95] = 1.0;
   out_4761019588596262838[96] = 0.0;
   out_4761019588596262838[97] = 0.0;
   out_4761019588596262838[98] = 0.0;
   out_4761019588596262838[99] = 0.0;
   out_4761019588596262838[100] = 0.0;
   out_4761019588596262838[101] = 0.0;
   out_4761019588596262838[102] = 0.0;
   out_4761019588596262838[103] = 0.0;
   out_4761019588596262838[104] = 0.0;
   out_4761019588596262838[105] = 0.0;
   out_4761019588596262838[106] = 0.0;
   out_4761019588596262838[107] = 0.0;
   out_4761019588596262838[108] = 0.0;
   out_4761019588596262838[109] = 0.0;
   out_4761019588596262838[110] = 0.0;
   out_4761019588596262838[111] = 0.0;
   out_4761019588596262838[112] = 0.0;
   out_4761019588596262838[113] = 0.0;
   out_4761019588596262838[114] = 1.0;
   out_4761019588596262838[115] = 0.0;
   out_4761019588596262838[116] = 0.0;
   out_4761019588596262838[117] = 0.0;
   out_4761019588596262838[118] = 0.0;
   out_4761019588596262838[119] = 0.0;
   out_4761019588596262838[120] = 0.0;
   out_4761019588596262838[121] = 0.0;
   out_4761019588596262838[122] = 0.0;
   out_4761019588596262838[123] = 0.0;
   out_4761019588596262838[124] = 0.0;
   out_4761019588596262838[125] = 0.0;
   out_4761019588596262838[126] = 0.0;
   out_4761019588596262838[127] = 0.0;
   out_4761019588596262838[128] = 0.0;
   out_4761019588596262838[129] = 0.0;
   out_4761019588596262838[130] = 0.0;
   out_4761019588596262838[131] = 0.0;
   out_4761019588596262838[132] = 0.0;
   out_4761019588596262838[133] = 1.0;
   out_4761019588596262838[134] = 0.0;
   out_4761019588596262838[135] = 0.0;
   out_4761019588596262838[136] = 0.0;
   out_4761019588596262838[137] = 0.0;
   out_4761019588596262838[138] = 0.0;
   out_4761019588596262838[139] = 0.0;
   out_4761019588596262838[140] = 0.0;
   out_4761019588596262838[141] = 0.0;
   out_4761019588596262838[142] = 0.0;
   out_4761019588596262838[143] = 0.0;
   out_4761019588596262838[144] = 0.0;
   out_4761019588596262838[145] = 0.0;
   out_4761019588596262838[146] = 0.0;
   out_4761019588596262838[147] = 0.0;
   out_4761019588596262838[148] = 0.0;
   out_4761019588596262838[149] = 0.0;
   out_4761019588596262838[150] = 0.0;
   out_4761019588596262838[151] = 0.0;
   out_4761019588596262838[152] = 1.0;
   out_4761019588596262838[153] = 0.0;
   out_4761019588596262838[154] = 0.0;
   out_4761019588596262838[155] = 0.0;
   out_4761019588596262838[156] = 0.0;
   out_4761019588596262838[157] = 0.0;
   out_4761019588596262838[158] = 0.0;
   out_4761019588596262838[159] = 0.0;
   out_4761019588596262838[160] = 0.0;
   out_4761019588596262838[161] = 0.0;
   out_4761019588596262838[162] = 0.0;
   out_4761019588596262838[163] = 0.0;
   out_4761019588596262838[164] = 0.0;
   out_4761019588596262838[165] = 0.0;
   out_4761019588596262838[166] = 0.0;
   out_4761019588596262838[167] = 0.0;
   out_4761019588596262838[168] = 0.0;
   out_4761019588596262838[169] = 0.0;
   out_4761019588596262838[170] = 0.0;
   out_4761019588596262838[171] = 1.0;
   out_4761019588596262838[172] = 0.0;
   out_4761019588596262838[173] = 0.0;
   out_4761019588596262838[174] = 0.0;
   out_4761019588596262838[175] = 0.0;
   out_4761019588596262838[176] = 0.0;
   out_4761019588596262838[177] = 0.0;
   out_4761019588596262838[178] = 0.0;
   out_4761019588596262838[179] = 0.0;
   out_4761019588596262838[180] = 0.0;
   out_4761019588596262838[181] = 0.0;
   out_4761019588596262838[182] = 0.0;
   out_4761019588596262838[183] = 0.0;
   out_4761019588596262838[184] = 0.0;
   out_4761019588596262838[185] = 0.0;
   out_4761019588596262838[186] = 0.0;
   out_4761019588596262838[187] = 0.0;
   out_4761019588596262838[188] = 0.0;
   out_4761019588596262838[189] = 0.0;
   out_4761019588596262838[190] = 1.0;
   out_4761019588596262838[191] = 0.0;
   out_4761019588596262838[192] = 0.0;
   out_4761019588596262838[193] = 0.0;
   out_4761019588596262838[194] = 0.0;
   out_4761019588596262838[195] = 0.0;
   out_4761019588596262838[196] = 0.0;
   out_4761019588596262838[197] = 0.0;
   out_4761019588596262838[198] = 0.0;
   out_4761019588596262838[199] = 0.0;
   out_4761019588596262838[200] = 0.0;
   out_4761019588596262838[201] = 0.0;
   out_4761019588596262838[202] = 0.0;
   out_4761019588596262838[203] = 0.0;
   out_4761019588596262838[204] = 0.0;
   out_4761019588596262838[205] = 0.0;
   out_4761019588596262838[206] = 0.0;
   out_4761019588596262838[207] = 0.0;
   out_4761019588596262838[208] = 0.0;
   out_4761019588596262838[209] = 1.0;
   out_4761019588596262838[210] = 0.0;
   out_4761019588596262838[211] = 0.0;
   out_4761019588596262838[212] = 0.0;
   out_4761019588596262838[213] = 0.0;
   out_4761019588596262838[214] = 0.0;
   out_4761019588596262838[215] = 0.0;
   out_4761019588596262838[216] = 0.0;
   out_4761019588596262838[217] = 0.0;
   out_4761019588596262838[218] = 0.0;
   out_4761019588596262838[219] = 0.0;
   out_4761019588596262838[220] = 0.0;
   out_4761019588596262838[221] = 0.0;
   out_4761019588596262838[222] = 0.0;
   out_4761019588596262838[223] = 0.0;
   out_4761019588596262838[224] = 0.0;
   out_4761019588596262838[225] = 0.0;
   out_4761019588596262838[226] = 0.0;
   out_4761019588596262838[227] = 0.0;
   out_4761019588596262838[228] = 1.0;
   out_4761019588596262838[229] = 0.0;
   out_4761019588596262838[230] = 0.0;
   out_4761019588596262838[231] = 0.0;
   out_4761019588596262838[232] = 0.0;
   out_4761019588596262838[233] = 0.0;
   out_4761019588596262838[234] = 0.0;
   out_4761019588596262838[235] = 0.0;
   out_4761019588596262838[236] = 0.0;
   out_4761019588596262838[237] = 0.0;
   out_4761019588596262838[238] = 0.0;
   out_4761019588596262838[239] = 0.0;
   out_4761019588596262838[240] = 0.0;
   out_4761019588596262838[241] = 0.0;
   out_4761019588596262838[242] = 0.0;
   out_4761019588596262838[243] = 0.0;
   out_4761019588596262838[244] = 0.0;
   out_4761019588596262838[245] = 0.0;
   out_4761019588596262838[246] = 0.0;
   out_4761019588596262838[247] = 1.0;
   out_4761019588596262838[248] = 0.0;
   out_4761019588596262838[249] = 0.0;
   out_4761019588596262838[250] = 0.0;
   out_4761019588596262838[251] = 0.0;
   out_4761019588596262838[252] = 0.0;
   out_4761019588596262838[253] = 0.0;
   out_4761019588596262838[254] = 0.0;
   out_4761019588596262838[255] = 0.0;
   out_4761019588596262838[256] = 0.0;
   out_4761019588596262838[257] = 0.0;
   out_4761019588596262838[258] = 0.0;
   out_4761019588596262838[259] = 0.0;
   out_4761019588596262838[260] = 0.0;
   out_4761019588596262838[261] = 0.0;
   out_4761019588596262838[262] = 0.0;
   out_4761019588596262838[263] = 0.0;
   out_4761019588596262838[264] = 0.0;
   out_4761019588596262838[265] = 0.0;
   out_4761019588596262838[266] = 1.0;
   out_4761019588596262838[267] = 0.0;
   out_4761019588596262838[268] = 0.0;
   out_4761019588596262838[269] = 0.0;
   out_4761019588596262838[270] = 0.0;
   out_4761019588596262838[271] = 0.0;
   out_4761019588596262838[272] = 0.0;
   out_4761019588596262838[273] = 0.0;
   out_4761019588596262838[274] = 0.0;
   out_4761019588596262838[275] = 0.0;
   out_4761019588596262838[276] = 0.0;
   out_4761019588596262838[277] = 0.0;
   out_4761019588596262838[278] = 0.0;
   out_4761019588596262838[279] = 0.0;
   out_4761019588596262838[280] = 0.0;
   out_4761019588596262838[281] = 0.0;
   out_4761019588596262838[282] = 0.0;
   out_4761019588596262838[283] = 0.0;
   out_4761019588596262838[284] = 0.0;
   out_4761019588596262838[285] = 1.0;
   out_4761019588596262838[286] = 0.0;
   out_4761019588596262838[287] = 0.0;
   out_4761019588596262838[288] = 0.0;
   out_4761019588596262838[289] = 0.0;
   out_4761019588596262838[290] = 0.0;
   out_4761019588596262838[291] = 0.0;
   out_4761019588596262838[292] = 0.0;
   out_4761019588596262838[293] = 0.0;
   out_4761019588596262838[294] = 0.0;
   out_4761019588596262838[295] = 0.0;
   out_4761019588596262838[296] = 0.0;
   out_4761019588596262838[297] = 0.0;
   out_4761019588596262838[298] = 0.0;
   out_4761019588596262838[299] = 0.0;
   out_4761019588596262838[300] = 0.0;
   out_4761019588596262838[301] = 0.0;
   out_4761019588596262838[302] = 0.0;
   out_4761019588596262838[303] = 0.0;
   out_4761019588596262838[304] = 1.0;
   out_4761019588596262838[305] = 0.0;
   out_4761019588596262838[306] = 0.0;
   out_4761019588596262838[307] = 0.0;
   out_4761019588596262838[308] = 0.0;
   out_4761019588596262838[309] = 0.0;
   out_4761019588596262838[310] = 0.0;
   out_4761019588596262838[311] = 0.0;
   out_4761019588596262838[312] = 0.0;
   out_4761019588596262838[313] = 0.0;
   out_4761019588596262838[314] = 0.0;
   out_4761019588596262838[315] = 0.0;
   out_4761019588596262838[316] = 0.0;
   out_4761019588596262838[317] = 0.0;
   out_4761019588596262838[318] = 0.0;
   out_4761019588596262838[319] = 0.0;
   out_4761019588596262838[320] = 0.0;
   out_4761019588596262838[321] = 0.0;
   out_4761019588596262838[322] = 0.0;
   out_4761019588596262838[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_1515506424900782745) {
   out_1515506424900782745[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_1515506424900782745[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_1515506424900782745[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_1515506424900782745[3] = dt*state[12] + state[3];
   out_1515506424900782745[4] = dt*state[13] + state[4];
   out_1515506424900782745[5] = dt*state[14] + state[5];
   out_1515506424900782745[6] = state[6];
   out_1515506424900782745[7] = state[7];
   out_1515506424900782745[8] = state[8];
   out_1515506424900782745[9] = state[9];
   out_1515506424900782745[10] = state[10];
   out_1515506424900782745[11] = state[11];
   out_1515506424900782745[12] = state[12];
   out_1515506424900782745[13] = state[13];
   out_1515506424900782745[14] = state[14];
   out_1515506424900782745[15] = state[15];
   out_1515506424900782745[16] = state[16];
   out_1515506424900782745[17] = state[17];
}
void F_fun(double *state, double dt, double *out_6672883653144067305) {
   out_6672883653144067305[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6672883653144067305[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6672883653144067305[2] = 0;
   out_6672883653144067305[3] = 0;
   out_6672883653144067305[4] = 0;
   out_6672883653144067305[5] = 0;
   out_6672883653144067305[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6672883653144067305[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6672883653144067305[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6672883653144067305[9] = 0;
   out_6672883653144067305[10] = 0;
   out_6672883653144067305[11] = 0;
   out_6672883653144067305[12] = 0;
   out_6672883653144067305[13] = 0;
   out_6672883653144067305[14] = 0;
   out_6672883653144067305[15] = 0;
   out_6672883653144067305[16] = 0;
   out_6672883653144067305[17] = 0;
   out_6672883653144067305[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6672883653144067305[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6672883653144067305[20] = 0;
   out_6672883653144067305[21] = 0;
   out_6672883653144067305[22] = 0;
   out_6672883653144067305[23] = 0;
   out_6672883653144067305[24] = 0;
   out_6672883653144067305[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6672883653144067305[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6672883653144067305[27] = 0;
   out_6672883653144067305[28] = 0;
   out_6672883653144067305[29] = 0;
   out_6672883653144067305[30] = 0;
   out_6672883653144067305[31] = 0;
   out_6672883653144067305[32] = 0;
   out_6672883653144067305[33] = 0;
   out_6672883653144067305[34] = 0;
   out_6672883653144067305[35] = 0;
   out_6672883653144067305[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6672883653144067305[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6672883653144067305[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6672883653144067305[39] = 0;
   out_6672883653144067305[40] = 0;
   out_6672883653144067305[41] = 0;
   out_6672883653144067305[42] = 0;
   out_6672883653144067305[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6672883653144067305[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6672883653144067305[45] = 0;
   out_6672883653144067305[46] = 0;
   out_6672883653144067305[47] = 0;
   out_6672883653144067305[48] = 0;
   out_6672883653144067305[49] = 0;
   out_6672883653144067305[50] = 0;
   out_6672883653144067305[51] = 0;
   out_6672883653144067305[52] = 0;
   out_6672883653144067305[53] = 0;
   out_6672883653144067305[54] = 0;
   out_6672883653144067305[55] = 0;
   out_6672883653144067305[56] = 0;
   out_6672883653144067305[57] = 1;
   out_6672883653144067305[58] = 0;
   out_6672883653144067305[59] = 0;
   out_6672883653144067305[60] = 0;
   out_6672883653144067305[61] = 0;
   out_6672883653144067305[62] = 0;
   out_6672883653144067305[63] = 0;
   out_6672883653144067305[64] = 0;
   out_6672883653144067305[65] = 0;
   out_6672883653144067305[66] = dt;
   out_6672883653144067305[67] = 0;
   out_6672883653144067305[68] = 0;
   out_6672883653144067305[69] = 0;
   out_6672883653144067305[70] = 0;
   out_6672883653144067305[71] = 0;
   out_6672883653144067305[72] = 0;
   out_6672883653144067305[73] = 0;
   out_6672883653144067305[74] = 0;
   out_6672883653144067305[75] = 0;
   out_6672883653144067305[76] = 1;
   out_6672883653144067305[77] = 0;
   out_6672883653144067305[78] = 0;
   out_6672883653144067305[79] = 0;
   out_6672883653144067305[80] = 0;
   out_6672883653144067305[81] = 0;
   out_6672883653144067305[82] = 0;
   out_6672883653144067305[83] = 0;
   out_6672883653144067305[84] = 0;
   out_6672883653144067305[85] = dt;
   out_6672883653144067305[86] = 0;
   out_6672883653144067305[87] = 0;
   out_6672883653144067305[88] = 0;
   out_6672883653144067305[89] = 0;
   out_6672883653144067305[90] = 0;
   out_6672883653144067305[91] = 0;
   out_6672883653144067305[92] = 0;
   out_6672883653144067305[93] = 0;
   out_6672883653144067305[94] = 0;
   out_6672883653144067305[95] = 1;
   out_6672883653144067305[96] = 0;
   out_6672883653144067305[97] = 0;
   out_6672883653144067305[98] = 0;
   out_6672883653144067305[99] = 0;
   out_6672883653144067305[100] = 0;
   out_6672883653144067305[101] = 0;
   out_6672883653144067305[102] = 0;
   out_6672883653144067305[103] = 0;
   out_6672883653144067305[104] = dt;
   out_6672883653144067305[105] = 0;
   out_6672883653144067305[106] = 0;
   out_6672883653144067305[107] = 0;
   out_6672883653144067305[108] = 0;
   out_6672883653144067305[109] = 0;
   out_6672883653144067305[110] = 0;
   out_6672883653144067305[111] = 0;
   out_6672883653144067305[112] = 0;
   out_6672883653144067305[113] = 0;
   out_6672883653144067305[114] = 1;
   out_6672883653144067305[115] = 0;
   out_6672883653144067305[116] = 0;
   out_6672883653144067305[117] = 0;
   out_6672883653144067305[118] = 0;
   out_6672883653144067305[119] = 0;
   out_6672883653144067305[120] = 0;
   out_6672883653144067305[121] = 0;
   out_6672883653144067305[122] = 0;
   out_6672883653144067305[123] = 0;
   out_6672883653144067305[124] = 0;
   out_6672883653144067305[125] = 0;
   out_6672883653144067305[126] = 0;
   out_6672883653144067305[127] = 0;
   out_6672883653144067305[128] = 0;
   out_6672883653144067305[129] = 0;
   out_6672883653144067305[130] = 0;
   out_6672883653144067305[131] = 0;
   out_6672883653144067305[132] = 0;
   out_6672883653144067305[133] = 1;
   out_6672883653144067305[134] = 0;
   out_6672883653144067305[135] = 0;
   out_6672883653144067305[136] = 0;
   out_6672883653144067305[137] = 0;
   out_6672883653144067305[138] = 0;
   out_6672883653144067305[139] = 0;
   out_6672883653144067305[140] = 0;
   out_6672883653144067305[141] = 0;
   out_6672883653144067305[142] = 0;
   out_6672883653144067305[143] = 0;
   out_6672883653144067305[144] = 0;
   out_6672883653144067305[145] = 0;
   out_6672883653144067305[146] = 0;
   out_6672883653144067305[147] = 0;
   out_6672883653144067305[148] = 0;
   out_6672883653144067305[149] = 0;
   out_6672883653144067305[150] = 0;
   out_6672883653144067305[151] = 0;
   out_6672883653144067305[152] = 1;
   out_6672883653144067305[153] = 0;
   out_6672883653144067305[154] = 0;
   out_6672883653144067305[155] = 0;
   out_6672883653144067305[156] = 0;
   out_6672883653144067305[157] = 0;
   out_6672883653144067305[158] = 0;
   out_6672883653144067305[159] = 0;
   out_6672883653144067305[160] = 0;
   out_6672883653144067305[161] = 0;
   out_6672883653144067305[162] = 0;
   out_6672883653144067305[163] = 0;
   out_6672883653144067305[164] = 0;
   out_6672883653144067305[165] = 0;
   out_6672883653144067305[166] = 0;
   out_6672883653144067305[167] = 0;
   out_6672883653144067305[168] = 0;
   out_6672883653144067305[169] = 0;
   out_6672883653144067305[170] = 0;
   out_6672883653144067305[171] = 1;
   out_6672883653144067305[172] = 0;
   out_6672883653144067305[173] = 0;
   out_6672883653144067305[174] = 0;
   out_6672883653144067305[175] = 0;
   out_6672883653144067305[176] = 0;
   out_6672883653144067305[177] = 0;
   out_6672883653144067305[178] = 0;
   out_6672883653144067305[179] = 0;
   out_6672883653144067305[180] = 0;
   out_6672883653144067305[181] = 0;
   out_6672883653144067305[182] = 0;
   out_6672883653144067305[183] = 0;
   out_6672883653144067305[184] = 0;
   out_6672883653144067305[185] = 0;
   out_6672883653144067305[186] = 0;
   out_6672883653144067305[187] = 0;
   out_6672883653144067305[188] = 0;
   out_6672883653144067305[189] = 0;
   out_6672883653144067305[190] = 1;
   out_6672883653144067305[191] = 0;
   out_6672883653144067305[192] = 0;
   out_6672883653144067305[193] = 0;
   out_6672883653144067305[194] = 0;
   out_6672883653144067305[195] = 0;
   out_6672883653144067305[196] = 0;
   out_6672883653144067305[197] = 0;
   out_6672883653144067305[198] = 0;
   out_6672883653144067305[199] = 0;
   out_6672883653144067305[200] = 0;
   out_6672883653144067305[201] = 0;
   out_6672883653144067305[202] = 0;
   out_6672883653144067305[203] = 0;
   out_6672883653144067305[204] = 0;
   out_6672883653144067305[205] = 0;
   out_6672883653144067305[206] = 0;
   out_6672883653144067305[207] = 0;
   out_6672883653144067305[208] = 0;
   out_6672883653144067305[209] = 1;
   out_6672883653144067305[210] = 0;
   out_6672883653144067305[211] = 0;
   out_6672883653144067305[212] = 0;
   out_6672883653144067305[213] = 0;
   out_6672883653144067305[214] = 0;
   out_6672883653144067305[215] = 0;
   out_6672883653144067305[216] = 0;
   out_6672883653144067305[217] = 0;
   out_6672883653144067305[218] = 0;
   out_6672883653144067305[219] = 0;
   out_6672883653144067305[220] = 0;
   out_6672883653144067305[221] = 0;
   out_6672883653144067305[222] = 0;
   out_6672883653144067305[223] = 0;
   out_6672883653144067305[224] = 0;
   out_6672883653144067305[225] = 0;
   out_6672883653144067305[226] = 0;
   out_6672883653144067305[227] = 0;
   out_6672883653144067305[228] = 1;
   out_6672883653144067305[229] = 0;
   out_6672883653144067305[230] = 0;
   out_6672883653144067305[231] = 0;
   out_6672883653144067305[232] = 0;
   out_6672883653144067305[233] = 0;
   out_6672883653144067305[234] = 0;
   out_6672883653144067305[235] = 0;
   out_6672883653144067305[236] = 0;
   out_6672883653144067305[237] = 0;
   out_6672883653144067305[238] = 0;
   out_6672883653144067305[239] = 0;
   out_6672883653144067305[240] = 0;
   out_6672883653144067305[241] = 0;
   out_6672883653144067305[242] = 0;
   out_6672883653144067305[243] = 0;
   out_6672883653144067305[244] = 0;
   out_6672883653144067305[245] = 0;
   out_6672883653144067305[246] = 0;
   out_6672883653144067305[247] = 1;
   out_6672883653144067305[248] = 0;
   out_6672883653144067305[249] = 0;
   out_6672883653144067305[250] = 0;
   out_6672883653144067305[251] = 0;
   out_6672883653144067305[252] = 0;
   out_6672883653144067305[253] = 0;
   out_6672883653144067305[254] = 0;
   out_6672883653144067305[255] = 0;
   out_6672883653144067305[256] = 0;
   out_6672883653144067305[257] = 0;
   out_6672883653144067305[258] = 0;
   out_6672883653144067305[259] = 0;
   out_6672883653144067305[260] = 0;
   out_6672883653144067305[261] = 0;
   out_6672883653144067305[262] = 0;
   out_6672883653144067305[263] = 0;
   out_6672883653144067305[264] = 0;
   out_6672883653144067305[265] = 0;
   out_6672883653144067305[266] = 1;
   out_6672883653144067305[267] = 0;
   out_6672883653144067305[268] = 0;
   out_6672883653144067305[269] = 0;
   out_6672883653144067305[270] = 0;
   out_6672883653144067305[271] = 0;
   out_6672883653144067305[272] = 0;
   out_6672883653144067305[273] = 0;
   out_6672883653144067305[274] = 0;
   out_6672883653144067305[275] = 0;
   out_6672883653144067305[276] = 0;
   out_6672883653144067305[277] = 0;
   out_6672883653144067305[278] = 0;
   out_6672883653144067305[279] = 0;
   out_6672883653144067305[280] = 0;
   out_6672883653144067305[281] = 0;
   out_6672883653144067305[282] = 0;
   out_6672883653144067305[283] = 0;
   out_6672883653144067305[284] = 0;
   out_6672883653144067305[285] = 1;
   out_6672883653144067305[286] = 0;
   out_6672883653144067305[287] = 0;
   out_6672883653144067305[288] = 0;
   out_6672883653144067305[289] = 0;
   out_6672883653144067305[290] = 0;
   out_6672883653144067305[291] = 0;
   out_6672883653144067305[292] = 0;
   out_6672883653144067305[293] = 0;
   out_6672883653144067305[294] = 0;
   out_6672883653144067305[295] = 0;
   out_6672883653144067305[296] = 0;
   out_6672883653144067305[297] = 0;
   out_6672883653144067305[298] = 0;
   out_6672883653144067305[299] = 0;
   out_6672883653144067305[300] = 0;
   out_6672883653144067305[301] = 0;
   out_6672883653144067305[302] = 0;
   out_6672883653144067305[303] = 0;
   out_6672883653144067305[304] = 1;
   out_6672883653144067305[305] = 0;
   out_6672883653144067305[306] = 0;
   out_6672883653144067305[307] = 0;
   out_6672883653144067305[308] = 0;
   out_6672883653144067305[309] = 0;
   out_6672883653144067305[310] = 0;
   out_6672883653144067305[311] = 0;
   out_6672883653144067305[312] = 0;
   out_6672883653144067305[313] = 0;
   out_6672883653144067305[314] = 0;
   out_6672883653144067305[315] = 0;
   out_6672883653144067305[316] = 0;
   out_6672883653144067305[317] = 0;
   out_6672883653144067305[318] = 0;
   out_6672883653144067305[319] = 0;
   out_6672883653144067305[320] = 0;
   out_6672883653144067305[321] = 0;
   out_6672883653144067305[322] = 0;
   out_6672883653144067305[323] = 1;
}
void h_4(double *state, double *unused, double *out_2268895945492712976) {
   out_2268895945492712976[0] = state[6] + state[9];
   out_2268895945492712976[1] = state[7] + state[10];
   out_2268895945492712976[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_989212378752355266) {
   out_989212378752355266[0] = 0;
   out_989212378752355266[1] = 0;
   out_989212378752355266[2] = 0;
   out_989212378752355266[3] = 0;
   out_989212378752355266[4] = 0;
   out_989212378752355266[5] = 0;
   out_989212378752355266[6] = 1;
   out_989212378752355266[7] = 0;
   out_989212378752355266[8] = 0;
   out_989212378752355266[9] = 1;
   out_989212378752355266[10] = 0;
   out_989212378752355266[11] = 0;
   out_989212378752355266[12] = 0;
   out_989212378752355266[13] = 0;
   out_989212378752355266[14] = 0;
   out_989212378752355266[15] = 0;
   out_989212378752355266[16] = 0;
   out_989212378752355266[17] = 0;
   out_989212378752355266[18] = 0;
   out_989212378752355266[19] = 0;
   out_989212378752355266[20] = 0;
   out_989212378752355266[21] = 0;
   out_989212378752355266[22] = 0;
   out_989212378752355266[23] = 0;
   out_989212378752355266[24] = 0;
   out_989212378752355266[25] = 1;
   out_989212378752355266[26] = 0;
   out_989212378752355266[27] = 0;
   out_989212378752355266[28] = 1;
   out_989212378752355266[29] = 0;
   out_989212378752355266[30] = 0;
   out_989212378752355266[31] = 0;
   out_989212378752355266[32] = 0;
   out_989212378752355266[33] = 0;
   out_989212378752355266[34] = 0;
   out_989212378752355266[35] = 0;
   out_989212378752355266[36] = 0;
   out_989212378752355266[37] = 0;
   out_989212378752355266[38] = 0;
   out_989212378752355266[39] = 0;
   out_989212378752355266[40] = 0;
   out_989212378752355266[41] = 0;
   out_989212378752355266[42] = 0;
   out_989212378752355266[43] = 0;
   out_989212378752355266[44] = 1;
   out_989212378752355266[45] = 0;
   out_989212378752355266[46] = 0;
   out_989212378752355266[47] = 1;
   out_989212378752355266[48] = 0;
   out_989212378752355266[49] = 0;
   out_989212378752355266[50] = 0;
   out_989212378752355266[51] = 0;
   out_989212378752355266[52] = 0;
   out_989212378752355266[53] = 0;
}
void h_10(double *state, double *unused, double *out_8157035487248433085) {
   out_8157035487248433085[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_8157035487248433085[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_8157035487248433085[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_3040720830858167540) {
   out_3040720830858167540[0] = 0;
   out_3040720830858167540[1] = 9.8100000000000005*cos(state[1]);
   out_3040720830858167540[2] = 0;
   out_3040720830858167540[3] = 0;
   out_3040720830858167540[4] = -state[8];
   out_3040720830858167540[5] = state[7];
   out_3040720830858167540[6] = 0;
   out_3040720830858167540[7] = state[5];
   out_3040720830858167540[8] = -state[4];
   out_3040720830858167540[9] = 0;
   out_3040720830858167540[10] = 0;
   out_3040720830858167540[11] = 0;
   out_3040720830858167540[12] = 1;
   out_3040720830858167540[13] = 0;
   out_3040720830858167540[14] = 0;
   out_3040720830858167540[15] = 1;
   out_3040720830858167540[16] = 0;
   out_3040720830858167540[17] = 0;
   out_3040720830858167540[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_3040720830858167540[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_3040720830858167540[20] = 0;
   out_3040720830858167540[21] = state[8];
   out_3040720830858167540[22] = 0;
   out_3040720830858167540[23] = -state[6];
   out_3040720830858167540[24] = -state[5];
   out_3040720830858167540[25] = 0;
   out_3040720830858167540[26] = state[3];
   out_3040720830858167540[27] = 0;
   out_3040720830858167540[28] = 0;
   out_3040720830858167540[29] = 0;
   out_3040720830858167540[30] = 0;
   out_3040720830858167540[31] = 1;
   out_3040720830858167540[32] = 0;
   out_3040720830858167540[33] = 0;
   out_3040720830858167540[34] = 1;
   out_3040720830858167540[35] = 0;
   out_3040720830858167540[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_3040720830858167540[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_3040720830858167540[38] = 0;
   out_3040720830858167540[39] = -state[7];
   out_3040720830858167540[40] = state[6];
   out_3040720830858167540[41] = 0;
   out_3040720830858167540[42] = state[4];
   out_3040720830858167540[43] = -state[3];
   out_3040720830858167540[44] = 0;
   out_3040720830858167540[45] = 0;
   out_3040720830858167540[46] = 0;
   out_3040720830858167540[47] = 0;
   out_3040720830858167540[48] = 0;
   out_3040720830858167540[49] = 0;
   out_3040720830858167540[50] = 1;
   out_3040720830858167540[51] = 0;
   out_3040720830858167540[52] = 0;
   out_3040720830858167540[53] = 1;
}
void h_13(double *state, double *unused, double *out_7347695323865909197) {
   out_7347695323865909197[0] = state[3];
   out_7347695323865909197[1] = state[4];
   out_7347695323865909197[2] = state[5];
}
void H_13(double *state, double *unused, double *out_6621418829564345663) {
   out_6621418829564345663[0] = 0;
   out_6621418829564345663[1] = 0;
   out_6621418829564345663[2] = 0;
   out_6621418829564345663[3] = 1;
   out_6621418829564345663[4] = 0;
   out_6621418829564345663[5] = 0;
   out_6621418829564345663[6] = 0;
   out_6621418829564345663[7] = 0;
   out_6621418829564345663[8] = 0;
   out_6621418829564345663[9] = 0;
   out_6621418829564345663[10] = 0;
   out_6621418829564345663[11] = 0;
   out_6621418829564345663[12] = 0;
   out_6621418829564345663[13] = 0;
   out_6621418829564345663[14] = 0;
   out_6621418829564345663[15] = 0;
   out_6621418829564345663[16] = 0;
   out_6621418829564345663[17] = 0;
   out_6621418829564345663[18] = 0;
   out_6621418829564345663[19] = 0;
   out_6621418829564345663[20] = 0;
   out_6621418829564345663[21] = 0;
   out_6621418829564345663[22] = 1;
   out_6621418829564345663[23] = 0;
   out_6621418829564345663[24] = 0;
   out_6621418829564345663[25] = 0;
   out_6621418829564345663[26] = 0;
   out_6621418829564345663[27] = 0;
   out_6621418829564345663[28] = 0;
   out_6621418829564345663[29] = 0;
   out_6621418829564345663[30] = 0;
   out_6621418829564345663[31] = 0;
   out_6621418829564345663[32] = 0;
   out_6621418829564345663[33] = 0;
   out_6621418829564345663[34] = 0;
   out_6621418829564345663[35] = 0;
   out_6621418829564345663[36] = 0;
   out_6621418829564345663[37] = 0;
   out_6621418829564345663[38] = 0;
   out_6621418829564345663[39] = 0;
   out_6621418829564345663[40] = 0;
   out_6621418829564345663[41] = 1;
   out_6621418829564345663[42] = 0;
   out_6621418829564345663[43] = 0;
   out_6621418829564345663[44] = 0;
   out_6621418829564345663[45] = 0;
   out_6621418829564345663[46] = 0;
   out_6621418829564345663[47] = 0;
   out_6621418829564345663[48] = 0;
   out_6621418829564345663[49] = 0;
   out_6621418829564345663[50] = 0;
   out_6621418829564345663[51] = 0;
   out_6621418829564345663[52] = 0;
   out_6621418829564345663[53] = 0;
}
void h_14(double *state, double *unused, double *out_1667724773537730315) {
   out_1667724773537730315[0] = state[6];
   out_1667724773537730315[1] = state[7];
   out_1667724773537730315[2] = state[8];
}
void H_14(double *state, double *unused, double *out_4072000811047727562) {
   out_4072000811047727562[0] = 0;
   out_4072000811047727562[1] = 0;
   out_4072000811047727562[2] = 0;
   out_4072000811047727562[3] = 0;
   out_4072000811047727562[4] = 0;
   out_4072000811047727562[5] = 0;
   out_4072000811047727562[6] = 1;
   out_4072000811047727562[7] = 0;
   out_4072000811047727562[8] = 0;
   out_4072000811047727562[9] = 0;
   out_4072000811047727562[10] = 0;
   out_4072000811047727562[11] = 0;
   out_4072000811047727562[12] = 0;
   out_4072000811047727562[13] = 0;
   out_4072000811047727562[14] = 0;
   out_4072000811047727562[15] = 0;
   out_4072000811047727562[16] = 0;
   out_4072000811047727562[17] = 0;
   out_4072000811047727562[18] = 0;
   out_4072000811047727562[19] = 0;
   out_4072000811047727562[20] = 0;
   out_4072000811047727562[21] = 0;
   out_4072000811047727562[22] = 0;
   out_4072000811047727562[23] = 0;
   out_4072000811047727562[24] = 0;
   out_4072000811047727562[25] = 1;
   out_4072000811047727562[26] = 0;
   out_4072000811047727562[27] = 0;
   out_4072000811047727562[28] = 0;
   out_4072000811047727562[29] = 0;
   out_4072000811047727562[30] = 0;
   out_4072000811047727562[31] = 0;
   out_4072000811047727562[32] = 0;
   out_4072000811047727562[33] = 0;
   out_4072000811047727562[34] = 0;
   out_4072000811047727562[35] = 0;
   out_4072000811047727562[36] = 0;
   out_4072000811047727562[37] = 0;
   out_4072000811047727562[38] = 0;
   out_4072000811047727562[39] = 0;
   out_4072000811047727562[40] = 0;
   out_4072000811047727562[41] = 0;
   out_4072000811047727562[42] = 0;
   out_4072000811047727562[43] = 0;
   out_4072000811047727562[44] = 1;
   out_4072000811047727562[45] = 0;
   out_4072000811047727562[46] = 0;
   out_4072000811047727562[47] = 0;
   out_4072000811047727562[48] = 0;
   out_4072000811047727562[49] = 0;
   out_4072000811047727562[50] = 0;
   out_4072000811047727562[51] = 0;
   out_4072000811047727562[52] = 0;
   out_4072000811047727562[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_2226712134560383445) {
  err_fun(nom_x, delta_x, out_2226712134560383445);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5991885512389717825) {
  inv_err_fun(nom_x, true_x, out_5991885512389717825);
}
void pose_H_mod_fun(double *state, double *out_4761019588596262838) {
  H_mod_fun(state, out_4761019588596262838);
}
void pose_f_fun(double *state, double dt, double *out_1515506424900782745) {
  f_fun(state,  dt, out_1515506424900782745);
}
void pose_F_fun(double *state, double dt, double *out_6672883653144067305) {
  F_fun(state,  dt, out_6672883653144067305);
}
void pose_h_4(double *state, double *unused, double *out_2268895945492712976) {
  h_4(state, unused, out_2268895945492712976);
}
void pose_H_4(double *state, double *unused, double *out_989212378752355266) {
  H_4(state, unused, out_989212378752355266);
}
void pose_h_10(double *state, double *unused, double *out_8157035487248433085) {
  h_10(state, unused, out_8157035487248433085);
}
void pose_H_10(double *state, double *unused, double *out_3040720830858167540) {
  H_10(state, unused, out_3040720830858167540);
}
void pose_h_13(double *state, double *unused, double *out_7347695323865909197) {
  h_13(state, unused, out_7347695323865909197);
}
void pose_H_13(double *state, double *unused, double *out_6621418829564345663) {
  H_13(state, unused, out_6621418829564345663);
}
void pose_h_14(double *state, double *unused, double *out_1667724773537730315) {
  h_14(state, unused, out_1667724773537730315);
}
void pose_H_14(double *state, double *unused, double *out_4072000811047727562) {
  H_14(state, unused, out_4072000811047727562);
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
