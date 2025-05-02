#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_2343109546089990432);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6250262588326011624);
void pose_H_mod_fun(double *state, double *out_4730078843917365686);
void pose_f_fun(double *state, double dt, double *out_5266432288628039552);
void pose_F_fun(double *state, double dt, double *out_6430483000299200317);
void pose_h_4(double *state, double *unused, double *out_5871837789572178941);
void pose_H_4(double *state, double *unused, double *out_5928026325800263790);
void pose_h_10(double *state, double *unused, double *out_4831208843302909645);
void pose_H_10(double *state, double *unused, double *out_3150410560262957570);
void pose_h_13(double *state, double *unused, double *out_3542107992820507541);
void pose_H_13(double *state, double *unused, double *out_9140300151132596591);
void pose_h_14(double *state, double *unused, double *out_129812995230636672);
void pose_H_14(double *state, double *unused, double *out_8555476891569803297);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}