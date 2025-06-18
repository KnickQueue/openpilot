#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_2226712134560383445);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5991885512389717825);
void pose_H_mod_fun(double *state, double *out_4761019588596262838);
void pose_f_fun(double *state, double dt, double *out_1515506424900782745);
void pose_F_fun(double *state, double dt, double *out_6672883653144067305);
void pose_h_4(double *state, double *unused, double *out_2268895945492712976);
void pose_H_4(double *state, double *unused, double *out_989212378752355266);
void pose_h_10(double *state, double *unused, double *out_8157035487248433085);
void pose_H_10(double *state, double *unused, double *out_3040720830858167540);
void pose_h_13(double *state, double *unused, double *out_7347695323865909197);
void pose_H_13(double *state, double *unused, double *out_6621418829564345663);
void pose_h_14(double *state, double *unused, double *out_1667724773537730315);
void pose_H_14(double *state, double *unused, double *out_4072000811047727562);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}