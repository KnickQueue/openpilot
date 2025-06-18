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
void car_err_fun(double *nom_x, double *delta_x, double *out_8944739138199939797);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8518252015721902519);
void car_H_mod_fun(double *state, double *out_2999799436753241489);
void car_f_fun(double *state, double dt, double *out_8076686351349619864);
void car_F_fun(double *state, double dt, double *out_9005642596725164851);
void car_h_25(double *state, double *unused, double *out_9023267735589281550);
void car_H_25(double *state, double *unused, double *out_1695460326561269869);
void car_h_24(double *state, double *unused, double *out_8947275820086677161);
void car_H_24(double *state, double *unused, double *out_477189272444229697);
void car_h_30(double *state, double *unused, double *out_1660211081299597625);
void car_H_30(double *state, double *unused, double *out_8612150668052886624);
void car_h_26(double *state, double *unused, double *out_4264616402108295276);
void car_H_26(double *state, double *unused, double *out_2046042992312786355);
void car_h_27(double *state, double *unused, double *out_6386690286675937381);
void car_H_27(double *state, double *unused, double *out_6437387356252461713);
void car_h_29(double *state, double *unused, double *out_6661884348960443270);
void car_H_29(double *state, double *unused, double *out_4724024629382910680);
void car_h_28(double *state, double *unused, double *out_8090852339424763643);
void car_H_28(double *state, double *unused, double *out_358374387686619894);
void car_h_31(double *state, double *unused, double *out_5483320317294463600);
void car_H_31(double *state, double *unused, double *out_1726106288438230297);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}