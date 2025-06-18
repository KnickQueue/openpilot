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
void err_fun(double *nom_x, double *delta_x, double *out_8944739138199939797) {
   out_8944739138199939797[0] = delta_x[0] + nom_x[0];
   out_8944739138199939797[1] = delta_x[1] + nom_x[1];
   out_8944739138199939797[2] = delta_x[2] + nom_x[2];
   out_8944739138199939797[3] = delta_x[3] + nom_x[3];
   out_8944739138199939797[4] = delta_x[4] + nom_x[4];
   out_8944739138199939797[5] = delta_x[5] + nom_x[5];
   out_8944739138199939797[6] = delta_x[6] + nom_x[6];
   out_8944739138199939797[7] = delta_x[7] + nom_x[7];
   out_8944739138199939797[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8518252015721902519) {
   out_8518252015721902519[0] = -nom_x[0] + true_x[0];
   out_8518252015721902519[1] = -nom_x[1] + true_x[1];
   out_8518252015721902519[2] = -nom_x[2] + true_x[2];
   out_8518252015721902519[3] = -nom_x[3] + true_x[3];
   out_8518252015721902519[4] = -nom_x[4] + true_x[4];
   out_8518252015721902519[5] = -nom_x[5] + true_x[5];
   out_8518252015721902519[6] = -nom_x[6] + true_x[6];
   out_8518252015721902519[7] = -nom_x[7] + true_x[7];
   out_8518252015721902519[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2999799436753241489) {
   out_2999799436753241489[0] = 1.0;
   out_2999799436753241489[1] = 0.0;
   out_2999799436753241489[2] = 0.0;
   out_2999799436753241489[3] = 0.0;
   out_2999799436753241489[4] = 0.0;
   out_2999799436753241489[5] = 0.0;
   out_2999799436753241489[6] = 0.0;
   out_2999799436753241489[7] = 0.0;
   out_2999799436753241489[8] = 0.0;
   out_2999799436753241489[9] = 0.0;
   out_2999799436753241489[10] = 1.0;
   out_2999799436753241489[11] = 0.0;
   out_2999799436753241489[12] = 0.0;
   out_2999799436753241489[13] = 0.0;
   out_2999799436753241489[14] = 0.0;
   out_2999799436753241489[15] = 0.0;
   out_2999799436753241489[16] = 0.0;
   out_2999799436753241489[17] = 0.0;
   out_2999799436753241489[18] = 0.0;
   out_2999799436753241489[19] = 0.0;
   out_2999799436753241489[20] = 1.0;
   out_2999799436753241489[21] = 0.0;
   out_2999799436753241489[22] = 0.0;
   out_2999799436753241489[23] = 0.0;
   out_2999799436753241489[24] = 0.0;
   out_2999799436753241489[25] = 0.0;
   out_2999799436753241489[26] = 0.0;
   out_2999799436753241489[27] = 0.0;
   out_2999799436753241489[28] = 0.0;
   out_2999799436753241489[29] = 0.0;
   out_2999799436753241489[30] = 1.0;
   out_2999799436753241489[31] = 0.0;
   out_2999799436753241489[32] = 0.0;
   out_2999799436753241489[33] = 0.0;
   out_2999799436753241489[34] = 0.0;
   out_2999799436753241489[35] = 0.0;
   out_2999799436753241489[36] = 0.0;
   out_2999799436753241489[37] = 0.0;
   out_2999799436753241489[38] = 0.0;
   out_2999799436753241489[39] = 0.0;
   out_2999799436753241489[40] = 1.0;
   out_2999799436753241489[41] = 0.0;
   out_2999799436753241489[42] = 0.0;
   out_2999799436753241489[43] = 0.0;
   out_2999799436753241489[44] = 0.0;
   out_2999799436753241489[45] = 0.0;
   out_2999799436753241489[46] = 0.0;
   out_2999799436753241489[47] = 0.0;
   out_2999799436753241489[48] = 0.0;
   out_2999799436753241489[49] = 0.0;
   out_2999799436753241489[50] = 1.0;
   out_2999799436753241489[51] = 0.0;
   out_2999799436753241489[52] = 0.0;
   out_2999799436753241489[53] = 0.0;
   out_2999799436753241489[54] = 0.0;
   out_2999799436753241489[55] = 0.0;
   out_2999799436753241489[56] = 0.0;
   out_2999799436753241489[57] = 0.0;
   out_2999799436753241489[58] = 0.0;
   out_2999799436753241489[59] = 0.0;
   out_2999799436753241489[60] = 1.0;
   out_2999799436753241489[61] = 0.0;
   out_2999799436753241489[62] = 0.0;
   out_2999799436753241489[63] = 0.0;
   out_2999799436753241489[64] = 0.0;
   out_2999799436753241489[65] = 0.0;
   out_2999799436753241489[66] = 0.0;
   out_2999799436753241489[67] = 0.0;
   out_2999799436753241489[68] = 0.0;
   out_2999799436753241489[69] = 0.0;
   out_2999799436753241489[70] = 1.0;
   out_2999799436753241489[71] = 0.0;
   out_2999799436753241489[72] = 0.0;
   out_2999799436753241489[73] = 0.0;
   out_2999799436753241489[74] = 0.0;
   out_2999799436753241489[75] = 0.0;
   out_2999799436753241489[76] = 0.0;
   out_2999799436753241489[77] = 0.0;
   out_2999799436753241489[78] = 0.0;
   out_2999799436753241489[79] = 0.0;
   out_2999799436753241489[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8076686351349619864) {
   out_8076686351349619864[0] = state[0];
   out_8076686351349619864[1] = state[1];
   out_8076686351349619864[2] = state[2];
   out_8076686351349619864[3] = state[3];
   out_8076686351349619864[4] = state[4];
   out_8076686351349619864[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8076686351349619864[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8076686351349619864[7] = state[7];
   out_8076686351349619864[8] = state[8];
}
void F_fun(double *state, double dt, double *out_9005642596725164851) {
   out_9005642596725164851[0] = 1;
   out_9005642596725164851[1] = 0;
   out_9005642596725164851[2] = 0;
   out_9005642596725164851[3] = 0;
   out_9005642596725164851[4] = 0;
   out_9005642596725164851[5] = 0;
   out_9005642596725164851[6] = 0;
   out_9005642596725164851[7] = 0;
   out_9005642596725164851[8] = 0;
   out_9005642596725164851[9] = 0;
   out_9005642596725164851[10] = 1;
   out_9005642596725164851[11] = 0;
   out_9005642596725164851[12] = 0;
   out_9005642596725164851[13] = 0;
   out_9005642596725164851[14] = 0;
   out_9005642596725164851[15] = 0;
   out_9005642596725164851[16] = 0;
   out_9005642596725164851[17] = 0;
   out_9005642596725164851[18] = 0;
   out_9005642596725164851[19] = 0;
   out_9005642596725164851[20] = 1;
   out_9005642596725164851[21] = 0;
   out_9005642596725164851[22] = 0;
   out_9005642596725164851[23] = 0;
   out_9005642596725164851[24] = 0;
   out_9005642596725164851[25] = 0;
   out_9005642596725164851[26] = 0;
   out_9005642596725164851[27] = 0;
   out_9005642596725164851[28] = 0;
   out_9005642596725164851[29] = 0;
   out_9005642596725164851[30] = 1;
   out_9005642596725164851[31] = 0;
   out_9005642596725164851[32] = 0;
   out_9005642596725164851[33] = 0;
   out_9005642596725164851[34] = 0;
   out_9005642596725164851[35] = 0;
   out_9005642596725164851[36] = 0;
   out_9005642596725164851[37] = 0;
   out_9005642596725164851[38] = 0;
   out_9005642596725164851[39] = 0;
   out_9005642596725164851[40] = 1;
   out_9005642596725164851[41] = 0;
   out_9005642596725164851[42] = 0;
   out_9005642596725164851[43] = 0;
   out_9005642596725164851[44] = 0;
   out_9005642596725164851[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_9005642596725164851[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_9005642596725164851[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_9005642596725164851[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_9005642596725164851[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_9005642596725164851[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_9005642596725164851[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_9005642596725164851[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_9005642596725164851[53] = -9.8000000000000007*dt;
   out_9005642596725164851[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_9005642596725164851[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_9005642596725164851[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9005642596725164851[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9005642596725164851[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_9005642596725164851[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_9005642596725164851[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_9005642596725164851[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9005642596725164851[62] = 0;
   out_9005642596725164851[63] = 0;
   out_9005642596725164851[64] = 0;
   out_9005642596725164851[65] = 0;
   out_9005642596725164851[66] = 0;
   out_9005642596725164851[67] = 0;
   out_9005642596725164851[68] = 0;
   out_9005642596725164851[69] = 0;
   out_9005642596725164851[70] = 1;
   out_9005642596725164851[71] = 0;
   out_9005642596725164851[72] = 0;
   out_9005642596725164851[73] = 0;
   out_9005642596725164851[74] = 0;
   out_9005642596725164851[75] = 0;
   out_9005642596725164851[76] = 0;
   out_9005642596725164851[77] = 0;
   out_9005642596725164851[78] = 0;
   out_9005642596725164851[79] = 0;
   out_9005642596725164851[80] = 1;
}
void h_25(double *state, double *unused, double *out_9023267735589281550) {
   out_9023267735589281550[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1695460326561269869) {
   out_1695460326561269869[0] = 0;
   out_1695460326561269869[1] = 0;
   out_1695460326561269869[2] = 0;
   out_1695460326561269869[3] = 0;
   out_1695460326561269869[4] = 0;
   out_1695460326561269869[5] = 0;
   out_1695460326561269869[6] = 1;
   out_1695460326561269869[7] = 0;
   out_1695460326561269869[8] = 0;
}
void h_24(double *state, double *unused, double *out_8947275820086677161) {
   out_8947275820086677161[0] = state[4];
   out_8947275820086677161[1] = state[5];
}
void H_24(double *state, double *unused, double *out_477189272444229697) {
   out_477189272444229697[0] = 0;
   out_477189272444229697[1] = 0;
   out_477189272444229697[2] = 0;
   out_477189272444229697[3] = 0;
   out_477189272444229697[4] = 1;
   out_477189272444229697[5] = 0;
   out_477189272444229697[6] = 0;
   out_477189272444229697[7] = 0;
   out_477189272444229697[8] = 0;
   out_477189272444229697[9] = 0;
   out_477189272444229697[10] = 0;
   out_477189272444229697[11] = 0;
   out_477189272444229697[12] = 0;
   out_477189272444229697[13] = 0;
   out_477189272444229697[14] = 1;
   out_477189272444229697[15] = 0;
   out_477189272444229697[16] = 0;
   out_477189272444229697[17] = 0;
}
void h_30(double *state, double *unused, double *out_1660211081299597625) {
   out_1660211081299597625[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8612150668052886624) {
   out_8612150668052886624[0] = 0;
   out_8612150668052886624[1] = 0;
   out_8612150668052886624[2] = 0;
   out_8612150668052886624[3] = 0;
   out_8612150668052886624[4] = 1;
   out_8612150668052886624[5] = 0;
   out_8612150668052886624[6] = 0;
   out_8612150668052886624[7] = 0;
   out_8612150668052886624[8] = 0;
}
void h_26(double *state, double *unused, double *out_4264616402108295276) {
   out_4264616402108295276[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2046042992312786355) {
   out_2046042992312786355[0] = 0;
   out_2046042992312786355[1] = 0;
   out_2046042992312786355[2] = 0;
   out_2046042992312786355[3] = 0;
   out_2046042992312786355[4] = 0;
   out_2046042992312786355[5] = 0;
   out_2046042992312786355[6] = 0;
   out_2046042992312786355[7] = 1;
   out_2046042992312786355[8] = 0;
}
void h_27(double *state, double *unused, double *out_6386690286675937381) {
   out_6386690286675937381[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6437387356252461713) {
   out_6437387356252461713[0] = 0;
   out_6437387356252461713[1] = 0;
   out_6437387356252461713[2] = 0;
   out_6437387356252461713[3] = 1;
   out_6437387356252461713[4] = 0;
   out_6437387356252461713[5] = 0;
   out_6437387356252461713[6] = 0;
   out_6437387356252461713[7] = 0;
   out_6437387356252461713[8] = 0;
}
void h_29(double *state, double *unused, double *out_6661884348960443270) {
   out_6661884348960443270[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4724024629382910680) {
   out_4724024629382910680[0] = 0;
   out_4724024629382910680[1] = 1;
   out_4724024629382910680[2] = 0;
   out_4724024629382910680[3] = 0;
   out_4724024629382910680[4] = 0;
   out_4724024629382910680[5] = 0;
   out_4724024629382910680[6] = 0;
   out_4724024629382910680[7] = 0;
   out_4724024629382910680[8] = 0;
}
void h_28(double *state, double *unused, double *out_8090852339424763643) {
   out_8090852339424763643[0] = state[0];
}
void H_28(double *state, double *unused, double *out_358374387686619894) {
   out_358374387686619894[0] = 1;
   out_358374387686619894[1] = 0;
   out_358374387686619894[2] = 0;
   out_358374387686619894[3] = 0;
   out_358374387686619894[4] = 0;
   out_358374387686619894[5] = 0;
   out_358374387686619894[6] = 0;
   out_358374387686619894[7] = 0;
   out_358374387686619894[8] = 0;
}
void h_31(double *state, double *unused, double *out_5483320317294463600) {
   out_5483320317294463600[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1726106288438230297) {
   out_1726106288438230297[0] = 0;
   out_1726106288438230297[1] = 0;
   out_1726106288438230297[2] = 0;
   out_1726106288438230297[3] = 0;
   out_1726106288438230297[4] = 0;
   out_1726106288438230297[5] = 0;
   out_1726106288438230297[6] = 0;
   out_1726106288438230297[7] = 0;
   out_1726106288438230297[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8944739138199939797) {
  err_fun(nom_x, delta_x, out_8944739138199939797);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8518252015721902519) {
  inv_err_fun(nom_x, true_x, out_8518252015721902519);
}
void car_H_mod_fun(double *state, double *out_2999799436753241489) {
  H_mod_fun(state, out_2999799436753241489);
}
void car_f_fun(double *state, double dt, double *out_8076686351349619864) {
  f_fun(state,  dt, out_8076686351349619864);
}
void car_F_fun(double *state, double dt, double *out_9005642596725164851) {
  F_fun(state,  dt, out_9005642596725164851);
}
void car_h_25(double *state, double *unused, double *out_9023267735589281550) {
  h_25(state, unused, out_9023267735589281550);
}
void car_H_25(double *state, double *unused, double *out_1695460326561269869) {
  H_25(state, unused, out_1695460326561269869);
}
void car_h_24(double *state, double *unused, double *out_8947275820086677161) {
  h_24(state, unused, out_8947275820086677161);
}
void car_H_24(double *state, double *unused, double *out_477189272444229697) {
  H_24(state, unused, out_477189272444229697);
}
void car_h_30(double *state, double *unused, double *out_1660211081299597625) {
  h_30(state, unused, out_1660211081299597625);
}
void car_H_30(double *state, double *unused, double *out_8612150668052886624) {
  H_30(state, unused, out_8612150668052886624);
}
void car_h_26(double *state, double *unused, double *out_4264616402108295276) {
  h_26(state, unused, out_4264616402108295276);
}
void car_H_26(double *state, double *unused, double *out_2046042992312786355) {
  H_26(state, unused, out_2046042992312786355);
}
void car_h_27(double *state, double *unused, double *out_6386690286675937381) {
  h_27(state, unused, out_6386690286675937381);
}
void car_H_27(double *state, double *unused, double *out_6437387356252461713) {
  H_27(state, unused, out_6437387356252461713);
}
void car_h_29(double *state, double *unused, double *out_6661884348960443270) {
  h_29(state, unused, out_6661884348960443270);
}
void car_H_29(double *state, double *unused, double *out_4724024629382910680) {
  H_29(state, unused, out_4724024629382910680);
}
void car_h_28(double *state, double *unused, double *out_8090852339424763643) {
  h_28(state, unused, out_8090852339424763643);
}
void car_H_28(double *state, double *unused, double *out_358374387686619894) {
  H_28(state, unused, out_358374387686619894);
}
void car_h_31(double *state, double *unused, double *out_5483320317294463600) {
  h_31(state, unused, out_5483320317294463600);
}
void car_H_31(double *state, double *unused, double *out_1726106288438230297) {
  H_31(state, unused, out_1726106288438230297);
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
