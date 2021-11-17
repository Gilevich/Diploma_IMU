/*
 * File: Kalman_filer_function.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 07-Nov-2021 14:55:24
 */

/* Include Files */
#include "Kalman_filer_function.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double acc_data[3]
 *                const double gyr_data[3]
 *                const double old_x_post_data[6]
 *                const double old_p_post_data[36]
 *                double x_post_data[6]
 *                double p_post_data[36]
 * Return Type  : void
 */
void Kalman_filer_function(const double acc_data[3], const double gyr_data[3],
  const double old_x_post_data[6], const double old_p_post_data[36], double
  x_post_data[6], double p_post_data[36])
{
  static const double a[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -0.00390625, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, -0.00390625, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -0.00390625, 0.0, 0.0, 1.0 };

  static const double dv[36] = { 1.0, 0.0, 0.0, -0.00390625, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, -0.00390625, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -0.00390625, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0 };

  static const double b_a[18] = { 0.00390625, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.00390625, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00390625, 0.0, 0.0, 0.0 };

  static const signed char d_I[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  static const signed char b_b[18] = { 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0 };

  static const signed char iv[18] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0 };

  double P_pri[36];
  double c_I[36];
  double K[18];
  double b[18];
  double b_I[9];
  double c_a[9];
  double x[9];
  double x_pri[6];
  double b_acc_data[3];
  double absx11;
  double absx21;
  double absx31;
  int itmp;
  int p1;
  int p2;
  int p3;

  /* x(k) = A * x(k-1) + B * u(k-1) + W(k-1) - model procesu */
  /* x = [x y z g_bias_x g_bias_y g_bias_z]'; %wektor stanu 6x1 */
  /* x = zeros(6,length(time)); */
  /* macierz stanu 6x6 */
  /* macierz wejscia(sterowania) 6x3 */
  /* z(k) = H * x(k) + V(k) - model pomiaru */
  /* macierz wyjscia 3x6 */
  memset(&P_pri[0], 0, 36U * sizeof(double));
  for (p1 = 0; p1 < 6; p1++) {
    P_pri[p1 + 6 * p1] = 1.0;
  }

  /* macierz kowariancji modelu */
  memset(&b_I[0], 0, 9U * sizeof(double));
  b_I[0] = 1.0;
  b_I[4] = 1.0;
  b_I[8] = 1.0;

  /* macierz kowariancji pomiarow */
  /* macierz kowariancji stanu */
  /* Predykcja */
  for (p2 = 0; p2 < 6; p2++) {
    absx11 = 0.0;
    for (p3 = 0; p3 < 6; p3++) {
      absx11 += a[p2 + 6 * p3] * old_x_post_data[p3];
    }

    x_pri[p2] = absx11 + ((b_a[p2] * gyr_data[0] + b_a[p2 + 6] * gyr_data[1]) +
                          b_a[p2 + 12] * gyr_data[2]);
  }

  for (p2 = 0; p2 < 36; p2++) {
    P_pri[p2] = a[p2] * old_p_post_data[p2] * dv[p2] + P_pri[p2] * 1.0E-5;
  }

  /* Korekcja */
  for (p2 = 0; p2 < 3; p2++) {
    for (p3 = 0; p3 < 6; p3++) {
      absx11 = 0.0;
      for (p1 = 0; p1 < 6; p1++) {
        absx11 += (double)b_b[p2 + 3 * p1] * P_pri[p1 + 6 * p3];
      }

      b[p2 + 3 * p3] = absx11;
    }

    for (p3 = 0; p3 < 3; p3++) {
      absx11 = 0.0;
      for (p1 = 0; p1 < 6; p1++) {
        absx11 += b[p2 + 3 * p1] * (double)iv[p1 + 6 * p3];
      }

      p1 = p2 + 3 * p3;
      c_a[p1] = absx11 + b_I[p1] * 8500.0;
    }
  }

  memcpy(&x[0], &c_a[0], 9U * sizeof(double));
  p1 = 0;
  p2 = 3;
  p3 = 6;
  absx11 = fabs(c_a[0]);
  absx21 = fabs(c_a[1]);
  absx31 = fabs(c_a[2]);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    p1 = 3;
    p2 = 0;
    x[0] = c_a[1];
    x[1] = c_a[0];
    x[3] = c_a[4];
    x[4] = c_a[3];
    x[6] = c_a[7];
    x[7] = c_a[6];
  } else {
    if (absx31 > absx11) {
      p1 = 6;
      p3 = 0;
      x[0] = c_a[2];
      x[2] = c_a[0];
      x[3] = c_a[5];
      x[5] = c_a[3];
      x[6] = c_a[8];
      x[8] = c_a[6];
    }
  }

  x[1] /= x[0];
  x[2] /= x[0];
  x[4] -= x[1] * x[3];
  x[5] -= x[2] * x[3];
  x[7] -= x[1] * x[6];
  x[8] -= x[2] * x[6];
  if (fabs(x[5]) > fabs(x[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    absx11 = x[1];
    x[1] = x[2];
    x[2] = absx11;
    absx11 = x[4];
    x[4] = x[5];
    x[5] = absx11;
    absx11 = x[7];
    x[7] = x[8];
    x[8] = absx11;
  }

  x[5] /= x[4];
  x[8] -= x[5] * x[7];
  absx11 = (x[5] * x[1] - x[2]) / x[8];
  absx21 = -(x[1] + x[7] * absx11) / x[4];
  b_I[p1] = ((1.0 - x[3] * absx21) - x[6] * absx11) / x[0];
  b_I[p1 + 1] = absx21;
  b_I[p1 + 2] = absx11;
  absx11 = -x[5] / x[8];
  absx21 = (1.0 - x[7] * absx11) / x[4];
  b_I[p2] = -(x[3] * absx21 + x[6] * absx11) / x[0];
  b_I[p2 + 1] = absx21;
  b_I[p2 + 2] = absx11;
  absx11 = 1.0 / x[8];
  absx21 = -x[7] * absx11 / x[4];
  b_I[p3] = -(x[3] * absx21 + x[6] * absx11) / x[0];
  b_I[p3 + 1] = absx21;
  b_I[p3 + 2] = absx11;
  for (p2 = 0; p2 < 6; p2++) {
    for (p3 = 0; p3 < 3; p3++) {
      absx11 = 0.0;
      for (p1 = 0; p1 < 6; p1++) {
        absx11 += P_pri[p2 + 6 * p1] * (double)iv[p1 + 6 * p3];
      }

      b[p2 + 6 * p3] = absx11;
    }

    absx11 = b[p2];
    absx21 = b[p2 + 6];
    absx31 = b[p2 + 12];
    for (p3 = 0; p3 < 3; p3++) {
      K[p2 + 6 * p3] = (absx11 * b_I[3 * p3] + absx21 * b_I[3 * p3 + 1]) +
        absx31 * b_I[3 * p3 + 2];
    }
  }

  for (p2 = 0; p2 < 3; p2++) {
    absx11 = 0.0;
    for (p3 = 0; p3 < 6; p3++) {
      absx11 += (double)b_b[p2 + 3 * p3] * x_pri[p3];
    }

    b_acc_data[p2] = acc_data[p2] - absx11;
  }

  for (p2 = 0; p2 < 6; p2++) {
    absx11 = K[p2 + 6];
    absx21 = K[p2 + 12];
    x_post_data[p2] = x_pri[p2] + ((K[p2] * b_acc_data[0] + absx11 * b_acc_data
      [1]) + absx21 * b_acc_data[2]);
    for (p3 = 0; p3 < 6; p3++) {
      p1 = p2 + 6 * p3;
      c_I[p1] = (double)d_I[p1] - ((K[p2] * (double)b_b[3 * p3] + absx11 *
        (double)b_b[3 * p3 + 1]) + absx21 * (double)b_b[3 * p3 + 2]);
    }

    for (p3 = 0; p3 < 6; p3++) {
      absx11 = 0.0;
      for (p1 = 0; p1 < 6; p1++) {
        absx11 += c_I[p2 + 6 * p1] * P_pri[p1 + 6 * p3];
      }

      p_post_data[p2 + 6 * p3] = absx11;
    }
  }
}

/*
 * File trailer for Kalman_filer_function.c
 *
 * [EOF]
 */
