/*
 * File: quatern2euler.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 07-Nov-2021 21:11:19
 */

/* Include Files */
#include "quatern2euler.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static double rt_atan2d_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2(b_u0, b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/*
 * QUATERN2EULER Converts a quaternion orientation to ZYX Euler angles
 *
 *    q = quatern2euler(q)
 *
 *    Converts a quaternion orientation to ZYX Euler angles where phi is a
 *    rotation around X, theta around Y and psi around Z.
 *
 *    For more information see:
 *    http://www.x-io.co.uk/node/8#quaternions
 *
 *  Date          Author          Notes
 *  27/09/2011    SOH Madgwick    Initial release
 * Arguments    : const double q[4]
 *                double euler[3]
 * Return Type  : void
 */
void quatern2euler(const double q[4], double euler[3])
{
  double R_idx_2;
  double euler_tmp;
  R_idx_2 = 2.0 * (q[1] * q[3] + q[0] * q[2]);
  euler_tmp = 2.0 * (q[0] * q[0]) - 1.0;
  euler[0] = rt_atan2d_snf(2.0 * (q[2] * q[3] - q[0] * q[1]), euler_tmp + 2.0 *
    (q[3] * q[3]));
  euler[1] = -atan(R_idx_2 / sqrt(1.0 - R_idx_2 * R_idx_2));
  euler[2] = rt_atan2d_snf(2.0 * (q[1] * q[2] - q[0] * q[3]), euler_tmp + 2.0 *
    (q[1] * q[1]));
}

/*
 * File trailer for quatern2euler.c
 *
 * [EOF]
 */
