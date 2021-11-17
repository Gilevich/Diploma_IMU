/*
 * File: Kalman_filer_function.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 07-Nov-2021 14:55:24
 */

#ifndef KALMAN_FILER_FUNCTION_H
#define KALMAN_FILER_FUNCTION_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>
#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  extern void Kalman_filer_function(const double acc_data[3], const double
    gyr_data[3], const double old_x_post_data[6], const double old_p_post_data
    [36], double x_post_data[6], double p_post_data[36]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for Kalman_filer_function.h
 *
 * [EOF]
 */
