/*
 * File: _coder_Kalman_filer_function_api.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 07-Nov-2021 14:55:24
 */

#ifndef _CODER_KALMAN_FILER_FUNCTION_API_H
#define _CODER_KALMAN_FILER_FUNCTION_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  void Kalman_filer_function(real_T acc_data[3], real_T gyr_data[3], real_T
    old_x_post_data[6], real_T old_p_post_data[36], real_T x_post_data[6],
    real_T p_post_data[36]);
  void Kalman_filer_function_api(const mxArray * const prhs[4], int32_T nlhs,
    const mxArray *plhs[2]);
  void Kalman_filer_function_atexit(void);
  void Kalman_filer_function_initialize(void);
  void Kalman_filer_function_terminate(void);
  void Kalman_filer_function_xil_shutdown(void);
  void Kalman_filer_function_xil_terminate(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for _coder_Kalman_filer_function_api.h
 *
 * [EOF]
 */
