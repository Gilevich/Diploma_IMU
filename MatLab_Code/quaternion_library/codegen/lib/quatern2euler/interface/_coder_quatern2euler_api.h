/*
 * File: _coder_quatern2euler_api.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 22-Nov-2021 20:09:44
 */

#ifndef _CODER_QUATERN2EULER_API_H
#define _CODER_QUATERN2EULER_API_H

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
  void quatern2euler(real_T q[4], real_T euler[3]);
  void quatern2euler_api(const mxArray * const prhs[1], const mxArray *plhs[1]);
  void quatern2euler_atexit(void);
  void quatern2euler_initialize(void);
  void quatern2euler_terminate(void);
  void quatern2euler_xil_shutdown(void);
  void quatern2euler_xil_terminate(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for _coder_quatern2euler_api.h
 *
 * [EOF]
 */
