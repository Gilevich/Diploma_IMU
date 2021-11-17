/*
 * File: _coder_quatern2euler_mex.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 07-Nov-2021 21:11:19
 */

#ifndef _CODER_QUATERN2EULER_MEX_H
#define _CODER_QUATERN2EULER_MEX_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T
    nrhs, const mxArray *prhs[]);
  emlrtCTX mexFunctionCreateRootTLS(void);
  void quatern2euler_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
    const mxArray *prhs[1]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for _coder_quatern2euler_mex.h
 *
 * [EOF]
 */
