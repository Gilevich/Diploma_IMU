/*
 * File: main.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 07-Nov-2021 14:55:24
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include Files */
#include "main.h"
#include "Kalman_filer_function.h"
#include "Kalman_filer_function_terminate.h"

/* Function Declarations */
static void argInit_1x3_real_T(double result[3]);
static void argInit_6x1_real_T(double result[6]);
static void argInit_6x6_real_T(double result[36]);
static double argInit_real_T(void);
static void main_Kalman_filer_function(void);

/* Function Definitions */
/*
 * Arguments    : double result[3]
 * Return Type  : void
 */
static void argInit_1x3_real_T(double result[3])
{
  int idx1;

  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 3; idx1++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx1] = argInit_real_T();
  }
}

/*
 * Arguments    : double result[6]
 * Return Type  : void
 */
static void argInit_6x1_real_T(double result[6])
{
  int idx0;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 6; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : double result[36]
 * Return Type  : void
 */
static void argInit_6x6_real_T(double result[36])
{
  int idx0;
  int idx1;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 6; idx0++) {
    for (idx1 = 0; idx1 < 6; idx1++) {
      /* Set the value of the array element.
         Change this value to the value that the application requires. */
      result[idx0 + 6 * idx1] = argInit_real_T();
    }
  }
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_Kalman_filer_function(void)
{
  double dv1[36];
  double p_post_data[36];
  double dv[6];
  double x_post_data[6];
  double acc_data_tmp[3];

  /* Initialize function 'Kalman_filer_function' input arguments. */
  /* Initialize function input argument 'acc_data'. */
  argInit_1x3_real_T(acc_data_tmp);

  /* Initialize function input argument 'gyr_data'. */
  /* Initialize function input argument 'old_x_post_data'. */
  /* Initialize function input argument 'old_p_post_data'. */
  /* Call the entry-point 'Kalman_filer_function'. */
  argInit_6x1_real_T(dv);
  argInit_6x6_real_T(dv1);
  Kalman_filer_function(acc_data_tmp, acc_data_tmp, dv, dv1, x_post_data,
                        p_post_data);
}

/*
 * Arguments    : int argc
 *                const char * const argv[]
 * Return Type  : int
 */
int main(int argc, const char * const argv[])
{
  (void)argc;
  (void)argv;

  /* The initialize function is being called automatically from your entry-point function. So, a call to initialize is not included here. */
  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
  main_Kalman_filer_function();

  /* Terminate the application.
     You do not need to do this more than one time. */
  Kalman_filer_function_terminate();
  return 0;
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
