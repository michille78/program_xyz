/*
 * pythagoras.h
 *
 * Code generation for function 'pythagoras'
 *
 */

#ifndef __PYTHAGORAS_H__
#define __PYTHAGORAS_H__

/* Include files */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "mwmathutil.h"
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include "blas.h"
#include "rtwtypes.h"
#include "pythagoras_types.h"

/* Function Declarations */
extern real_T pythagoras(const emlrtStack *sp, real_T a, real_T b, char_T color);

#ifdef __WATCOMC__

#pragma aux pythagoras value [8087];

#endif
#endif

/* End of code generation (pythagoras.h) */
