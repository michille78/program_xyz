/*
 * test.h
 *
 * Code generation for function 'test'
 *
 */

#ifndef __TEST_H__
#define __TEST_H__

/* Include files */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include "blas.h"
#include "rtwtypes.h"
#include "test_types.h"

/* Function Declarations */
extern real_T test(real_T b);

#ifdef __WATCOMC__

#pragma aux test value [8087];

#endif
#endif

/* End of code generation (test.h) */
