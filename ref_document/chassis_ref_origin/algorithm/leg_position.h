/*
 * File: leg_position.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2025-09-05 09:42:16
 */

#ifndef LEG_POSITION_H
#define LEG_POSITION_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void leg_position(double phi1, double phi4, double *L,double *phi0);
extern void LegToJoint(float L_0,float phi_0,float *phi_1,float *phi_4);
extern void JointToLeg(float phi1, float phi4, float *L, float *phi0);
#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for leg_position.h
 *
 * [EOF]
 */
