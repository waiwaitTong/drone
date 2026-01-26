#ifndef __RM_ALGORITHM_H__
#define __RM_ALGORITHM_H__

#include "main.h"
#include "globaluse_basic_function.h"
#include "stdlib.h"

//#define abs(x) ((x)>0? (x):(-(x)))

void CalPID(ST_PID *pstPid);
void CalISeparatedPID(ST_PID *pstPid);
void CalIResistedPID(ST_PID *pstPid);
void CalIWeakenPID(ST_PID *pstPid);
void LMS_Filter(FP32* Input, FP32* Output, FP32* RefOutput, UCHAR8 order, FP32 Cov);
//double Chebyshev50HzLPF(Filter_t *F);
void TD_Function(TD *ptd);
float abs_fl(float value);
float SMC_SatFunc(float in, float d);
void CalSMC(ST_SMC *pStSMC);
void Kalman_Filter(kalman_filter *pkf);
void LpFilter(ST_LPF* lpf);
//void LESO(ST_LADRC *pStLADRC);
//void CalLADRC(ST_LADRC *pStLADRC);
#endif 
