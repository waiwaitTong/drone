#ifndef _GlobalUse_Basic_Function_H_
#define _GlobalUse_Basic_Function_H_

#include "rm_types_my.h"
#include "global_declare.h"
#include "delay.h"

SINT32 Sign_Judge(FP32 fp_Any_Number);
SINT32 Round(FP32 fp_Any_Number);
FP32 Square(FP32 x);
UCHAR8 KEY_SingleClick_Q(void);
UCHAR8 KEY_SingleClick_E(void);
UCHAR8 KEY_SingleClick_ML(void);
UCHAR8 KEY_SingleClick_MR(void);
UCHAR8 KEY_SingleClick_R(void);
UCHAR8 KEY_SingleClick_W(void);
UCHAR8 KEY_SingleClick_A(void);
UCHAR8 KEY_SingleClick_S(void);
UCHAR8 KEY_SingleClick_D(void);
UCHAR8 KEY_SingleClick_Z(void);
UCHAR8 KEY_SingleClick_X(void);
UCHAR8 KEY_SingleClick_G(void);
UCHAR8 KEY_SingleClick_C(void);
UCHAR8 KEY_SingleClick_V(void);
UCHAR8 KEY_SingleClick_B(void);
UCHAR8 KEY_SingleClick_F(void);  
UCHAR8 KEY_SingleClick_Ctrl(void);
UCHAR8 KEY_SingleClick_Shift(void);
UCHAR8 ClockDivide(USHORT16 order);

extern FP32 Clip(FP32 siValue, FP32 siMin, FP32 siMax);
extern SINT32 Absolute_value(FP32 Number);
extern inline bool Is_Float_Equal(float a, float b);

#endif
