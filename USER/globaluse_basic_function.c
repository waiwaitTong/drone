#include "GlobalUse_Basic_Function.h"

/*******************************************************************
函数名称：SINT32 fabs(FP32 Number)
函数功能：绝对值函数
********************************************************************/
SINT32 Absolute_value(FP32 Number)
{
   if(Number>=0) return Number;
	 else          return -Number;
}
///*******************************************************************
//函数名称：Sign_Judge(FP32 fp_Any_Number)
//函数功能：判断正负
//备    注：返回值为1和-1，来改变数的符号
//********************************************************************/
SINT32 Sign_Judge(FP32 fp_Judge_Number)
{
	if(fp_Judge_Number >= 0)
	{
		return 1;
	}
	else 
	{
		return -1;
	}
}
///*-------------------------------------------------------------------
//函数功能：判断浮点数是否相等
//-------------------------------------------------------------------*/
inline bool Is_Float_Equal(float a, float b)
{
    return fabs(a-b)<1e-4 ? TRUE:FALSE;
}
///*******************************************************************
//函数名称：Round()
//函数功能：将浮点数四舍五入，返回32位整型数
//********************************************************************/
//SINT32 Round(FP32 fp_Round_Number)
//{   
//    if (fp_Round_Number >= 0)
//    {
//		return (SINT32)(fp_Round_Number + 0.5f);
//    }
//    else 
//    {
//		return (SINT32)(fp_Round_Number - 0.5f);
//    }
//}



///*--------------------------------------------------------------------------------------------------
//函数名称：Clip()
//函数功能：削波函数，去除超出最大值与最小值之间的值，代之以最大或最小值
//--------------------------------------------------------------------------------------------------*/
FP32 Clip(FP32 fpValue, FP32 fpMin, FP32 fpMax)
{
	if(fpValue <= fpMin)
	{
		return fpMin;
	}
	else if(fpValue >= fpMax)
	{
		return fpMax;
	}
	else 
	{
		return fpValue;
	}
}
///*--------------------------------------------------------------------------------------------------
//函数名称：Square(FP32 x)
//函数功能：计算平方
//--------------------------------------------------------------------------------------------------*/
//FP32 Square(FP32 x)
//{
//    return x*x;	
//}


///*--------------------------------------------------------------------------------------------------
//函数名称：KeySingleClick(UCHAR8 Key)
//函数功能：按键单次按下判断
//备注：1-按下一次，0-没有按下或者始终连按
//--------------------------------------------------------------------------------------------------*/
UCHAR8 KEY_SingleClick_Q(void)
{	 
	static u8 key_up=1;//按键按松开标志	  
	if(key_up && (PRESSED_Q))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(PRESSED_Q) return 1;
		else return 0;  //连按
	}
	else if(!PRESSED_Q) 
	{
		key_up=1; 	     
	    return 0;// 无按键按下
	}
	return 0;
}

UCHAR8 KEY_SingleClick_E(void)
{	 
	static u8 key_up=1;//按键按松开标志	  
	if(key_up && (PRESSED_E))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(PRESSED_E) return 1;
		else return 0;
	}
	else if(!PRESSED_E)
	{
		key_up=1; 	     
	    return 0;// 无按键按下
	}
	return 0;
}

UCHAR8 KEY_SingleClick_ML(void)
{	 
	static u8 key_up=1;//按键按松开标志	  
	if(key_up && (DR16_rec.stMouse.Left))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(DR16_rec.stMouse.Left) return 1;
		else return 0;
	}
	else if(!DR16_rec.stMouse.Left)
	{
		key_up=1; 	     
		return 0;// 无按键按下
	}
	return 0;
}

UCHAR8 KEY_SingleClick_MR(void)
{	 
	static u8 key_up=1;//按键按松开标志	  
	if(key_up && (DR16_rec.stMouse.Right))
	{
		//delay_ms(10);//去抖动 
		key_up=0;
		if(DR16_rec.stMouse.Right) return 1;
		else return 0;
	}
	else if(!DR16_rec.stMouse.Right) key_up=1; 	     
	return 0;// 无按键按下
}

UCHAR8 KEY_SingleClick_R(void)
{
	static u8 key_up=1;//按键按松开标志	  
	if(key_up && (PRESSED_R))
	{
		//delay_ms(10);//去抖动 
		key_up=0;
		if(PRESSED_R) return 1;
		else return 0;
	}
	else if(!PRESSED_R)
	{
		key_up=1; 	     
	return 0;// 无按键按下
	}
	return 0;
}

UCHAR8 KEY_SingleClick_W(void)
{
	static u8 key_up=1;//按键按松开标志	  
	if(key_up && (PRESSED_W))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(PRESSED_W) return 1;
		else return 0;
	}
	else if(!PRESSED_W) 
	{
		key_up=1; 	     
	return 0;// 无按键按下
	}
	return 0;
}

UCHAR8 KEY_SingleClick_A(void)
{
	static u8 key_up=1;//按键按松开标志	  
	if(key_up && (PRESSED_A))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(PRESSED_A) return 1;
		else return 0;
	}
	else if(!PRESSED_A) 
	{
		key_up=1; 	     
	return 0;// 无按键按下
	}
	return 0;
}

UCHAR8 KEY_SingleClick_S(void)
{
	static u8 key_up=1;//按键按松开标志	  
	if(key_up && (PRESSED_S))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(PRESSED_S) return 1;
		else return 0;
	}
	else if(!PRESSED_S)
	{
		key_up=1; 	     
	return 0;// 无按键按下
	}
	return 0;
}

UCHAR8 KEY_SingleClick_D(void)
{
	static u8 key_up=1;//按键按松开标志	  
	if(key_up && (PRESSED_D))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(PRESSED_D) return 1;
		else return 0;
	}
	else if(!PRESSED_D) 
	{
		key_up=1; 	     
	return 0;// 无按键按下
	}
	return 0;
}

UCHAR8 KEY_SingleClick_Ctrl(void)
{
	static u8 key_up=1;//按键按松开标志	  
	if(key_up && (PRESSED_CTRL))
	{
	//	delay_ms(10);//去抖动 
		key_up=0;
		if(PRESSED_CTRL) return 1;
		else return 0;
	}
	else if(!PRESSED_CTRL)
		{ 
			key_up=1; 	     
	return 0;// 无按键按下
		}
		return 0;
}
UCHAR8 KEY_SingleClick_Shift(void)
{
	static u8 key_up=1;//按键按松开标志	  
	if(key_up && (PRESSED_SHIFT))
	{
	//	delay_ms(10);//去抖动 
		key_up=0;
		if(PRESSED_SHIFT) return 1;
		else return 0;
	}
	else if(!PRESSED_SHIFT) 
	{
		key_up=1; 	     
	return 0;// 无按键按下
	}
	return 0;
}


UCHAR8 KEY_SingleClick_Z(void)
{
	static u8 key_up=1;//按键按松开标志	  
	if(key_up && (PRESSED_Z))
	{
//		delay_ms(10);//去抖动 
		key_up=0;
		if(PRESSED_Z) return 1;
		else return 0;
	}
	else if(!PRESSED_Z) 
	{
		key_up=1; 	     
		return 0;// 无按键按下
	}
	return 0;
}

UCHAR8 KEY_SingleClick_X(void)
{
	static u8 key_up=1;//按键按松开标志	  
	if(key_up && (PRESSED_X))
	{
	//	delay_ms(10);//去抖动 
		key_up=0;
		if(PRESSED_X) 
            return 1;
		else return 0;
	}
	else if(!PRESSED_X) 
	{
		key_up=1; 	     
	return 0;// 无按键按下
	}
	return 0;
}

UCHAR8 KEY_SingleClick_G(void)
{
	static u8 key_up=1;//按键按松开标志	  
	if(key_up && (PRESSED_G))
	{
	//	delay_ms(10);//去抖动 
		key_up=0;
		if(PRESSED_G) 
            return 1;
		else return 0;
	}
	else if(!PRESSED_G) 
	{
		key_up=1; 	     
	return 0;// 无按键按下
	}
	return 0;
}
UCHAR8 KEY_SingleClick_C(void)
{
	static u8 key_up=1;//按键按松开标志	  
	if(key_up && (PRESSED_C))
	{
	//	delay_ms(10);//去抖动 
		key_up=0;
		if(PRESSED_C) return 1;
		else return 0;
	}
	else if(!PRESSED_C) 
	{
		key_up=1; 	     
	return 0;// 无按键按下
	}
	return 0;
}
UCHAR8 KEY_SingleClick_V(void)
{	 
	static u8 key_up=1;//按键按松开标志	  
	if(key_up && (PRESSED_V))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(PRESSED_V) return 1;
		else return 0;
	}
	else if(!PRESSED_V) 
	{
		key_up=1; 	     
	  return 0;// 无按键按下
	}
	return 0;
}


UCHAR8 KEY_SingleClick_B(void)
{	 
	static u8 key_up=1;//按键按松开标志	  
	if(key_up && (PRESSED_B))
	{
		//delay_ms(10);//去抖动 
		key_up=0;
		if(PRESSED_B) return 1;
		else return 0;
	}
	else if(!PRESSED_B)
	{
		key_up=1; 	     
	return 0;// 无按键按下
	}
	return 0;
}
UCHAR8 KEY_SingleClick_F(void)
{	 
	static u8 key_up=1;//按键按松开标志	  
	if(key_up && (PRESSED_F))
	{
		//delay_ms(10);//去抖动 
		key_up=0;
		if(PRESSED_F) return 1;
		else return 0;
	}
	else if(!PRESSED_F) 
	{
		key_up=1; 	     
	return 0;// 无按键按下
	}
	return 0;
}
///*--------------------------------------------------------------------------------------------------
//函数名称：ClockDivide()
//函数功能：时钟分频
//备注：1-触发，0-无触发
//--------------------------------------------------------------------------------------------------*/
//UCHAR8 ClockDivide(USHORT16 order)
//{
//	static UCHAR8 Cnt = 0;
//	Cnt++;
//	if(Cnt == order) 
//	{
//		Cnt = 0;
//		return 1;
//	}
//	else return 0;
//}
