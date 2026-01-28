#include "friction_wheel_task.h"

UCHAR8 FrictionWheel_UP_Ready = 0;
FP32 DesSpeed1 = 5850;	 
FP32 DesSpeed2 = 5850;	 
FP32 des_single = 11200; // 单位：r/min
int SpeedCompensationCnt = 0;
// float ShootHeat_RateProcessed = 0.0f;
ST_SLIDINGWINDOWS bullet_speed_filter = { {0}, {0}, 0 };


int Start_cnt = 0;
int Start_flag = 0;
//bool Pre_State=0;
/*-----------------------------------------------------------
函数名：FrictionWheelControl()
功能： 摩擦轮控制
-----------------------------------------------------------*/

void FrictionWheelControl(void)
{
	if (g_stTestFlag.FrictionTestFlag == FALSE)
	{
		if (RC_ON)
		{
			switch (g_emOperation_Mode)
			{
			case RC_Mode:
				FrictionWheel_Ready_cnt = 0;
				intRampSignal(0, 0, 0);
				FrictionWheel_Ready = FALSE;
				des_single = 0;
				break;
			case KeyMouse_Mode:
				FrictionWheel_Ready_cnt = 0;
				FrictionWheel_KeyMouse_Mode();

				break;
			case Secuirty_Mode:
				intRampSignal(0, 0, 0);

				break;
			case SingleShoot_Mode:
//                Friction_Start();
				FrictionWheel_RC_Mode();
				break;
			case SpeedShoot_Mode:
				FrictionWheel_RC_Mode();
				break;
			case VisionSpeed_Mode:
				FrictionWheel_RC_Mode();

				break;
			case VisionSingle_Mode:
				FrictionWheel_RC_Mode();

				break;
			case ImageControl_Mode:
				FrictionWheel_Ready_cnt = 0;
				FrictionWheel_KeyMouse_Mode();

				break;
			case VisionControl_Mode:
				FrictionWheel_Ready_cnt = 0;
				intRampSignal(0, 0, 0);
				FrictionWheel_Ready = FALSE;
			default:

				PWM1 = 500;
				PWM2 = 500;
				break;
			}
		}
		else
		{
			FrictionWheel_Ready_cnt = 0;
			FrictionWheel_Ready = FALSE;
			PWM1 = Flameout;
			PWM2 = Flameout;
		}
	}
	else if (g_stTestFlag.FrictionTestFlag == TRUE) // 更改标志位和des_single的值
	{
		if (RC_ON)
		{
			intRampSignal(DesSpeed1, DesSpeed2, Acc_Step);
			//			    Friction_Direction_Calibration();//遥控器控制摩擦轮
		}
	}
}

/*------------------------------------------------------------
函 数 名：FrictionWheel_RC_Mode()
函数功能：摩擦轮遥控器控制
------------------------------------------------------------*/
void FrictionWheel_RC_Mode(void)
{

		FrictionWheel_Ready_cnt++;
		if (FrictionWheel_Ready_cnt > 1000 && g_stFriction2SMC.fpFB >DesSpeed1-500 && g_stFriction1SMC.fpFB < -(DesSpeed1-500)) // 改为反馈值大于阈值
		{
			FrictionWheel_Ready = TRUE;
		}
		else
		{
			FrictionWheel_Ready = FALSE;
		}
       
        
		intRampSignal(DesSpeed1, DesSpeed2, 1);

}
/*------------------------------------------------------------
函 数 名：FrictionWheel_KeyMouse_Mode()
函数功能：摩擦轮键鼠控制
------------------------------------------------------------*/
void FrictionWheel_KeyMouse_Mode(void)
{
        if (KEY_SingleClick_E())
        {
            if (!PRESSED_CTRL)
                FrictionWheel_Ready = TRUE;
            if (PRESSED_CTRL)
            {
                FrictionWheel_Ready = FALSE;
            }
        }
        else if(remote_control_image_transmission.left_button_down)/////////按下左键就打开摩擦轮
            FrictionWheel_Ready = TRUE;

        
        if (FrictionWheel_Ready == TRUE)
        {
            intRampSignal(DesSpeed1, DesSpeed2, 1);
        }
        else if (FrictionWheel_Ready == FALSE)
        {
            intRampSignal(0, 0, 0);
        }
        if(PRESSED_Z)/////////调整摩擦轮转速ctrl+z转速增加
        {
            if(PRESSED_CTRL){
            DesSpeed1+=10*0.001;
            DesSpeed2+=10*0.001;
            }
            else{/////////按z转速减小
            DesSpeed1-=10*0.001;
            DesSpeed2-=10*0.001;
            }
            
        }
            

}

/*------------------------------------------------------------
函 数 名：void Friction_2006_control(u32 des1,u32 des2)
函数功能：2006电机控制摩擦轮
备    注：
------------------------------------------------------------*/
u32 pwm_1, pwm_2;
u32 fric1, fric2;
int fric_cnt = 0;
int fric_flag = 0;
void intRampSignal(FP32 DesValue1, FP32 DesValue2, UINT32 Step)
{

	g_stFriction1SMC.fpDes = DesValue1;
	g_stFriction2SMC.fpDes = -DesValue2;

	CalSMC(&g_stFriction1SMC);
	CalSMC(&g_stFriction2SMC);

	if (g_stFriction1SMC.fpE < 100 && g_stFriction2SMC.fpE < 100 && !Step)
	{
		Send_Current_To_Friction(CAN2, 0, 0);
	}

	else
		Send_Current_To_Friction(CAN2, g_stFriction1SMC.fpU, g_stFriction2SMC.fpU);

	FrictionWheel_UP_Ready = 0;

}

void Friction_Start(void)
{
    if(Start_flag==0){
	if (Start_cnt <= 3000)
	{
		g_stFriction1SMC.fpDes = DesSpeed1;
		g_stFriction2SMC.fpDes = DesSpeed2;
		CalSMC(&g_stFriction1SMC);
		CalSMC(&g_stFriction2SMC);
        
		Send_Current_To_Friction(CAN2, g_stFriction1SMC.fpU, g_stFriction2SMC.fpU);
        
	}
	if (Start_cnt > 3000 && Start_cnt <= 6000)
	{
		g_stFriction1SMC.fpDes = 0;
		g_stFriction2SMC.fpDes = DesSpeed2;
		CalSMC(&g_stFriction1SMC);
		CalSMC(&g_stFriction2SMC);

		Send_Current_To_Friction(CAN2, 0, g_stFriction2SMC.fpU);
	}
}
	if (Start_cnt > 6000)
	{
		Start_flag = 1;
		Start_cnt = 0;
	}
    
    

//	if (Start_flag == 0)
//	{
//		CalSMC(&g_stFriction1SMC);
//		CalSMC(&g_stFriction2SMC);

//		Send_Current_To_Friction(CAN2, g_stFriction1SMC.fpU, g_stFriction2SMC.fpU);
//	}
	Start_cnt++;
}

///*------------------------------------------------------------
// 函 数 名：void Friction_Direction_Calibration(void)
// 函数功能：摩擦轮遥控器控制
//------------------------------------------------------------*/
// u32 PWM_1,PWM_2;
// void Friction_Direction_Calibration(void)
//{
////电机正反向校准
//		//校准步骤，把不需要校准的改成500
//		//！！！！然后然后一定要注意先下载程序，然后再上电，不然有可能疯转
//		//选择需要校准转向的电机在B-BB-B-BB之后先1000，后500，然后580，
//		//电机出现正常的开机的声音，既可以校准转向
//////
//  if( DR16_rec.stRC.SW_L == 3 )
//		{
//		    PWM1=500;
//			PWM2=500;
//		}
//		if(DR16_rec.stRC.SW_L == 2)
//		{
//			PWM2=1000;
//			PWM1=500;
//		    FrictionWheel_Ready =FALSE;
//		}

//		if(DR16_rec.stRC.SW_L == 1)
//		{
//			PWM2=580;
//			PWM1=500;
//		    FrictionWheel_Ready =FALSE;
//		}
//		PWM_1=PWM1;
//		PWM_2=PWM2;
//}
float SlidingWindowFilter(ST_SLIDINGWINDOWS *swf, float newdata)
{
    float sum = 0.0f;
    float temp = 0.0f;
    
    if(newdata!=swf->pre_datanum)
    {
        // FIFO数据更新
        for (int i = MAX_DATA_NUM - 1; i > 0; i--) {
            swf->DataList[i] = swf->DataList[i-1];
        }
        swf->DataList[0] = newdata;

        // 更新数据计数器
        swf->datanum = (swf->datanum >= MAX_DATA_NUM) ? MAX_DATA_NUM : (swf->datanum + 1);
        swf->pre_datanum=newdata;
    }
    // 创建数据副本
    memcpy(swf->DataList_Copy, swf->DataList, sizeof(swf->DataList));

    if (swf->datanum < MAX_DATA_NUM) {
        // 计算简单平均值
        for (int i = 0; i < MAX_DATA_NUM; i++) {
            sum += swf->DataList_Copy[i];
        }
        return sum / swf->datanum;
    }
    else {
        // 优化冒泡排序（升序）
        for (int j = 0; j < MAX_DATA_NUM - 1; j++) {
            for (int k = 0; k < MAX_DATA_NUM - 1 - j; k++) {
                if (swf->DataList_Copy[k] > swf->DataList_Copy[k+1]) {
                    temp = swf->DataList_Copy[k];
                    swf->DataList_Copy[k] = swf->DataList_Copy[k+1];
                    swf->DataList_Copy[k+1] = temp;
                }
            }
        }

        // 计算中间有效数据平均值
        const int offset = (MAX_DATA_NUM - WINDOW_DATA_NUM) / 2;
        for (int i = offset; i < offset + WINDOW_DATA_NUM; i++) {
            sum += swf->DataList_Copy[i];
        }
        return sum / WINDOW_DATA_NUM;
    }
}
