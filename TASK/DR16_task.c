#include "DR16_task.h"

/*************************************************************************
函 数 名：RC_Protocol
函数功能：遥控器接收机数据处理
备    注：
*************************************************************************/
extern UCHAR8 UA1RxMailbox[USART1_RXMB_LEN];
extern void Aim_Mode_Choose(void);
extern void Aim_Mode_Switch(void);
extern void Operation_Mode_Choose(void);
extern void Sentry_Cmd(void);
extern void Balanced_Infantry_Number_Choose(void);
void RC_Protocol(void)
{

	/**************************************************遥控器数据解码**************************************************/
	/*右摇杆*/
	DR16_rec.stRC.Ch0 = (UA1RxMailbox[0] | (UA1RxMailbox[1] << 8)) & 0x07ff;		// Channe0——水平通道
	DR16_rec.stRC.Ch1 = ((UA1RxMailbox[1] >> 3) | (UA1RxMailbox[2] << 5)) & 0x07ff; // Channe1——垂直通道
	/*左摇杆*/
	DR16_rec.stRC.Ch2 = ((UA1RxMailbox[2] >> 6) | (UA1RxMailbox[3] << 2) | (UA1RxMailbox[4] << 10)) & 0x07ff; // Channe2——水平通道
	DR16_rec.stRC.Ch3 = ((UA1RxMailbox[4] >> 1) | (UA1RxMailbox[5] << 7)) & 0x07ff;							  // Channe3——垂直通道
	/*拨盘*/
	DR16_rec.stRC.Dial = (UA1RxMailbox[16] | UA1RxMailbox[17] << 8) & 0x07ff; // Channe4——左上拨盘 ，向下拨最大值，向上拨最小值
	/*左3位开关*/
	DR16_rec.stRC.SW_L = ((UA1RxMailbox[5] >> 4) & 0x000C) >> 2; //(上——1；中——3；下——2)
	/*右3位开关*/
	DR16_rec.stRC.SW_R = ((UA1RxMailbox[5] >> 4) & 0x0003); //(上——1；中——3；下——2)

	/**************************************************鼠标数据解码***************************************************/
	DR16_rec.stMouse.X = UA1RxMailbox[6] | (UA1RxMailbox[7] << 8);	 // X坐标
	DR16_rec.stMouse.Y = UA1RxMailbox[8] | (UA1RxMailbox[9] << 8);	 // Y坐标
	DR16_rec.stMouse.Z = UA1RxMailbox[10] | (UA1RxMailbox[11] << 8); // Z坐标
	MOUSE_XYZ_Deal();
	DR16_rec.stMouse.Left = UA1RxMailbox[12];  // 左键状态（1：按下；0：没按下）
	DR16_rec.stMouse.Right = UA1RxMailbox[13]; // 右键状态（1：按下；0：没按下）

	/**************************************************键盘数据解码***************************************************/
	DR16_rec.usKeyboard = UA1RxMailbox[14] | (UA1RxMailbox[15] << 8); // 键盘值

	/*************************************遥控协议解析******************************************/
	/*鼠标*/
    if(g_emOperation_Mode!=ImageControl_Mode)
    {  
	if (DR16_rec.stMouse.Left & 0x01)
		PRESSED_ML = TRUE;
	else
		PRESSED_ML = FALSE; // left为1则TRUL
	if (DR16_rec.stMouse.Right & 0x01)
		PRESSED_MR = TRUE;
	else
		PRESSED_MR = FALSE;
	/*键盘按键控制底盘及其他任务*/
	if (DR16_rec.usKeyboard & KEY_PRESSED_OFFSET_W)
		PRESSED_W = TRUE;
	else
		PRESSED_W = FALSE; // 按下及改变TRUE
	if (DR16_rec.usKeyboard & KEY_PRESSED_OFFSET_S)
		PRESSED_S = TRUE;
	else
		PRESSED_S = FALSE;
	if (DR16_rec.usKeyboard & KEY_PRESSED_OFFSET_A)
		PRESSED_A = TRUE;
	else
		PRESSED_A = FALSE;
	if (DR16_rec.usKeyboard & KEY_PRESSED_OFFSET_D)
		PRESSED_D = TRUE;
	else
		PRESSED_D = FALSE;
	if (DR16_rec.usKeyboard & KEY_PRESSED_OFFSET_Q)
		PRESSED_Q = TRUE;
	else
		PRESSED_Q = FALSE;
	if (DR16_rec.usKeyboard & KEY_PRESSED_OFFSET_E)
		PRESSED_E = TRUE;
	else
		PRESSED_E = FALSE;
	if (DR16_rec.usKeyboard & KEY_PRESSED_OFFSET_R)
		PRESSED_R = TRUE;
	else
		PRESSED_R = FALSE;
	if (DR16_rec.usKeyboard & KEY_PRESSED_OFFSET_F)
		PRESSED_F = TRUE;
	else
		PRESSED_F = FALSE;
	if (DR16_rec.usKeyboard & KEY_PRESSED_OFFSET_G)
		PRESSED_G = TRUE;
	else
		PRESSED_G = FALSE;
	if (DR16_rec.usKeyboard & KEY_PRESSED_OFFSET_Z)
		PRESSED_Z = TRUE;
	else
		PRESSED_Z = FALSE;
	if (DR16_rec.usKeyboard & KEY_PRESSED_OFFSET_X)
		PRESSED_X = TRUE;
	else
		PRESSED_X = FALSE;
	if (DR16_rec.usKeyboard & KEY_PRESSED_OFFSET_C)
		PRESSED_C = TRUE;
	else
		PRESSED_C = FALSE;
	if (DR16_rec.usKeyboard & KEY_PRESSED_OFFSET_V)
		PRESSED_V = TRUE;
	else
		PRESSED_V = FALSE;
	if (DR16_rec.usKeyboard & KEY_PRESSED_OFFSET_B)
		PRESSED_B = TRUE;
	else
		PRESSED_B = FALSE;
	if (DR16_rec.usKeyboard & KEY_PRESSED_OFFSET_SHIFT)
		PRESSED_SHIFT = TRUE;
	else
		PRESSED_SHIFT = FALSE;
	if (DR16_rec.usKeyboard & KEY_PRESSED_OFFSET_CTRL)
		PRESSED_CTRL = TRUE;
	else
		PRESSED_CTRL = FALSE;
    }
	/*鼠标控制云台*/
	DR16_rec.stMouse.X = UA1RxMailbox[6] | (UA1RxMailbox[7] << 8);	 // X坐标
	DR16_rec.stMouse.Y = UA1RxMailbox[8] | (UA1RxMailbox[9] << 8);	 // Y坐标
	DR16_rec.stMouse.Z = UA1RxMailbox[10] | (UA1RxMailbox[11] << 8); // Z坐标
	DR16_rec.stMouse.Left = UA1RxMailbox[12];						 // 左键状态（1：按下；0：没按下）
	DR16_rec.stMouse.Right = UA1RxMailbox[13];						 // 右键状态（1：按下；0：没按下）


	Operation_Mode_Choose(); // 操作方式选择

	Balanced_Infantry_Number_Choose();

	Aim_Mode_Choose(); // 是否打前哨站or基地or切换相机等

	Aim_Mode_Switch(); // 是否开辅瞄
}

/*************************************************************************
函 数 名：MOUSE_XYZ_Deal
函数功能：鼠标坐标处理
备    注：
*************************************************************************/
float M_buflen = 10.0f;
void MOUSE_XYZ_Deal(void)
{
	Mouse_X.P %= 10;
	Mouse_Y.P %= 10;
	Mouse_X.AVE -= Mouse_X.buff[Mouse_X.P];
	Mouse_Y.AVE -= Mouse_Y.buff[Mouse_Y.P];
	Mouse_X.buff[Mouse_X.P++] = (float)DR16_rec.stMouse.X / M_buflen;
	Mouse_Y.buff[Mouse_Y.P++] = (float)DR16_rec.stMouse.Y / M_buflen;
	Mouse_X.AVE += Mouse_X.buff[Mouse_X.P - 1];
	Mouse_Y.AVE += Mouse_Y.buff[Mouse_Y.P - 1];
}

/*************************************************************************
函 数 名：Operation_Mode_Choose
函数功能：模式选择
备    注：
			l  拨杆 r
			1       1
			3       3
			2       2
*************************************************************************/
u8 Judge = 0;
void Operation_Mode_Choose(void)
{
	if (DR16_rec.stRC.SW_R == 2 && DR16_rec.stRC.SW_L == 2)
		Judge = 0;
	else if (DR16_rec.stRC.SW_R == 3 && DR16_rec.stRC.SW_L == 2)
		Judge = 1;
	else if (DR16_rec.stRC.SW_R == 1 && DR16_rec.stRC.SW_L == 2)
		Judge = 2;
	else if (DR16_rec.stRC.SW_R == 3 && DR16_rec.stRC.SW_L == 3)
		Judge = 3;
	else if (DR16_rec.stRC.SW_R == 3 && DR16_rec.stRC.SW_L == 1)
		Judge = 4;
	else if (DR16_rec.stRC.SW_R == 2 && DR16_rec.stRC.SW_L == 3)
		Judge = 5;
	else if (DR16_rec.stRC.SW_R == 1 && DR16_rec.stRC.SW_L == 1)
		Judge = 6;
	else if (DR16_rec.stRC.SW_R == 2 && DR16_rec.stRC.SW_L == 1)
		Judge = 7;
	else if (DR16_rec.stRC.SW_R == 1 && DR16_rec.stRC.SW_L == 3)
		Judge = 8;
	switch (Judge)
	{
	case 0:
		g_emOperation_Mode = Secuirty_Mode;
		break;
	case 1:
		g_emOperation_Mode = RC_Mode;
		break;
	case 2:
		g_emOperation_Mode = VisionControl_Mode;
		break;
	case 3:
		g_emOperation_Mode = SingleShoot_Mode;
		break;
	case 4:
		g_emOperation_Mode = SpeedShoot_Mode;
		break;
	case 5:
		g_emOperation_Mode = KeyMouse_Mode;
		break;
	case 6:
		g_emOperation_Mode = VisionSpeed_Mode;
		break;
	case 7:
		g_emOperation_Mode = ImageControl_Mode;
		break;
	case 8:
		g_emOperation_Mode = VisionSingle_Mode;
		break;
	default:
		break;
	}

}

/*--------------------------------------------------------------------------
函数名称:Aim_Mode_Switch()
功能：右键视觉瞄准
--------------------------------------------------------------------------*/
void Aim_Mode_Switch(void)
{
    
	if (g_emOperation_Mode == KeyMouse_Mode)
	{
		if (DR16_rec.stMouse.Right == 1)
		{
			//		if(Gimbal_Control == NO_Aim_Mode)
			//		{
			Gimbal_Control = Aim_Mode;
		}
		//  else if(Gimbal_Control == Aim_Mode)
		else
		{
			Gimbal_Control = NO_Aim_Mode;
			YawBaseCnt = 0;
			PitchBaseCnt = 0;
		}
	}
    
    	if (g_emOperation_Mode == ImageControl_Mode)
	{
		if (remote_control_image_transmission.right_button_down == 1)
		{

			Gimbal_Control = Aim_Mode;
			
		}
		else
		{
			Gimbal_Control = NO_Aim_Mode;
			
			YawBaseCnt = 0;
			PitchBaseCnt = 0;
		}
	}
}
/*--------------------------------------------------------------------------
函数名称:Aim_Mode_Choose()
功能：目标选择
--------------------------------------------------------------------------*/
void Aim_Mode_Choose(void)
{
	if (KEY_SingleClick_C()) // 按c切换相机
	{
		if (change_xiangji_flag)
			change_xiangji_flag = 0;
		else
			change_xiangji_flag = 1;
	}

	if (KEY_SingleClick_G()) // 按g切换小陀螺模式
	{
       if (PRESSED_CTRL)
			spinning_flag = 0;
		else
			spinning_flag = 1;
	}

	if (KEY_SingleClick_Z()) // crtl+z切换动静态（动态目标静态目标）
	{
		if (PRESSED_CTRL)
		{
			if (d_or_s_flag == 0) // 哨兵
				d_or_s_flag = 1;
			else
				d_or_s_flag = 0;
		}
		else // 按z锁基地
		{
			if (shooting_or_not_s == 0)
				shooting_or_not_s = 1;
			else
				shooting_or_not_s = 0;
		}
	}
	else if (KEY_SingleClick_X()) // ctrl+x刷新ui
	{
		if (PRESSED_CTRL)
		{
			if (testf == FALSE)
				testf = TRUE;
			else
				testf = FALSE;
		}
		else // 按x锁前哨站
		{
			if (danger_flag == 0)
				danger_flag = 1;
			else
				danger_flag = 0;
		}
	}
}

/*--------------------------------------------------------------------------
函数名称:Aim_Mode_Choose()
功能：平步号码确定
ctrl+R 切换 单击 R 取消

--------------------------------------------------------------------------*/
void Balanced_Infantry_Number_Choose(void)
{
	if (KEY_SingleClick_R()) // ctrl+R 3
	{
		if (PRESSED_CTRL)
		{
            if(infantry3_flag==0&&infantry4_flag==0&&infantry5_flag==0)
			{
                infantry3_flag=1;
                infantry4_flag=0;
                infantry5_flag=0;
		    }
           else if(infantry3_flag==1&&infantry4_flag==0&&infantry5_flag==0)
			{
                infantry3_flag=0;
                infantry4_flag=1;
                infantry5_flag=0;
		    }
            else if(infantry3_flag==0&&infantry4_flag==1&&infantry5_flag==0)
			{
                infantry3_flag=0;
                infantry4_flag=0;
                infantry5_flag=1;
		    }
            else if(infantry3_flag==0&&infantry4_flag==0&&infantry5_flag==1)
			{
                infantry3_flag=0;
                infantry4_flag=0;
                infantry5_flag=0;
		    }
        }
		else
		{
                infantry3_flag=0;
                infantry4_flag=0;
                infantry5_flag=0;
		}
	}
	if(infantry3_flag == 1)
    {
       infantry_number = 3;   
    }
    else if(infantry4_flag == 1)
    {
       infantry_number = 4;
    }
    else if(infantry5_flag)
    {
       infantry_number = 5;
    }
    else
    {
       infantry_number = 0;
    }

}
