#include "supply_pellet_task.h"

#define SupplyStep  36864           //8192*36/8=36864  步长   8192*36/12*减速比=57017
//#define SupplyStep  19664
/*--------------------------------------------------------------------------
函数名：SupplyPelletControl()
功  能：拨弹控制
--------------------------------------------------------------------------*/
static u16 Freq_Cntsingle=0;
u8      shooting_or_not_s=0,shooting_or_not_r=0;
u8      danger_flag=0;
s32     shooterspeed = 200;  //纯速度环的目标值
u16     total = 100;
u8      Freq_test=0;
u8      Shoot_freq=0;
int     freq_test_time=0;
u8      Shoot_des_freq=20  ;   //20
u32     Shootnumber,ShootDesnumber=100;
u32     Shoot_pre_number=0;
u8      shoot_testflag=0;
FP32    shooterDes=0,shooterDes1=0;
FP32    Des_supply=0;
FP32    Supply_Speed_Des=0;
/*****************************************************
 * 拨弹电机控制逻辑
 * （模式调度 → 拨弹闭环 → 输出电流）
 *****************************************************/
void SupplyPelletControl(void)
{
    /* ========== 测试模式优先级最高 ========== */
    if (g_stTestFlag.SupplyPelletTestFlag == TRUE)
    {
        Supply_PosPID.fpDes = Des_supply;   // 外部测试目标
        PelletSupply_Loop();                // 闭环执行
        Send_Current_To_SupplyPelletMotor(CAN2, (SSHORT16)Supply_SpeedPID.fpU);
        return;                             // 测试模式提前退出
    }

    /* ========== 正常工作模式 ========== */
    if (!RC_ON)  // 遥控器未连接
    {
        Supply_PosPID.fpDes = Supply_PosPID.fpFB;   // 位置保持
        Supply_SpeedPID.fpU = 0;                    // 停止输出
        Send_Current_To_SupplyPelletMotor(CAN2, 0);
        return;
    }

    /* ========== 根据工作模式选择拨弹策略 ========== */
    switch (g_emOperation_Mode)
    {
        case RC_Mode:                     // 遥控器控制（默认静止）
            Supply_PosPID.fpDes  = Supply_PosPID.fpFB;
            Supply_SpeedPID.fpU  = 0;
            Supply_Speed_Des     = 0;
            break;

        case SingleShoot_Mode:            // 单发
            Gimbal_Control = NO_Aim_Mode;
            SupplyMotor_Single_Mode();
            break;

        case KeyMouse_Mode:               // 键鼠
            SupplyMotor_KeyMouse_Mode();
            break;

        case SpeedShoot_Mode:             // 高速连发
            Gimbal_Control = NO_Aim_Mode;
            SupplyMotor_Speed_Mode();
            break;

        case VisionSpeed_Mode:            // 视觉高速连发
            SupplyMotor_Speed_Mode();
            break;

        case VisionSingle_Mode:           // 视觉单发
            SupplyMotor_Single_Mode();
            break;

        case ImageControl_Mode:           // 图像算法控制（类似键鼠）
            SupplyMotor_KeyMouse_Mode();
            break;

        case Secuirty_Mode:               // 安全模式（强制停机）
            Supply_PosPID.fpDes  = Supply_PosPID.fpFB;
            Supply_SpeedPID.fpU  = 0;
            break;

        default:                          // 未知模式 → 安全处理
            Supply_PosPID.fpDes  = Supply_PosPID.fpFB;
            Supply_SpeedPID.fpU  = 0;
            Supply_Speed_Des     = 0;
            break;
    }

    /* ========== 闭环控制执行（只要不是安全停机） ========== */
    if (g_emOperation_Mode != Secuirty_Mode)
    {
        PelletSupply_Loop();
    }

    /* ========== 输出电流 ========== */
    Send_Current_To_SupplyPelletMotor(CAN2, (SSHORT16)Supply_SpeedPID.fpU);

    /* ========== 更新射击发数统计 ========== */
    Shootnumber = Supply_PosPID.fpFB / SupplyStep;
}
/*--------------------------------------------------------------------------
函数名：PelletSupply_Loop()
功  能：拨弹电机闭环控制
--------------------------------------------------------------------------*/
void PelletSupply_Loop(void)
{
     CalIWeakenPID(&Supply_PosPID);
	 Supply_SpeedPID.fpDes = Supply_PosPID.fpU/8192 * 360.0f;//rpm
	//////////////////////////////////防止快速拨弹
	if(Supply_SpeedPID.fpDes < -200)
	{
		Supply_SpeedPID.fpDes = -200;
	}
//	 
//     Supply_SpeedPID.fpDes=-Supply_Speed_Des;
	 CalIWeakenPID(&Supply_SpeedPID);
}

/*--------------------------------------------------------------------------
函数名：SupplyMotor_KeyMouse_Mode()
功  能：键鼠模式下拨弹控制
--------------------------------------------------------------------------*/
float Bullet_prespeed = 0 ;
int  Bullet_num_actaul = 0;
int  Bullet_num_fb=0;

u8 shoot_freq_actual=0;
int shoot_num_buchang = 0;
void SupplyMotor_KeyMouse_Mode(void)
{
	 
	if (g_emOperation_Mode == KeyMouse_Mode)
	{
			if(DR16_rec.stMouse.Left)  //打开拨弹电机
		  {
				SupplyMotor_Speed_Mode();
		  }
	 
    }
	
	if (g_emOperation_Mode == ImageControl_Mode)
	{
        if(remote_control_image_transmission.left_button_down 
//           && g_stFriction2SMC.fpFB >DesSpeed1-400 &&
//            g_stFriction1SMC.fpFB < -(DesSpeed1-400)
		)  //按下左键且摩擦轮转速达到目标值附近打开拨弹电机
		  {
				SupplyImage_Speed_Mode();
		  }
          else 
          {
              Supply_PosPID.fpDes=Supply_PosPID.fpFB;
          }
	}
	////////////////////////////////////////打弹拨盘补偿
//    	if (g_emOperation_Mode == ImageControl_Mode)
//	{
//		     static FP32 Last_shoottime_actual=0;

//			
//			if(remote_control_image_transmission.left_button_down )  //打开拨弹电机
//		  {
//				SupplyImage_Speed_Mode();
//			  
//			  
//			   if(Shoot_Data.bullet_speed != Bullet_prespeed)
//			 {
//				 Bullet_num_actaul ++ ;
//				 
//				 Bullet_prespeed = Shoot_Data.bullet_speed;
//			 }

//			 
//			 
//		  }
//		  
//		  
//		  
//		  else 
//	{
//		if(Shoot_Data.bullet_speed != Bullet_prespeed)
//		  {
//			  Bullet_num_actaul ++ ;
//			  Bullet_prespeed = Shoot_Data.bullet_speed;			  
//		  } 
//		  Bullet_num_fb = -Supply_PosPID.fpFB/(SupplyStep);
//	  if(Bullet_num_actaul > Bullet_num_fb)
//		 {
//			  shoot_freq_actual=1000/(system_monitor.System_time-Last_shoottime_actual);

//			 if(shoot_freq_actual<Shoot_des_freq)
//				{
//					Bullet_num_fb = -Supply_PosPID.fpFB/(SupplyStep);
//					 if(shoot_num_buchang<( Bullet_num_actaul -Bullet_num_fb-1 ))//////////////bucahng连发
//					 {
//						 Supply_PosPID.fpDes -=(SupplyStep);
//						 shoot_num_buchang++;
//						 Last_shoottime_actual=system_monitor.System_time;//记录上次打蛋时间
//					 }
//					 else{
//						 shoot_num_buchang=0;
//					 
//					 }

//				}		
//					 
//		 }
//		 else {
//			 Bullet_num_actaul=Bullet_num_fb;
//		 }
//	  }
//		  
//	  
//    }
}

/*--------------------------------------------------------------------------
函数名：SupplyMotor_Single_Mode()
功  能：遥控器控制单发模式
--------------------------------------------------------------------------*/
void SupplyMotor_Single_Mode(void)
{
  

	if(FrictionWheel_Ready==TRUE)
	{
			if(Freq_Cntsingle > 400)//计时
			{
				if(DR16_rec.stRC.Ch2==RC_CH_VALUE_MIN)//左摇杆水平通道364
				{
					Supply_PosPID.fpDes = Supply_PosPID.fpDes - (SupplyStep);   //目标值加步长
                  	Freq_Cntsingle = 0;						
				}
			}
		  else{	
              Freq_Cntsingle++; 
          }
	}
	else
	{
		Supply_PosPID.fpDes=Supply_PosPID.fpFB;//不动
		Supply_SpeedPID.fpU=0;
	}
}


/*----------------------------------------------------------------------------------------
函数名: SupplyMotor_Speed_Mode()
功  能: 连发模式
----------------------------------------------------------------------------------------*/
u8 shoot_freq=0;
int text_shoot_freq=0;
int shoot_num=0;
void SupplyMotor_Speed_Mode(void)
{
	/************** 实际射速测量 **************/
    freq_test_time++;
    static float Last_shoot_time = 0;

    float dt = system_monitor.System_time - Last_shoot_time;
    if (dt > 0)
        shoot_freq = 1000.0f / dt;            // 当前射击频率估算

    text_shoot_freq = (int)shoot_freq;        // 调试显示

   /************** 3. 遥控器触发 **************/
   uint8_t shoot_trig =
       (DR16_rec.stRC.Ch2 == RC_CH_VALUE_MIN) ||
      (DR16_rec.stRC.Ch2 == RC_CH_VALUE_MAX);

	
	if(FrictionWheel_Ready==TRUE)//判断摩擦轮速度是否达到目标值
	{
		if (shoot_trig)
		{
			//限制射击速度
        	if (shoot_freq < Shoot_des_freq)
        {
            // 热量控制

                Supply_PosPID.fpDes -= SupplyStep;//+为反转
                shoot_num++;
                Last_shoot_time = system_monitor.System_time;   // 更新射击时间
            }
        }
    }

    else    // 松开射击键
    {
       shoot_num = 0;
    }

    /************** 统计输出射速（每 30s） **************/
    if (freq_test_time >= 30000)
    {
        Shoot_freq = (Shootnumber - Shoot_pre_number) / 30;
        freq_test_time = 0;
        Shoot_pre_number = Shootnumber;
    }
}


/*----------------------------------------------------------------------------------------
函数名:SupplyImage_Speed_Mode(void)
功能:图传固定弹频连发模式
----------------------------------------------------------------------------------------*/

void SupplyImage_Speed_Mode(void)
{
     Supply_Speed_Des=Shoot_des_freq*0.125*60;
	 freq_test_time++;
     static FP32 Last_shoottime=0;
	 shoot_freq=1000/(system_monitor.System_time-Last_shoottime);
     text_shoot_freq=(int)shoot_freq;
	//////////////////////////回退
		if((fabs(Supply_PosPID.fpDes)>fabs(Supply_PosPID.fpFB)+6*SupplyStep)&&fabs(Supply_SpeedPID.fpFB)<60)
		 {
			 Supply_PosPID.fpDes = Supply_PosPID.fpFB + SupplyStep;
		 }
     if(shoot_freq<Shoot_des_freq)
     {
             Supply_PosPID.fpDes -=(SupplyStep);
             Last_shoottime=system_monitor.System_time;//记录上次打蛋时间
     }
     

        if(freq_test_time >= 1000*30)
    {
        Shoot_freq=(Shootnumber-Shoot_pre_number)/30;
        freq_test_time=0;
        Shoot_pre_number=Shootnumber;	
    }
}
/*                              热量控制函数          */
float Heat_Left = 0.0f;				  //剩余热量
int Pre_shoot_num = 0;                    //		上次发射子弹颗数
s16 Allowed_PelletNum = 0;           //允许发射子弹颗数
bool Is_Heat_Safe(void)
{
	
	  static float Pre_ShootHeat = 0.0f;    //记录上一次枪口热量
    static float Pre_ShootSpeed = 0.0f;		//记录上一次弹丸速度
	static float Last_heat_time = 0.0f;		//上次热量更新的时间
	s32 Heatlimit = ShootHeat_Limit;      //枪口热量上限

    if(system_monitor.UART5_Rx_fps>4)         //裁判系统帧率正常
    {
        if(ShootSpeed_mes != Pre_ShootSpeed)
        {
            Allowed_PelletNum--;    //发射速度更新，说明打出弹丸，计数
        }
        else if(Is_Float_Equal(ShootHeat_mes, Pre_ShootHeat)==FALSE ||
                Is_Float_Equal(ShootHeat_mes, 0.0f)==TRUE)              //枪口热量更新周期为100ms
        {
            Allowed_PelletNum =(s16)((Heatlimit - ShootHeat_mes) / 10); //一个周期更新会计算该周期可发射弹丸数量
        }

        Pre_ShootHeat = ShootHeat_mes;  //记录枪口热量
        Pre_ShootSpeed = ShootSpeed_mes;   //记录弹丸速度
        Heat_Left = (float)Heatlimit - (float)ShootHeat_mes;
    }
    else
    {
		
        Heat_Left -=10.0f * (shoot_num - Pre_shoot_num);
		float dt = system_monitor.System_time - Last_heat_time;
        Heat_Left += ShooterHeat_Rate * dt / 1000.0f;   //热量恢复，冷却速率为每秒X点热量
		Last_heat_time = system_monitor.System_time;
        Heat_Left = Heat_Left<(float)Heatlimit ? Heat_Left:(float)Heatlimit;
        Heat_Left = Heat_Left>0.0f ? Heat_Left:0.0f;    //避免出现负数u8类型溢出
        Allowed_PelletNum = (s16)(Heat_Left/10.0f);
        Pre_shoot_num = shoot_num;
    }

    if( Allowed_PelletNum<=1||system_monitor.CAN_Rx_LeftFritionWheel_fps<=0||system_monitor.CAN_Rx_RightFritionWheel_fps<=0  )  return FALSE;  //不允许发送
    else                        return TRUE;            //允许发射
}


/*----------------------------------------------------------------------------------------
函数名:Shoot_freq_calculate(void)
功能:测试模式下弹频的计算
----------------------------------------------------------------------------------------*/
u32 Shoot_test_time_1,Shoot_test_time_2;
u8 Shoot_test_freq;
void Shoot_freq_calculate(void)
{
	static u8 Shoot_firstflag=0;
  if(!shoot_testflag) Shoot_test_time_1=system_monitor.System_time;
  if(shoot_testflag==2 && !Shoot_firstflag) 
	{
		Shoot_firstflag=1;
		Shoot_test_time_2=system_monitor.System_time;
	  Shoot_test_freq=Shootnumber*1000/(Shoot_test_time_2-Shoot_test_time_1);
	}
}



