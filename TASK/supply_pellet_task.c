#include "supply_pellet_task.h"

#define SupplyStep  57017           //8192*36/8=36864  步长   8192*36/12*减速比=57017
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
FP32    Shootencoder = 0;
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
void SupplyPelletControl(void)
{
	if(g_stTestFlag.SupplyPelletTestFlag==FALSE)
	{
		if(RC_ON)//(system_monitor.USART1_fps>=60)
	  {
		  switch(g_emOperation_Mode)
		  {
			  case RC_Mode://遥控Pitch、Yaw
               Supply_PosPID.fpDes  = Supply_PosPID.fpFB;
               Supply_SpeedPID.fpU  = 0;	
               Supply_Speed_Des=0;
              
			  break;	
			  case SingleShoot_Mode:
				 Gimbal_Control = NO_Aim_Mode;
//				 Gimbal_RC_Mode();
				 SupplyMotor_Single_Mode();
			  break;
			
			  case KeyMouse_Mode:

				   SupplyMotor_KeyMouse_Mode();
				
			  break;
			  case Secuirty_Mode:
                  
              break;				
			  case SpeedShoot_Mode:
				Gimbal_Control = NO_Aim_Mode;
//					Gimbal_RC_Mode();
 //         Gimbal_VisionControl_Mode();//视觉打弹用
				   SupplyMotor_Speed_Mode();
			  break;		
				case VisionSpeed_Mode:
				   SupplyMotor_Speed_Mode();
					 break;
				case VisionSingle_Mode:
				 SupplyMotor_Single_Mode();

					 break;
                case ImageControl_Mode:		
                    SupplyMotor_KeyMouse_Mode();

			  break; 
			  default:
				   Supply_PosPID.fpDes  = Supply_PosPID.fpFB;
                   Supply_SpeedPID.fpU  = 0;
                    Supply_Speed_Des=0;
			  break;
		  }
			PelletSupply_Loop();//拨弹电机闭环控制
			if(g_emOperation_Mode==Secuirty_Mode)//不添加的话由于先进安全模式再进运算，导致原先赋值为0的数重新赋值导致发送电流不为0
			{
				Supply_SpeedPID.fpU  = 0;		
				Supply_PosPID.fpDes  = Supply_PosPID.fpFB;
			}
			Send_Current_To_SupplyPelletMotor(CAN2, (SSHORT16)Supply_SpeedPID.fpU);
	  }
	 	else  
		{
			 Supply_PosPID.fpDes  = Supply_PosPID.fpFB;
             Supply_SpeedPID.fpU  = 0;
			 Send_Current_To_SupplyPelletMotor(CAN2, 0);	
		} 
           Shootnumber=Supply_PosPID.fpFB/(SupplyStep);
				
		
	}
    
	else if (g_stTestFlag.SupplyPelletTestFlag==TRUE)
	{
            Supply_PosPID.fpDes=Des_supply;
			PelletSupply_Loop();//拨弹电机闭环控制
			Send_Current_To_SupplyPelletMotor(CAN2, (SSHORT16)Supply_SpeedPID.fpU);	
	}
			
			
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
  

//	if(FrictionWheel_Ready==TRUE)
//	{
			if(Freq_Cntsingle > 400)//计时
			{
				if(DR16_rec.stRC.Ch2==RC_CH_VALUE_MIN)//左摇杆水平通道364
				{
					Supply_PosPID.fpDes = Supply_PosPID.fpDes - (SupplyStep);   //目标值加步长
                    Supply_Speed_Des=Shoot_des_freq * 0.125 * 60;
					Freq_Cntsingle = 0;						
				}
			}
		  else{	
              Freq_Cntsingle++; 
              Supply_Speed_Des=0;
          }
		  Shootencoder = Supply_PosPID.fpFB;
//	}
//	else
//	{
//		Supply_PosPID.fpDes=Supply_PosPID.fpFB;//不动
//		Supply_SpeedPID.fpU=0;
//	}
}

/*----------------------------------------------------------------------------------------
函数名:SupplyMotor_Speed_Mode(void)
功能:固定弹频连发模式
----------------------------------------------------------------------------------------*/
u8 shoot_freq=0;
int text_shoot_freq=0;
int shoot_num=0;
void SupplyMotor_Speed_Mode(void)
{
     Supply_Speed_Des=Shoot_des_freq*0.125*60;
	 freq_test_time++;
     static FP32 Last_shoottime=0;
	 shoot_freq=1000/(system_monitor.System_time-Last_shoottime);
     text_shoot_freq=(int)shoot_freq;
//		 if(FrictionWheel_Ready==TRUE&&g_stFriction2SMC.fpFB>DesSpeed1-400&&g_stFriction1SMC.fpFB<-(DesSpeed1-400))
//		 if(g_stFriction2SMC.fpFB>DesSpeed1-400&&g_stFriction1SMC.fpFB<-(DesSpeed1-400))

//        {		
	if(DR16_rec.stRC.Ch2==RC_CH_VALUE_MIN)//左摇杆水平通道364
    {
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
     }
	if(DR16_rec.stRC.Ch2==RC_CH_VALUE_MAX)//左摇杆水平通道max
    {
		//////////////////////////回退
		if((fabs(Supply_PosPID.fpDes)>fabs(Supply_PosPID.fpFB)+6*SupplyStep)&&fabs(Supply_SpeedPID.fpFB)<60)
		 {
			 Supply_PosPID.fpDes = Supply_PosPID.fpFB + SupplyStep;
		 }
			 if(shoot_freq<Shoot_des_freq)
	     {
             if(shoot_num<10)//////////////50连发
             {
                 Supply_PosPID.fpDes -=(SupplyStep);
                 shoot_num++;
		         Last_shoottime=system_monitor.System_time;//记录上次打蛋时间
             }

	     }
     }
    	if(DR16_rec.stRC.Ch2==RC_CH_VALUE_OFFSET)

            shoot_num=0;

//        if(abs_fl(Supply_PosPID.fpDes-Supply_PosPID.fpFB)>90*SupplyStep)
//            
//        Supply_PosPID.fpDes=Supply_PosPID.fpFB-90*SupplyStep;
//        
        
	 		if(freq_test_time >= 1000*30)
		{
			Shoot_freq=(Shootnumber-Shoot_pre_number)/30;
			freq_test_time=0;
			Shoot_pre_number=Shootnumber;	
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



