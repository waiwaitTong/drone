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