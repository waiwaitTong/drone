/** 比赛机器人状态：0x0201   发送频率：10Hz */

/** typedef __packed struct
{
	UCHAR8 robot_id;					   // 本机器人 ID
	UCHAR8 robot_level;					   // 机器人等级
	USHORT16 remain_HP;					   // 机器人剩余血量
	USHORT16 max_HP;					   // 机器人上限血量
	USHORT16 shooter_barrel_cooling_value; // 机器人枪口每秒冷却值
	USHORT16 shooter_barrel_heat_limit;	   // 机器人枪口热量上限
	USHORT16 chassis_power_limit;		   // 机器人底盘功率限制上限
	UCHAR8 mains_power_gimbal_output : 1;  // gimbal口输出:1有输出0无输出
	UCHAR8 mains_power_chassis_output : 1; // chassis口输出:1有输出0无输出
	UCHAR8 mains_power_shooter_output : 1; // shooter口输出:1有输出0无输出
} ext_game_robot_state_t; */ 
 //ext_game_robot_state_t Drone_State = {0};	//机器人状态								

 
 /** 实时功率热量数据：0x0202  发送频率：10Hz */
/*typedef __packed struct
{
	uint16_t reserved; 
	uint16_t reserved; 
	float reserved; 
	uint16_t buffer_energy; 
	uint16_t shooter_17mm_1_barrel_heat; //第一个发射机构的射击热量
	uint16_t shooter_42mm_barrel_heat; 
} ext_power_heat_data_t; */
//ext_power_heat_data_t Shoot_Power = {0};									// 当前射击热量值 

/** 实时射击信息：0x0207    发送频率：射击后发送 */
/*typedef __packed struct
{
	UCHAR8 bullet_type; // 弹丸种类
	UCHAR8 shooter_id;	// 发射机构 ID
	UCHAR8 bullet_freq; // 发射频率
	FP32 bullet_speed;	// 弹丸出射速度
} ext_shoot_data_t;*/
//ext_shoot_data_t Shoot_Data = {0};											// 射击状态   


/** 子弹剩余发射数：0x0208   发送频率：10Hz周期发送   发送范围：所有机器人发送 */
/*typedef __packed struct
{
	USHORT16 bullet_remaining_num_17mm; // 17mm 子弹剩余发射数目
	USHORT16 bullet_remaining_num_42mm; // 42mm 子弹剩余发射数目
	USHORT16 coin_remaining_num;		// 剩余金币数量
	USHORT16 projectile_allowance_fortress; //堡垒增益点提供的储备17mm弹丸允许发弹量；
} ext_bullet_remaining_t;				// 0x0208 子弹剩余 2字节*/
//ext_bullet_remaining_t Ammo_remain = {0};									// 剩余子弹数   ID:0208 